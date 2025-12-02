#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import Bool
from rclpy.executors import Executor, MultiThreadedExecutor

import threading
import os
import math
import asyncio
import json
import time
from mavsdk import System
from mavsdk.offboard import (OffboardError, PositionNedYaw)


class mavsdk_drone(Node):

    def __init__(self):

        super().__init__('mavsdk_drone')
        self.get_logger().info("Starting mavsdk_drone")
        self.loop = asyncio.get_event_loop()

        self.declare_parameter('udp_bool')
        self.declare_parameter('dev_path')
        self.declare_parameter('baudrate')
        self.udp_bool = self.get_parameter('udp_bool').value
        self.dev_path = self.get_parameter('dev_path').value
        self.baudrate = self.get_parameter('baudrate').value
        
        self.os1Start = self.create_publisher(Bool, '/os1/connect', 10)
        self.os1Stop = self.create_publisher(Bool, '/os1/disconnect', 10)

        self.drone = None
        self.sendOs1Command = False

        self.drone_init()
        self.loop.create_task(self.handle_services())
        self.loop.run_forever()

    def drone_init(self):

        async def connect():
            self.get_logger().info("Starting connect sequence")
            self.drone = System()
            
            if (self.udp_bool):
                self.get_logger().info("Connecting over UDP")
                await self.drone.connect(system_address="udp://:14551")
            else:
                self.get_logger().info("Connecting over Serial")
                await self.drone.connect(system_address="serial://" + self.dev_path + ":" + self.baudrate)

            self.get_logger().info("Waiting for drone to connect...")
            async for state in self.drone.core.connection_state():
                if state.is_connected:
                    self.get_logger().info("Connected to drone!")
                    break
            self.get_arming_state()

        self.loop.create_task(connect())

    async def handle_services(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)
            await asyncio.sleep(0.01)

    def get_arming_state(self):
        
        async def getArmingState(**kwargs):
            async for is_armed in self.drone.telemetry.armed():
                if(is_armed and not self.sendOs1Command):
                    os1_start = Bool()
                    os1_start.data = True
                    self.os1Start.publish(os1_start)
                    self.sendOs1Command = True
                if(not is_armed and self.sendOs1Command):
                    os1_stop = Bool()
                    os1_stop.data = True
                    self.os1Stop.publish(os1_stop)
                    self.sendOs1Command = False

        self.loop.create_task(getArmingState())

def main(args=None):
    rclpy.init(args=args)
    mavsdk_init = mavsdk_drone()
    rclpy.spin(mavsdk_init)
    rclpy.destroy_node(mavsdk_init)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

