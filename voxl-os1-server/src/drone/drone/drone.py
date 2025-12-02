#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

import threading
import time
import os
import sys

from pymavlink import mavutil

class drone(Node):
    def __init__(self):
        super().__init__('drone')
        self.get_logger().info('Starting drone connection')

        self.declare_parameter('simulation')
        self.declare_parameter('simulation_ip')
        self.declare_parameter('simulation_port')
        self.declare_parameter('connection_path')
        self.declare_parameter('baudrate')

        self.simulation = self.get_parameter('simulation').value
        self.simulation_ip = self.get_parameter('simulation_ip').value
        self.simulation_port = self.get_parameter('simulation_port').value
        self.connection_path = self.get_parameter('connection_path').value
        self.baudrate = self.get_parameter('baudrate').value

        self.armed_timer = self.create_timer(.1, self.checkArmed)
        self.disarmed_timer = self.create_timer(.1, self.checkDisarmed)
        
        self.armed_publisher = self.create_publisher(Bool, "/ouster/start", 10)
        self.disarmed_publisher = self.create_publisher(Bool, "/ouster/stop", 10)

        self.drone = None
        self.disarmed = True
        self.connected = False
        self.connectDrone()

    def armDrone(self):
        self.drone.mav.command_long_send(
            self.drone.target_system,
            self.drone.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0)
    
    def connectDrone(self):
        while(not self.connected):
            try:
                if(not self.simulation):
                    # Assuming serial connection
                    self.drone = mavutil.mavlink_connection(self.connection_path, baud = self.baudrate)
                else:
                    # Assuming connection over udp
                    self.drone = mavutil.mavlink_connection('udpin:' + str(self.simulation_ip) + ':' + str(self.simulation_port))
                self.drone.wait_heartbeat()
                self.get_logger().info("Connected to drone!")
            except Exception as e:
                pass
            time.sleep(1)
                
    def checkArmed(self):
        #Check message from drone
        self.drone.recv_match(type = 'HEARTBEAT', blocking = False)
        if(self.drone.motors_armed() and self.disarmed):
            self.disarmed = False
            self.get_logger().info("Drone is armed!")

    def checkDisarmed(self):
        self.drone.recv_match(type = 'HEARTBEAT', blocking = False)
        if(not self.drone.motors_armed()):
            self.disarmed = True

def main(args=None):
    rclpy.init(args=args)
    drone_init = drone()
    rclpy.spin(drone_init)
    rclpy.destroy_node(drone_init)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

