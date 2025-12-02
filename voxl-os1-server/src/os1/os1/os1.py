#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

import threading
import time
import os
import sys
import subprocess
import ouster
import shutil

from contextlib import closing
from datetime import datetime
from ouster import client, pcap
from more_itertools import time_limited
from ouster.sdk.examples.pcap import pcap_to_ply

class os1(Node):
    def __init__(self):
        super().__init__('os1')
        self.get_logger().info('Starting os1 connection')

        self.declare_parameter('os1_ip')
        self.declare_parameter('os1_lidar_port')
        self.declare_parameter('os1_imu_port')

        self.os1_ip = self.get_parameter('os1_ip').value
        self.os1_lidar_port = self.get_parameter('os1_lidar_port').value
        self.os1_imu_port = self.get_parameter('os1_imu_port').value

        self.os1_connect_subscriber = self.create_subscription(
            Bool, '/os1/connect', self.connectOs1, 10)
        self.os1_disconnect_subscriber = self.create_subscription(
            Bool, '/os1/disconnect', self.disconnectOs1, 10)

        self.fname_base = ''

        while(True):
            try:
                self.config = client.SensorConfig()
                self.config.udp_port_lidar = self.os1_lidar_port
                self.config.udp_port_imu = self.os1_imu_port
                self.config.operating_mode = client.OperatingMode.OPERATING_NORMAL
                client.set_config(self.os1_ip, self.config, persist=True, udp_dest_auto=True)
                break
            except Exception as e:
                time.sleep(1)
                print(e)
                pass
        
        self.source = None

    def connectOs1(self, msg):
        def run(**kwargs):
            self.get_logger().info("Recieved arm command, starting recording")
            try:
                with closing(client.Sensor(self.os1_ip, self.os1_lidar_port, self.os1_imu_port)) as self.source:
                    time_part = datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
                    meta = self.source.metadata
                    self.fname_base = f"/media/ouster/{meta.prod_line}_{meta.sn}_{meta.mode}_{time_part}"
                    self.source.write_metadata(f"{self.fname_base}.json")
                    source_it = time_limited(86400, self.source)
                    n_packets = pcap.record(source_it, f"{self.fname_base}.pcap")
            except Exception as e:
                self.get_logger().info(str(e))
                pass

        threading.Thread(target=run, args=()).start()

    def disconnectOs1(self, msg):
        try:
            self.source.close()
            self.get_logger().info("Closed OS1 source")
            
            time.sleep(0.5)

            self.get_logger().info("Starting conversions")

            subprocess.run(['ouster-cli', 'mapping', 'slam', f'{self.fname_base}.pcap', '-o', f'{self.fname_base}.osf'])
            self.get_logger().info("Finished conversion to OSF - starting LAS conversion")

            subprocess.run(['ouster-cli', 'mapping', 'convert', f'{self.fname_base}.osf', f'{self.fname_base}.las'])
            self.get_logger().info("Finished LAS conversion")

            for fname in os.listdir('.'):
                if(fname.endswith('.las')):
                    shutil.move(fname, f'{self.fname_base}.las')

        except Exception as e:
            self.get_logger().info(str(e))
            pass

def main(args=None):
    rclpy.init(args=args)
    os1_init = os1()
    rclpy.spin(os1_init)
    rclpy.destroy_node(os1_init)
    rclpy.shutdown()

if __name__ == '__main__':
    main()


