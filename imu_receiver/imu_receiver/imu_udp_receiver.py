#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import socket
import json
from sensor_msgs.msg import Imu

# UDP settings
UDP_IP = "0.0.0.0"  # Listen on all interfaces
UDP_PORT = 12345

class IMUReceiverNode(Node):
    def __init__(self):
        super().__init__('imu_udp_receiver')  # Node name
        self.imu_pub = self.create_publisher(Imu, '/imu_data', 10)  # Publisher for IMU data
        
        # Create UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((UDP_IP, UDP_PORT))
        self.get_logger().info(f"Listening for UDP packets on port {UDP_PORT}")
        
        # Create a timer to handle receiving UDP packets in a non-blocking manner
        self.timer = self.create_timer(0.1, self.receive_udp_data)  # Call every 0.1s

    def receive_udp_data(self):
        try:
            data, _ = self.sock.recvfrom(1024)  # Buffer size (bytes)
            if not data:
                self.get_logger().warn("Received empty data.")
                return

            # Decode received data and parse as JSON
            data_str = data.decode('utf-8').strip()
            self.get_logger().info(f"Received data: {data_str}")

            # Try to load the data into a JSON object
            message = json.loads(data_str)
            
            # Parse data
            #ax = message["ax"]
            #ay = message["ay"]
            #az = message["az"]
            #wx = message["wx"]
            #wy = message["wy"]
            #wz = message["wz"]

             # Extract values from JSON
            ax = message.get("ax", 0.0)
            ay = message.get("ay", 0.0)
            az = message.get("az", 0.0)
            wx = message.get("wx", 0.0)
            wy = message.get("wy", 0.0)
            wz = message.get("wz", 0.0)

            # Create and publish IMU message
            imu_msg = Imu()
            imu_msg.linear_acceleration.x = ax
            imu_msg.linear_acceleration.y = ay
            imu_msg.linear_acceleration.z = az
            imu_msg.angular_velocity.x = wx
            imu_msg.angular_velocity.y = wy
            imu_msg.angular_velocity.z = wz

            self.imu_pub.publish(imu_msg)
            #self.get_logger().info(f"Published: {imu_msg}")
        except Exception as e:
            self.get_logger().error(f"Error receiving or processing data: {str(e)}")


def main(args=None):
    rclpy.init(args=args)

    # Create and spin the IMU receiver node
    imu_receiver_node = IMUReceiverNode()
    rclpy.spin(imu_receiver_node)

    # Cleanup after spinning
    imu_receiver_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
