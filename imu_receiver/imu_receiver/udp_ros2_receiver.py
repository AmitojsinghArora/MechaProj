#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import socket
import json
from geometry_msgs.msg import TwistStamped

class UDPReceiverNode(Node):
    def __init__(self):
        super().__init__('udp_receiver_node')
        self.declare_parameter('udp_port', 12345)  # Default port to match ESP32
        self.imu_pub= self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)

        self.udp_port = self.get_parameter('udp_port').value
        self.get_logger().info(f'Listening for UDP packets on port {self.udp_port}...')
        
        # Setup UDP Socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', self.udp_port))  # Listen on all available interfaces
        
        # Create a timer to periodically check for data
        self.timer = self.create_timer(0.05, self.receive_data)  # 50ms (20Hz loop)
    
    def receive_data(self):
        try:
            self.sock.settimeout(0.01)  # Non-blocking mode
            data, addr = self.sock.recvfrom(1024)  # Buffer size of 1024 bytes
            data = data.decode('utf-8')
            
            # Parse JSON data
            parsed_data = json.loads(data)
            self.get_logger().info(f'Received data from {addr}: {parsed_data}')

            imu_msg = TwistStamped()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = "base_link"
            imu_msg.twist.linear.x = parsed_data.get("ux", 0.0)
            imu_msg.twist.linear.y = parsed_data.get('uy', 0.0)
            imu_msg.twist.linear.z = parsed_data.get('uz', 0.0)
            imu_msg.twist.angular.x = parsed_data.get('wx', 0.0)
            imu_msg.twist.angular.y = parsed_data.get('wy', 0.0)
            imu_msg.twist.angular.z = parsed_data.get('wz', 0.0)

            self.imu_pub.publish(imu_msg)

        except socket.timeout:
            pass  # No data received, continue
        except json.JSONDecodeError:
            self.get_logger().warn('Received malformed JSON data.')
        except Exception as e:
            self.get_logger().error(f'Error receiving data: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = UDPReceiverNode()
    rclpy.spin(node)
    
    node.sock.close()  # Close UDP socket on shutdown
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
