#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import socket
import json
from geometry_msgs.msg import TwistStamped

from std_srvs.srv import Trigger

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

        # Create a service client to start the Servo node
        self.servo_start_client = self.create_client(Trigger, '/servo_node/start_servo')
        while not self.servo_start_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        
        self.send_request()
        
    def send_request(self):
        request = Trigger.Request()
        future = self.servo_start_client.call_async(request)
        future.add_done_callback(self.callback)

    def callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Service call succeeded: {response.success}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {str(e)}')

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


            # ux = parsed_data.get("ux", 0.0)
            # imu_msg.twist.linear.x = min(max(ux, -0.6), 0.6)

            # uy = parsed_data.get("uy", 0.0)
            # imu_msg.twist.linear.y = min(max(uy, -0.6), 0.6)

            # uz = parsed_data.get("uz", 0.0)
            # imu_msg.twist.linear.z = min(max(uz, -0.6), 0.6)

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
