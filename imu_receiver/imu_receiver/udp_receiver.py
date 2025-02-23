import rclpy
from rclpy.node import Node
import socket
import json

class UDPReceiverNode(Node):
    def __init__(self):
        super().__init__('udp_receiver_node')
        
        # UDP settings
        self.udp_ip = "192.168.43.124"
        #self.udp_ip = "192.168.118.156"  # IP address to listen on
        self.udp_port = 12345           # Port to listen on
        
        # Create a UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.udp_ip, self.udp_port))
        
        self.get_logger().info(f"Listening for UDP packets on {self.udp_ip}:{self.udp_port}")
        
        # Timer to periodically check for incoming data
        self.timer = self.create_timer(0.01, self.check_for_data)  # 100 Hz

    def check_for_data(self):
        try:
            # Check if there is any data available to receive
            data, addr = self.sock.recvfrom(1024)  # Buffer size is 1024 bytes
            self.process_data(data)
        except socket.error as e:
            self.get_logger().error(f"Socket error: {e}")

    def process_data(self, data):
        try:
            # Decode the received data
            data_str = data.decode('utf-8')
            self.get_logger().info(f"Received data: {data_str}")
            
            # Parse the JSON data
            data_dict = json.loads(data_str)
            
            # Extract the values
            ux = data_dict.get("ux", 0.0)
            uy = data_dict.get("uy", 0.0)
            uz = data_dict.get("uz", 0.0)
            wx = data_dict.get("wx", 0.0)
            wy = data_dict.get("wy", 0.0)
            wz = data_dict.get("wz", 0.0)
            
            # Log the extracted values
            self.get_logger().info(f"Linear Velocities: ux={ux}, uy={uy}, uz={uz}")
            self.get_logger().info(f"Angular Velocities: wx={wx}, wy={wy}, wz={wz}")
            
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to decode JSON: {e}")
        except Exception as e:
            self.get_logger().error(f"Error processing data: {e}")

def main(args=None):
    rclpy.init(args=args)
    udp_receiver_node = UDPReceiverNode()
    rclpy.spin(udp_receiver_node)
    udp_receiver_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
