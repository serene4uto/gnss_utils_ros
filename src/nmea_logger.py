import rclpy
from rclpy.node import Node
from nmea_msgs.msg import Sentence
import os
from datetime import datetime

class NMEALogger(Node):
    def __init__(self):
        super().__init__('nmea_logger')
        self.declare_parameter('log_file', 'nmea_log.txt')  # Default log file
        log_filename = self.get_parameter('log_file').value

        # Get full path
        self.log_file = os.path.expanduser(log_filename)
        self.subscription = self.create_subscription(
            Sentence,
            '/nmea',
            self.nmea_callback,
            10)
        self.get_logger().info(f"NMEA Logger started. Logging to: {self.log_file}")

    def nmea_callback(self, msg):
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        log_entry = f"[{timestamp}] {msg.sentence}\n"
        with open(self.log_file, 'a') as f:
            f.write(log_entry)
        self.get_logger().info(f"Logged: {msg.sentence}")

def main(args=None):
    rclpy.init(args=args)
    node = NMEALogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
