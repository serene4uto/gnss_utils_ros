import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

import numpy as np
import pyproj
import logging
import pytz
from datetime import datetime
import os
import collections


logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s,%(message)s',
    datefmt='%Y-%m-%d,%H:%M:%S'
)
logger = logging.getLogger(__name__)

# Create transformer once as a global object
wgs84 = pyproj.CRS("EPSG:4326")  # WGS84 coordinate system
utm_crs = pyproj.CRS("EPSG:32652")  # UTM Zone 52N coordinate system (for Daegu South Korea)
transformer = pyproj.Transformer.from_crs(wgs84, utm_crs, always_xy=True)

def get_utm_coordinates(latitude, longitude):
    # Use the global transformer for better performance
    utm_easting, utm_northing = transformer.transform(longitude, latitude)
    return utm_easting, utm_northing

class GnssEval(Node):
    def __init__(self):
        super().__init__('gnss_eval')
        self.get_logger().info('GNSS Eval Node Started')

        # ros2 params
        self.declare_parameter('log_enable', True)
        self.declare_parameter('log_path', '.gnss_eval_log')
        self.declare_parameter('ground_truth.latitude', 0.0)
        self.declare_parameter('ground_truth.longitude', 0.0)
        self.declare_parameter('ground_truth.altitude', 0.0)
        self.declare_parameter('log_frequency', 10)  # Only log every Nth message
        self.declare_parameter('rate_window_size', 50)  # Size of window for rate calculation

        # get parameters
        self.gt_lat = self.get_parameter('ground_truth.latitude').get_parameter_value().double_value
        self.gt_lon = self.get_parameter('ground_truth.longitude').get_parameter_value().double_value
        self.gt_alt = self.get_parameter('ground_truth.altitude').get_parameter_value().double_value
        self.log_enable = self.get_parameter('log_enable').value
        self.log_path = self.get_parameter('log_path').value
        self.log_frequency = self.get_parameter('log_frequency').get_parameter_value().integer_value
        self.rate_window_size = self.get_parameter('rate_window_size').get_parameter_value().integer_value

        self.get_logger().info(f'ground_truth: lat({self.gt_lat}), lon({self.gt_lon}), alt({self.gt_alt})')
        self.get_logger().info(f'log_enable: {self.log_enable}')
        self.get_logger().info(f'log_path: {self.log_path}')

        # Pre-compute ground truth UTM coordinates
        gt_utm_easting, gt_utm_northing = get_utm_coordinates(self.gt_lat, self.gt_lon)
        self.gt_utm_easting = float(gt_utm_easting)
        self.gt_utm_northing = float(gt_utm_northing)

        # Message counter and rate calculation
        self.msg_counter = 0
        self.msg_timestamps = collections.deque(maxlen=self.rate_window_size)
        self.current_msg_rate = 0.0

        # setting up logger
        if self.log_enable:
            os.makedirs(self.log_path, exist_ok=True)
            kst = pytz.timezone('Asia/Seoul')
            current_time = datetime.now(pytz.utc).astimezone(kst)
            # Format timestamp for filename - replace colons with underscores to be Linux-compatible
            timestamp_str = current_time.strftime("%Y-%m-%d_%H-%M-%S")
            file_handler = logging.FileHandler(os.path.join(self.log_path, f'gnss_eval_{timestamp_str}.csv'))
            file_handler.setLevel(logging.DEBUG)
            file_handler.setFormatter(logging.Formatter('%(asctime)s,%(message)s'))
            logger.addHandler(file_handler)

        self.sub_fix = self.create_subscription(NavSatFix, '/fix', self.sub_fix_callback, 10)

    def calculate_message_rate(self):
        if len(self.msg_timestamps) < 2:
            return 0.0
        
        # Calculate time difference between first and last message in the window
        time_diff = (self.msg_timestamps[-1] - self.msg_timestamps[0]).total_seconds()
        if time_diff <= 0:
            return 0.0
            
        # Calculate messages per second
        msg_rate = (len(self.msg_timestamps) - 1) / time_diff
        return msg_rate

    def sub_fix_callback(self, msg):
        # Record timestamp for rate calculation
        current_time = datetime.now()
        self.msg_timestamps.append(current_time)
        self.msg_counter += 1
        
        # Calculate message rate
        self.current_msg_rate = self.calculate_message_rate()
        
        # Fast path calculation
        hpe_coords, hpe_dist = self.evaluate(msg.latitude, msg.longitude, msg.altitude)
        
        # Only log at reduced frequency to improve performance
        if self.msg_counter % self.log_frequency == 0:
            self.get_logger().info(f'lat: {msg.latitude}, lon: {msg.longitude}, hpe_dist: {hpe_dist:.3f} m, msg_rate: {self.current_msg_rate:.2f} Hz')
            if self.log_enable:
                logger.info(f'lat: {msg.latitude}, lon: {msg.longitude}, hpe_coords: [{hpe_coords[0]:.3f},{hpe_coords[1]:.3f}], hpe_dist: {hpe_dist:.3f} m, msg_rate: {self.current_msg_rate:.2f} Hz')

    def evaluate(self, lat, lon, alt):
        # Get UTM coordinates efficiently
        utm_easting, utm_northing = get_utm_coordinates(lat, lon)
        
        # Use pre-allocated arrays for better performance
        hpe_coords = np.array([
            float(utm_easting) - self.gt_utm_easting,
            float(utm_northing) - self.gt_utm_northing
        ])
        
        # Faster NumPy calculation
        hpe_dist = np.linalg.norm(hpe_coords)

        return hpe_coords, hpe_dist

def main(args=None):
    rclpy.init(args=args)
    gnss_eval = GnssEval()
    rclpy.spin(gnss_eval)
    gnss_eval.destroy_node()
    rclpy.shutdown()