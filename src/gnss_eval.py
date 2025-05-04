import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

import numpy as np
import pyproj
import logging
import pytz
from datetime import datetime
import os


logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s,%(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)
logger = logging.getLogger(__name__)

def get_utm_coordinates(latitude, longitude):
    # Define the source (WGS84) and target (UTM) coordinate systems
    wgs84 = pyproj.CRS("EPSG:4326")  # WGS84 coordinate system
    crs = pyproj.CRS("EPSG:32652")  # UTM Zone 52N coordinate system (for Daegu South Korea)
    # Create a transformer for the conversion
    transformer = pyproj.Transformer.from_crs(wgs84, crs, always_xy=True)
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

        # get parameters
        self.gt_lat = self.get_parameter('ground_truth.latitude').get_parameter_value().double_value
        self.gt_lon = self.get_parameter('ground_truth.longitude').get_parameter_value().double_value
        self.gt_alt = self.get_parameter('ground_truth.altitude').get_parameter_value().double_value
        self.log_enable = self.get_parameter('log_enable').value
        self.log_path = self.get_parameter('log_path').value

        self.get_logger().info(f'ground_truth: lat({self.gt_lat}), lon({self.gt_lon}), alt({self.gt_alt})')
        self.get_logger().info(f'log_enable: {self.log_enable}')
        self.get_logger().info(f'log_path: {self.log_path}')


        gt_utm_easting, gt_utm_northing = get_utm_coordinates(self.gt_lat, self.gt_lon)
        self.gt_utm = {
            'easting': gt_utm_easting,
            'northing': gt_utm_northing
        }

        # setting up logger
        os.makedirs(self.log_path, exist_ok=True)
        kst = pytz.timezone('Asia/Seoul')
        current_time = datetime.now(pytz.utc).astimezone(kst)
        if self.log_enable:
            file_handler = logging.FileHandler(os.path.join(self.log_path, f'gnss_eval_{current_time}.csv'))
            file_handler.setLevel(logging.DEBUG)
            file_handler.setFormatter(logging.Formatter('%(asctime)s,%(message)s'))
            logger.addHandler(file_handler)

        self.sub_fix = self.create_subscription(NavSatFix, '/fix', self.sub_fix_callback, 10)

    
    def sub_fix_callback(self, msg):
        hpe_coords, hpe_dist = self.evaluate(msg.latitude, msg.longitude, msg.altitude)
        self.get_logger().info(f'lat: {msg.latitude}, lon: {msg.longitude}, hpe_coords: {hpe_coords}, hpe_dist: {hpe_dist} m')
        logger.info(f' lat: {msg.latitude}, lon: {msg.longitude}, hpe_coords: {hpe_coords}, hpe_dist: {hpe_dist} m')


    def evaluate(self, lat, lon, alt):
        utm_easting, utm_northing = get_utm_coordinates(lat, lon)

        hpe_coords = np.array(
            [   float(utm_easting) - float(self.gt_utm['easting']) ,
                float(utm_northing) - float(self.gt_utm['northing']) ]
        )

        hpe_dist = np.sqrt(np.sum(hpe_coords**2))

        return hpe_coords, hpe_dist




def main(args=None):
    rclpy.init(args=args)
    gnss_eval = GnssEval()
    rclpy.spin(gnss_eval)
    gnss_eval.destroy_node()
    rclpy.shutdown()