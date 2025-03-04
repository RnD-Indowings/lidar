import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from mavros_msgs.msg import OBSTACLE_DISTANCE
from sensor_msgs_py.point_cloud2 import read_points
import numpy as np
import time

class LidarToObstacleDistance(Node):
    def __init__(self):
        super().__init__('lidar_to_obstacle_distance')
        self.subscriber = self.create_subscription(PointCloud2, '/pointcloud', self.callback, 10)
        self.publisher = self.create_publisher(OBSTACLE_DISTANCE, '/mavros/obstacle/send', 10)

    def callback(self, msg):
        points = list(read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        if not points:
            return
        
        # Convert point cloud to polar coordinates
        angles = np.arctan2([y for x, y, z in points], [x for x, y, z in points]) * 180.0 / np.pi
        distances = [np.linalg.norm([x, y, z]) * 100 for x, y, z in points]  # Convert to cm

        # Create 72-bin obstacle distance array
        bin_size = 360 / 72
        obstacle_distances = [65535] * 72  # Default to no obstacle

        for angle, distance in zip(angles, distances):
            bin_index = int((angle + 180) // bin_size) % 72  # Map to 0-71 index
            obstacle_distances[bin_index] = min(obstacle_distances[bin_index], distance)

        # Publish OBSTACLE_DISTANCE message
        obs_msg = OBSTACLE_DISTANCE()
        obs_msg.time_usec = int(time.time() * 1e6)
        obs_msg.sensor_type = 0  # MAV_DISTANCE_SENSOR_LASER
        obs_msg.distances = obstacle_distances
        obs_msg.increment = int(bin_size)
        obs_msg.min_distance = 10  # 10 cm
        obs_msg.max_distance = 4000  # 40 meters (adjust if needed)
        obs_msg.angle_offset = 0.0
        obs_msg.frame = 12  # MAV_FRAME_BODY_FRD (forward-right-down)
        
        self.publisher.publish(obs_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LidarToObstacleDistance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
