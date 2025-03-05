import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from mavros_msgs.msg import ObstacleDistance
import numpy as np
import time

class LidarToPX4(Node):
    def __init__(self):
        super().__init__('lidar_to_px4')
        
        # Subscribe to LiDAR PointCloud2 topic
        self.subscriber = self.create_subscription(PointCloud2, '/pointcloud', self.callback, 10)
        
        # Publisher for PX4 MAVROS obstacle avoidance
        self.publisher = self.create_publisher(ObstacleDistance, '/mavros/obstacle/send', 10)

    def callback(self, msg):
        points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        if not points:
            self.get_logger().warn("No valid LiDAR points received!")
            return

        # Convert points to polar coordinates
        angles = np.arctan2([y for x, y, z in points], [x for x, y, z in points]) * (180 / np.pi)
        distances = [np.linalg.norm([x, y, z]) * 100 for x, y, z in points]  # Convert to cm
        
        # Initialize 72 bins (5° increments)
        obstacle_distances = [65535] * 72
        bin_size = 5  # 5-degree bins
        
        for angle, distance in zip(angles, distances):
            index = int((angle + 180) // bin_size)  # Normalize angle to 0-71 bins
            if 0 <= index < 72:
                obstacle_distances[index] = min(obstacle_distances[index], distance)

        # Create OBSTACLE_DISTANCE message
        obs_msg = ObstacleDistance()
        obs_msg.time_usec = int(time.time() * 1e6)  # Timestamp in microseconds
        obs_msg.sensor_type = 0  # Default sensor type
        obs_msg.distances = obstacle_distances
        obs_msg.increment = bin_size
        obs_msg.min_distance = 10  # 10cm min
        obs_msg.max_distance = 4000  # 40m max
        obs_msg.angle_offset = 0  # 0° offset
        obs_msg.frame = 12  # MAV_FRAME_BODY_FRD
        
        self.publisher.publish(obs_msg)
        self.get_logger().info("Published ObstacleDistance to MAVROS!")

def main(args=None):
    rclpy.init(args=args)
    node = LidarToPX4()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
