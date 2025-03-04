import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Range
from sensor_msgs_py import point_cloud2 as pc2
import numpy as np

class LidarToRange(Node):
    def __init__(self):
        super().__init__('lidar_to_range')
        self.subscriber = self.create_subscription(PointCloud2, '/pointcloud', self.callback, 10)
        self.publisher = self.create_publisher(Range, '/mavros/distance_sensor/rangefinder', 10)

    def callback(self, msg):
        points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        if not points:
            return

        # Extract nearest point (smallest Z value)
        distances = [np.linalg.norm([x, y, z]) for x, y, z in points]
        min_distance = min(distances)

        range_msg = Range()
        range_msg.header = msg.header
        range_msg.radiation_type = Range.INFRARED
        range_msg.field_of_view = 0.1  # Adjust based on LiDAR spec
        range_msg.min_range = 0.1
        range_msg.max_range = 50.0  # Max range of SF45B
        range_msg.range = float(min_distance)

        self.publisher.publish(range_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LidarToRange()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
