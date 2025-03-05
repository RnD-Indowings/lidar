import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from mavros_msgs.msg import DistanceSensor

class LaserToMavlink(Node):
    def __init__(self):
        super().__init__('laser_to_mavlink')
        self.subscription = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        self.publisher = self.create_publisher(DistanceSensor, '/mavros/distance_sensor', 10)

    def laser_callback(self, msg):
        if len(msg.ranges) == 0:
            return  # No data, exit
        
        distance_msg = DistanceSensor()
        distance_msg.header = msg.header
        distance_msg.min_range = msg.range_min
        distance_msg.max_range = msg.range_max
        distance_msg.current_distance = min(msg.ranges)  # Take the closest detected point
        distance_msg.type = DistanceSensor.MAV_DISTANCE_SENSOR_LASER
        distance_msg.id = 0
        distance_msg.orientation = DistanceSensor.ROTATION_FORWARD
        distance_msg.field_of_view = 0.1  # Adjust based on sensor specs

        self.publisher.publish(distance_msg)

rclpy.init()
node = LaserToMavlink()
rclpy.spin(node)
node.destroy_node()
rclpy.shutdown()
