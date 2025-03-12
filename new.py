import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from mavros_msgs.srv import SetMode
from mavros_msgs.msg import State, PositionTarget
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('sf45_mavros_obstacle_avoidance')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # Ensure reliability matches MAVROS
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.distance_thresh = 2.0  # Obstacle detection threshold (meters)
        self.alt_inc  = 15.0  # Increase altitude by 15m per avoidance step
        self.current_mode = None
        self.previous_mode = None
        self.last_valid_mode = None  # Store mode before obstacle detection
        self.w_alt  = None  # Initial altitude before avoidance
        self.ca_offset = 0.0  # Track total altitude increase
        self.lob = None  # Store last detected obstacle distance
        self.current_position = PoseStamped()

        # Subscribers
        self.create_subscription(PointCloud2, '/pointcloud', self.lidar_callback, 1)
        self.create_subscription(State, '/mavros/state', self.state_callback, 1)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.altitude_callback, qos_profile)

        # Publishers
        self.local_pos_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)

        # Clients
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
    
    def set_mode(self, mode):
        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for /mavros/set_mode service...')
        
        request = SetMode.Request()
        request.custom_mode = mode
        future = self.set_mode_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() and future.result().mode_sent:
            self.get_logger().info(f'Mode changed to {mode}')
            return True
        else:
            self.get_logger().warn(f'Failed to change mode to {mode}')
            return False

    def state_callback(self, msg):
        if self.current_mode and self.current_mode != 'AUTO.LOITER':
            self.last_valid_mode = self.current_mode  # Store last valid mode before loiter
        self.current_mode = msg.mode

    def lidar_callback(self, data):
        points = list(pc2.read_points(data, field_names=("x", "y", "z"), skip_nans=True))
        if not points:
            self.get_logger().warn(" lawde nhi mila LiDAR se")
            return
        
        distances = np.sqrt(np.array([p[0] for p in points]) ** 2 + np.array([p[1] for p in points]) ** 2)
        valid_distances = [d for d in distances if not np.isclose(d, 1.00, atol=0.01)]
        if not valid_distances:
            self.get_logger().warn("All valid distances filtered out, skipping obstacle check.")
            return

        min_distance = np.min(valid_distances)
        self.get_logger().info(f"Closest obstacle distance: {min_distance:.2f}m")

        if self.lob is not None and np.isclose(min_distance, self.lob, atol=0.05):
            self.get_logger().info("Obstacle distance unchanged, ignoring reading.")
            return

        self.lob = min_distance

        if min_distance < self.distance_thresh:
            self.get_logger().warn("Obstacle detected! Switching to OFFBOARD and increasing altitude!")
            if self.w_alt  is None:
                self.w_alt  = self.get_current_altitude()
            self.previous_mode = self.current_mode  
            self.set_mode("AUTO.LOITER")
            if self.set_mode("OFFBOARD"):
                self.increase_altitude_offboard()

    def increase_altitude_offboard(self):
        if self.w_alt  is None:
            self.get_logger().warn("Base not set, skipping altitude increase.")
            return
        
        new_altitude = self.w_alt  + self.alt_inc 
        pose = PoseStamped()
        pose.pose.position.x = self.current_position.pose.position.x
        pose.pose.position.y = self.current_position.pose.position.y

        self.get_logger().info("Publishing initial position setpoints before OFFBOARD mode...")
        for _ in range(100):
            pose.pose.position.z = self.w_alt 
            self.local_pos_pub.publish(pose)
            rclpy.sleep(0.05)

        if self.set_mode("OFFBOARD"):
            self.get_logger().info("OFFBOARD mode enabled!")
        else:
            self.get_logger().warn("OFFBOARD mode failed to enable!")

        step = 3.0
        while self.current_position.pose.position.z + 0.1 < new_altitude:
            pose.pose.position.z += step
            if pose.pose.position.z > new_altitude:
                pose.pose.position.z = new_altitude
            self.get_logger().info(f"New altitude target: {pose.pose.position.z:.2f}m")
            for _ in range(50):
                self.local_pos_pub.publish(pose)
                rclpy.sleep(0.1)

        self.get_logger().info("Target altitude reached, rechecking for obstacles...")
        self.verify_obstacle_clearance()

    def verify_obstacle_clearance(self):
        self.get_logger().info("Rechecking for obstacles...")
        if self.current_mode == "OFFBOARD":
            self.set_mode(self.previous_mode)
            if self.previous_mode != "OFFBOARD" and self.previous_mode in ["AUTO.MISSION", "AUTO.RTL"]:
                success = self.set_mode(self.previous_mode)
                if success:
                    self.get_logger().info(f"Successfully switched back to {self.previous_mode}")
                else:
                    self.get_logger().warn(f"Failed to switch back to {self.previous_mode}, retrying...")
                    rclpy.sleep(2)
                    self.set_mode(self.previous_mode)
            else:
                self.set_mode("AUTO.LOITER")

    def altitude_callback(self, data):
        self.current_position = data

    def get_current_altitude(self):
        return self.current_position.pose.position.z


def main():
    rclpy.init()
    node = ObstacleAvoidance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
