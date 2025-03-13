#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from mavros_msgs.srv import SetMode, CommandBool
from mavros_msgs.msg import State, PositionTarget
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# MAVROS parameters
min_threshold_distance = 0.0
max_threshold_distance = 2.0  # Obstacle detection threshold (meters)
altitude_increment = 20.0  # Increase altitude per avoidance step
min_altitude = 5.0  # Minimum altitude before avoidance is allowed

# State variables
current_mode = None
previous_mode = None
last_valid_mode = None
base_altitude = None
current_altitude_offset = 0.0
last_obstacle_distance = None

class MAVROSObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('sf45_mavros_obstacle_avoidance')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # Ensure reliability matches MAVROS
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribers
        self.create_subscription(PointCloud2, "/pointcloud", self.lidar_callback, 10)
        self.create_subscription(State, "/mavros/state", self.state_callback, 10)
        self.create_subscription(PoseStamped, "/mavros/local_position/pose", self.altitude_callback, qos_profile)

        # Publisher
        self.local_pos_pub = self.create_publisher(PoseStamped, "/mavros/setpoint_position/local", 10)
        self.current_position = PoseStamped()

        # Clients for MAVROS services
        self.set_mode_client = self.create_client(SetMode, "/mavros/set_mode")
        self.arming_client = self.create_client(CommandBool, "/mavros/cmd/arming")

    def set_mode(self, mode):
        """ Change the flight mode """
        if not self.set_mode_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f"SetMode service not available!")
            return False
        
        request = SetMode.Request()
        request.custom_mode = mode
        future = self.set_mode_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() and future.result().mode_sent:
            self.get_logger().info(f"Mode changed to {mode}")
            return True
        else:
            self.get_logger().warn(f"Failed to change mode to {mode}")
            return False

    def state_callback(self, msg):
        """ Store the current MAVROS state """
        global current_mode, last_valid_mode
        if current_mode and current_mode != "AUTO.LOITER":
            last_valid_mode = current_mode  # Store last valid mode before loiter
        current_mode = msg.mode

    def lidar_callback(self, data):
        """ Process LiDAR data and detect obstacles """
        global current_mode, previous_mode, last_valid_mode, base_altitude, current_altitude_offset, last_obstacle_distance

        points = list(pc2.read_points(data, field_names=("x", "y", "z"), skip_nans=True))
        if len(points) == 0:
            self.get_logger().warn("No points received from SF45 LiDAR")
            return

        distances = np.sqrt(np.array([p[0] for p in points])**2 + np.array([p[1] for p in points])**2)
        valid_distances = [d for d in distances if not np.isclose(d, 1.00, atol=0.01)]

        if len(valid_distances) == 0:
            self.get_logger().warn("All valid distances were filtered out, skipping obstacle check.")
            return

        min_distance = np.min(valid_distances)
        self.get_logger().info(f"Closest obstacle distance: {min_distance:.2f}m")

        if last_obstacle_distance is not None and np.isclose(min_distance, last_obstacle_distance, atol=0.05):
            self.get_logger().info("Obstacle distance is the same as before, ignoring this reading.")
            return

        last_obstacle_distance = min_distance

        if min_threshold_distance < min_distance < max_threshold_distance:
            self.get_logger().warn("Obstacle detected! Switching to TAKEOFF and increasing altitude!")
            if base_altitude is None:
                base_altitude = self.get_current_altitude()

            if self.get_current_altitude() > min_altitude:
                previous_mode = current_mode
                self.set_mode("AUTO.LOITER")
                if self.set_mode("TAKEOFF"):
                    self.increase_altitude_takeoff()
            else:
                #self.get_logger().warn(f"Low altitude ({self.get_current_altitude()}m), avoiding altitude increase.")
                self.increase_altitude_takeoff()

    def increase_altitude_takeoff(self):
        """ Increase altitude in OFFBOARD mode """
        global current_altitude_offset, base_altitude

        if base_altitude is None:
            self.get_logger().warn("Base altitude not set, skipping altitude increase.")
            return

       

        pose = PoseStamped()
        pose.pose.position.x = self.current_position.pose.position.x
        pose.pose.position.y = self.current_position.pose.position.y
        new_altitude = self.get_current_altitude() + altitude_increment

        self.get_logger().info("Sending initial position setpoints before switching to OFFBOARD...")
        for _ in range(100):
            pose.pose.position.z = base_altitude
            self.local_pos_pub.publish(pose)
            #rclpy.sleep(0.05)

        self.get_logger().info("Attempting to switch to TAKEOFF mode...")
        if self.set_mode("TAKEOFF"):
            self.get_logger().info("TAKEOFF mode enabled!")

        step = 4.0
        while self.get_current_altitude() + 0.1 < new_altitude:
            alti = self.get_current_altitude()
            pose.pose.position.z += step
            if pose.pose.position.z > new_altitude:
                pose.pose.position.z = new_altitude
            if pose.pose.position.z < alti:
                pose.pose.position.z = alti
            self.get_logger().info(f"New altitude target: {pose.pose.position.z:.2f}m")
            for _ in range(50):
                self.local_pos_pub.publish(pose)
                #rclpy.sleep(0.1)

        self.get_logger().info("Target altitude reached, rechecking for obstacles...")
        self.verify_obstacle_clearance()

    def verify_obstacle_clearance(self):
        """ Recheck for obstacles and restore previous mode """
        global previous_mode
        self.get_logger().info("Rechecking for obstacles...")
        self.create_subscription(PointCloud2, "/pointcloud", self.lidar_callback, 10)

        if previous_mode != "OFFBOARD" and previous_mode == "AUTO.MISSION" or previous_mode == "AUTO.RTL":
            self.get_logger().info(f"Switching back to previous mode: {previous_mode}")
            if not self.set_mode(previous_mode):
                self.get_logger().warn(f"Failed to switch to {previous_mode}, retrying...")
                rclpy.sleep(2)
                self.set_mode(previous_mode)
        else:
            self.set_mode("AUTO.LOITER")

    def altitude_callback(self, data):
        """ Store current altitude """
        self.current_position = data

    def get_current_altitude(self):
        """ Get the current altitude of the UAV """
        return self.current_position.pose.position.z

def main(args=None):
    rclpy.init(args=args)
    node = MAVROSObstacleAvoidance()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()