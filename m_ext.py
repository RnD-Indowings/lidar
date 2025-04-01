#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from mavros_msgs.srv import SetMode
from mavros_msgs.msg import State, PositionTarget
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import WaypointPull
from mavros_msgs.msg import Waypoint


# MAVROS parameters
min_threshold_distrance = 0.0
max_threshold_distance = 2.0  # Obstacle detection threshold (meters)
altitude_increment = 20.0  # Increase altitude by 15m per avoidance step
current_mode = None
previous_mode = None
last_valid_mode = None  # Store mode before obstacle detection
base_altitude = None  # Initial altitude before avoidance
current_altitude_offset = 0.0  # Track total altitude increase
last_obstacle_distance = None  # Store last detected obstacle distance
min_altitude = 5
# Position publisher
local_pos_pub = None
current_position = PoseStamped()

# MAVROS Service to Change Flight Mode
def set_mode(mode):
    rospy.wait_for_service("/mavros/set_mode")
    try:
        set_mode_service = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        response = set_mode_service(0, mode)
        if response.mode_sent:
            rospy.loginfo(f"Mode changed to {mode}")
            return True
        else:
            rospy.logwarn(f"Failed to change mode to {mode}")
            return False
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return False

# Get Current Flight Mode
def state_callback(msg):
    global current_mode, last_valid_mode

    if current_mode and current_mode != "AUTO.LOITER":
        last_valid_mode = current_mode  # Store last valid mode before loiter

    current_mode = msg.mode

# Read LiDAR Data and Detect Obstacles
def lidar_callback(data):
    global current_mode, previous_mode, last_valid_mode, base_altitude, current_altitude_offset, last_obstacle_distance

    points = list(pc2.read_points(data, field_names=("x", "y", "z"), skip_nans=True))
    if len(points) == 0:
        rospy.logwarn("No points received from SF45 LiDAR")
        return

    distances = np.sqrt(np.array([p[0] for p in points]) ** 2 + np.array([p[1] for p in points]) ** 2)
    
    valid_distances = [d for d in distances if not np.isclose(d, 1.00, atol=0.01)]
    if len(valid_distances) == 0:
        rospy.logwarn("All valid distances were filtered out, skipping obstacle check.")
        return

    min_distance = np.min(valid_distances)
    rospy.loginfo(f"Closest obstacle distance: {min_distance:.2f}m")

    if last_obstacle_distance is not None and np.isclose(min_distance, last_obstacle_distance, atol=0.05):
        rospy.loginfo("Obstacle distance is the same as before, ignoring this reading.")
        return

    last_obstacle_distance = min_distance

    # **If obstacle is detected, store previous mode and switch to OFFBOARD**
    if min_threshold_distrance < min_distance < max_threshold_distance:
        rospy.logwarn("Obstacle detected! Switching to OFFBOARD and increasing altitude!")
          # Retry once

        if base_altitude is None:
            base_altitude = get_current_altitude()
        cat_alt = get_current_altitude()
        print("base ", cat_alt)
        if cat_alt > min_altitude:
        # **Store the mode before switching**
            previous_mode = current_mode  
            set_mode("AUTO.LOITER")
            if set_mode("OFFBOARD"):
                #rospy.sleep(2)
                increase_altitude_offboard()
        else:
            print("low alt", cat_alt)



# Function to Increase Altitude (Now in OFFBOARD Mode)
def increase_altitude_offboard():
    global current_altitude_offset, base_altitude, local_pos_pub, current_position

    if base_altitude is None:
        rospy.logwarn("Base altitude not set, skipping altitude increase.")
        return

    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)

    # Arm the drone if not already armed


    # Wait to ensure the publisher is ready
    rospy.sleep(1)
    tt_altitude = get_current_altitude()
    new_altitude = tt_altitude + altitude_increment  # Target altitude
    pose = PoseStamped()
    pose.pose.position.x = current_position.pose.position.x  # Maintain X
    pose.pose.position.y = current_position.pose.position.y  # Maintain Y

    rospy.loginfo("Sending initial position setpoints before switching to OFFBOARD...")

    # **Publish 100 setpoints before switching to OFFBOARD**
    for _ in range(100):
        pose.pose.position.z = base_altitude  # Hold current altitude initially
        local_pos_pub.publish(pose)
        rospy.sleep(0.05)

    rospy.loginfo("Attempting to switch to OFFBOARD mode...")

    # Switch to OFFBOARD mode
    if set_mode("OFFBOARD"):
        rospy.loginfo("OFFBOARD mode enabled!")
    else:
        rospy.logwarn("OFFBOARD mode failed to enable!")

    # Gradual altitude increase in steps of 3m
    rospy.loginfo(f"Gradually increasing altitude to {new_altitude}m in OFFBOARD mode")

    step = 4.0
    while current_position.pose.position.z + 0.1 < new_altitude:
        alti = get_current_altitude()
        pose.pose.position.z += step
        if pose.pose.position.z > new_altitude:
            pose.pose.position.z = new_altitude  # Don't exceed target altitude
        if pose.pose.position.z < alti:
            pose.pose.position.z = alti
        rospy.loginfo(f"New altitude target: {pose.pose.position.z:.2f}m")
        alti = get_current_altitude()
        print("current:  ", alti )
        for _ in range(50):  # Send multiple setpoints to maintain OFFBOARD mode
            local_pos_pub.publish(pose)
            rospy.sleep(0.1)

        #rospy.sleep(1)  # Allow the UAV to stabilize

    rospy.loginfo("Target altitude reached, rechecking for obstacles...")
    #rospy.sleep(5)  # Allow time for stabilization
    verify_obstacle_clearance()

# Function to Recheck for Obstacles and Restore Flight Mode
def verify_obstacle_clearance():
    global previous_mode
    print("mode           " ,  previous_mode)
    print("modey          ", current_mode)
    #rospy.sleep(2)  # Delay before checking again
    if current_mode == "OFFBOARD" :
        rospy.loginfo("Rechecking for obstacles...")
        print("mode" ,  previous_mode)
        rospy.Subscriber("/pointcloud", PointCloud2, lidar_callback, queue_size=1)
        #rospy.sleep(3)  # Allow LiDAR data to update
        set_mode(previous_mode) 
        # **Force switching back to previous mode**
        if previous_mode != "OFFBOARD" and previous_mode == "AUTO.MISSION" or previous_mode == "AUTO.RTL":
            rospy.loginfo(f"Switching back to previous mode: {previous_mode}")
            success = set_mode(previous_mode)
            
            if success:
                rospy.loginfo(f"Successfully switched back to {previous_mode}")
            else:
                rospy.logwarn(f"Failed to switch back to {previous_mode}, retrying in 2s...")
                rospy.sleep(2)
                set_mode(previous_mode)  # Retry once
        else:
            set_mode("AUTO.LOITER") 




# Function to Get Current Altitude
def altitude_callback(data):
    global current_position
    current_position = data  # Store latest position

def get_current_altitude():
    """Get the drone's current altitude."""
    return current_position.pose.position.z  # Fetch from last received pose

from mavros_msgs.msg import WaypointList

# Global waypoint list
mission_waypoints = []

def mission_waypoints_callback(msg):
    global mission_waypoints
    mission_waypoints = msg.waypoints  # List of mavros_msgs/Waypoint

from mavros_msgs.srv import WaypointPull
from mavros_msgs.msg import WaypointList

def fetch_and_display_mission_waypoints():
    rospy.wait_for_service('/mavros/mission/pull')
    try:
        pull_srv = rospy.ServiceProxy('/mavros/mission/pull', WaypointPull)
        resp = pull_srv()
        if resp.success:
            rospy.loginfo(f"Fetched {resp.wp_received} mission waypoints.")

            # ✅ Wait for actual waypoints from /mavros/mission/waypoints topic
            waypoints_msg = rospy.wait_for_message("/mavros/mission/waypoints", WaypointList, timeout=5.0)
            waypoints = waypoints_msg.waypoints

            if not waypoints:
                rospy.logwarn("No mission waypoints received.")
                return

            filename = "/home/indowings/mission_waypoints.txt"
            with open(filename, "w") as f:
                for i, wp in enumerate(waypoints):
                    lat = wp.x_lat
                    lon = wp.y_long
                    alt = wp.z_alt
                    log_msg = f"Waypoint {i}: lat={lat:.6f}, lon={lon:.6f}, alt={alt:.2f}"
                    rospy.loginfo(log_msg)
                    f.write(log_msg + "\n")
            rospy.loginfo(f"Waypoints saved to {filename}")
        else:
            rospy.logwarn("Failed to pull mission waypoints.")
    except rospy.ROSException:
        rospy.logerr("Timeout while waiting for mission waypoints.")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed while pulling waypoints: {e}")






# Main Function
def main():
    global local_pos_pub
    rospy.init_node("sf45_mavros_obstacle_avoidance", anonymous=True)

    rospy.Subscriber("/pointcloud", PointCloud2, lidar_callback, queue_size=1)
    rospy.Subscriber("/mavros/state", State, state_callback)
    rospy.Subscriber("/mavros/local_position/pose", PoseStamped, altitude_callback)

    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
    rospy.loginfo("Fetching mission waypoints from GCS...")
    fetch_and_display_mission_waypoints()
    
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
