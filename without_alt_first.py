#!/usr/bin/env python3

import asyncio
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

import numpy as np
import sensor_msgs.msg
import struct

from mavsdk import System
#from mavsdk.action import FlightMode
from mavsdk.offboard import PositionNedYaw

class ROS2PointCloudHandler(Node):
    def __init__(self):
        super().__init__("ros2_pointcloud_handler")
        self.subscription = self.create_subscription(
            sensor_msgs.msg.PointCloud2,
            '/pointcloud',
            self.pointcloud_callback,
            10
        )
        self.obstacle_detected = False
        self.hold_required = False
        self.latest_msg = None  

    def pointcloud_callback(self, msg):
        """Process point cloud data and detect obstacles."""
        self.latest_msg = msg  
        close_threshold = 0.1  
        hold_threshold_min = 2.0
        hold_threshold_max = 10.0

        points = self.extract_points(msg)
        if points is None:
            return

        distances = np.linalg.norm(points, axis=1)

        if np.any(distances < close_threshold):
            self.obstacle_detected = True
            self.hold_required = True
            print("⚠️ WARNING: Obstacle detected! Switching to Hold mode...")
        elif np.any((distances >= hold_threshold_min) & (distances <= hold_threshold_max)):
            self.obstacle_detected = False
            self.hold_required = True
            print("⏸ Obstacle detected in range 2-50m, switching to Hold mode...")
        else:
            self.obstacle_detected = False
            self.hold_required = False

    def extract_points(self, msg):
        """Extract XYZ points from PointCloud2 message."""
        try:
            point_step = msg.point_step
            data = msg.data
            num_points = len(data) // point_step

            points = []
            for i in range(num_points):
                offset = i * point_step
                x, y, z = struct.unpack_from('fff', data, offset)
                points.append([x, y, z])

            return np.array(points)
        except Exception as e:
            print(f"❌ Error processing point cloud: {e}")
            return None

async def avoid_obstacle(drone, initial_mode):
    """Switches to Hold mode first, then increases altitude to 15m in Offboard mode."""
    await hold_drone(drone)
    await asyncio.sleep(2)  # Small delay before altitude change
    
    print("🔄 Attempting to switch to Offboard mode...")
    await drone.action.arm()
    await asyncio.sleep(1)
    
    try:
        await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -15.0, 0.0))
        await drone.offboard.start()
        print("✅ Offboard mode active, increasing altitude to 15m...")
    except Exception as e:
        print(f"❌ Failed to switch to Offboard mode: {e}")
        return
    
    async for position in drone.telemetry.position():
        if position:
            await drone.action.goto_location(
                position.latitude_deg,
                position.longitude_deg,
                15.0, 0
            )
            print("⬆️ Altitude increased to 15m to avoid obstacle.")
            break
    
    print(f"🔄 Restoring initial flight mode: {initial_mode}")
    #await drone.action.set_flight_mode(initial_mode)
    if initial_mode == "TAKEOFF":
        await drone.action.takeoff()
    elif initial_mode == "LAND":
        await drone.action.land()
    elif initial_mode == "HOLD":
        await drone.action.hold()
    elif initial_mode == "MISSION":
        await drone.mission.start_mission()
    elif initial_mode == "MANUAL":
        await drone.manual_control.start_position_control()
    elif initial_mode == "RETURN_TO_LAUNCH":
        await drone.action.return_to_launch()
    else:
        print("⚠️ Unknown initial mode, keeping Offboard mode active.")
    

async def hold_drone(drone):
    """Switches the drone to Hold mode."""
    await drone.action.hold()
    print("⏸ Drone switched to Hold mode due to obstacle.")

async def monitor_pointcloud(drone):
    """Continuously checks for obstacles and applies avoidance strategies."""
    rclpy.init()
    node = ROS2PointCloudHandler()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    async for flight_mode in drone.telemetry.flight_mode():
        initial_mode = flight_mode
        print(f"📌 Initial flight mode recorded: {initial_mode}")
        break

    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.hold_required:
            await hold_drone(drone)
        if node.obstacle_detected:
            await avoid_obstacle(drone, initial_mode)

    node.destroy_node()
    rclpy.shutdown()

async def run():
    """Main function to monitor point cloud and avoid obstacles."""
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("⏳ Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"✅ Connected to drone!")
            break

    asyncio.create_task(monitor_pointcloud(drone))
    await asyncio.Future()  

if __name__ == "__main__":
    asyncio.run(run())
