import asyncio
from mavsdk import System
from mavsdk.telemetry import FlightMode
from mavsdk.mission import MissionItem, MissionPlan

waypoints = []  # Stores (lat, lon, alt)

async def log_waypoints(drone):
    """ Logs waypoints dynamically while the drone is in flight """
    previous_gps = None  # Track GPS changes

    async for gps in drone.telemetry.position():
        lat, lon, alt = gps.latitude_deg, gps.longitude_deg, max(gps.relative_altitude_m, 10)

        if lat == 0 and lon == 0:
            print("❌ Invalid GPS data received! Skipping...")
            continue

        if previous_gps and (lat, lon) == (previous_gps.latitude_deg, previous_gps.longitude_deg):
            print("⚠️ GPS not updating, skipping duplicate waypoint...")
            continue

        waypoints.append((lat, lon, alt))
        print(f"✅ Waypoint Logged: {lat}, {lon}, {alt}")

        previous_gps = gps  # Update previous GPS position
        await asyncio.sleep(2)  # Log every 2 seconds


async def check_rtl_trigger(drone):
    """ Monitors if RTL mode is triggered and starts the reverse mission """
    async for mode in drone.telemetry.flight_mode():
        if mode == FlightMode.RETURN_TO_LAUNCH:
            print("✈️ RTL triggered! Running Reverse Route Mission.")
            await execute_reverse_route_mission(drone)
            break


async def execute_reverse_route_mission(drone):
    """ Uploads and starts a reverse mission """
    if len(waypoints) < 2:
        print("❌ Not enough waypoints recorded! Falling back to normal RTL.")
        await drone.action.return_to_launch()
        return

    print("🛑 Stopping RTL Mode...")
    await drone.action.hold()  # Prevents conflicts with RTL
    await asyncio.sleep(2)

    # Reverse the waypoints
    mission_waypoints = list(reversed(waypoints))

    print("📌 Creating Reverse Mission...")
    mission_items = [
        MissionItem(
            lat, lon, max(alt, 10),  # Latitude, Longitude, Altitude (Ensure min 10m)
            12.0,  # Speed in m/s
            False,  # Stop at each waypoint
            float('nan'),  # Gimbal pitch
            float('nan'),  # Gimbal yaw
            MissionItem.CameraAction.NONE,
            0.0,  # Loiter time
            float('nan'),
            5.0,  # Acceptance radius
            float('nan'),
            float('nan'),
            MissionItem.VehicleAction.NONE
        )
        for lat, lon, alt in mission_waypoints
    ]

    print("🧹 Clearing old mission...")
    await drone.mission.clear_mission()  # Ensure no old missions interfere

    mission_plan = MissionPlan(mission_items)

    print("📤 Uploading mission...")
    try:
        await drone.mission.upload_mission(mission_plan)
        print("✅ Mission uploaded successfully")
    except Exception as e:
        print(f"❌ Mission upload failed: {e}")
        return

    print("🚀 Starting Reverse Route Mission...")
    await drone.mission.start_mission()

    # Wait for the drone to enter Mission Mode
    async for mode in drone.telemetry.flight_mode():
        if mode == FlightMode.MISSION:
            print("✅ Drone is now in Mission Mode!")
            break
        await asyncio.sleep(1)

    # Monitor mission progress
    await monitor_mission_completion(drone)


async def monitor_mission_completion(drone):
    """ Monitors mission status and triggers RTL upon completion """
    async for mission_progress in drone.mission.mission_progress():
        print(f"📍 Mission Progress: {mission_progress.current}/{mission_progress.total}")

        # If mission is completed, trigger RTL
        if mission_progress.current >= mission_progress.total:
            print("🏁 Mission completed. Returning to launch!")
            await drone.action.return_to_launch()
            break

        await asyncio.sleep(2)  # Check progress every 2 seconds


async def main():
    drone = System()
    await drone.connect(system_address="udp://:14540")
    print("🛰️ Drone connected!")

    asyncio.create_task(log_waypoints(drone))  # Start waypoint logging
    await check_rtl_trigger(drone)  # Wait for RTL trigger


if __name__ == "__main__":
    asyncio.run(main())
