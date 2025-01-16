import asyncio
from mavsdk import System

async def read_distance_sensor(system_address="serial:///dev/ttyACM2:1152000"):
    # Initialize the MAVSDK system
    drone = System()
    await drone.connect(system_address=system_address)

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"Drone connected at {system_address}")
            break

    print("Waiting for distance sensor data...")
    async for distance_sensor in drone.telemetry.distance_sensor():
        # Extract distance data
        current_distance = distance_sensor.current_distance_m
        min_distance = distance_sensor.minimum_distance_m
        max_distance = distance_sensor.maximum_distance_m

        print(f"Distance: {current_distance:.2f} m "
              f"(Min: {min_distance:.2f} m, Max: {max_distance:.2f} m)")
        await asyncio.sleep(0.1)  # Reduce output frequency if needed

if __name__ == "__main__":
    asyncio.run(read_distance_sensor("serial:///dev/ttyACM2:1152000"))
