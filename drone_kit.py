from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import math

def haversine_distance(lat1, lon1, lat2, lon2):
    """Calculate distance in meters between two GPS coordinates."""
    R = 6371000  # Earth radius in meters
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)

    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2
    return 2 * R * math.asin(math.sqrt(a))


def run_mission(connection_str, target_lat, target_lon, target_alt=10.0):
    start_time = time.time()

    print("----------Mission Log----------")
    print(f"Connecting to vehicle on: {connection_str}")
    vehicle = connect(connection_str, wait_ready=True)

    # --- Switch to GUIDED mode ---
    print("Switching to GUIDED mode...")
    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode.name != "GUIDED":
        print("Waiting for GUIDED mode...")
        time.sleep(1)

    # --- Arm the vehicle ---
    print("Arming motors...")
    vehicle.armed = True
    while not vehicle.armed:
        print("Waiting for arming...")
        time.sleep(1)
    print(f"Vehicle armed: {vehicle.armed}")

    # --- Takeoff ---
    print(f"Taking off to target altitude: {target_alt} m")
    vehicle.simple_takeoff(target_alt)

    while True:
        current_alt = vehicle.location.global_relative_frame.alt
        print(f"Altitude: {current_alt:.1f} m")
        if current_alt >= target_alt * 0.95:
            print(f"Reached target altitude: {target_alt} m")
            break
        time.sleep(1)

    # --- Navigate to target GPS location ---
    print(f"Navigating to GPS: ({target_lat}, {target_lon})")
    
    vehicle.groundspeed=15

    target_location = LocationGlobalRelative(target_lat, target_lon, target_alt)
    vehicle.simple_goto(target_location)

    while True:
        current = vehicle.location.global_relative_frame
        distance = haversine_distance(current.lat, current.lon, target_lat, target_lon)
        print(f"Current Location: ({current.lat:.6f}, {current.lon:.6f}) | Distance to target: {distance:.1f} m")
        if distance < 5:
            print(f"Checkpoint reached: ({target_lat}, {target_lon})")
            break
        time.sleep(1)

    # --- Mission complete ---
    elapsed = time.time() - start_time
    print(f"Mission complete. Total flight time: {elapsed:.1f} seconds")
    print("-------------------------------")

    # Optional: land and close connection
    print("Returning to launch and landing...")
    vehicle.mode = VehicleMode("RTL")
    time.sleep(5)
    vehicle.close()


if __name__ == "__main__":
    run_mission(
        connection_str="127.0.0.1:14550",
        target_lat=28.744444,
        target_lon=77.138056,
        target_alt=10.0
    )
