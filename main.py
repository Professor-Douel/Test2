from dronekit import connect, VehicleMode
import time
from pymavlink import mavutil
import math


HOME_LAT = 50.450739
HOME_LON = 30.461242
TARGET_LAT = 50.443326
TARGET_LON = 30.448078
TARGET_ALT = 100

def connect_vehicle():
    print("[+] Connecting to SITL at 127.0.0.1:5762...")
    vehicle = connect('tcp:127.0.0.1:5762', wait_ready=True)
    return vehicle

def set_mode(vehicle, mode):
    print(f"[+] Setting mode to {mode}...")
    vehicle.mode = VehicleMode(mode)
    while not vehicle.mode.name == mode:
        print(f"Current mode: {vehicle.mode.name}")
        time.sleep(1)

def arm(vehicle):
    print("[+] Arming motors...")
    vehicle.armed = True
    while not vehicle.armed:
        print("[!] Waiting for arming...")
        time.sleep(1)
    print("[+] Armed!")

def wait_altitude(vehicle, target_alt, tolerance=2):
    print(f"[+] Waiting to reach altitude: {target_alt} m...")
    while True:
        alt = vehicle.location.global_relative_frame.alt
        print(f" Altitude: {alt:.1f} m")
        if alt >= target_alt - tolerance:
            print("[+] Target altitude reached.")
            break
        time.sleep(1)

def get_distance_meters(loc1, loc2):
    R = 6371000
    dlat = math.radians(loc2[0] - loc1[0])
    dlon = math.radians(loc2[1] - loc1[1])
    a = math.sin(dlat/2)**2 + math.cos(math.radians(loc1[0])) * math.cos(math.radians(loc2[0])) * math.sin(dlon/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    d = R * c
    return d

def get_bearing(loc1, loc2):
    lat1 = math.radians(loc1[0])
    lat2 = math.radians(loc2[0])
    diffLong = math.radians(loc2[1] - loc1[1])

    x = math.sin(diffLong) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - (math.sin(lat1) * math.cos(lat2) * math.cos(diffLong))
    initial_bearing = math.atan2(x, y)
    bearing = (math.degrees(initial_bearing) + 360) % 360
    return bearing

def fly_towards(vehicle, bearing, duration=1):
    vehicle.channels.overrides['2'] = 1600
    vehicle.channels.overrides['1'] = 1500

    time.sleep(duration)

def stop_motion(vehicle):
    vehicle.channels.overrides['1'] = 1500
    vehicle.channels.overrides['2'] = 1500

def condition_yaw(vehicle, heading, relative=False):
    print(f"[+] Yawing to heading {heading}°")
    msg = vehicle.message_factory.command_long_encode(
        0, 0,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0,
        heading,
        10,
        1,
        int(relative),
        0, 0, 0
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()
    time.sleep(5)

def land(vehicle):
    print("[+] Landing...")
    vehicle.channels.overrides = {}
    vehicle.mode = VehicleMode("LAND")
    time.sleep(5)

def main():
    vehicle = connect_vehicle()

    set_mode(vehicle, "ALT_HOLD")

    arm(vehicle)

    print("[+] Taking off...")
    for throttle in range(1500, 1801, 50):
        vehicle.channels.overrides['3'] = throttle
        time.sleep(1)
    wait_altitude(vehicle, TARGET_ALT)

    start_loc = (HOME_LAT, HOME_LON)
    target_loc = (TARGET_LAT, TARGET_LON)
    distance = get_distance_meters(start_loc, target_loc)
    bearing = get_bearing(start_loc, target_loc)
    print(f"[+] Distance to target: {distance:.1f} m, Bearing: {bearing:.1f}°")

    flight_time = distance / 5
    print(f"[+] Flying forward for approx {flight_time:.1f} seconds")
    fly_towards(vehicle, bearing, duration=flight_time)

    stop_motion(vehicle)

    condition_yaw(vehicle, 350)

    land(vehicle)

    print("[+] Done.")
    vehicle.close()

if __name__ == "__main__":
    main()
