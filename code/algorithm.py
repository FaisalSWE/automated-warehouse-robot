def run_diagnostics():
    battery_status = check_battery_level(min_threshold=20)  # Ensure >20% charge
    sensor_status = check_sensors(lidar=True, ultrasonic=True, camera=True)
    arm_status = calibrate_arm(joints=6)
    return battery_status and sensor_status and arm_status

def scan_warehouse():
    slam = initialize_slam("cartographer", lidar_port="/dev/ttyUSB0")
    map_data = generate_map(lidar_data, resolution=0.05)  # 5cm grid
    save_map("warehouse_map.pgm")
    return map_data

def read_item_tags():
    camera = initialize_camera(device=0)
    qr_data = detect_qr_codes(camera, library="opencv")
    if qr_data:
        item_id, location = parse_qr_data(qr_data)
        return item_id, location
    return None, None

def receive_task():
    task = fetch_task_from_wms(url="http://wms.local/tasks", protocol="REST")
    if task:
        return task["item_id"], task["target_shelf"]
    return None, None

def plan_path(start_pose, goal_pose, map_data):
    path = compute_a_star_path(start_pose, goal_pose, map_data, cost_function="distance")
    return path

def avoid_obstacles(path, sensor_data):
    obstacles = detect_obstacles(lidar_data, ultrasonic_data)
    if obstacles:
        new_path = replan_path(path, obstacles, algorithm="DWA")
        return new_path
    return path

def ensure_hygiene():
    gripper_status = check_gripper_cleanliness()
    if not gripper_status:
        sterilize_gripper(method="UV", duration=30)  # 30 seconds UV
    return gripper_status

def monitor_temperature():
    temp = read_temperature_sensor(pin=18)  # DS18B20 sensor
    if temp < 0 or temp > 5:  # Cold storage range
        log_warning("Temperature out of range: {}°C".format(temp))
    return temp

def pick_item(item_id, shelf_location):
    ensure_hygiene()
    arm_pose = compute_arm_pose(shelf_location, item_id)
    move_arm_to_pose(arm_pose, speed=0.5)
    close_gripper(type="soft", force=10)  # Food-safe grip
    return verify_pick_success(weight_sensor=True)

def navigate_to_dropoff(path, dropoff_zone):
    monitor_temperature()
    follow_path(path, speed=1.0, ros_node="move_base")
    if at_dropoff_zone(dropoff_zone):
        return True
    return False

def place_item(dropoff_pose):
    arm_pose = compute_arm_pose(dropoff_pose)
    move_arm_to_pose(arm_pose, speed=0.5)
    open_gripper()
    return verify_place_success(vision_check=True)

def update_inventory(item_id, new_location):
    send_update_to_wms(item_id, new_location, protocol="MQTT")
    return confirm_update_received()

def main():
    if not run_diagnostics():
        log_error("Diagnostics failed")
        return
    map_data = scan_warehouse()
    while True:
        item_id, shelf = receive_task()
        if not item_id:
            return_to_charging_station()
            break
        item_id, location = read_item_tags()
        path = plan_path(robot_pose, shelf, map_data)
        path = avoid_obstacles(path, sensor_data)
        if pick_item(item_id, location):
            dropoff_path = plan_path(location, dropoff_zone, map_data)
            if navigate_to_dropoff(dropoff_path, dropoff_zone):
                place_item(dropoff_zone)
                update_inventory(item_id, dropoff_zone)
