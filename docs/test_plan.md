# Test Plan for Automated Food Warehouse Robot

This document outlines the testing strategy to validate the functionality of the autonomous food warehouse robot in simulation and hardware, ensuring compliance with food safety and operational requirements.

## Objectives
- Verify navigation, picking, placing, and inventory updates.
- Ensure food safety (hygiene, temperature control).
- Validate performance in a 50m x 30m warehouse with 0.8m aisles.

## Test Environment
- **Simulation**: Gazebo with `code/simulation/warehouse.world` and `warehouse_robot.urdf`.
- **Hardware**: Prototype with Raspberry Pi 4, Arduino Uno, RPLIDAR A1, HC-SR04, Pi Camera v2, 6-DOF arm, Mecanum wheels.
- **Warehouse Setup**: 5m x 5m test area with shelves (2m high), QR-coded items, and cold storage zone (0–5°C).

## Test Cases
### 1. System Diagnostics
- **Objective**: Ensure all components are functional.
- **Steps**:
  1. Run `run_diagnostics()` from `code/algorithm.py`.
  2. Check battery (>20%), LIDAR, ultrasonic, camera, and arm calibration.
- **Success Criteria**: All checks return `True`.
- **Environment**: Simulation and hardware.

### 2. Warehouse Mapping
- **Objective**: Generate accurate warehouse map.
- **Steps**:
  1. Run `scan_warehouse()` with Cartographer SLAM.
  2. Save map as `warehouse_map.pgm`.
- **Success Criteria**: Map resolution ≤5cm, covers 50m x 30m area.
- **Environment**: Simulation.

### 3. QR Code Detection
- **Objective**: Localize items via QR codes.
- **Steps**:
  1. Run `code/vision/qr_scanner.py`.
  2. Place QR-coded items at 1–5m from camera.
- **Success Criteria**: Detects and publishes item ID and location within 1s.
- **Environment**: Simulation and hardware.

### 4. Path Planning and Navigation
- **Objective**: Navigate to target shelf and avoid obstacles.
- **Steps**:
  1. Run `nav_node.py` with A* algorithm.
  2. Set goal pose (e.g., shelf at 10m, 10m).
  3. Introduce dynamic obstacle (0.5m proximity).
- **Success Criteria**: Reaches goal within ±5cm, avoids obstacles.
- **Environment**: Simulation.

### 5. Item Picking
- **Objective**: Pick food packages safely.
- **Steps**:
  1. Run `pick_item()` with 6-DOF arm.
  2. Target 1 kg and 4 kg packages on 1m shelf.
  3. Verify hygiene with `ensure_hygiene()`.
- **Success Criteria**: Picks item without damage, weight sensor confirms.
- **Environment**: Simulation and hardware.

### 6. Item Placing
- **Objective**: Place items accurately.
- **Steps**:
  1. Run `place_item()` at drop-off zone.
  2. Verify placement with camera.
- **Success Criteria**: Item placed within ±2cm of target.
- **Environment**: Simulation and hardware.

### 7. Inventory Update
- **Objective**: Communicate with WMS.
- **Steps**:
  1. Run `update_inventory()` with MQTT.
  2. Update item location after placing.
- **Success Criteria**: WMS confirms update within 2s.
- **Environment**: Simulation.

### 8. Temperature Monitoring
- **Objective**: Ensure cold storage compliance.
- **Steps**:
  1. Run `monitor_temperature()` in 0–5°C zone.
  2. Log temperature every 10s.
- **Success Criteria**: Logs warnings for temperatures outside 0–5°C.
- **Environment**: Hardware.

### 9. Charging Docking
- **Objective**: Auto-dock for charging.
- **Steps**:
  1. Navigate to docking station (50cm range).
  2. Align using IR sensors.
- **Success Criteria**: Docks within ±5cm.
- **Environment**: Simulation and hardware.

## Test Schedule
- **Simulation Tests**: Complete by [insert date].
- **Hardware Tests**: Complete after prototype assembly by [insert date].
- **Duration**: 2 weeks for simulation, 1 week for hardware.

## Metrics
- Navigation accuracy: ±5cm.
- Pick/place success rate: ≥95%.
- QR detection accuracy: ≥98%.
- Temperature compliance: 100% within 0–5°C.
- System uptime: ≥8 hours per charge.

## Notes
- Log results in `docs/test_results.md`.
- Address food safety (HACCP) with hygiene and temperature tests.
- Scale tests to multi-robot scenarios in future iterations.
