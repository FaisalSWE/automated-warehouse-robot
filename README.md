# Automated Food Warehouse Robot

This project designs and implements a **fully autonomous robot** to transform a traditional food warehouse into a **smart, human-free logistics system**. The robot handles picking, placing, navigation, and inventory updates while ensuring food safety compliance (e.g., HACCP).

## Project Goals
- Pick and place food packages without human intervention.
- Navigate safely around the warehouse.
- Communicate with a central inventory system.
- Automatically return to charge when idle.

## Execution Algorithm
The robot follows a detailed algorithm with food safety considerations:
1. Run diagnostics (battery, sensors, arm).
2. Scan warehouse layout using LIDAR and Cartographer SLAM.
3. Read QR codes using OpenCV for item localization.
4. Ensure gripper hygiene with UV sterilization.
5. Monitor temperature for cold storage (0–5°C).
6. Receive tasks from WMS via REST API.
7. Plan paths using A* in ROS.
8. Avoid obstacles using LIDAR and ultrasonic sensors.
9. Pick items with a 6-DOF arm and soft gripper.
10. Navigate to drop-off zone.
11. Place items accurately, verify with camera.
12. Update inventory via MQTT.
13. Return to charging station if idle.

See [`algorithm/warehouse_robot_algorithm.txt`](algorithm/warehouse_robot_algorithm.txt) and [`code/algorithm.py`](code/algorithm.py).

## Robot Design
- **Base**: 4 Mecanum wheels (100mm, DC motors), stainless steel chassis (1m x 0.6m x 0.3m).
- **Arm**: 6-DOF manipulator, food-safe silicone gripper, 5 kg payload.
- **Sensors**: RPLIDAR A1, HC-SR04 ultrasonic, Raspberry Pi Camera v2, DS18B20 temperature sensor.
- **Controller**: Raspberry Pi 4 (ROS2), Arduino Uno (motor control).
- **Power**: 24V, 20Ah Li-ion battery, auto-docking charger.
- **Comms**: Wi-Fi (MQTT/REST for WMS integration).

See [`design/robot_design_diagram.txt`](design/robot_design_diagram.txt) for details. CAD diagram pending at `design/robot_design_diagram.png`.

## Working Envelope
| Component       | Specification                              |
|----------------|--------------------------------------------|
| Operating Area  | 50m x 30m floor, 0.8m min aisle width      |
| Arm Reach       | 0.6–1.0m horizontal, 0–2.0m vertical       |
| Payload         | Up to 5 kg                                 |
| Navigation      | 360° LIDAR, 5cm map resolution             |
| Vision Range    | 1–5m (60° FOV)                             |
| Docking Range   | ±5cm (IR-guided)                           |
| Temperature     | 0–5°C (cold storage)                       |

See [`docs/working_envelope.md`](docs/working_envelope.md) and [`design/working_envelope_diagram.txt`](design/working_envelope_diagram.txt).

## System Components
### Hardware
- Raspberry Pi 4 (4GB)
- Arduino Uno
- RPLIDAR A1
- HC-SR04 ultrasonic sensors
- Raspberry Pi Camera v2
- DS18B20 temperature sensor
- 6-DOF arm with silicone gripper
- Mecanum wheels (100mm)
- 24V Li-ion battery

### Software
- ROS2 (Humble): Navigation, arm control
- Python: Vision (OpenCV), WMS communication
- Arduino IDE: Motor and sensor control
- Gazebo: Simulation environment

## Repository Structure
- `.gitignore` → Excludes build artifacts and temporary files
- `algorithm/` → Execution logic
  - `warehouse_robot_algorithm.txt` → Algorithm overview
- `code/` → Source code
  - `algorithm.py` → Pseudocode for robot control
  - `arduino/` → Arduino sketches
    - `motor_control.ino` → Mecanum wheel control
  - `ros_workspace/src/warehouse_robot/` → ROS nodes and launch files
    - `nav_node.py` → Navigation node
    - `launch/simulate_robot.launch.py` → Simulation launch file
  - `simulation/` → Gazebo files
    - `warehouse.world` → Warehouse environment
    - `warehouse_robot.urdf` → Robot model
  - `vision/` → Vision processing
    - `qr_scanner.py` → QR code detection
- `design/` → Robot and envelope diagrams
  - `robot_design_diagram.txt` → Robot design placeholder
  - `working_envelope_diagram.txt` → Working envelope placeholder
- `docs/` → Documentation
  - `simulation_setup.md` → Simulation setup guide
  - `test_plan.md` → Testing strategy
  - `test_results.md` → Test results log
  - `working_envelope.md` → Working envelope specs
- `README.md` → Project overview

## Progress
- **Algorithm**: Detailed pseudocode with food safety steps.
- **Design**: Chassis, arm, and sensor specs; URDF for simulation.
- **Working Envelope**: Detailed specs with food safety constraints.
- **Code**: ROS navigation, Arduino motor control, QR scanner.
- **Simulation**: Gazebo world and launch file.
- **Testing**: Test plan and results placeholder.
- **Next Steps**: Create CAD diagrams, run simulation tests, assemble hardware.

## Setup Instructions
1. Clone the repository: `git clone https://github.com/FaisalSWE/automated-warehouse-robot.git`
2. Install ROS2 Humble and Gazebo (see `docs/simulation_setup.md`).
3. Build ROS workspace: `cd code/ros_workspace && colcon build`
4. Run simulation: `ros2 launch warehouse_robot simulate_robot.launch.py`
5. Run QR scanner: `ros2 run warehouse_robot qr_scanner.py`

## Testing
- Follow `docs/test_plan.md` for simulation and hardware tests.
- Log results in `docs/test_results.md`.
