# 🤖 Automated Food Warehouse Robot

This project proposes the design and control logic for a **fully autonomous robot** that transforms a traditional food warehouse into a **smart, human-free logistics system**.

---

## Project Goals

- Pick and place food packages without human intervention.
- Navigate safely around the warehouse.
- Communicate with a central inventory system.
- Automatically return to charge when idle.

---

## Execution Algorithm

1. Start system and run diagnostics.
2. Scan the warehouse layout using LIDAR/depth cameras.
3. Read item tags (QR/RFID) to localize objects.
4. Receive task from central server or local queue.
5. Plan optimal path to target shelf using A*.
6. Avoid real-time obstacles using sensor feedback.
7. Use robotic arm to pick item.
8. Navigate to drop-off zone.
9. Place item accurately.
10. Update inventory over Wi-Fi.
11. Repeat or return to charging station.

See [`algorithm/warehouse_robot_algorithm.txt`](algorithm/warehouse_robot_algorithm.txt)

---

## Robot Design

- **Base**: 4-wheel differential or mecanum wheels.
- **Sensors**: LIDAR, ultrasonic, IR for obstacle detection.
- **Arm**: 4–6 DOF with soft gripper or suction.
- **Controller**: Raspberry Pi + Arduino, running ROS.
- **Camera**: For visual feedback and tag scanning.
- **Comms**: Wi-Fi / LoRa for system integration.
- **Power**: Rechargeable battery with auto-docking.

Robot diagram and design will be added to `design/robot_design_diagram.png`

---

## Working Envelope

| Component       | Specification                  |
|----------------|----------------------------------|
| Operating area  | Entire warehouse floor          |
| Arm reach       | 0.6 – 1.0 meters                |
| Payload         | Up to 5 kg                      |
| Navigation      | 360° LIDAR & real-time mapping  |
| Vision Range    | 1 – 5 meters                    |
| Docking Range   | Auto-docking within 50 cm       |

See [`docs/working_envelope.md`](docs/working_envelope.md)

---

## System Components

### Hardware
- Arduino Uno / Mega
- Raspberry Pi 4
- LIDAR module (e.g., RPLIDAR)
- 6DOF robotic arm
- Ultrasonic sensors (HC-SR04)
- IR obstacle sensors
- Mecanum wheels
- Li-ion battery pack
- Camera (Pi Camera or USB cam)
- RFID reader / QR scanner

### Software
- ROS (Robot Operating System)
- Python (vision, communication)
- Arduino IDE (motor/sensor control)
- OpenCV (QR detection)

---

## Repository Structure

algorithm/ → Task execution logic
design/ → Robot architecture and diagrams
code/ → Arduino & ROS source files
docs/ → Working envelope, specs, and planning
README.md → Project overview
