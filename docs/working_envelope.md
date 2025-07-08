# Working Envelope

The working envelope defines the 3D operational space of the autonomous warehouse robot, tailored for a food warehouse with hygiene and cold storage requirements.

| Component       | Specification                              |
|----------------|--------------------------------------------|
| Operating Area  | 50m x 30m warehouse floor, 0.8m min aisle width |
| Arm Reach       | 0.6–1.0m horizontal, 0–2.0m vertical       |
| Payload         | Up to 5 kg (food packages, e.g., rice bags, crates) |
| Navigation      | 360° LIDAR (10m range), 5cm map resolution |
| Vision Range    | 1–5m (60° FOV, Raspberry Pi Camera v2)     |
| Docking Range   | Auto-docking within ±5cm (IR-guided)       |
| Temperature     | Operates in 0–5°C (cold storage), DS18B20 sensor |
| Safety Zone     | 0.5m proximity buffer, 0.2m emergency stop |

## Details
- **Operating Area**: Supports multi-level shelving up to 2.5m height. Aisles ≥0.8m allow Mecanum wheel navigation.
- **Arm Reach**: 6-DOF arm accesses shelves at various angles. Vertical reach supports high shelves; horizontal reach covers both sides of aisles.
- **Payload**: Handles typical food items (e.g., 1 kg bags, 4 kg crates). Soft silicone gripper ensures food safety.
- **Navigation**: RPLIDAR A1 provides 360° mapping with Cartographer SLAM. Dynamic obstacle avoidance using ultrasonic sensors (HC-SR04).
- **Vision**: Camera detects QR codes for item localization, effective in low-light with flash.
- **Docking**: IR sensors ensure precise docking for charging within 50 cm.
- **Temperature**: Monitors cold storage compliance (0–5°C) to protect perishables.
- **Safety**: Stops if objects/humans enter 0.2m zone; 0.5m buffer for navigation planning.

## Notes
- Diagram of the working envelope will be added as `design/working_envelope_diagram.png`.
- Food-grade materials and UV sterilization ensure HACCP compliance.
