# Arduino-Based Gimbal

An Arduino-based two-axis gimbal project designed to stabilize an object (like a camera) using servo motors and an MPU 6050 motion sensor. This project integrates hardware design, 3D modeling, and software programming for a functional real-time stabilization system.

---

## Features
- **Two-axis stabilization**: Controls roll and pitch using servo motors.
- **MPU 6050 integration**: Provides real-time orientation data (yaw, pitch, roll).
- **Custom 3D design**: Gimbal body designed in Fusion 360 and fabricated using 3D printing.
- **Processing IDE Simulation**: Visualizes sensor data in a 3D simulation environment.

---

## Components Used
### Hardware
- Arduino UNO/Nano
- MPU 6050 (6-axis motion sensor)
- 2 Servo Motors
- 3D-printed gimbal frame
- Jumper Wires
- Breadboard

### Software
- Arduino IDE
- Fusion 360 (for 3D modeling)
- Processing IDE (for simulation)
- Libraries: `I2Cdev.h`, `MPU6050.h`, `Servo.h`

---

## Getting Started
### Prerequisites
1. **Install Arduino IDE**: [Download Here](https://www.arduino.cc/en/software)
2. **Install Processing IDE** (optional for simulation): [Download Here](https://processing.org/download/)
3. **Arduino Libraries**: Add the following libraries:
   - [I2Cdev Library](https://github.com/jrowberg/i2cdevlib)
   - [MPU6050 Library](https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050)
   - Built-in `Servo` library.

---

### Circuit Setup
1. Connect the MPU 6050 to the Arduino using the I2C protocol:
   - `VCC` → 3.3V/5V  
   - `GND` → GND  
   - `SCL` → A5 (for Arduino UNO)  
   - `SDA` → A4 (for Arduino UNO)

2. Connect the servo motors:
   - Servo X → Pin 5  
   - Servo Y → Pin 6  
   - Power the servos with a separate power source if needed.

---

### Code Upload
1. Open the `maincode.ino` file in Arduino IDE.
2. Select the appropriate Arduino board and COM port.
3. Upload the code to the Arduino.

---

### Simulation (Optional)
1. Open the provided Processing IDE file.
2. Run the sketch to visualize real-time motion data from the MPU 6050.

---

## Output
- Stabilizes an object along the roll and pitch axes based on motion sensor data.
- Visualizes yaw, pitch, and roll using a 3D simulation in Processing IDE.

---

## Future Improvements
- Add stabilization along the Z-axis (yaw).
- Optimize control algorithms using a PID controller.
- Explore alternative sensors for enhanced accuracy.

---



---


