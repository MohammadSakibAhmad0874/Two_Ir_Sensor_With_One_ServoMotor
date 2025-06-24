# Two_Ir_Sensor_With_One_ServoMotor
# Line Following Robot 🤖
![Terrific Turing-Krunk](https://github.com/user-attachments/assets/132d9b98-9d40-4ee3-9ce1-196ca5cc5505)

A comprehensive Arduino-based line following robot project that uses infrared sensors to detect and follow a line path autonomously. This project is perfect for robotics enthusiasts, students, and hobbyists looking to learn about autonomous navigation systems.

## 📋 Table of Contents
- [Introduction](#introduction)
- [How It Works](#how-it-works)
- [Types of Line Following Robots](#types-of-line-following-robots)
- [Components Required](#components-required)
- [Circuit Diagram](#circuit-diagram)
- [Installation Process](#installation-process)
- [Code Upload](#code-upload)
- [Calibration & Testing](#calibration--testing)
- [Applications & Uses](#applications--uses)
- [Troubleshooting](#troubleshooting)
- [Advanced Features](#advanced-features)
- [Additional Knowledge](#additional-knowledge)
- [Contributing](#contributing)
- [License](#license)

## 🔍 Introduction

A line following robot is an autonomous vehicle that follows a predetermined path marked by a line (usually black on white surface or vice versa). The robot uses sensors to detect the line and adjusts its movement accordingly to stay on track.

This project implements a **servo-based steering system** where the robot's direction is controlled by rotating a servo motor rather than controlling individual wheel motors. This approach is commonly used in RC cars and provides smooth steering control.

### Key Features
- ✅ Autonomous line following capability
- ✅ Real-time sensor feedback via Serial Monitor
- ✅ Adjustable servo positions for fine-tuning
- ✅ Built-in calibration system
- ✅ Modular code structure for easy modifications
- ✅ Low-cost implementation

## ⚙️ How It Works

The robot uses two infrared (IR) sensors positioned at the front to detect the line:

1. **Line Detection**: IR sensors emit infrared light and measure the reflection
   - **Dark surfaces** (black line) absorb IR light → Low reflection → Digital LOW (0)
   - **Light surfaces** (white background) reflect IR light → High reflection → Digital HIGH (1)

2. **Decision Making**: Based on sensor readings, the robot decides its next move:
   - `Left sensor ON, Right sensor OFF` → Turn LEFT
   - `Left sensor OFF, Right sensor ON` → Turn RIGHT  
   - `Both sensors OFF` → Go STRAIGHT
   - `Both sensors ON` → STOP/SEARCH mode

3. **Servo Control**: A servo motor controls the steering mechanism
   - Left turn: 30° position
   - Center/Straight: 60° position  
   - Right turn: 90° position

## 🔧 Types of Line Following Robots

### 1. **Differential Drive Robot**
- Uses two motors for left and right wheels
- Direction controlled by varying wheel speeds
- More common in educational robots

### 2. **Servo Steering Robot** (This Project)
- Uses a servo motor for steering
- Similar to RC car mechanism
- Smoother turns and better control

### 3. **Omnidirectional Robot**
- Can move in any direction
- Uses mecanum or omni wheels
- Most complex but most flexible

### 4. **Tank Drive Robot**
- Similar to differential drive
- Uses tracks instead of wheels
- Better for rough terrain

## 🛠️ Components Required

### Electronic Components
| Component | Quantity | Description |
|-----------|----------|-------------|
| Arduino Uno/Nano | 1x | Main microcontroller |
| IR Sensor Modules | 2x | Line detection (TCRT5000 recommended) |
| Servo Motor | 1x | Steering control (SG90 or similar) |
| Jumper Wires | 10-15x | Connections |
| Breadboard | 1x | For connections (optional) |
| Battery Pack | 1x | Power supply (6V-9V recommended) |
| Resistors | 2x | 220Ω for LED indicators (optional) |

### Mechanical Components
| Component | Quantity | Description |
|-----------|----------|-------------|
| Robot Chassis | 1x | Main body structure |
| Wheels | 3-4x | 2 drive wheels + 1-2 caster wheels |
| DC Motors | 1-2x | For drive wheels |
| Motor Driver | 1x | L298N or similar |
| Screws & Bolts | As needed | Assembly hardware |
| Mounting Brackets | As needed | Sensor and component mounting |

### Tools Required
- Screwdriver set
- Wire strippers
- Soldering iron (if permanent connections needed)
- Multimeter (for troubleshooting)
- Computer with Arduino IDE

## 📐 Circuit Diagram

```
Arduino Uno Connections:
┌─────────────────┐
│     Arduino     │
│                 │
│  Pin 2 ←──────  │ ← Left IR Sensor (Digital Out)
│  Pin 3 ←──────  │ ← Right IR Sensor (Digital Out)
│  Pin 9 ────────→│ → Servo Motor (Signal)
│                 │
│  5V  ────────→  │ → Sensors & Servo Power
│  GND ────────→  │ → Common Ground
└─────────────────┘

IR Sensor Module (TCRT5000):
┌─────────────┐
│   TCRT5000  │
│             │
│ VCC → 5V    │
│ GND → GND   │
│ OUT → Pin2/3│
└─────────────┘

Servo Motor (SG90):
┌─────────────┐
│    SG90     │
│             │
│ Red → 5V    │
│ Brown → GND │
│ Orange → Pin9│
└─────────────┘
```

## 🔧 Installation Process

### Step 1: Prepare the Arduino IDE
1. Download and install [Arduino IDE](https://www.arduino.cc/en/software)
2. Connect your Arduino board via USB
3. Select correct board type: `Tools → Board → Arduino Uno`
4. Select correct port: `Tools → Port → COM_X` (Windows) or `/dev/ttyUSB0` (Linux)

### Step 2: Install Required Libraries
The Servo library is built into Arduino IDE, but verify it's available:
```cpp
Sketch → Include Library → Servo
```

### Step 3: Hardware Assembly
1. **Mount IR Sensors**:
   - Position sensors 2-3cm above ground
   - Space them 3-5cm apart
   - Ensure they're parallel to the ground
   - Angle slightly downward (15-20°)

2. **Install Servo Motor**:
   - Mount servo to chassis
   - Connect servo horn to steering mechanism
   - Ensure servo can rotate freely

3. **Make Connections**:
   - Follow the circuit diagram above
   - Use breadboard for temporary connections
   - Solder for permanent installation

4. **Power Setup**:
   - Use 6V-9V battery pack
   - Consider separate power for motors if needed
   - Add power switch for convenience

### Step 4: Mechanical Assembly
1. **Chassis Setup**:
   - Install drive motors
   - Mount wheels and ensure they spin freely
   - Add caster wheel(s) for stability

2. **Sensor Mounting**:
   - Create sensor mount at front of robot
   - Ensure sensors can detect the line clearly
   - Test sensor height and angle

## 💻 Code Upload

### Step 1: Download Code
```bash
git clone https://github.com/your-repo/line-following-robot.git
cd line-following-robot
```

### Step 2: Open in Arduino IDE
1. Open `line_follower_robot.ino` in Arduino IDE
2. Verify the code compiles: `Sketch → Verify/Compile`
3. Upload to Arduino: `Sketch → Upload`

### Step 3: Configure Serial Monitor
1. Open Serial Monitor: `Tools → Serial Monitor`
2. Set baud rate to `9600`
3. Monitor sensor readings and robot actions

## 🎯 Calibration & Testing

### Initial Calibration
1. **Sensor Testing**:
   ```cpp
   // Use the built-in calibration function
   void setup() {
     Serial.begin(9600);
     calibrateSensors(); // Call this first
   }
   ```

2. **Line Detection Test**:
   - Place sensors over white surface → Should read `1`
   - Place sensors over black line → Should read `0`
   - If reversed, adjust your logic or sensor sensitivity

3. **Servo Calibration**:
   - Test center position (should go straight)
   - Adjust `SERVO_CENTER` value if needed
   - Test left/right positions and adjust accordingly

### Track Preparation
- **Line Width**: 2-3cm wide black tape
- **Surface**: White/light colored smooth surface
- **Curves**: Gradual curves work better than sharp turns
- **Lighting**: Consistent lighting, avoid shadows

### Testing Protocol
1. Start with straight line
2. Test gentle curves
3. Try sharp turns
4. Test intersections (advanced)
5. Validate different lighting conditions

## 🌍 Applications & Uses

### Educational Applications
- **STEM Learning**: Introduction to robotics and programming
- **Competition**: Robot competitions like micromouse
- **Research**: Autonomous navigation studies
- **Workshops**: Hands-on learning experiences

### Industrial Applications
- **Automated Guided Vehicles (AGVs)**: Warehouse automation
- **Manufacturing**: Assembly line guidance
- **Material Handling**: Automated transport systems
- **Quality Control**: Automated inspection systems

### Hobby & Personal Projects
- **Home Automation**: Mail delivery robot
- **Entertainment**: Pet following robot
- **Surveillance**: Patrol robot for security
- **Gardening**: Automated plant watering system

### Advanced Implementations
- **Multi-sensor Arrays**: Using 5-8 sensors for better accuracy
- **Camera-based**: Computer vision for complex path following
- **GPS Integration**: Outdoor navigation systems
- **IoT Integration**: Remote monitoring and control

## 🔍 Troubleshooting

### Common Issues & Solutions

| Problem | Possible Cause | Solution |
|---------|----------------|-----------|
| Robot doesn't move | Power issues | Check battery, connections |
| Sensors not responding | Wiring errors | Verify pin connections |
| Servo not turning | Power/signal issues | Check servo connections, power supply |
| Erratic behavior | Interference | Check for loose connections, EMI |
| Poor line detection | Sensor height/angle | Adjust sensor positioning |
| Robot overshoots turns | Delay too long | Reduce loop delay |

### Debugging Tips
1. **Use Serial Monitor**: Monitor sensor values in real-time
2. **Test Components**: Test each component individually
3. **Check Power**: Ensure adequate power supply
4. **Verify Connections**: Double-check all wiring
5. **Calibrate Regularly**: Recalibrate for different surfaces

## 🚀 Advanced Features

### Possible Enhancements
1. **PID Control**: Implement PID algorithm for smoother following
2. **Speed Control**: Variable speed based on turn radius
3. **Obstacle Detection**: Add ultrasonic sensors
4. **Wireless Control**: Add Bluetooth/WiFi for remote control
5. **Data Logging**: Store path data for analysis
6. **Machine Learning**: Adaptive learning algorithms

### Code Modifications
```cpp
// Example: Adding PID control
float kP = 1.0, kI = 0.0, kD = 0.1;
float lastError = 0, integral = 0;

float calculatePID(float error) {
  integral += error;
  float derivative = error - lastError;
  float output = kP * error + kI * integral + kD * derivative;
  lastError = error;
  return output;
}
```

## 📚 Additional Knowledge

### Sensor Technology Deep Dive
**Infrared Sensors (IR)**:
- **Wavelength**: Typically 940nm
- **Range**: 2-30cm depending on model
- **Response Time**: Usually < 1ms
- **Power Consumption**: ~20mA per sensor

**Alternative Sensors**:
- **Ultrasonic**: For distance measurement
- **Camera**: Computer vision applications
- **LiDAR**: High-precision mapping
- **Magnetic**: For magnetic tape following

### Programming Concepts
**State Machines**:
```cpp
enum RobotState {
  FOLLOWING_LINE,
  SEARCHING_LINE,
  TURNING_LEFT,
  TURNING_RIGHT,
  STOPPED
};
```

**Interrupt Handling**:
```cpp
// For encoder feedback
volatile int encoderCount = 0;
void encoderISR() {
  encoderCount++;
}
```

### Mathematical Concepts
**Proportional Control**:
```
Output = Kp × Error
Where Error = Desired_Position - Current_Position
```

**Coordinate Systems**:
- **Robot Frame**: Relative to robot's orientation
- **World Frame**: Fixed reference frame
- **Path Frame**: Relative to the path being followed

### Performance Optimization
1. **Sensor Fusion**: Combine multiple sensor types
2. **Predictive Control**: Anticipate path changes
3. **Dynamic Speed**: Adjust speed based on curvature
4. **Error Correction**: Implement error recovery mechanisms

### Safety Considerations
- **Emergency Stop**: Implement safety cutoff
- **Boundary Detection**: Prevent robot from leaving area
- **Collision Avoidance**: Add obstacle detection
- **Battery Monitoring**: Prevent over-discharge

## 🤝 Contributing

We welcome contributions! Please follow these guidelines:

1. **Fork the Repository**
2. **Create Feature Branch**: `git checkout -b feature/new-feature`
3. **Commit Changes**: `git commit -am 'Add new feature'`
4. **Push to Branch**: `git push origin feature/new-feature`
5. **Create Pull Request**

### Development Guidelines
- Follow Arduino coding standards
- Add comments for complex logic
- Test thoroughly before submitting
- Update documentation as needed

## 📄 License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details.

## 📞 Support & Community

- **Issues**: Report bugs via GitHub Issues
- **Discussions**: Join our community discussions
- **Documentation**: Check wiki for detailed guides
- **Examples**: Browse example projects

---

**Happy Building! 🎉**

*Made with ❤️ by the robotics community*
