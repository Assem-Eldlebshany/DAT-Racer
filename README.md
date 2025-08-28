# DAT-Racer 🏎️
**vroom vroom** - Hand Gesture Controlled 4WD Arduino Car

## Overview
DAT-Racer is a hand gesture-controlled 4WD car system that uses computer vision to track hand movements and wirelessly control an Arduino-based car. The system uses MediaPipe for hand tracking, OpenCV for computer vision processing, and nRF24L01 modules for wireless communication between a computer and the remote-controlled car.

## System Architecture

The project consists of three main components:

1. **Computer Vision Controller** (`controller.py`) - Captures hand gestures via webcam
2. **Arduino Uno Server** (`Uno_code_final.ino`) - Receives commands from computer and transmits wirelessly
3. **Arduino Nano Car** (`Nano_code_final.ino`) - Receives wireless commands and controls the 4WD car

```
[Computer + Webcam] → [Arduino Uno + nRF24L01] → [Arduino Nano + nRF24L01 + 4WD Car]
     controller.py        Uno_code_final.ino         Nano_code_final.ino
```

## Features

### Hand Gesture Controls
- **Speed Control**: Hand openness controls speed
  - Open hand (spread fingers) = 0% speed (stop)
  - Closed fist = 100% speed (full speed)
  - Partial closure = proportional speed
- **Steering Control**: Hand tilt controls direction
  - Tilt left = turn left (-45° max)
  - Tilt right = turn right (+45° max)
  - Flat hand = straight driving

### Safety Features
- **Timeout Protection**: Car stops if no commands received within 500ms
- **Signal Loss Detection**: Emergency stop if wireless connection lost
- **Hand Detection**: Car stops when no hand is detected in frame
- **Value Validation**: All speed and angle values are constrained to safe ranges

### Visual Feedback
- Real-time hand landmark visualization
- Speed bar display (0-100%)
- Angle indicator with visual compass
- Connection status display
- Control value smoothing for stable operation

## Hardware Requirements

### Computer Side
- Computer with webcam
- Arduino Uno
- nRF24L01 wireless module
- Connecting wires
- USB cable for Arduino

### Car Side
- Arduino Nano
- nRF24L01 wireless module
- L298N motor driver
- 4WD car chassis with motors
- 4 AA battery pack (6V)
- LED indicator
- Connecting wires

## Wiring Diagrams

### Arduino Uno (Server) + nRF24L01
```
nRF24L01    Arduino Uno
VCC      →  3.3V (NOT 5V!)
GND      →  GND
CE       →  Pin 9
CSN      →  Pin 10
SCK      →  Pin 13
MOSI     →  Pin 11
MISO     →  Pin 12
LED      →  Pin 13 (built-in)
```

### Arduino Nano (Car) + L298N + nRF24L01
```
nRF24L01    Arduino Nano
VCC      →  3.3V (NOT 5V!)
GND      →  GND
CE       →  Pin 9
CSN      →  Pin 10
SCK      →  Pin 13
MOSI     →  Pin 11
MISO     →  Pin 12

L298N       Arduino Nano
ENA      →  Pin 5 (PWM)
IN1      →  Pin 6
IN2      →  Pin 7
ENB      →  Pin 3 (PWM)
IN3      →  Pin 4
IN4      →  Pin 8

LED      →  Pin A0
Battery  →  Pin A1 (optional voltage monitoring)
```

### Motor Connections
- **Motor A (L298N)**: Left side motors (front-left + rear-left in parallel)
- **Motor B (L298N)**: Right side motors (front-right + rear-right in parallel)

## Software Dependencies

### Python Requirements
```bash
pip install opencv-python
pip install mediapipe
pip install numpy
pip install pyserial
```

### Arduino Libraries
- SPI (built-in)
- nRF24L01 - Install "RF24" library by TMRh20
- RF24 (part of nRF24L01 library)

## Installation & Setup

### 1. Hardware Assembly
1. Wire the Arduino Uno with nRF24L01 module according to the wiring diagram
2. Assemble the 4WD car with Arduino Nano, L298N motor driver, and nRF24L01
3. Connect motors to L298N driver
4. Install batteries and test power connections

### 2. Arduino Programming
1. Upload `Uno_code_final.ino` to the Arduino Uno
2. Upload `Nano_code_final.ino` to the Arduino Nano
3. Verify both Arduinos boot successfully and establish wireless connection

### 3. Python Setup
1. Install required Python packages
2. Update the serial port in `controller.py` (line 235):
   ```python
   # Change to your Arduino Uno's serial port
   controller = HandGestureController(serial_port='/dev/cu.usbmodem101', baudrate=9600)
   ```
   - **Windows**: Usually `'COM3'`, `'COM4'`, etc.
   - **macOS**: Usually `'/dev/cu.usbmodem101'` or similar
   - **Linux**: Usually `'/dev/ttyUSB0'` or `'/dev/ttyACM0'`

## Usage

### Starting the System
1. **Power on the car** (Arduino Nano + motors)
2. **Connect Arduino Uno** to computer via USB
3. **Run the Python controller**:
   ```bash
   python controller.py
   ```
4. **Position your hand** in front of the webcam
5. **Control the car** with hand gestures!

### Controls
- **Stop**: Open your hand completely (spread all fingers)
- **Go**: Close your hand into a fist
- **Speed**: Partially close your hand for variable speed
- **Turn Left**: Tilt your hand to the left
- **Turn Right**: Tilt your hand to the right
- **Quit**: Press 'q' key in the video window

### Troubleshooting

#### No Arduino Connection
- Check USB cable and drivers
- Verify correct serial port in code
- The system will run in "Demo Mode" without Arduino

#### Poor Hand Detection
- Ensure good lighting
- Keep hand clearly visible in frame
- Avoid background clutter
- Camera should be at eye level

#### Car Not Responding
- Check nRF24L01 wiring (especially 3.3V power!)
- Verify both Arduinos are powered
- Check motor driver connections
- Ensure batteries are charged

#### Weak Motor Performance
- Check battery voltage (should be >5V for 4 AA batteries)
- Verify L298N connections
- Motors may need higher PWM values (adjust in code)

## Technical Details

### Communication Protocol
- **Python ↔ Arduino Uno**: Serial communication via USB
  - Format: `S<speed>A<angle>\n`
  - Example: `S75A-20\n` (75% speed, -20° turn)

- **Arduino Uno ↔ Arduino Nano**: nRF24L01 wireless
  - Data structure: `{int speed; int angle;}`
  - Frequency: 2.4GHz
  - Range: ~100m (line of sight)

### Hand Tracking Algorithm
- Uses MediaPipe hand landmarks (21 points per hand)
- **Speed calculation**: Distance ratio between fingertips and palm
- **Angle calculation**: Tilt angle of line between knuckles
- **Smoothing**: Low-pass filter for stable control

### Motor Control
- **Differential steering**: Different speeds for left/right motors
- **Sharp turns**: Inner wheel reversal for tighter turning radius
- **PWM control**: Variable speed using PWM signals (120-255 range)

## Customization

### Adjusting Sensitivity
In `controller.py`, modify these parameters:
```python
# Hand detection confidence
min_detection_confidence=0.7    # Lower = more sensitive
min_tracking_confidence=0.5     # Lower = more sensitive

# Control smoothing
alpha = 0.7  # Higher = less smooth, more responsive
```

### Motor Performance Tuning
In `Nano_code_final.ino`, adjust PWM ranges:
```cpp
// Minimum PWM for motor startup
baseSpeed = map(speed, 1, 100, 120, 255);  // Increase 120 if motors don't start

// Turn sensitivity
leftSpeed = baseSpeed * (1 - turnRatio * 0.6);  // Increase 0.6 for sharper turns
```

### Wireless Range
Increase transmission power for longer range:
```cpp
radio.setPALevel(RF24_PA_HIGH);  // Options: LOW, HIGH, MAX
```

## Future Enhancements
- Multiple car control (racing mode)
- Obstacle avoidance using ultrasonic sensors
- FPV camera integration
- Mobile app control interface
- Voice commands integration
- Racing lap timer and scoring system

## License
This project is open source. Feel free to modify and improve!

## Contributing
Pull requests are welcome! Please feel free to submit bug reports, feature requests, or improvements.

---

**Happy Racing! 🏁**
