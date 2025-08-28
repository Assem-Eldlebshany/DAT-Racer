import cv2
import mediapipe as mp
import numpy as np
import serial
import time
import math


class HandGestureController:
    def __init__(self, serial_port='/dev/cu.usbmodem101', baudrate=9600):
        """
        Initialize the hand gesture controller for a single player.

        Args:
            serial_port: Serial port for Arduino communication
            baudrate: Baud rate for serial communication
        """
        # Initialize MediaPipe hands
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,  # Single player
            min_detection_confidence=0.7,
            min_tracking_confidence=0.5
        )
        self.mp_drawing = mp.solutions.drawing_utils

        # Initialize serial communication (uncomment when Arduino is connected)
        try:
            self.serial_conn = serial.Serial(serial_port, baudrate, timeout=1)
            time.sleep(2)  # Wait for Arduino to initialize
            print(f"Connected to Arduino on {serial_port}")
        except:
            self.serial_conn = None
            print("Arduino not connected - running in demo mode")

        # Control parameters
        self.current_speed = 0
        self.current_angle = 0

    def calculate_hand_openness(self, hand_landmarks):
        """
        Calculate how open the hand is (0 = fully closed fist, 1 = fully open).
        Based on the distance between fingertips and palm.
        """
        # Get wrist position (base of palm)
        wrist = hand_landmarks.landmark[self.mp_hands.HandLandmark.WRIST]

        # Get fingertip positions
        fingertips = [
            hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_TIP],
            hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_TIP],
            hand_landmarks.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_TIP],
            hand_landmarks.landmark[self.mp_hands.HandLandmark.RING_FINGER_TIP],
            hand_landmarks.landmark[self.mp_hands.HandLandmark.PINKY_TIP]
        ]

        # Get MCP (metacarpophalangeal) joints for reference
        mcps = [
            hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_MCP],
            hand_landmarks.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_MCP],
            hand_landmarks.landmark[self.mp_hands.HandLandmark.RING_FINGER_MCP],
            hand_landmarks.landmark[self.mp_hands.HandLandmark.PINKY_MCP]
        ]

        # Calculate average distance ratio
        openness_ratios = []

        for i in range(1, 5):  # Skip thumb for more reliable detection
            # Distance from fingertip to wrist
            tip_to_wrist = math.sqrt(
                (fingertips[i].x - wrist.x) ** 2 +
                (fingertips[i].y - wrist.y) ** 2
            )

            # Distance from MCP to wrist (reference length)
            mcp_to_wrist = math.sqrt(
                (mcps[i - 1].x - wrist.x) ** 2 +
                (mcps[i - 1].y - wrist.y) ** 2
            )

            # Ratio (normalized by MCP distance)
            if mcp_to_wrist > 0:
                ratio = tip_to_wrist / mcp_to_wrist
                # Clamp between typical closed (0.8) and open (2.0) values
                normalized_ratio = (ratio - 0.8) / (2.0 - 0.8)
                normalized_ratio = max(0, min(1, normalized_ratio))
                openness_ratios.append(normalized_ratio)

        return np.mean(openness_ratios) if openness_ratios else 0

    def calculate_hand_tilt(self, hand_landmarks):
        """
        Calculate the horizontal tilt of the hand in degrees.
        Range: -45 to +45 degrees (negative = left tilt, positive = right tilt)
        """
        # Get wrist and middle finger MCP for hand orientation
        wrist = hand_landmarks.landmark[self.mp_hands.HandLandmark.WRIST]
        middle_mcp = hand_landmarks.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_MCP]

        # Get index and pinky MCP for tilt calculation
        index_mcp = hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_MCP]
        pinky_mcp = hand_landmarks.landmark[self.mp_hands.HandLandmark.PINKY_MCP]

        # Calculate tilt based on the line between index and pinky MCPs
        # In normalized coordinates, x goes from 0 (left) to 1 (right)
        # y goes from 0 (top) to 1 (bottom)

        # Calculate angle of the line from index to pinky
        dx = pinky_mcp.x - index_mcp.x
        dy = pinky_mcp.y - index_mcp.y

        # Calculate angle in radians and convert to degrees
        angle_rad = math.atan2(dy, dx)
        angle_deg = math.degrees(angle_rad)

        # Normalize to -45 to +45 range
        # When hand is flat (horizontal line), angle should be close to 0
        # Adjust based on typical hand orientation
        tilt_angle = angle_deg

        # Clamp to -45 to +45 range
        tilt_angle = max(-45, min(45, tilt_angle))

        return tilt_angle

    def send_command_to_arduino(self, speed, angle):
        """
        Send control commands to Arduino.

        Args:
            speed: 0-100 (percentage)
            angle: -45 to +45 degrees
        """
        if self.serial_conn:
            # Format: "S<speed>A<angle>\n"
            command = f"S{int(speed)}A{int(angle)}\n"
            try:
                self.serial_conn.write(command.encode())
            except:
                print(f"Failed to send command: {command}")

    def draw_control_info(self, image, speed, angle):
        """
        Draw control information on the image.
        """
        h, w, _ = image.shape

        # Draw speed bar
        cv2.putText(image, f"Speed: {speed:.0f}%", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        bar_length = int(200 * (speed / 100))
        cv2.rectangle(image, (10, 40), (210, 60), (100, 100, 100), 2)
        cv2.rectangle(image, (10, 40), (10 + bar_length, 60), (0, 255, 0), -1)

        # Draw angle indicator
        cv2.putText(image, f"Angle: {angle:.0f}°", (10, 90),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        # Draw angle visualization
        center_x = 110
        center_y = 120
        radius = 40
        cv2.circle(image, (center_x, center_y), radius, (100, 100, 100), 2)

        # Draw angle line
        angle_rad = math.radians(-angle)  # Negative for correct visual orientation
        end_x = int(center_x + radius * math.cos(angle_rad))
        end_y = int(center_y + radius * math.sin(angle_rad))
        cv2.line(image, (center_x, center_y), (end_x, end_y), (0, 0, 255), 3)

        # Draw status
        status = "Arduino Connected" if self.serial_conn else "Demo Mode"
        cv2.putText(image, status, (10, h - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

    def run(self):
        """
        Main loop for hand gesture control.
        """
        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        print("Hand Gesture Car Controller Started")
        print("Controls:")
        print("- Open hand = 0% speed (stop)")
        print("- Closed fist = 100% speed (full speed)")
        print("- Tilt hand left/right = turn left/right (-45° to +45°)")
        print("- Press 'q' to quit")

        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                print("Failed to grab frame")
                break

            # Flip the frame horizontally for selfie-view
            frame = cv2.flip(frame, 1)

            # Convert BGR to RGB
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            rgb_frame.flags.writeable = False

            # Process the frame for hands
            results = self.hands.process(rgb_frame)

            # Convert back to BGR for OpenCV
            rgb_frame.flags.writeable = True
            frame = cv2.cvtColor(rgb_frame, cv2.COLOR_RGB2BGR)

            # If hand detected
            if results.multi_hand_landmarks:
                hand_landmarks = results.multi_hand_landmarks[0]

                # Draw hand landmarks
                self.mp_drawing.draw_landmarks(
                    frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS,
                    self.mp_drawing.DrawingSpec(color=(0, 0, 255), thickness=2, circle_radius=2),
                    self.mp_drawing.DrawingSpec(color=(0, 255, 0), thickness=2)
                )

                # Calculate control values
                openness = self.calculate_hand_openness(hand_landmarks)
                speed = openness * 100  # Convert to percentage

                tilt = self.calculate_hand_tilt(hand_landmarks)

                # Smooth the values (simple low-pass filter)
                alpha = 0.7  # Smoothing factor
                self.current_speed = alpha * speed + (1 - alpha) * self.current_speed
                self.current_angle = alpha * tilt + (1 - alpha) * self.current_angle

                # Send command to Arduino
                self.send_command_to_arduino(self.current_speed, self.current_angle)

            else:
                # No hand detected - stop the car
                self.current_speed = 0
                self.current_angle = 0
                self.send_command_to_arduino(0, 0)
                cv2.putText(frame, "No hand detected", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            # Draw control information
            self.draw_control_info(frame, self.current_speed, self.current_angle)

            # Display the frame
            cv2.imshow('Hand Gesture Car Controller', frame)

            # Break on 'q' key press
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        # Cleanup
        cap.release()
        cv2.destroyAllWindows()
        if self.serial_conn:
            self.send_command_to_arduino(0, 0)  # Stop the car
            self.serial_conn.close()
            print("Arduino connection closed")


if __name__ == "__main__":
    # Create controller instance
    # Change 'COM3' to your Arduino's serial port (e.g., '/dev/ttyUSB0' on Linux)
    controller = HandGestureController(serial_port='/dev/cu.usbmodem101', baudrate=9600)

    # Run the controller
    controller.run()