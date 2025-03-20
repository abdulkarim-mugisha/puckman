import cv2
import numpy as np
from collections import deque
from datetime import datetime
import serial
import time

class PuckTracker:
    def __init__(self, buffer_size=32):
        # Initialize deque to store puck positions for trajectory analysis
        self.pts = deque(maxlen=buffer_size)
        self.last_positions = deque(maxlen=5)  # Store last 5 positions with timestamps

    def detect_puck(self, frame):
        # Convert to HSV color space for color detection
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define range for yellow color
        lower_yellow = np.array([20, 100, 100])  # Changed from purple to yellow
        upper_yellow = np.array([30, 255, 255])  # Changed from purple to yellow

        # Create mask for yellow
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # Apply Gaussian blur to reduce noise
        blurred = cv2.GaussianBlur(mask, (9, 9), 2)

        # Find contours
        contours, _ = cv2.findContours(
            blurred, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        puck_position = None

        # Find the circular contour that matches puck characteristics
        for contour in contours:
            area = cv2.contourArea(contour)

            # Filter based on area (adjusted for smaller puck)
            if 100 < area < 2000:  # Changed from 500-10000 to 100-2000
                # Check circularity
                perimeter = cv2.arcLength(contour, True)
                circularity = 4 * np.pi * area / (perimeter * perimeter)

                if circularity > 0.6:  # Keeping the same circularity threshold
                    M = cv2.moments(contour)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        puck_position = (cx, cy)

                        # Store position with timestamp
                        self.last_positions.append(
                            {"position": puck_position, "timestamp": datetime.now()}
                        )

                        # Add to trajectory points
                        self.pts.appendleft(puck_position)
        
        return puck_position

    def predict_position(self, time_ahead_ms=100):
        """Predict puck position some milliseconds in the future"""
        if len(self.last_positions) < 2:
            return None

        # Calculate velocity from last two positions
        pos1 = self.last_positions[-1]
        pos2 = self.last_positions[-2]

        time_diff = (pos1["timestamp"] - pos2["timestamp"]).total_seconds()
        if time_diff == 0:
            return None

        # Calculate velocity (pixels per second)
        velocity_x = (pos1["position"][0] - pos2["position"][0]) / time_diff
        velocity_y = (pos1["position"][1] - pos2["position"][1]) / time_diff

        # Predict future position
        time_ahead = time_ahead_ms / 1000.0  # convert to seconds
        pred_x = int(pos1["position"][0] + velocity_x * time_ahead)
        pred_y = int(pos1["position"][1] + velocity_y * time_ahead)

        return (pred_x, pred_y)

    def draw_debug(self, frame):
        """Draw debug visualization"""
        # Draw trajectory
        for i in range(1, len(self.pts)):
            if self.pts[i - 1] is None or self.pts[i] is None:
                continue
            cv2.line(frame, self.pts[i - 1], self.pts[i], (0, 255, 0), 2)

        # Draw current position
        if len(self.last_positions) > 0:
            cv2.circle(frame, self.last_positions[-1]["position"], 5, (0, 0, 255), -1)

        # Draw predicted position
        pred_pos = self.predict_position()
        if pred_pos is not None:
            cv2.circle(frame, pred_pos, 5, (255, 0, 0), -1)

        return frame


def detect_pusher(frame):
    """
    Detect the green pusher in the frame
    """
    # Convert BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Define green color range in HSV (adjusted for the green pusher shown in image)
    lower_green = np.array([35, 40, 40])
    upper_green = np.array([90, 255, 255])

    # Create a mask for green color
    mask = cv2.inRange(hsv, lower_green, upper_green)

    # Apply morphological operations to reduce noise
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.erode(mask, kernel, iterations=1)
    mask = cv2.dilate(mask, kernel, iterations=2)

    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        # Get the largest contour (assuming it's the pusher)
        largest_contour = max(contours, key=cv2.contourArea)

        # Get bounding box only if the contour area is significant
        area = cv2.contourArea(largest_contour)
        if area > 100:  # Minimum area threshold
            x, y, w, h = cv2.boundingRect(largest_contour)
            return (x, y, w, h)

    return None


class SerialController:
    def __init__(self, port, baudrate=115200):
        try:
            self.ser = serial.Serial(port, baudrate, timeout=1)
            self.current_command = None  # Track current command to avoid repeats
            time.sleep(2)
            print(f"Connected to {port}")
        except Exception as e:
            print(f"Error connecting to serial port: {e}")
            self.ser = None
    
    def send_command(self, command):
        """Send a command to the Pi Pico only if it's different from current command"""
        if self.ser and command != self.current_command:
            try:
                self.ser.write(f"{command}\n".encode())
                print(f"Sent command: {command}")  # Debug print
                self.current_command = command
            except Exception as e:
                print(f"Error sending command: {e}")


def main():
    # Replace with your camera's URL or use 0 for webcam
    url = "ACTUAL_URL"  # IP camera stream URL
    
    # Use your actual Pi Pico serial port
    serial_port = "ACTUAL_PORT"
    
    # Initialize tracker and serial controller
    tracker = PuckTracker()
    controller = SerialController(serial_port)
    
    # Open video capture
    cap = cv2.VideoCapture(url)
    
    # Define alignment threshold (in pixels)
    ALIGNMENT_THRESHOLD = 5       
    
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Failed to get frame")
                break
            
            # Detect puck
            puck_pos = tracker.detect_puck(frame)
            
            # Detect pusher
            pusher_bbox = detect_pusher(frame)
            
            # Control motors based on alignment
            if puck_pos is not None and pusher_bbox is not None:
                pusher_center_y = pusher_bbox[1] + pusher_bbox[3] // 2
                puck_y = puck_pos[1]
                print(f"Pusher Y: {pusher_center_y}, Puck Y: {puck_y}")
                
                # Only send stop when actually aligned
                if abs(pusher_center_y - puck_y) <= ALIGNMENT_THRESHOLD:
                    controller.send_command("stop")
                    print("Aligned - stopping")
                else:
                    # Continuous movement until aligned
                    if pusher_center_y < puck_y:
                        controller.send_command("right")
                        print("Moving right")
                    else:
                        controller.send_command("left")
                        print("Moving left")

            
            # Draw debug visualization
            if frame is not None:
                # Draw pusher bounding box if detected
                if pusher_bbox is not None:
                    x, y, w, h = pusher_bbox
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.circle(frame, (x + w // 2, y + h // 2), 3, (0, 255, 0), -1)  # Center
                
                # Draw puck tracking visuals
                frame = tracker.draw_debug(frame)
                
                # Display frame
                cv2.imshow("Puck Tracking", frame)
            
            # Exit on 'q' key
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    
    finally:
        # Cleanup
        controller.stop()
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()