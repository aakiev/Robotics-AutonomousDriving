import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge
import cv2
import numpy as np

class LaneDetector(Node):
    def __init__(self):
        super().__init__('lane_detector')
        self.subscription = self.create_subscription(
            Image,
            'camera_image',
            self.image_callback,
            10)
        self.publisher_ = self.create_publisher(Int32, 'drive_inputs', 10)
        self.br = CvBridge()
        self.vector_length = 50  # Desired length of the vector
        self.min_line_length = 50  # Minimum length of a line to be detected
        self.max_line_gap = 10  # Maximum allowed gap between line segments to treat them as a single line
        self.last_turn = None  # To keep track of the last significant turn
        self.block_turn_4 = False  # To block turning left after a sharp right turn (5)

    def image_callback(self, msg):
        frame = self.br.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # Convert to HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define range for white color
        lower_white = np.array([0, 0, 200])
        upper_white = np.array([180, 25, 255])

        # Threshold the image to get only white colors
        mask = cv2.inRange(hsv, lower_white, upper_white)

        # Bitwise-AND mask and original image
        filtered = cv2.bitwise_and(frame, frame, mask=mask)

        # Convert to grayscale
        gray = cv2.cvtColor(filtered, cv2.COLOR_BGR2GRAY)

        # Apply Gaussian blur to reduce noise and improve edge detection
        blur = cv2.GaussianBlur(gray, (5, 5), 0)

        # Perform Canny edge detection
        edges = cv2.Canny(blur, 50, 150)

        # Apply Hough Line Transform to detect lines
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 50, minLineLength=self.min_line_length, maxLineGap=self.max_line_gap)

        left_lines = []
        right_lines = []
        front_lines = []

        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                slope = (y2 - y1) / (x2 - x1) if x2 - x1 != 0 else float('inf')
                intercept = y1 - slope * x1
                if abs(slope) > 0.5:  # Filter out nearly horizontal lines
                    if slope < 0:
                        left_lines.append((slope, intercept))
                    else:
                        right_lines.append((slope, intercept))
                elif abs(slope) <= 0.5:
                    front_lines.append((slope, intercept))

        # Calculate car position (center of the bottom half of the frame)
        car_center_x = frame.shape[1] // 2
        car_center_y = frame.shape[0] * 3 // 4

        # Determine lane direction
        lane_direction = self.determine_lane_direction(left_lines, right_lines, front_lines)

        # Publish lane direction and print it to the terminal
        self.publisher_.publish(Int32(data=lane_direction))
        print(f"Lane direction: {lane_direction}")

        # Draw lines on the image for visualization
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

        # Draw car (square) at the bottom center of the frame
        cv2.rectangle(frame, (car_center_x - 10, car_center_y - 10), (car_center_x + 10, car_center_y + 10), (0, 0, 255), -1)

        # Calculate and draw vector based on lane direction
        vector_x, vector_y = self.calculate_direction_vector(lane_direction)
        if lane_direction not in [4, 5]:  # Draw the arrow only if the direction is not 4 or 5
            cv2.arrowedLine(frame, (car_center_x, car_center_y), (car_center_x + vector_x, car_center_y + vector_y), (255, 0, 0), 2)

        # Display the modified frame
        cv2.imshow('Lane Detection', frame)
        cv2.waitKey(1)

    def determine_lane_direction(self, left_lines, right_lines, front_lines):
        if not left_lines and not right_lines and not front_lines:
            return 3  # Case 1: No lines detected

        if left_lines and right_lines and not front_lines:
            return 3  # Case 6: Two lines detected on right and left at the same time

        if left_lines and not right_lines and not front_lines:
            self.last_turn = 1
            self.block_turn_4 = False
            return 1  # Case 2: White line detected on the left

        if right_lines and not left_lines and not front_lines:
            self.last_turn = 2
            self.block_turn_4 = False
            return 2  # Case 3: White line detected on the right

        if front_lines and not left_lines and not right_lines:
            if self.block_turn_4:
                return 5  # Continue sharp right if blocked from turning left
            return 4  # Case 4: Line directly in front, turn sharp left

        if front_lines and right_lines and not left_lines:
            if self.block_turn_4:
                return 5  # Continue sharp right if blocked from turning left
            return 4  # Case 5: Line directly in front and right, turn sharp left

        if front_lines and left_lines and not right_lines:
            self.last_turn = 5
            self.block_turn_4 = True
            return 5  # Case 6: Line directly in front and left, turn sharp right

        # New condition for the specific difficult curve
        if front_lines and left_lines and right_lines:
            if self.last_turn == 1:
                self.block_turn_4 = True
                return 5  # After turning left, if a line is detected in front and both sides, turn sharp right
            if self.last_turn == 2:
                return 2  # After turning right, if a line is detected in front and both sides, turn sharp right

        self.last_turn = None  # Reset last turn if lines on the left or right are detected
        self.block_turn_4 = False  # Reset block on turn left
        return 3  # Default case: Go straight


    def calculate_direction_vector(self, lane_direction):
        if lane_direction == 1:  # Turn right
            return self.vector_length, 0
        elif lane_direction == 2:  # Turn left
            return -self.vector_length, 0
        elif lane_direction == 3:  # Go straight
            return 0, -self.vector_length
        elif lane_direction in [4, 5, 6]:  # Stop/No movement or slow straight
            return 0, 0

def main(args=None):
    rclpy.init(args=args)
    lane_detector = LaneDetector()
    rclpy.spin(lane_detector)
    lane_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

