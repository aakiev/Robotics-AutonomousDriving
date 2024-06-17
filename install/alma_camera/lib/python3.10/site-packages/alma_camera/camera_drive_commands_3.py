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
        self.publisher_ = self.create_publisher(Int32, 'micro_ros_alma_subscriber', 10) 
        self.br = CvBridge()
        self.vector_length = 50  # Desired length of the vector
        self.min_contour_area = 500  # Minimum area for a contour to be considered a lane

    def image_callback(self, msg):
        frame = self.br.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # Convert image to HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define white color range in HSV
        lower_white = np.array([0, 0, 180])
        upper_white = np.array([180, 25, 255])

        # Threshold the HSV image to get only white colors
        mask = cv2.inRange(hsv, lower_white, upper_white)

        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Filter out small contours
        contours = [contour for contour in contours if cv2.contourArea(contour) > self.min_contour_area]

        # Draw the contours on the original frame
        cv2.drawContours(frame, contours, -1, (0, 255, 0), 2)

        # Calculate car position (center of the bottom half of the frame)
        car_center_x = frame.shape[1] // 2
        car_center_y = frame.shape[0] * 3 // 4

        # Draw bounding rectangles around the two white lanes
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)

        # Calculate the middle of the road (ideal position for the car)
        middle_of_road = self.calculate_middle_of_road(contours, car_center_x)

        # Determine lane direction
        lane_direction = self.determine_lane_direction(middle_of_road, car_center_x)

        # Publish lane direction and print it to the terminal
        self.publisher_.publish(Int32(data=lane_direction))
        self.get_logger().info(f"Lane direction: {lane_direction}")

        # Draw middle line as a green line
        if middle_of_road is not None:
            cv2.line(frame, (middle_of_road, 0), (middle_of_road, frame.shape[0]), (0, 255, 0), 2)

        # Draw car (square) at the bottom center of the frame
        cv2.rectangle(frame, (car_center_x - 10, car_center_y - 10), (car_center_x + 10, car_center_y + 10), (0, 0, 255), -1)

        # Calculate and draw vector based on lane direction
        vector_x, vector_y = self.calculate_direction_vector(lane_direction)
        if lane_direction != 4:  # Draw the arrow only if the direction is not 4 (stop/no movement)
            cv2.arrowedLine(frame, (car_center_x, car_center_y), (car_center_x + vector_x, car_center_y + vector_y), (255, 0, 0), 2)

        # Display the modified frame
        cv2.imshow('Lane Detection', frame)
        cv2.waitKey(1)

    def calculate_middle_of_road(self, contours, car_center_x):
        if len(contours) >= 2:
            # Find the bounding boxes of the two largest contours (assumed to be the white lanes)
            contours = sorted(contours, key=cv2.contourArea, reverse=True)[:2]
            x1, _, w1, _ = cv2.boundingRect(contours[0])
            x2, _, w2, _ = cv2.boundingRect(contours[1])
            
            # Calculate the middle of the road (middle point between the left boundary of the first white lane
            # and the right boundary of the second white lane)
            middle_of_road = (x1 + w1 + x2) // 2
            return middle_of_road
        elif len(contours) == 1:
            # If only one contour is detected, it could mean the car is close to the edge of the lane
            x, _, w, _ = cv2.boundingRect(contours[0])
            if x + w // 2 < car_center_x:
                return x + w  # White line is on the left
            else:
                return x  # White line is on the right
        else:
            return None

    def determine_lane_direction(self, middle_of_road, car_center_x):
        if middle_of_road is None:
            return 3  # No lines detected
        elif middle_of_road < car_center_x - 10:
            return 1  # White line detected on the left
        elif middle_of_road > car_center_x + 10:
            return 2  # White line detected on the right
        else:
            return 4  # White line detected directly in front

    def calculate_direction_vector(self, lane_direction):
        if lane_direction == 1:  # Turn right
            return self.vector_length, 0
        elif lane_direction == 2:  # Turn left
            return -self.vector_length, 0
        elif lane_direction == 3:  # Go straight
            return 0, -self.vector_length
        elif lane_direction == 4:  # Stop/No movement
            return 0, 0

def main(args=None):
    rclpy.init(args=args)
    lane_detector = LaneDetector()
    rclpy.spin(lane_detector)
    lane_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
