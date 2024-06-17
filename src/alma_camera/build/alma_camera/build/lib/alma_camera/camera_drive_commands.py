import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower')
        self.publisher_ = self.create_publisher(Int32, 'micro_ros_alma_subscriber', 10)
        self.subscription = self.create_subscription(
            Image,
            '/camera_image',
            self.image_callback,
            10)
        self.bridge = CvBridge()
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.latest_image = None

    def image_callback(self, msg):
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def timer_callback(self):
        if self.latest_image is None:
            self.get_logger().error('No image received yet')
            return

        # Convert to grayscale
        gray = cv2.cvtColor(self.latest_image, cv2.COLOR_BGR2GRAY)
        
        # Apply a binary threshold to the image
        _, binary = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)
        
        # Find contours
        contours, _ = cv2.findContours(binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            # Assume the largest contour is the line
            largest_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest_contour)
            
            if M['m00'] != 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                
                # Control logic based on contour position
                if cx < self.latest_image.shape[1] // 3:
                    self.send_command(3)  # Turn left
                elif cx > 2 * self.latest_image.shape[1] // 3:
                    self.send_command(4)  # Turn right
                else:
                    self.send_command(1)  # Move forward
            else:
                self.send_command(0)  # Stop if no line detected
        else:
            self.send_command(0)  # Stop if no contours found

    def send_command(self, command):
        msg = Int32()
        msg.data = command
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    line_follower = LineFollower()
    rclpy.spin(line_follower)
    line_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

