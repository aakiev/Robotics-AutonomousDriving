import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class ImageDisplayNode(Node):
    def __init__(self):
        super().__init__('image_display_node')
        self.subscription = self.create_subscription(
            Image,
            '/camera_image',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()
        #create trackbars for color tracking
        cv2.namedWindow('ColorAdjuster')
        cv2.createTrackbar('H', 'ColorAdjuster', 0, 179, lambda x:None)
                     
    def listener_callback(self, msg):
        try:
            # Convert your ROS Image message to OpenCV2
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
        else:       
            # Display the image in an OpenCV window
            h = cv2.getTrackbarPos('H', 'ColorAdjuster')
            lower_color = np.array([h, 0, 0])
            upper_color = np.array([h+5, 255, 255])
            #Convert BGR to HSV
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            #Threshold the HSV image to get only blue colors
            mask = cv2.inRange(hsv, lower_color, upper_color)
            #Bitwise-AND mask and original image
            result = cv2.bitwise_and(frame, frame, mask = mask)
        
            #cv2.imshow('image',img)
            #cv2.imshow('mask', mask)
            cv2.imshow('res',result)
            cv2.imshow("Camera Feed", frame)
            cv2.waitKey(1)  # Refresh the image on the window

def main(args=None):
    rclpy.init(args=args)  # Initialize communications with ROS2
    image_display_node = ImageDisplayNode()
    try:
        rclpy.spin(image_display_node)  # Spin the node so the callback function is called.
    except KeyboardInterrupt:
        pass  # Allow the node to be interrupted with Ctrl+C
    finally:
        image_display_node.destroy_node()
        rclpy.shutdown()  # Shutdown ROS2 Python interface
        cv2.destroyAllWindows()  # Close all OpenCV windows

if __name__ == '__main__':
    main()
