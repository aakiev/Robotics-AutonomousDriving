import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

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

    def listener_callback(self, msg):
        try:
            # Convert your ROS Image message to OpenCV2
            cv2_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
        else:
            # Display the image in an OpenCV window
            cv2.imshow("Camera Feed", cv2_img)
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
