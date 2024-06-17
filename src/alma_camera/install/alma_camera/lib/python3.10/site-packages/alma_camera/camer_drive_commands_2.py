import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
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
        self.publisher_ = self.create_publisher(Int32, 'engines', 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()
        self.error = 0

    def listener_callback(self, msg):
        try:
            # Convert your ROS Image message to OpenCV2
            cv2_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
        else:
            frame_gray = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2GRAY)
            ret, frame_binary = cv2.threshold(frame_gray, 210, 255, cv2.THRESH_BINARY)
            rows = 3
            cols = 3
            lastSideline = 0
            retval = []
            retvalsum = 0
            threshold = 1000
            height, width, _ = cv2_img.shape

            tile_height = height // rows
            tile_width = width // cols

            for row in range(rows):
                for col in range(cols):
                    y1 = row * tile_height
                    y2 = (row + 1) * tile_height
                    x1 = col * tile_width
                    x2 = (col + 1) * tile_width

                    tile = frame_binary[y1:y2, x1:x2]
                    temp = cv2.countNonZero(tile)
                    retval.append(temp)

            retvalsum = retval[3] + retval[6] - retval[5] - retval[8]

            # Links == -1 # Rechts == 1 # VorwÃ¤rts == 0
            if (retval[4] > 2000):
                self.error = -1
            else:
                if (retvalsum > threshold):
                    self.error = 1
                elif (retvalsum < -threshold):
                    self.error = -1
                else:
                    self.error = 0
            
            cv2.waitKey(1)

    def timer_callback(self):
        msg = Int32()
        msg.data = self.error
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')

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