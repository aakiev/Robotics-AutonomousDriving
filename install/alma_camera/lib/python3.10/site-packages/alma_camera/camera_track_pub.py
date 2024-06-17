import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraPublisher(Node):
    def __init__(self):
      super().__init__('camera_publisher')
      self.publisher_ = self.create_publisher(Image, 'camera_image', 10)
      timer_period = 0.033 #original 0.1
      self.timer = self.create_timer(timer_period, self.timer_callback)
      self.br = CvBridge()
      self.cap = cv2.VideoCapture(0)
      if not self.cap.isOpened():
        self.get_logger().error('Could not open videostream')
        exit(1)

    def timer_callback(self):
      ret, frame = self.cap.read()
      gframe =  cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
      #print(gframe.shape)
      gframe = cv2.resize(gframe, (0, 0), fx = 0.33, fy = 0.33)
      if ret:
         msg = self.br.cv2_to_imgmsg(gframe, encoding="mono8")
         self.publisher_.publish(msg)
      else:
         self.get_logger().error('Error in capturing image')
            
    def destroy_node(self):
      super().destroy_node()
      self.cap.release()
        
def main(args=None):
    rclpy.init(args=args)
    camera_publisher = CameraPublisher()
    rclpy.spin(camera_publisher)
    camera_publisher.destroy_node()
    rclpy.shutdown()
    
if __name__=='__main__':
    main()