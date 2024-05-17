import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float64
import serial

class UltrasoundNode(Node):
  def __init__(self):
      super().__init__('ultrasound_node')
      self.publisher_= self.create_publisher(String, 'ultrasound_topic',10)
      self.serial_port = serial.Serial('/dev/ttyACM0', 9600, timeout = 2)
      timer_period = 0.1 #0.1 Sekunden!
      self.timer = self.create_timer(timer_period, self.timer_callback)
      self.i = 0
		
  def timer_callback(self):
    if(self.serial_port.in_waiting > 0):
      data_line = self.serial_port.readline().decode('ascii').strip()
      msg = String()
      msg.data = data_line
      self.publisher_.publish(msg)
      self.get_logger().info('Publishing: "%s"' % data_line)
			
def main(args=None):
  rclpy.init(args=args)
  ultrasound_node = UltrasoundNode()
  rclpy.spin(ultrasound_node)
  ultrasound_node.destroy_node()
  rclpy.shutdown()
		
if __name__ == '__main__':
    main()  
