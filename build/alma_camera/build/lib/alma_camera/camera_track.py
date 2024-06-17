import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import time

class WoWasIst(Node):
    def __init__(self):
        super().__init__('wowasist')
        self.subscription = self.create_subscription(
            Image,
            '/camera_image',
            self.listener_callback,
            1)
        self.bridge = CvBridge()
        self.publisher_ = self.create_publisher(Int32, 'danlias_commands', 1)
        
        # Initialize control variables
        self.c_msg = Int32()
        self.lastWall = ""
        self.lastWallupdate = time.time()
        
        # Create trackbars for parameter adjustment
        cv2.namedWindow('Adjuster')
        cv2.createTrackbar('Bright', 'Adjuster', 200, 255, lambda x:None)
        cv2.createTrackbar('Start/Stop', 'Adjuster', 0, 1, lambda x:None)
        cv2.setTrackbarMin('Start/Stop', 'Adjuster', -1)
        cv2.createTrackbar('minArea', 'Adjuster', 1352, 5000, lambda x:None)
        cv2.createTrackbar('Crop', 'Adjuster', 80, 100, lambda x:None)
        cv2.createTrackbar('hDistDZ', 'Adjuster', 124, 1000, lambda x:None)
        cv2.createTrackbar('vCutoff', 'Adjuster', 66, 100, lambda x: None)

    def listener_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "mono8")
        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge Error: {e}")
            return
        
        frame_height, frame_width = frame.shape
        crop = cv2.getTrackbarPos('Crop', 'Adjuster') / 100.0
        b = cv2.getTrackbarPos('Bright', 'Adjuster')
        stasto = cv2.getTrackbarPos('Start/Stop', 'Adjuster')
        minArea = cv2.getTrackbarPos('minArea', 'Adjuster')
        h_distance_dz = cv2.getTrackbarPos('hDistDZ', 'Adjuster') / 1000
        vCutoff = cv2.getTrackbarPos('vCutoff', 'Adjuster') / 100
        
        mask = cv2.inRange(frame, b, 255)
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        index = [i for i in range(len(contours)) if cv2.contourArea(contours[i]) > minArea]
        
        result = cv2.cvtColor(frame, cv2.COLOR_GRAY2RGB)
        mask = np.zeros_like(mask)
        for i in index:
            cv2.drawContours(mask, contours, i, 1, -1)
            cv2.drawContours(result, contours, i, (0, 255, 0), 1)
        
        ch_mask = mask[int(frame_height * crop):frame_height,]
        ch_mask_height, ch_mask_width = ch_mask.shape
        
        mass_y, mass_x = np.where(mask == 1)
        cent_x = np.average(mass_x) if mass_x.size else frame_width // 2
        cent_y = np.average(mass_y) if mass_y.size else frame_height
        
        ch_mass_y, ch_mass_x = np.where(ch_mask == 1)
        ch_cent_x = np.average(ch_mass_x) if ch_mass_x.size else ch_mask_width // 2
        ch_cent_y = np.average(ch_mass_y) if ch_mass_y.size else ch_mask_height
        
        origin_x = frame_width // 2
        origin_y = frame_height
        ch_origin_x = ch_mask_width // 2
        ch_origin_y = ch_mask_height
        
        rot_y_unter_cutoff = ((cent_y > frame_height * vCutoff) and not np.isnan(cent_y))
        rotx_blaux_nah_dz = (abs(ch_cent_x - cent_x) < h_distance_dz * frame_width)
        
        if stasto == 1:
            if rot_y_unter_cutoff:
                if rotx_blaux_nah_dz:
                    if self.lastWall == "right":
                        self.c_msg.data = 3
                        if time.time() - self.lastWallupdate > 5:
                            self.lastWall = "left"
                            self.lastWallupdate = time.time()
                    elif self.lastWall == "left":
                        self.c_msg.data = 4
                        if time.time() - self.lastWallupdate > 5:
                            self.lastWall = "right"
                            self.lastWallupdate = time.time()
                    else:
                        self.c_msg.data = 3
                        if time.time() - self.lastWallupdate > 5:
                            self.lastWall = "right"
                            self.lastWallupdate = time.time()
                    cv2.rectangle(result, (int(ch_cent_x), int(ch_cent_y + (origin_y - ch_origin_y))),
                                  (int(cent_x), int(cent_y)), (255, 255, 0), 5)
                else:
                    self.c_msg.data = 4 if cent_x > ch_cent_x else 3
            else:
                if ch_cent_x < origin_x * 0.2:
                    self.c_msg.data = 4
                elif ch_cent_x > origin_x * 1.2:
                    self.c_msg.data = 3
                else:
                    self.c_msg.data = 1
        elif stasto == -1:
            self.c_msg.data = 2
        else:
            self.c_msg.data = 0

        cv2.circle(result, (int(cent_x), int(cent_y)), 5, (0, 0, 255), -1)
        cv2.circle(result, (int(ch_cent_x), int(ch_cent_y + (origin_y - ch_origin_y))), 5, (255, 0, 0), -1)
        cv2.line(result, (0, int(origin_y - ch_origin_y)), (frame_width, int(origin_y - ch_origin_y)), (255, 0, 0), 2)
        cv2.line(result, (0, int(frame_height * vCutoff)), (frame_width, int(frame_height * vCutoff)), (0, 0, 255), 1)
        cv2.line(result, (int(ch_cent_x), int(ch_cent_y + (origin_y - ch_origin_y))),
                 (int(cent_x), int(cent_y)), (255, 0, 255), 1)

        cv2.imshow('result', result)
        cv2.waitKey(1)
        self.publisher_.publish(self.c_msg)
        self.get_logger().info(f'Publishing: {self.c_msg.data}')

def main(args=None):
    rclpy.init(args=args)
    wowasist = WoWasIst()
    try:
        rclpy.spin(wowasist)
    except KeyboardInterrupt:
        pass
    finally:
        wowasist.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
