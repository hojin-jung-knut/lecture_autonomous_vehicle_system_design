#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, cv2, numpy as np
from sensor_msgs.msg import CompressedImage

class IMGParser:
    def __init__(self):
        rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback)
        self.img_bgr = None

        # Load parameters from the ROS parameter server or use default values
        self.lower_wlane = np.array(rospy.get_param('~lower_wlane', [0, 0, 185]))
        self.upper_wlane = np.array(rospy.get_param('~upper_wlane', [30, 60, 255]))
        self.lower_ylane = np.array(rospy.get_param('~lower_ylane', [20, 100, 100]))
        self.upper_ylane = np.array(rospy.get_param('~upper_ylane', [30, 255, 255]))

    def callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            self.img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except Exception as e:
            rospy.logerr(f"Error decoding image: {e}")
            return

        if self.img_bgr is not None:
            img_hsv = cv2.cvtColor(self.img_bgr, cv2.COLOR_BGR2HSV)

            # Create masks for white and yellow lanes
            mask_wlane = cv2.inRange(img_hsv, self.lower_wlane, self.upper_wlane)
            mask_ylane = cv2.inRange(img_hsv, self.lower_ylane, self.upper_ylane)
            
            # Combine both masks
            mask_combined = cv2.bitwise_or(mask_wlane, mask_ylane)
            
            # Convert the mask to BGR for concatenation
            img_mask = cv2.cvtColor(mask_combined, cv2.COLOR_GRAY2BGR)

            img_concat = np.concatenate([self.img_bgr, img_hsv, img_mask], axis=1)

            cv2.imshow("Image window", img_concat)
            cv2.waitKey(1)

if __name__ == '__main__':
    rospy.init_node('image_parser', anonymous=True)
    IMGParser()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
    cv2.destroyAllWindows()