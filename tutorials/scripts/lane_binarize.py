#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, cv2, numpy as np
from sensor_msgs.msg import CompressedImage

class IMGParser:
    def __init__(self):
        rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback)
        self.img_bgr = None

    def callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            self.img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except Exception as e:
            rospy.logerr(f"Error decoding image: {e}")
            return

        if self.img_bgr is not None:
            img_hsv = cv2.cvtColor(self.img_bgr, cv2.COLOR_BGR2HSV)

            # Tune these values based on your specific scenario
            lower_wlane = np.array([0, 0, 185])
            upper_wlane = np.array([30, 60, 255])

            img_wlane = cv2.inRange(img_hsv, lower_wlane, upper_wlane)
            img_wlane = cv2.cvtColor(img_wlane, cv2.COLOR_GRAY2BGR)

            img_concat = np.concatenate([self.img_bgr, img_hsv, img_wlane], axis=1)

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