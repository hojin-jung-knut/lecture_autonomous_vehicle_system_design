#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage

class IMGParser:
    def __init__(self):
        rospy.init_node('image_parser', anonymous=True)
        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback)
        rospy.spin()

    def callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            cv2.imshow("Image window", img_bgr)
            cv2.waitKey(1)
        except Exception as e:
            rospy.logerr("Failed to decode image: %s", e)

if __name__ == '__main__':
    IMGParser()