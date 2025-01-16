#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage

class IMGParser:
    def __init__(self):
        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback)
        rospy.loginfo("Subscriber initialized")

        # Dynamic configuration through ROS parameters (could also be set via launch files)
        self.crop_pts = rospy.get_param('~crop_pts', [
            [320 / 2 - 20, 240 / 2 + 10],
            [320 / 2 + 20, 240 / 2 + 10],
            [320, 240],
            [0, 240]
        ])
        self.crop_pts = np.array([self.crop_pts], dtype=np.int32)

    def callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except Exception as e:
            rospy.logerr(f"Error decoding image: {e}")
            return

        self.mask = self.mask_roi(img_bgr)

        if len(self.mask.shape) == 3:
            img_concat = np.concatenate([img_bgr, self.mask], axis=1)
        else:
            img_concat = np.concatenate([img_bgr, cv2.cvtColor(self.mask, cv2.COLOR_GRAY2BGR)], axis=1)

        cv2.imshow("Image window", img_concat)
        cv2.waitKey(1)

    def mask_roi(self, img):
        h, w = img.shape[:2]

        if len(img.shape) == 3:
            mask = np.zeros((h, w, img.shape[2]), dtype=np.uint8)
            mask_value = (255, 255, 255)
        else:
            mask = np.zeros((h, w), dtype=np.uint8)
            mask_value = 255

        cv2.fillPoly(mask, self.crop_pts, mask_value)
        mask = cv2.bitwise_and(img, mask)

        return mask

def main():
    rospy.init_node('image_parser', anonymous=True)
    rospy.loginfo("Node initialized")

    image_parser = IMGParser()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
