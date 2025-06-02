#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, cv2, numpy as np
from sensor_msgs.msg import CompressedImage

def non_maximum_suppression(bboxes, threshold=0.75):
    bboxes = sorted(bboxes, key=lambda det: det[2] * det[3], reverse=True)
    selected = []
    for bbox in bboxes:
        keep = True
        for sel in selected:
            x1 = max(bbox[0], sel[0])
            y1 = max(bbox[1], sel[1])
            x2 = min(bbox[0] + bbox[2], sel[0] + sel[2])
            y2 = min(bbox[1] + bbox[3], sel[1] + sel[3])
            
            inter_area = max(0, x2 - x1) * max(0, y2 - y1)
            area1 = bbox[2] * bbox[3]
            area2 = sel[2] * sel[3]
            union_area = area1 + area2 - inter_area
            
            iou = inter_area / float(union_area) if union_area != 0 else 0
            if iou > threshold:
                keep = False
                break
        if keep:
            selected.append(bbox)
    return selected

class PEDESDetector:
    def __init__(self):
        rospy.init_node('pedes_detector', anonymous=True)
        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback)
        self.pedes_detector = cv2.HOGDescriptor()
        self.pedes_detector.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

    def callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        img_gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)

        rects_temp, _ = self.pedes_detector.detectMultiScale(
            img_gray,
            winStride=(4, 4),
            padding=(8, 8),
            scale=1.05
        )

        if len(rects_temp) > 0:
            rects = non_maximum_suppression(rects_temp)
            for (x, y, w, h) in rects:
                cv2.rectangle(img_bgr, (x, y), (x + w, y + h), (0, 255, 0), 2)

        cv2.imshow("Pedestrian Detection", img_bgr)
        cv2.waitKey(1)

if __name__ == '__main__':
    try:
        detector = PEDESDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
