#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, cv2, numpy as np
from sensor_msgs.msg import CompressedImage

def non_maximum_supression(bboxes, threshold=0.3):
    if len(bboxes) == 0:
        return []

    bboxes = sorted(bboxes, key=lambda b: b[2] * b[3], reverse=True)
    selected_bboxes = []

    while bboxes:
        chosen = bboxes.pop(0)
        selected_bboxes.append(chosen)
        bboxes = [box for box in bboxes if compute_iou(chosen, box) < threshold]

    return selected_bboxes

def compute_iou(box1, box2):
    x1 = max(box1[0], box2[0])
    y1 = max(box1[1], box2[1])
    x2 = min(box1[0]+box1[2], box2[0]+box2[2])
    y2 = min(box1[1]+box1[3], box2[1]+box2[3])

    inter_area = max(0, x2 - x1) * max(0, y2 - y1)
    box1_area = box1[2] * box1[3]
    box2_area = box2[2] * box2[3]
    union_area = box1_area + box2_area - inter_area

    return inter_area / union_area if union_area != 0 else 0

class PEDESDetector:
    def __init__(self):
        rospy.init_node('pedes_detector', anonymous=True)
        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback)
        self.pedes_detector = cv2.HOGDescriptor()
        self.pedes_detector.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
        rospy.on_shutdown(self.cleanup)

    def callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        img_gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)

        (rects_temp, _) = self.pedes_detector.detectMultiScale(
            img_gray,
            winStride=(4, 4),
            padding=(8, 8),
            scale=1.05
        )

        if len(rects_temp) != 0:
            rects = non_maximum_supression(rects_temp)
            for (x, y, w, h) in rects:
                cv2.rectangle(img_bgr, (x, y), (x + w, y + h), (0, 255, 0), 2)

        cv2.imshow("Pedestrian Detection", img_bgr)
        cv2.waitKey(1)

    def cleanup(self):
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        detector = PEDESDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
