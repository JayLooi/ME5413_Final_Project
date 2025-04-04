#!/usr/bin/python3
import rospy
import cv2
import os
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
import easyocr


class DigitRecognizer:
    def __init__(self):
        rospy.init_node("digit_recognizer")
        self.bridge = CvBridge()
        self.reader = easyocr.Reader(["en"], gpu=False)
        self.detected_numbers = {digit: 0 for digit in range(1, 10)}
        self.image_sub = rospy.Subscriber('/box_image', Image, self.image_callback)
        self.target_box_img_sub = rospy.Subscriber('/target_box', Image, self._target_detect_cb)
        self.result_pub = rospy.Publisher("/least_occurrence", Int32, queue_size=1)
        self.target_box_digit_pub = rospy.Publisher("/target_box_digit", Int32, queue_size=1)
        directory = os.path.dirname(os.path.realpath(__file__))
        self._temp_directory = os.path.abspath(directory + '/../temp')
        if not os.path.isdir(self._temp_directory):
            os.mkdir(self._temp_directory)
        rospy.loginfo("Digit recognizer node started.")

    def image_callback(self, msg):
        cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        result = self.reader.readtext(cv_img, allowlist="0123456789")
        display_img = cv_img.copy()

        max_bbox_size = 0
        max_bbox = None
        digit = None
        for detection in result:
            bbox, text, conf = detection
            if len(text) != 1 or conf < 0.4 or not text.isdigit():
                continue

            topleft = np.array(bbox[0])
            bottomright = np.array(bbox[2])
            bbox_size = np.multiply(*(bottomright - topleft))
            rospy.loginfo(f'digit={text}, bbox={bbox}')
            if bbox_size > max_bbox_size:
                digit = int(text)
                max_bbox = bbox
                max_bbox_size = bbox_size

        if digit is not None:
            self.detected_numbers[digit] += 1
            rospy.loginfo(f"Detected digit: {digit}")
            cv2.rectangle(display_img, tuple(map(int, max_bbox[0])), tuple(map(int, max_bbox[2])), (0, 255, 0), 2)
            cv2.putText(display_img, f"{digit})", (int(max_bbox[0][0]), int(max_bbox[0][1]) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.imwrite(f'{self._temp_directory}/{digit}_{self.detected_numbers[digit]}.png', display_img)

    def _target_detect_cb(self, msg):
        cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        result = self.reader.readtext(cv_img, allowlist="0123456789")
        display_img = cv_img.copy()

        max_bbox_size = 0
        digit = None
        for detection in result:
            bbox, text, conf = detection
            if len(text) != 1 or conf < 0.4 or not text.isdigit():
                continue

            topleft = np.array(bbox[0])
            bottomright = np.array(bbox[2])
            bbox_size = np.multiply(*(bottomright - topleft))
            if bbox_size > max_bbox_size:
                digit = int(text)
                max_bbox_size = bbox_size

        if digit is not None:
            target_digit = Int32()
            target_digit.data = digit
            self.target_box_digit_pub.publish(target_digit)
            rospy.loginfo(f"Detected target box digit: {digit}")

    def mainloop(self):
        loop_rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if sum(self.detected_numbers.values()) > 0:
                minimum = np.Inf
                least_occ_digit = None
                for k, v in self.detected_numbers.items():
                    if v < minimum:
                        least_occ_digit = k
                        minimum = v

                if least_occ_digit is not None:
                    result = Int32()
                    result.data = least_occ_digit
                    self.result_pub.publish(result)

            loop_rate.sleep()

if __name__ == "__main__":
    try:
        digit_detector = DigitRecognizer()
        digit_detector.mainloop()
    except rospy.ROSInterruptException:
        pass
