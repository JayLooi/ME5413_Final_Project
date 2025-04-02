#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import easyocr
import tf

class DigitRecognizer:
    def __init__(self):
        rospy.init_node("digit_recognizer")
        self.bridge = CvBridge()
        self.reader = easyocr.Reader(["en"], gpu=False)
        self.detected_numbers = {}  
        self.image_sub = rospy.Subscriber("/front/image_raw", Image, self.image_callback)
        self.scan_sub = rospy.Subscriber("/front/scan", LaserScan, self.scan_callback)
        self.result_pub = rospy.Publisher("/digit_perception/numbers_pose", String, queue_size=1)
        self.scan_ranges = None
        self.scan_params = None
        self.tf_listener = tf.TransformListener()
        rospy.loginfo("Digit recognizer node started.")

    def scan_callback(self, msg):
        self.scan_ranges = msg.ranges
        self.scan_params = [msg.angle_min, msg.angle_increment]

    def image_callback(self, msg):
        if self.scan_ranges is None:
            return
        cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        result = self.reader.readtext(cv_img, allowlist="0123456789")
        display_img = cv_img.copy()

        for detection in result:
            bbox, text, conf = detection
            if len(text) != 1 or conf < 0.9:
                continue
            digit = text
            center = [(bbox[0][0] + bbox[2][0]) / 2, (bbox[0][1] + bbox[2][1]) / 2]
            angle = (center[0] - (cv_img.shape[1] / 2)) / (cv_img.shape[1] / 2) * (np.pi / 4)  # 简单估算
            index = int((angle - self.scan_params[0]) / self.scan_params[1])
            index = max(0, min(index, len(self.scan_ranges) - 1))
            distance = self.scan_ranges[index]
            x = distance * np.cos(angle)
            y = distance * np.sin(angle)

            
            if digit in self.detected_numbers:
                old_x, old_y = self.detected_numbers[digit]
                if np.linalg.norm([x - old_x, y - old_y]) < 0.5:
                    continue

            
            self.detected_numbers[digit] = (x, y)
            rospy.loginfo(f"Detected digit: {digit}, position: ({x:.2f}, {y:.2f})")
            self.result_pub.publish(f"{digit}: ({x:.2f}, {y:.2f})")

            
            cv2.rectangle(display_img, tuple(map(int, bbox[0])), tuple(map(int, bbox[2])), (0, 255, 0), 2)
            cv2.putText(display_img, f"{digit} ({distance:.2f}m)", (int(bbox[0][0]), int(bbox[0][1]) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        cv2.imshow("Digit Detection", display_img)
        cv2.waitKey(1)

if __name__ == "__main__":
    try:
        DigitRecognizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

