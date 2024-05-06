#!/usr/bin/env python3
import rospy
import numpy as np
import cv2
import torch
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from yolov5_ros.utils import load_mask_rcnn_model

class MaskRCNNDetector:
    def __init__(self):
        rospy.init_node('mask_rcnn_detector', anonymous=True)

        self.model = load_mask_rcnn_model()
        self.bridge = CvBridge()
        self.camera_topic = rospy.get_param('~camera_topic', '/camera/color/image_raw')

        rospy.Subscriber(self.camera_topic, Image, self.image_callback)

        rospy.loginfo(f"Subscribed to {self.camera_topic}")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        results = self.model(cv_image)
        rospy.loginfo(results.pandas().xyxy[0])  # Log results

        cv2.imshow("Mask R-CNN Detections", np.squeeze(results.render()))
        cv2.waitKey(1)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    detector = MaskRCNNDetector()
    detector.run()
