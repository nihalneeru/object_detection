#!/usr/bin/env python3
import rospy
import numpy as np
import cv2
import torch
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# Load the YOLOv5 model
model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)

# Initialize the ROS node
rospy.init_node('yolov5_detector', anonymous=True)

# Image converter
bridge = CvBridge()

def image_callback(msg):
    try:
        # Convert ROS Image message to OpenCV image
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)

    # Perform inference
    results = model(cv_image)

    # Print results
    print(results.pandas().xyxy[0])  # Print dataframe of results

    # Optionally: Display the image with detections
    cv2.imshow("YOLOv5 Detections", np.squeeze(results.render()))
    cv2.waitKey(1)

def main():
    # Subscribe to RealSense camera image topic
    rospy.Subscriber('/camera/color/image_raw', Image, image_callback)
    rospy.spin()

if __name__ == '__main__':
    main()


