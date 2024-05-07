# ROS-YOLOv5 Object Detection

This repository contains the implementation of a ROS wrapper for the YOLOv5 object detection model, integrated with an Intel RealSense D435 camera for real-time object detection in robotic applications. This project aims to provide a robust system for detecting and identifying objects with high accuracy and speed, suitable for a variety of robotic tasks.

## Features

- **Real-Time Object Detection**: Integration of YOLOv5 model for high-speed and accurate object detection.
- **ROS Integration**: A ROS wrapper that facilitates communication and data exchange within robotic systems.
- **Graphical User Interface (GUI)**: Real-time display of detected objects with bounding boxes, labels, and confidence scores.
- **Publisher Node**: A ROS publisher node that broadcasts detailed object information to other ROS nodes.
- **Multiple Model Support**: Tested with several leading object detection models including YOLOv8, EfficientDet, SSD-Mobilenet, Faster R-CNN, and Mask R-CNN.
- **Fine-Tuned Model**: Upload the .ipynb file to Google Colab and run it there to test the transfer learning model.


## Installation

### Prerequisites

- ROS Noetic (other versions may work but are not tested)
- Python 3.8+
- OpenCV 4.2+
- PyTorch 1.7+

### Setup

1. **Clone the repository**:
   ```bash
   git clone https://github.com/yourgithub/ROS-YOLOv5.git
2. **Install dependencies**:
   ```bash
   cd ROS-YOLOv5
   pip install -r requirements.txt

### Usage
1. **Connect the Intel RealSense camera to your computer**
2. **Launch the ROS core**:
   ```bash
   roscore
3. **Run the object detection node**:
   ```bash
   rosrun src/yolov5_ros/scripts/detect.py

## Configuration

Edit the `config.yml` file in the `config` directory to adjust detection settings like model path, confidence thresholds, etc.

## Contributing

Contributions to this project are welcome! Please submit pull requests or create issues for bugs, suggestions, or enhancements.

## Acknowledgements

- Dr. Ben Abbatematteo for providing us with a workspace in the lab along with the ROS and RealSense camera.
- YOLOv5 / YOLOv8 by Ultralytics for the object detection model.
- EfficientDet / SSD-Mobilenet by Google for the object detection model.
- Faster R-CNN / Mask R-CNN by Ross Girshick et al. for the object detection model.
- ROS community for the extensive tools and libraries.
- Intel for the RealSense D435 camera.

