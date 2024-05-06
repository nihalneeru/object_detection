import torch

def load_yolov8_model():
    return torch.hub.load('ultralytics/yolov8', 'yolov8s', pretrained=True)

def load_efficientdet_model():
    return torch.hub.load('rwightman/efficientdet-pytorch', 'tf_efficientdet_d0', pretrained=True)

def load_ssd_mobilenet_model():
    return torch.hub.load('NVIDIA/DeepLearningExamples:torchhub', 'nvidia_ssd', pretrained=True)

def load_faster_rcnn_model():
    return torch.hub.load('pytorch/vision:v0.10.0', 'fasterrcnn_resnet50_fpn', pretrained=True)

def load_mask_rcnn_model():
    return torch.hub.load('pytorch/vision:v0.10.0', 'maskrcnn_resnet50_fpn', pretrained=True)
