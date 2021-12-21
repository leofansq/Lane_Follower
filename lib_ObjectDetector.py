import torch
import numpy as np
import cv2
from time import time


class ObjectDetector:
    """
    Class for the object detector based on Yolo5
    """
    def __init__(self):
        """
        Init
        """
        self.model = self.load_model()
        self.classes = self.model.names
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'

        # the class mask
        self.class_care = ['person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'truck', 'traffic light', 'fire hydrant','stop sign', 'parking meter', 'bench', 'cat', 'dog', 'chair']

    def load_model(self):
        """
        Load Yolo5 model from pytorch hub

        Return: 
            model: the trained pytorch model
        """
        model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
        print ("Load Success")
        return model

    def detect(self, frame, img_area):
        """
        Predict and analyze using yolo5

        Parameters:
            frame: input frame in numpy/list/tuple format
            img_area: image mask of the road area
        
        Return:
            labels: labels of the predictions
            cord: coordinates of the predictions
            colors: colors of the bounding-box for visualization, which distinguishes the dangerous level. i.e. red---dagerous   green---safe
        """
        self.model.to(self.device)
        frame = [frame]
        results = self.model(frame)
        d_labels, d_cord = results.xyxyn[0][:, -1].numpy(), results.xyxyn[0][:, :-1].numpy()
        labels = []
        cord = []
        colors = []
        for i in range(len(d_labels)):
            if self.class_to_label(d_labels[i]) in self.class_care and d_cord[i][4]>=0.4:
                labels.append(d_labels[i])
                cord.append(d_cord[i])

                # Decide the dangerous level of the detected object according to whether it falls into the area
                color = (0,0,255) if sum(img_area[int(d_cord[i][3]*img_area.shape[0])-1, int(d_cord[i][0]*img_area.shape[1])-1])+sum(img_area[int(d_cord[i][3]*img_area.shape[0])-1, int(d_cord[i][2]*img_area.shape[1])-1]) else (0,255,0)
                colors.append(color)

        return labels, cord, colors

    def class_to_label(self, idx):
        """
        Return the corresponding string label for a given label value

        Parameters:
            idx: numeric label
        
        Return:
            corresponding string label
        """
        return self.classes[int(idx)]

    def plot_detections(self, results, frame):
        """
        Takes a frame and its results as input, and plots the bounding boxes and label on to the frame

        Parameters:
            results: contains labels and coordinates predicted by model on the given frame
            frame: Frame which has been scored
        
        Return:
            Frame with bounding boxes and labels ploted on it.
        """
        labels, cord, colors = results
        x_shape, y_shape = frame.shape[1], frame.shape[0]
        for i in range(len(labels)):
            row = cord[i]
            x1, y1, x2, y2 = int(row[0]*x_shape), int(row[1]*y_shape), int(row[2]*x_shape), int(row[3]*y_shape)
            cv2.rectangle(frame, (x1, y1), (x2, y2), colors[i], 4)
            cv2.putText(frame, self.class_to_label(labels[i]), (x1, y1-2), cv2.FONT_HERSHEY_SIMPLEX, 1, colors[i], 2)

        return frame
            