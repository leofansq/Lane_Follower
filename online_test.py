#coding=utf-8
import cv2
import time

from mvsdk import CameraEnumerateDevice
from lib_camera import Camera
from lib_can import CAN
from lib_vehicle import Vehicle

from basic_function import show_img
from lib_LaneDetector import detect_line
from lib_ObjectDetector import ObjectDetector

######################################################
###                 INITIALIZATION                 ###
######################################################

# Init the CAN
can = CAN()

# Init the vehicle model
vehicle = Vehicle(wheel_base=2020, width=1655, length=2894, can=can)

# Init the detector
Detector = ObjectDetector()

# Init the industrial camera
DevList = CameraEnumerateDevice()
nDev = len(DevList)
for i, DevInfo in enumerate(DevList):
	print("{}: {} {}".format(i, DevInfo.GetFriendlyName(), DevInfo.GetPortType()))
cams = []
for i in map(lambda x: int(x), input("Select cameras: ").split()):
	cam = Camera(DevList[i])
	if cam.open():
		cams.append(cam)

######################################################
###                    BEGINING                    ###
######################################################

memory = {"left_fit":[0.0, 0.0, 0.0], "right_fit":[0.0, 0.0, 0.0], "left_x":0.0, "right_x":0.0}
time_tik = 0

while (cv2.waitKey(1) & 0xFF) != ord('q'):
	for cam in cams:
		frame = cam.grab()
		if frame is not None:

			# try:
			#     steer = vehicle.steer_get()
			# except:
			#     steer = "Can not get steer!"
			steer = "Can not get steer!"

			# Detection Part
			# 1: Lane detection
			img_result, dist_from_center, curvature, memory, img_area =  detect_line(frame, steer, memory, debug=False)
			# 2: Traffic object detection
			detections = Detector.detect(frame, img_area)
			# 3: Merge the detection results
			img_result = Detector.plot_detections(detections, img_result)
			
			# Control Part
			if dist_from_center==None: dist_from_center = 0.0
			vehicle.steer_cal(curvature*1000, dist_from_center*100)
			vehicle.steer_ctrl()

			# Log Part
			time_tok = time.time()
			time_cost = time_tok - time_tik
			time_tik = time_tok
			print (dist_from_center, curvature, time_cost)
			show_img("result", img_result)
			show_img("cap", frame)

for cam in cams:
	cam.close()

cv2.destroyAllWindows()
