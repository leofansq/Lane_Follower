import cv2
import time

from basic_function import show_img
from lib_LaneDetector import detect_line
from lib_ObjectDetector import ObjectDetector

######################################################
###                 INITIALIZATION                 ###
######################################################

# Init the detector
Detector = ObjectDetector()

######################################################
###                    BEGINING                    ###
######################################################

cap = cv2.VideoCapture("./video.mp4")

# lane_fit_memory
memory = {}

time_tik = 0

while 1:
    # Video input
    _, frame = cap.read()

    steer = "Can not get steer!"

    # Detection Part
    # 1: Lane detection
    img_result, dist_from_center, curvature, memory, img_area =  detect_line(frame, steer, memory, debug=False)
    # 2: Traffic object detection
    detections = Detector.detect(frame, img_area)
    # 3: Merge the detection results
    img_result = Detector.plot_detections(detections, img_result)

    # Log Part
    # time_tok = time.time()
    # time_cost = time_tok - time_tik
    # time_tik = time_tok
    # print (dist_from_center, curvature, time_cost)
    show_img("result", img_result)
    # show_img("cap", frame)

    if cv2.waitKey(1) & 0xff == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
