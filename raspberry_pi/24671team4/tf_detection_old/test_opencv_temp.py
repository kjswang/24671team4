"""Main script to run the object detection routine."""
import argparse
import sys
import time

import cv2
import numpy as np
import serial
import time
from matplotlib import pyplot as plt
from tflite_support.task import core
from tflite_support.task import processor
from tflite_support.task import vision
import utils_bbox


def opencv_bbox(model: str, camera_id: int, width: int, height: int, num_threads: int,
        enable_edgetpu: bool) -> None:

  counter, fps = 0, 0
  start_time = time.time()
  ser = serial.Serial('/dev/ttyACM0',115200,timeout =1)
  ser.flush()

  # Start capturing video input from the camera
  cap = cv2.VideoCapture(camera_id)
  cap.set(cv2.CAP_PROP_FPS,10)
  cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
  cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
  #end_time = time.time()
  #print(start_time-end_time)
  #tart_time = end_time
  # Visualization parameters
  row_size = 20  # pixels
  left_margin = 24  # pixels
  text_color = (0, 0, 255)  # red
  font_size = 1
  font_thickness = 1
  fps_avg_frame_count = 10
# 
#   # Initialize the object detection model
  # Continuously capture images from the camera and run inference
  while cap.isOpened():
    success, image = cap.read()
#     if not success:
#       sys.exit(
#           'ERROR: Unable to read from webcam. Please verify your webcam settings.'
#       )
    if (counter%1000 == 0):
      key = input("E")
    if key == "w":
      command = b"forward\n"
    elif key == "s":
      command = b"backward\n"
    elif key == "a":
      command = b"ccw\n"
    elif key == "d":
      command = b"cw\n"
    elif key == "e":
      command = b"stop\n"

    ser.write(command)
    line = ser.readline().decode('utf-8').rstrip()
    counter += 1
    print(counter)
    image = cv2.flip(image, 1)
#     blur = cv2.GaussianBlur(image,(7,7),0)
    img_gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    template = cv2.imread('Layer1_blackbg.png',0)
    h, w = template.shape
    
    methods = [cv2.TM_CCOEFF, cv2.TM_CCOEFF_NORMED, cv2.TM_CCORR,cv2.TM_CCORR_NORMED,cv2.TM_SQDIFF,cv2.TM_SQDIFF_NORMED]
    
    method = methods[2]
    d=cv2.matchShapes(img_gray,template,cv2.CONTOURS_MATCH_I2,0)
    result = cv2.matchTemplate(img_gray,template,method)
    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)
    location = max_loc
    print(location)
    bottom_right = (location[0]+w, location[1]+h)
    cv2.rectangle(img_gray,location,bottom_right,255,5)

    
    
#     stop_data = cv2.CascadeClassifier('stop_data.xml')
    
#     found = stop_data.detectMultiScale(img_gray, minSize = (20,20))
#     
#     amount_found = len(found)
#     
#     if amount_found !=0:
#         for (x,y,width,height) in found:
#             
#             cv2.rectangle(img_rgb,(x,y),(x+height,y+width),(0,255,0),5)
    if counter % fps_avg_frame_count == 0:
      end_time = time.time()
      fps = fps_avg_frame_count / (end_time - start_time)
      start_time = time.time()

    # Show the FPS
    fps_text = 'FPS = {:.1f}'.format(fps)
    text_location = (left_margin, row_size)
    cv2.putText(img_gray, fps_text, text_location, cv2.FONT_HERSHEY_PLAIN,
                font_size, text_color, font_thickness)
    cv2.imshow('haha',img_gray)

# 
#     # Convert the image from BGR to RGB as required by the TFLite model.
#     rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
#     gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
#     blur = cv2.GaussianBlur(gray,(7,7),0)
# #     blur = cv2.Laplacian(blur, cv2.CV_16S, (1,1))
#     #print(blur.shape)
#     thresh = cv2.adaptiveThreshold(blur, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY,7,2)
#     #edges = cv2.Canny(blur,100,100*2)
#     kernal = cv2.getStructuringElement(cv2.MORPH_CROSS,(3,3))
#     dilate = cv2.dilate(thresh, kernal, iterations=1)
#     cnts = cv2.findContours(dilate, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#     cnts = cnts[0] if len(cnts) == 2 else cents[1]
#     cnts = sorted(cnts, key=lambda x: cv2.boundingRect(x)[0])
#     print("-"*60)
#     print(cnts)
    
#     MIN_W, MAX_W = 80, 320
#     MIN_H, MAX_H = 60, 240
#     for c in cnts:
#         x,y,w,h = cv2.boundingRect(c)
#         if MIN_W < w < MAX_W and MIN_H < h < MAX_H:
#             cv2.rectangle(image,(x,y),(x+w,y+h),(36,255,12),2)
#     cv2.imshow('object_detector',image)
    if cv2.waitKey(1) == 27:
      break
    
    
def main():
  parser = argparse.ArgumentParser(
  formatter_class=argparse.ArgumentDefaultsHelpFormatter)
  parser.add_argument(
      '--model',
      help='Path of the object detection model.',
      required=False,
      default='efficientdet_lite0.tflite')
  parser.add_argument(
      '--cameraId', help='Id of camera.', required=False, type=int, default=0)
  parser.add_argument(
      '--frameWidth',
      help='Width of frame to capture from camera.',
      required=False,
      type=int,
      default=640)
  parser.add_argument(
      '--frameHeight',
      help='Height of frame to capture from camera.',
      required=False,
      type=int,
      default=480)
  parser.add_argument(
      '--numThreads',
      help='Number of CPU threads to run the model.',
      required=False,
      type=int,
      default=4)
  parser.add_argument(
      '--enableEdgeTPU',
      help='Whether to run the model on EdgeTPU.',
      action='store_true',
      required=False,
      default=False)
  args = parser.parse_args()
  opencv_bbox(args.model, int(args.cameraId), args.frameWidth, args.frameHeight,
      int(args.numThreads), bool(args.enableEdgeTPU))
    
if __name__ == '__main__':
    main()


