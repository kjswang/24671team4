# Copyright 2021 The TensorFlow Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Main script to run the object detection routine."""
import argparse
import sys
import time

import cv2
from tflite_support.task import core
from tflite_support.task import processor
from tflite_support.task import vision
import utils

detected = 0

def opencv_bbox(model: str, camera_id: int, width: int, height: int, num_threads: int,
        enable_edgetpu: bool) -> None:

  counter, fps = 0, 0
  start_time = time.time()

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
    counter += 1
    image = cv2.flip(image, 1)
#     blur = cv2.GaussianBlur(image,(7,7),0)
    img_gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    template = cv2.imread('black_cup.png',0)
    h, w = template.shape
    
    methods = [cv2.TM_CCOEFF, cv2.TM_CCOEFF_NORMED, cv2.TM_CCORR,cv2.TM_CCORR_NORMED,cv2.TM_SQDIFF,cv2.TM_SQDIFF_NORMED]
    
    method = methods[0]
    result = cv2.matchTemplate(img_gray,template,method)
    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)
    location = max_loc
    
    bottom_right = (location[0]+w, location[1]+h)
    cv2.rectangle(img_gray,location,bottom_right,255,5)
    print(bottom_right)
    
    
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
    #cv2.destroyAllWindows()

def run(model: str, camera_id: int, width: int, height: int, num_threads: int,
        enable_edgetpu: bool) -> None:
  """Continuously run inference on images acquired from the camera.

  Args:
    model: Name of the TFLite object detection model.
    camera_id: The camera id to be passed to OpenCV.
    width: The width of the frame captured from the camera.
    height: The height of the frame captured from the camera.
    num_threads: The number of CPU threads to run the model.
    enable_edgetpu: True/False whether the model is a EdgeTPU model.
  """

  # Variables to calculate FPS
  counter, fps = 0, 0
  start_time = time.time()

  # Start capturing video input from the camera
  cap = cv2.VideoCapture(camera_id)
  cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
  cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

  # Visualization parameters
  row_size = 20  # pixels
  left_margin = 24  # pixels
  text_color = (0, 0, 255)  # red
  font_size = 1
  font_thickness = 1
  fps_avg_frame_count = 10
  global detected
    
  # Initialize the object detection model
  base_options = core.BaseOptions(
      file_name=model, use_coral=enable_edgetpu, num_threads=num_threads)
  detection_options = processor.DetectionOptions(
      max_results=3, score_threshold=0.2)
  options = vision.ObjectDetectorOptions(
      base_options=base_options, detection_options=detection_options)
  detector = vision.ObjectDetector.create_from_options(options)

  # Continuously capture images from the camera and run inference
  while cap.isOpened():
    success, image = cap.read()
    if not success:
      sys.exit(
          'ERROR: Unable to read from webcam. Please verify your webcam settings.'
      )

    counter += 1
    image = cv2.flip(image, 1)
    
    image = cv2.GaussianBlur(image, (11, 11), 0)

    # Convert the image from BGR to RGB as required by the TFLite model.
    rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    # Create a TensorImage object from the RGB image.
    input_tensor = vision.TensorImage.create_from_array(rgb_image)

    # Run object detection estimation using the model.
    detection_result = detector.detect(input_tensor)
    objects_to_detect = set(['cup'])
    for detection in detection_result.detections:
    # Draw bounding_box
        category = detection.categories[0]
        category_name = category.category_name
        if category_name == 'cup':
            print(detection_result)
            detected =1
    
    # Draw keypoints and edges on input image
    #image = utils.visualize(image, detection_result)

    # Calculate the FPS
    if counter % fps_avg_frame_count == 0:
      end_time = time.time()
      fps = fps_avg_frame_count / (end_time - start_time)
      start_time = time.time()

    # Show the FPS
    fps_text = 'FPS = {:.1f}'.format(fps)
    text_location = (left_margin, row_size)
    cv2.putText(image, fps_text, text_location, cv2.FONT_HERSHEY_PLAIN,
                font_size, text_color, font_thickness)

    # Stop the program if the ESC key is pressed.
    if cv2.waitKey(1) == 27 or detected ==1:
      break
    cv2.imshow('object_detector', image)

  cap.release()
  cv2.destroyAllWindows()


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
  global detected

  run(args.model, int(args.cameraId), args.frameWidth, args.frameHeight,
          int(args.numThreads), bool(args.enableEdgeTPU))
  if detected == 1 :
      opencv_bbox(args.model, int(args.cameraId), args.frameWidth, args.frameHeight,
          int(args.numThreads), bool(args.enableEdgeTPU))


#if __name__ == '__main__':
 # main()

