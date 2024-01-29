from detect_ball import detect_balls
from detect_box import detect_boxes
from test_node import MotionTest
import cv2
import torch
import rclpy

cam_id = 2
cap = cv2.VideoCapture(cam_id)
rclpy.init()
motion_node = MotionTest()
cv2.namedWindow('YOLOv5 Inference', flags=cv2.WINDOW_NORMAL)


ball_model = torch.hub.load('ultralytics/yolov5', 'custom', path='last.pt', force_reload=True, trust_repo=True)
box_model = model = torch.hub.load('ultralytics/yolov5', 'custom', path='/home/a/Deisgn_YOLO/best (1).pt', force_reload=True, trust_repo=True)


while True:
    
    ball = detect_balls(motion_node, ball_model, cap)
    print("ball detected")

    if ball is False or ball is None:
        print('Could not catch ball')
        # motion_node.stop()
        # motion_node.closeservo()
        # motion_node.stepperup()
        # motion_node.rotate_left()
        break
    
    success = detect_boxes(motion_node, box_model, cap, ball)
    if not success:
        print('Could not put the ball in the box')
        break
    
    
    #motion_node.stepperup()
    #motion_node.rotate_left()
    print("flag")
    
    motion_node.stepperdown()