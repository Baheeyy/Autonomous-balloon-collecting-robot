import cv2
import torch
from test_node import MotionTest


def check_bbox_size(width_ratio, height_ratio, threshold=0.5):
    if width_ratio * height_ratio >= threshold:
        return True


def detect_balls(motion_test, model, cap):
    names = ['black ball', 'blue ball', 'green ball', 'orange ball', 'red ball', 'steel ball', 'violet ball',
             'white ball', 'yellow ball']
    
    motion_test.stop()

    difference_tolerance = 0.1
    frames_to_stop = 100

    successive_to_catch = 4

    no_detection = 0
    successive = 0

    # Check if the camera opened successfully
    if not cap.isOpened():
        print("Error: Could not open camera.")
        return False

    while True:
        if motion_test.have_ball:
            return True

        # Capture frame-by-frame
        ret, frame = cap.read()

        # If the frame is read correctly, ret is True
        if ret:

            frame_height = frame.shape[0]
            frame_width = frame.shape[1]

            frame_center = frame_width / 2
            # Perform YOLOv5 inference
            results = model(frame[:, :, ::-1])

            # Draw bounding boxes on the frame
            output_frame = results.render()[0][:, :, ::-1]

            # Extracting class_id, x_center, y_center, width, height
            pred = results.xyxy[0].cpu().numpy()

            if len(pred) > 0:
                no_detection += 1
            else:
                successive += 1
                no_detection = 0

            if no_detection == frames_to_stop:
                motion_test.stop()

            # Iterate through predictions and print relevant information
            for detection in pred:
                class_id = int(detection[5])
                x_center = (detection[0] + detection[2]) / 2
                confidence = detection[4]
                width = detection[2] - detection[0]
                height = detection[3] - detection[1]

                if confidence < 0.5:
                    output_frame = frame
                    continue

                if names[class_id] == 'red ball':
                    if abs(x_center - frame_center) / frame_width <= difference_tolerance:
                        motion_test.move_forward()
                        print('move forward')
                    elif (x_center - frame_center) / frame_width > difference_tolerance:
                        print('rotate right')
                        motion_test.rotate_right()
                    elif (frame_center - x_center) / frame_width > difference_tolerance:
                        print('rotate left')
                        motion_test.rotate_left()
                    if check_bbox_size(width / frame_width,
                                       height / frame_height) and successive >= successive_to_catch:
                        motion_test.stop()
                        print("stop")
                        motion_test.closeservo()    
                        print("servo close")
                        motion_test.stepperup()
                        print("stepper up")
                        
                        
                        #motion_test.have_ball = True
                        return 'red ball'
                    break
                if names[class_id] == 'blue ball':
                    if abs(x_center - frame_center) / frame_width <= difference_tolerance:
                        motion_test.move_forward()
                    elif (x_center - frame_center) / frame_width > difference_tolerance:
                        motion_test.rotate_right()
                    elif (frame_center - x_center) / frame_width > difference_tolerance:
                        motion_test.rotate_left()
                    if check_bbox_size(width / frame_width,
                                       height / frame_height) and successive >= successive_to_catch:
                        motion_test.stop()
                        print("stop")
                        motion_test.stepperup()
                        print("stepper up")
                        motion_test.closeservo()    
                        print("servo close")                        
                        #motion_test.have_ball = True
                        return 'blue ball'
                    break

            # Display the resulting frame
            cv2.imshow('YOLOv5 Inference', output_frame)

            # Break the loop if 'q' key is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
