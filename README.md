# Autonomous-balloon-collecting-robot
This project was created as a part of "Design of Mechatronic Systems" course 

this project combines the various engineering disciplines to create a fully functional autonomous robot that can detect and classify different color balloons and deliver them autonomously to their respective target location.
A custom YOLO model was created for object detection and ROS2 was used for communication with low-level hardware, in our case, an ESP32.

## Running the code

Firstly you need to initialize the microROS agent in order to communicate with the ESP32, you can do so using the following command <br>
`ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/{PORT}`

in a different terminal use the following command to navigate to the project folder<br>
```
cd Design_YOLO
```

to run the model and initiate the publisher node:<br>
```
python3 main.py
```
you can echo the topic to view what is being sent to the ESP32 using the following command:<br>
```
ros2 topic echo /esp
```
