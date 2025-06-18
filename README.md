# 2 DOF Gimbal with Object Detection and Tracking

<img src="https://github.com/user-attachments/assets/1dd2015a-c735-477a-a91f-9c08352504b8" width="300">

[DEMO VIDEO](https://youtu.be/xnSXLZFRSGM)

A 2 DOF gimbal that uses spline-based motor control and image filtering to scan, detect, and track objects. This project was developed as part of the Introduction to Robotics course at Caltech, focusing on mechanical design and fabrication, mathematical mechanism analysis, vision-based algorithms, and motion planning.

## Features
### Smooth motor control
We achieve smooth motor control by planning the position and velocity of each motor according to the following cubic spline formulas

p<sub>cmd<sub>(t) = a + b(t - t<sub>0<sub>) + c(t - t<sub>0<sub>)<sup>2<sup> + d(t - t<sub>0<sub>)<sup>3<sup>
v<sub>cmd<sub>(t) = b + 2c(t - t<sub>0<sub>) + 3d(t - t<sub>0<sub>)<sup>2<sup>

where, given t<sub>f<sub> is the time the entire trajectory takes and t<sub>0<sub> is the starting time,

T<sub>move<sub> = t<sub>f<sub> - t<sub>0<sub>
a = p<sub>0<sub>
b = v<sub>0<sub>
c = 3(p<sub>f<sub> - p<sub>0<sub>/T<sub>move<sub><sup>2<sup> - v<sub>f<sub>/T<sub>move<sub> - 2v<sub>0<sub>/T<move>
d = -2(p<sub>f<sub> - p<sub>0<sub>/T<sub>move<sub><sup>3<sup> - v<sub>f<sub>/T<sub>move<sub><sup>2<sup> - v<sub>0<sub>/T<move><sup>2<sup>

By calculating the position and velocity of the motors using these formulas every time step, the robot is able to move to specified positions without abruptly stopping or starting (i.e. going from a high velocity to 0 velocity in one time step). This is best seen in the graph below, where the position follows smooth curves and the velocity is the corresponding derivative.

<img src="https://github.com/user-attachments/assets/dd3806be-7a93-4d93-939a-bfe05c8d5995" width="300">

### Object detection and tracking
Our detector is designed to track small blue balls (as shown in the demo video). Every frame is processed by first filtering out pixels within the HSV range of the ball's color. Then these pixels are dilated and eroded to remove noise. For the balls, this leaves a circle of pixels, which we then bound with a circle contour. Every full contour detected is considered an object. Then the object's position is calculated and stored based on the position of the contour in the frame. We can then track the object by constantly updating the detected object locations and splining to those positions.

Here is an example done with an inflatable cube, although we ended up not using this object because the colors did not filter well.
<img src="https://github.com/user-attachments/assets/c8e2886c-b5b0-45d3-a311-c06da00e04c7" width="300">

### Multi-threading
Since the system must share motor position and object detection data between the controller and detector for object tracking, we utilize multi-threading to prevent data corruption. 

### User-interface
Our system operates according to the following key commands:

|Key |Behavior |
|:---|:---|
|s| Starts scanning, where the robot searches for and stores objects|
|z| Returns the robot to its home position|
|t| Tracks known objects|
|q| Breaks and quits the program|

We use these commands to update the *trajectory* and *mode* states, which are used to manage each behavior.

## Hardware
- Hebi motors (x2)
- Brackets (see CAD files) + M5 bolts
- Logitech webcam
- Raspberry pi

## Software
- Python
- Numpy
- Matplotlib
- OpenCV

## Learning Outcomes
### Skills
- Implemented finite state machines, smooth motion planning, multi-threading, object detection, and object tracking.
- Practiced hardware tuning and software debugging techniques. 
- Presented progress with weekly-demos.

### Challenges and Limitations
- We initially planned to use the cube shown above as our object of interest for detection, but found that it didn't filter well because it was difficult for the program to distinguish blue and purple pixels. We resolved this by switching to a small blue ball.
- Earlier in the process, we had issues with rapidly switching the goal position of the robot as it was moving, causing unnatural spikes in its movement. This was resolved by restructuring our code such that new splines were taken only when needed, rather than at every time step.
- If the object the robot is detecting moves too fast, the robot will lag or lose track of the object. This is because the motors are physically limited by a max velocity.

## Contributors
Chloe Wang (chloew@caltech.edu)
Luis Serrano Laguna (lserrano@caltech.edu)

## Acknowledgements
Thank you to our professor, Gunter Niemeyer, and our TA, Kevin Gauld, for guiding us through this project!
