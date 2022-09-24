# undergrad_thesis

With the gradual maturity of computer vision, cloud computing, modern sensing and other technologies, auto driving has achieved unimaginable fruits in the past ten years and is expected to be commercialized in the next ten years. Auto driving is an interdisciplinary field that integrates multiple disciplines, covering many disciplines and technologies such as computer vision, robotics systems, automation control, cloud computing, and vehicle engineering.

In order to explore the technology of auto driving, this thesis is based on the educational car platform of the Shenzhen Institute of Intelligent Robotics. It discusses the design and implementation of the unmanned car, and simulates multiple main tasks and technologies of auto driving: lane detection and path tracking, recognition of traffic lights and other traffic signals, driving in areas without lane lines (baffle area), and controlling the unmanned car based on multiple sensors.

In this thesis, lane detection and path tracking are realized by Pure Pursuit algorithm: the idea of this algorithm is direct, and it is suitable for low-speed and relatively simple laboratory scenes. For the realization of real-time object detection of traffic lights and traffic signals, the single-stage object detection network YOLOv5 is adopted, and its accuracy can reach 99% mAP_0.5, speed up to 40FPS (GeForce GTX 1080Ti). This thesis uses the custom data to train the model off-line on the server, and uses two deployment methods to implement and test: 1) only use RK3399 platform to deploy the whole model; 2) using additional Huawei HiLens platform to deploy object detection algorithm, and communicate with RK3399 platform to complete the whole set of functions. The driving in the lane-free area is realized by the laser on the car body, and the lane line is composed of laser point cloud, and then the Pure Pursuit algorithm is used to make the car pass through the middle of the baffle area.

Keywords: Unmanned Ground Vehicle;Pure Pursuit Path Tracking;YOLOv5; Real-time Object Detection

### Video Demo Link
https://drive.google.com/file/d/1Pe8Dsc6lbKdyUyWnIogsUrzYk6PR2Itq/view?usp=sharing

### Slide
https://docs.google.com/presentation/d/1zivGw8TQb6oXZoA0O37_YKGpdFnCv_UKt1uKwIlSmiQ/edit?usp=sharing

### Thesis Paper Link
https://drive.google.com/file/d/1qjYUsZSu3KZZfyvCQiMj4WBT7IPbDEN2/view?usp=sharing
