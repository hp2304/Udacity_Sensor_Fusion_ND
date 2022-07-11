# Camera Module Exercises

## Requirements

My specifications are as follows, 

* Xubuntu 18.04
* OpenCV - v4.5.5
* C++ v11
* gcc v7.5.0

## Exercises

1. [Image Basics with OpenCV](./Lesson%202%20-%20Autonomous%20Vehicles%20and%20Computer%20Vision/The%20OpenCV%20Library/OpenCV_exercises/): Reading, displaying and writing images using OpenCV. Manipulate pixel values given its location.

2. [Lidar TTC Estimation](./Lesson%203%20-%20Engineering%20a%20Collision%20Detection%20System/Estimating%20TTC%20with%20Lidar/TTC_lidar/): Estimate Time to Collision given two consecutive Lidar measurements. 

2. [Camera TTC Estimation](./Lesson%203%20-%20Engineering%20a%20Collision%20Detection%20System/Estimating%20TTC%20with%20Camera/TTC_camera/): Estimate Time to Collision using pixel distances of two consecutive frames. 

3. [Intensity Gradients](./Lesson%204%20-%20Tracking%20Image%20Features/Intensity%20Gradient%20and%20Filtering/gradient_filtering/): Use sobel filter to calculate gradient magnitude.

4. [Harris Corner Detection](./Lesson%204%20-%20Tracking%20Image%20Features/Harris%20Corner%20Detection/cornerness_harris/): Apply Non-max Suppression on Harris Cornerness Response matrix calculated using OpenCV.


5. [FAST Keypoint Detection](./Lesson%204%20-%20Tracking%20Image%20Features/Overview%20of%20Popular%20Keypoint%20Detectors/detect_keypoints/): Use OpenCV to detect keypoints using FAST technique and display it on the image.

6. [Describing Keypoints](./Lesson%204%20-%20Tracking%20Image%20Features/Gradient-based%20vs.%20Binary%20Descriptors/describe_keypoints/): Use OpenCV to detect and describe the keypoints using SIFT technique.

7. [Descriptor Matching](./Lesson%204%20-%20Tracking%20Image%20Features/Descriptor%20Matching/descriptor_matching/): Implement FLANN based matching using OpenCV. Filter matches which are calculated using OpenCV's *knnMatch* function.

8. [YOLO Object Detection](./Lesson%206%20-%20Combining%20Camera%20and%20Lidar/Object%20Detection%20with%20YOLO/detect_objects/): Use OpenCV to detect object using YOLO (You Only Look Once) object detection technique.

9. [Project Lidar Points to Camera](./Lesson%206%20-%20Combining%20Camera%20and%20Lidar/Lidar-to-Camera%20Point%20Projection/lidar_to_camera/): Use Extrinsic and Intrinsic parameters to project Lidar points on the camera frame.

10. [Cluster Lidar Points](./Lesson%206%20-%20Combining%20Camera%20and%20Lidar/Creating%203D-Objects/cluster_with_roi/): Group Lidar points given the Object Detection output (bounding boxes) on camera image.

## Usage

Go to respective directory attached with above links,

```bash
mkdir build && cd build
cmake ..
make
```

Running above commands will generate the executable.
