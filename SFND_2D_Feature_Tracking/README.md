# Feature Tracking Project

![](./media/keypoints.png)

## Introduction

Goal of this project is to benchmark the performance of different keypoint detector/descriptor combinations on the road environement data.

**Note**, I have added my own basic config parser to avoid re-making when configs change (same as the one used in Lidar Project). This [config file](./src/configs.txt) contains the hyperparameter settings.

## Requirements

My specifications are as follows, 

* Xubuntu 18.04
* CMake v2.23.2
* OpenCV - v4.5.5
* C++ v11
* gcc v7.5.0

## Benchmark Criteria

- **List of keypoint detectors are shown below**
    - Harris
    - Shi-Tomasi
    - FAST
    - BRISK
    - ORB
    - AKAZE
    - SIFT

- Types of Keypoint Descriptor:
    - DES_HOG (Histogram of Oriented Gradients)
    - DES_BIN (Binary descriptors)



- **List of Keypoint Descriptors:**
    - BRIEF
    - ORB
    - FREAK
    - AKAZE
    - SIFT
    - BRISK
    
    Only SIFT is HOG based descriptor on the above list.


- Types of Matcher:
    - MAT_BF (Brute Force)
    - MAT_FLANN

- Types of Selector:
    - SEL_NN (Nearest Neighbor)
    - SEL_KNN (*Distance ratio of 2 Nearest Neighbor* filter) 

### Tasks
1. For all 10 images, try all above shown detectors and count the number of detected keypoints.
2. For all **valid** combinations of detetor/descriptor, count total number of matched keypoints for all consecutive image frames.
3. Measure the average runtime of every detector and descriptors individually.

**Note**, time measurements are taken on intel-i7 7th gen CPU with 16 GB DDR4 RAM.

- [This excel file](./observations.xlsx) file contains 3 sheets named task-1, 2, and 3 corresponding to above tasks. 

- In the second sheet of the excel file each cell is an array of size 9. Each of which represents the number of matches for a consecutive image pair.

- In the third task, the keypoint detection doesn't depend on the descriptor algorithm. For each detector type, displayed time in the sheet is the average time taken over all 10 images. Descriptor algorithms work on the detected key points. So to calculate the average time in a fair way, the ORB detector is used in all of them.

## Build Instructions 

```bash
mkdir build && cd build
cmake ..
make
./2D_feature_tracking
```

## Implementation Details

- I have used OpenCV 4.5, which has SIFT in the cv namespace, not cv::xfeatures2d.
- FAST detector instance used here is of type of 5_8 (among three available options), which is the fastest of all three options but very poor at detecting key points. 
- Most of the detectors and descriptors are initialized using OpenCV's default argument values.
- Following extraneous values are also printed,
    1. Number of keypoints after filtering (keeping only which falls on the defined rectangle).
    2. In KNN selector, the discard percentage.
    3. Number of matches at the end.


## Top-3 Recommendations (detector/descriptor)

1. ORB-BRIEF
2. ORB-ORB
3. ORB-BRISK

- In the case of autonomous driving, a camera captures a lot of images per second, and many cameras are attached to the car. Tracking objects across cameras or consecutive images of one camera is an essential task to do object tracking, motion & behavior planning, and SLAM. Hence, real-time performance and high accuracy is expected from tracking algorithms. 

- According to collected time observations, the ORB detector is selected for its effectiveness to detect keypoints at multiple scales, unlike Harris and Shi-Tomasi. FAST on the other hand is quick, but poor in terms of accuracy as seen in the statistics. It has detected a relatively very less number of keypoints.

- Descriptor choices are made by considering the top 3 fastest descriptors from task 3's observations. ORB, BRIEF, and BRISK are several times quicker to compute compared to others.

- SIFT might give excellent results in both detection and description tasks due to its superior design, but it's slow compared to its counterparts. Also, AKAZE is a difficult choice due to its strict usage requirements.


