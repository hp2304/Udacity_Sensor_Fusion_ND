# SFND Camera Midterm Feature Tracking Project

### Note:

- Added a basic config parser to avoid re-making (same as the one used in Lidar Project). 
- Zip contains an excel file named observations.xlsx. It has 3 sheets named task-1, 2, and 3 corresponding to TASK MP 7, 8, and 9 's observations/results respectively.
- I use OpenCV 4.5 which has SIFT in the cv namespace, not cv::xfeatures2d.
- DES_HOG type is only used with SIFT descriptor, for all the other DES_BINARY is used. All the used detectors/descriptors are listed in the excel file.
- FAST detector instance used here is of type of 5_8 (among the given three options), which is the fastest of all three options but very poor at detecting key points. 
- Most of the detectors and descriptors are initialized using opencv's default argument values.
- Following extraneous values are also printed,
    1. Number of keypoints after filtering (keeping only which falls on the defined rectangle).
    2. In KNN selector, the discard percentage.
    3. Number of matches at the end.
- AKAZE-AKAZE detector-descriptor pair is only considered as instructed in class.
- SIFT-ORB detector-descriptor pair is not considered because it causes the out-of-memory exception. And, a question regarding this has been asked in the forum, in which the mentor suggested ignoring this combination.
- In the second sheet of the excel for TASK MP 8, each cell is an array of size 9. Each number represents the number of matches for a consecutive image pair.
- In TASK MP 9, the keypoint detection doesn't depend on the descriptor algorithm. For each detector type displayed time in the sheet is the average time taken over all 10 images. Descriptor algorithms work on the detected key points. So to calculate the average time in a fair way, the ORB detector (except AKAZE obviously) is used in all of them.
- Time measurements are taken on intel-i7 7th gen CPU with 16 GB DDR4 RAM.

## My Top-3 Recommendation (detector/descriptor)

1. ORB-BRIEF
2. ORB-ORB
3. ORB-BRISK

- In the case of autonomous driving, a camera captures a lot of images per second, and many cameras are attached to the car. Tracking objects across cameras or consecutive images of one camera is an essential task to do object detection, motion and behavior planning, and SLAM. Hence, real-time performance and high accuracy are expected from tracking algorithms. 

- According to time stats, the ORB detector is selected for its effectiveness to detect keypoints at multiple scales, unlike Harris and Shi-Tomasi. Both of which are faster than ORB. FAST on the other hand is also quick but poor in terms of accuracy as seen in the first sheet. It has detected a relatively very less number of keypoints. A more accurate option for FAST can be selected to replace ORB, which is as fast as ORB.

- Descriptor choices are made by considering the top 3 fastest descriptors from sheet 3 observations. ORB, BRIEF, and BRISK are several times quicker to compute compared to others.

- SIFT might give excellent results in both detection and description tasks due to its superior design but it's very slow compared to its counterparts, which makes it an obvious NO since the real-time performance is the requirement. Also, AKAZE is a difficult choice due to its strict usage limitations.


