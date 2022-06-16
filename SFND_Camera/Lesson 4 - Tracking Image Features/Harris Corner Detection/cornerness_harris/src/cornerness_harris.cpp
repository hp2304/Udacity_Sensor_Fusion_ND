#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <unordered_set>
#include <vector>


void nms(cv::Mat &cornerness_resp, int i, int j, int window_size){
    int selected_r, selected_c;
    float max_cornerness = -1;
    int inc_bound = window_size/2;
    for(int i_inc = -inc_bound; i_inc<=inc_bound; ++i_inc){
        for(int j_inc=-inc_bound; j_inc<=inc_bound; ++j_inc){
            int r = i + i_inc;
            int c = j + j_inc;
            if(r >= 0 && r < cornerness_resp.rows && c >= 0 && c < cornerness_resp.cols) {
                if (cornerness_resp.at<float>(r, c) > max_cornerness) {
                    selected_r = r;
                    selected_c = c;
                    max_cornerness = cornerness_resp.at<float>(r, c);
                }
            }
        }
    }
    for(int i_inc = -inc_bound; i_inc<=inc_bound; ++i_inc) {
        for (int j_inc = -inc_bound; j_inc <= inc_bound; ++j_inc) {
            int r = i + i_inc;
            int c = j + j_inc;
            if(r >= 0 && r < cornerness_resp.rows && c >= 0 && c < cornerness_resp.cols) {
                if (r == selected_r && c == selected_c)
                    continue;
                cornerness_resp.at<float>(r, c) = 0;
            }
        }
    }
}

void cornernessHarris()
{
    // load image from file
    cv::Mat img;
    img = cv::imread("../images/img1.png");
    cv::cvtColor(img, img, cv::COLOR_BGR2GRAY); // convert to grayscale

    // Detector parameters
    int blockSize = 2;     // for every pixel, a blockSize × blockSize neighborhood is considered
    int apertureSize = 3;  // aperture parameter for Sobel operator (must be odd)
    float minResponse = 100; // minimum value for a corner in the 8bit scaled response matrix
    double k = 0.04;       // Harris parameter (see equation for details)

    // Detect Harris corners and normalize output
    cv::Mat dst, dst_norm, dst_norm_scaled;
    dst = cv::Mat::zeros(img.size(), CV_32FC1);
    cv::cornerHarris(img, dst, blockSize, apertureSize, k, cv::BORDER_DEFAULT);
    cv::normalize(dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());
    cv::convertScaleAbs(dst_norm, dst_norm_scaled);

    // visualize results
    std::string windowName = "Harris Corner Detector Response Matrix";
//    cv::namedWindow(windowName, 4);
//    cv::imshow(windowName, dst_norm_scaled);
//    cv::waitKey(0);

    // TODO: Your task is to locate local maxima in the Harris response matrix 
    // and perform a non-maximum suppression (NMS) in a local neighborhood around 
    // each maximum. The resulting coordinates shall be stored in a list of keypoints 
    // of the type `vector<cv::KeyPoint>`.
    std::vector<cv::KeyPoint> keypoints;

    std::unordered_set<int> selected_locations;
    for(int i=0; i<dst_norm.rows; ++i){
        for(int j=0; j<dst_norm.cols; ++j){
            nms(dst_norm, i, j, 7);
        }
    }

    cv::Mat kp_image;
    for(int i=0; i<dst_norm.rows; ++i){
        for(int j=0; j<dst_norm.cols; ++j) {
            if (dst_norm.at<float>(i, j) > minResponse) {
                cv::KeyPoint kp(j, i, 3, -1, dst_norm.at<float>(i, j));
                keypoints.push_back(kp);
            }
        }
    }
    cv::drawKeypoints(dst_norm_scaled, keypoints, kp_image);

//    windowName = "KPs after NMS";
//    cv::namedWindow(windowName, 4);
//    cv::imshow(windowName, kp_image);
//    cv::waitKey(0);

    cv::imwrite("kp_harris_nms.jpg", kp_image);
}

int main()
{
    cornernessHarris();
}