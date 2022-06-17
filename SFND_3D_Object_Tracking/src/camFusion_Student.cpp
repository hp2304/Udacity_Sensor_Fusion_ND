
#include <iostream>
#include <algorithm>
#include <numeric>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <unordered_map>

#include "camFusion.hpp"
#include "dataStructures.h"

using namespace std;

void displayImg(cv::Mat const &img, string title = "untitled"){
    cv::namedWindow(title, 7);
    cv::imshow(title, img);
    cv::waitKey(0);
}

// Create groups of Lidar points whose projection into the camera falls into the same bounding box
void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, std::vector<LidarPoint> &lidarPoints, float shrinkFactor, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT)
{
    // loop over all Lidar points and associate them to a 2D bounding box
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1)
    {
        // assemble vector for matrix-vector-multiplication
        X.at<double>(0, 0) = it1->x;
        X.at<double>(1, 0) = it1->y;
        X.at<double>(2, 0) = it1->z;
        X.at<double>(3, 0) = 1;

        // project Lidar point into camera
        Y = P_rect_xx * R_rect_xx * RT * X;
        cv::Point pt;
        // pixel coordinates
        pt.x = Y.at<double>(0, 0) / Y.at<double>(2, 0); 
        pt.y = Y.at<double>(1, 0) / Y.at<double>(2, 0); 

        int flag = 0;
        int idx = -1;
        for (auto it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2)
        {
            // shrink current bounding box slightly to avoid having too many outlier points around the edges
            cv::Rect smallerBox;
            smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
            smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
            smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
            smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);

            // check wether point is within current bounding box
            if (smallerBox.contains(pt)){
                if(!flag){
                    idx = it2 - boundingBoxes.begin();
                    flag = 1;
                }
                else{
                    flag = 2;
                    break;
                }
            }

        } // eof loop over all bounding boxes

        // check wether point has been enclosed by one or by multiple boxes
        if(flag == 1)
            boundingBoxes[idx].lidarPoints.push_back(*it1);
    } // eof loop over all Lidar points
}

/* 
* The show3DObjects() function below can handle different output image sizes, but the text output has been manually tuned to fit the 2000x2000 size. 
* However, you can make this function work for other sizes too.
* For instance, to use a 1000x1000 size, adjusting the text positions by dividing them by 2.
*/
void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, cv::Mat &visImg)
{
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

    for(auto it1=boundingBoxes.begin(); it1!=boundingBoxes.end(); ++it1)
    {
        // create randomized color for current 3D object
        cv::RNG rng(it1->boxID);
        cv::Scalar currColor = cv::Scalar(rng.uniform(0,150), rng.uniform(0, 150), rng.uniform(0, 150));

        // plot Lidar points into top view image
        int top=1e8, left=1e8, bottom=0.0, right=0.0; 
        float xwmin=1e8, ywmin=1e8, ywmax=-1e8;
        for (auto it2 = it1->lidarPoints.begin(); it2 != it1->lidarPoints.end(); ++it2)
        {
            // world coordinates
            float xw = (*it2).x; // world position in m with x facing forward from sensor
            float yw = (*it2).y; // world position in m with y facing left from sensor
            xwmin = xwmin<xw ? xwmin : xw;
            ywmin = ywmin<yw ? ywmin : yw;
            ywmax = ywmax>yw ? ywmax : yw;

            // top-view coordinates
            int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
            int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

            // find enclosing rectangle
            top = top<y ? top : y;
            left = left<x ? left : x;
            bottom = bottom>y ? bottom : y;
            right = right>x ? right : x;

            // draw individual point
            cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);
        }

        // draw enclosing rectangle
        cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom),cv::Scalar(0,0,0), 2);

        // augment object with some key data
        char str1[200], str2[200];
        sprintf(str1, "id=%d, #pts=%d", it1->boxID, (int)it1->lidarPoints.size());
        putText(topviewImg, str1, cv::Point2f(left-250, bottom+50), cv::FONT_ITALIC, 2, currColor);
        sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax-ywmin);
        putText(topviewImg, str2, cv::Point2f(left-250, bottom+125), cv::FONT_ITALIC, 2, currColor);  
    }

    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    visImg = topviewImg;
}

double calculateStdDev(vector<double> const &dists, double const &mean){
    double sigma = 0;
    for(auto const &dist: dists)
        sigma += pow(dist - mean, 2);

    return sqrt(sigma / (dists.size() - 1));
}

vector<cv::DMatch> filterSigma(vector<cv::DMatch> const &matches, vector<cv::KeyPoint> const &kptsPrev, vector<cv::KeyPoint> const &kptsCurr, int nb_sigma = 3){
    vector<double> dists;
    vector<cv::DMatch> filtered_matches;

    double mean_dist = 0;
    for(auto const &match: matches){
        auto const cur_kp = kptsCurr[match.queryIdx];
        auto const prev_kp = kptsPrev[match.trainIdx];

        const double dist = cv::norm(cur_kp.pt - prev_kp.pt);
        mean_dist += dist;
        dists.push_back(dist);
    }
    mean_dist /= dists.size();

    double sigma = calculateStdDev(dists, mean_dist);
    double min_val = mean_dist - (nb_sigma * sigma), max_val = mean_dist + (nb_sigma * sigma);

    for(int i=0; i<dists.size(); ++i){
        if(dists[i] >= min_val && dists[i] <= max_val)
            filtered_matches.push_back(matches[i]);
    }
    return filtered_matches;
}

vector<cv::DMatch> filterInterQuantile(vector<cv::DMatch> const &matches, vector<cv::KeyPoint> const &kptsPrev, vector<cv::KeyPoint> const &kptsCurr){
    vector<double> dists;
    vector<cv::DMatch> filtered_matches;

    for(auto const &match: matches){
        auto const cur_kp = kptsCurr[match.queryIdx];
        auto const prev_kp = kptsPrev[match.trainIdx];

        const double dist = cv::norm(cur_kp.pt - prev_kp.pt);
        dists.push_back(dist);
    }
    nth_element(dists.begin(), dists.begin() + (dists.size() / 4), dists.end());
    double min_val = dists[dists.size() / 4];
    nth_element(dists.begin(), dists.begin() + ((3 * dists.size()) / 4), dists.end());
    double max_val = dists[(3 * dists.size()) / 4];

    for(auto const &match: matches){
        auto const cur_kp = kptsCurr[match.queryIdx];
        auto const prev_kp = kptsPrev[match.trainIdx];

        const double dist = cv::norm(cur_kp.pt - prev_kp.pt);
        if(dist >= min_val && dist <= max_val)
            filtered_matches.push_back(match);
    }
    return filtered_matches;
}

// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{
    vector<cv::DMatch> potentialMatches;

    for(auto const &match: kptMatches){
        auto const cur_kp = kptsCurr[match.queryIdx];
        if(boundingBox.roi.contains(cur_kp.pt))
            potentialMatches.push_back(match);
    }

//    boundingBox.kptMatches.clear();
//    boundingBox.keypoints.clear();
//
//    // Keep only one
//
//    // potentialMatches = filterSigma(potentialMatches, kptsPrev, kptsCurr, 1);
//    potentialMatches = filterInterQuantile(potentialMatches, kptsPrev, kptsCurr);
//
//    boundingBox.kptMatches = potentialMatches;
//    for(auto const &match: boundingBox.kptMatches){
//        boundingBox.keypoints.push_back(kptsCurr[match.queryIdx]);
//    }
}


// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
    vector<double> distRatios; // stores the distance ratios for all keypoints between curr. and prev. frame
    TTC = NAN;
    if(kptMatches.size() < 2){
        return;
    }
    for (auto it1 = kptMatches.begin(); it1 != kptMatches.end() - 1; ++it1)
    { // outer keypoint loop

        // get current keypoint and its matched partner in the prev. frame
        cv::KeyPoint kpOuterCurr = kptsCurr.at(it1->trainIdx);
        cv::KeyPoint kpOuterPrev = kptsPrev.at(it1->queryIdx);

        for (auto it2 = kptMatches.begin() + 1; it2 != kptMatches.end(); ++it2)
        { // inner keypoint loop

            double minDist = 100.0; // min. required distance

            // get next keypoint and its matched partner in the prev. frame
            cv::KeyPoint kpInnerCurr = kptsCurr.at(it2->trainIdx);
            cv::KeyPoint kpInnerPrev = kptsPrev.at(it2->queryIdx);

            // compute distances and distance ratios
            double distCurr = cv::norm(kpOuterCurr.pt - kpInnerCurr.pt);
            double distPrev = cv::norm(kpOuterPrev.pt - kpInnerPrev.pt);

            if (distPrev > std::numeric_limits<double>::epsilon() && distCurr >= minDist)
            { // avoid division by zero

                double distRatio = distCurr / distPrev;
                distRatios.push_back(distRatio);
            }
        } // eof inner loop over all matched kpts
    }     // eof outer loop over all matched kpts

    // only continue if list of distance ratios is not empty
    if (distRatios.size() == 0){
        return;
    }

    double dT = 1 / frameRate;
    double medianDistRatio;
    int nb_ratios = distRatios.size();
    int med_idx = nb_ratios / 2;

    std::nth_element(distRatios.begin(), distRatios.begin() + med_idx, distRatios.end());
    if((nb_ratios % 2) == 0){
        std::nth_element(distRatios.begin(), distRatios.begin() + med_idx - 1, distRatios.end());
        medianDistRatio = (distRatios[med_idx] + distRatios[med_idx-1]) / 2;
    }
    else{
        medianDistRatio = distRatios[med_idx];
    }

    TTC = -dT / (1 - medianDistRatio);
}


double findRobustClosestPoint(std::vector<LidarPoint> &lidarPoints, float percentile){
    assert(percentile >= 0 && percentile < 100);
    std::vector<double> x_vals;
    x_vals.reserve(lidarPoints.size());
    for (const auto &lidarPoint : lidarPoints) {
        x_vals.push_back(lidarPoint.x);
    }

    int pos = (int) (lidarPoints.size() * (percentile / 100));
    std::nth_element(x_vals.begin(), x_vals.begin() + pos, x_vals.end());

    return x_vals[pos];
}

void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC){
    double dT = 1/frameRate;        // time between two measurements in seconds

    double minXCurr = findRobustClosestPoint(lidarPointsCurr, 3);
    double minXPrev = findRobustClosestPoint(lidarPointsPrev, 3);

//    cout << minXPrev << ", " << minXCurr << endl;

    // compute TTC from both measurements
    TTC = minXCurr * dT / (minXPrev - minXCurr);
}


void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame){
    /*
     * Algorithm:
     *
     * For each match between the keypoints of current frame and previous frame:
     *      Find corresponding bbox for curr_kp
     *      Find corresponding bbox for prev_kp
     *
     *      if both boxes are found:
     *          increase the score for prev_kp_bboxId -> cur_kp_bboxId
     *
     * Go through mapping scores meaning for each prev_kp_bboxId:
     *      Find out highest scoring cur_kp_bboxId
     *      bbBestMatches[prev_kp_bboxId] = cur_kp_bboxId
     */

    // init variable to track matching scores
    std::unordered_map<int, std::unordered_map<int, std::vector<cv::DMatch>>> track_matches;
    std::unordered_map<int, std::unordered_map<int, int>> track_scores;


    // Loop through matches
    for(auto cur_match_it = matches.begin(); cur_match_it < matches.end(); ++cur_match_it){
        cv::KeyPoint curr_kp = currFrame.keypoints[cur_match_it->queryIdx];

        // find corresponding bbox for curr_kp here
        auto curr_kp_bbox_it = currFrame.boundingBoxes.end();
        for(auto cur_bbox_it = currFrame.boundingBoxes.begin(); cur_bbox_it < currFrame.boundingBoxes.end(); ++cur_bbox_it){
            if(cur_bbox_it->roi.contains(curr_kp.pt)){
                curr_kp_bbox_it = cur_bbox_it;
                break;
            }
        }

        cv::KeyPoint prev_kp = prevFrame.keypoints[cur_match_it->trainIdx];

        // find corresponding bbox for prev_kp here
        auto prev_kp_bbox_it = prevFrame.boundingBoxes.end();
        for(auto prev_bbox_it = prevFrame.boundingBoxes.begin(); prev_bbox_it < prevFrame.boundingBoxes.end(); ++prev_bbox_it){
            if(prev_bbox_it->roi.contains(prev_kp.pt)){
                prev_kp_bbox_it = prev_bbox_it;
                break;
            }
        }

        if(prev_kp_bbox_it != prevFrame.boundingBoxes.end() && curr_kp_bbox_it != currFrame.boundingBoxes.end()){
            track_matches[prev_kp_bbox_it - prevFrame.boundingBoxes.begin()][curr_kp_bbox_it - currFrame.boundingBoxes.begin()].push_back(*cur_match_it);
            ++track_scores[prev_kp_bbox_it - prevFrame.boundingBoxes.begin()][curr_kp_bbox_it - currFrame.boundingBoxes.begin()];
        }
    }

    // Loop through mappings of prev_kp_bbox -> map[curr_kp_bbox, score]
    for(auto const &tracks: track_matches){
        // find out highest scoring curr_kp_bbox
        int max_score = -1;
        int best_cur_bboxIdx = -1;
        for(auto const &track: track_scores[tracks.first]){
            if(track.second > max_score){
                max_score = track.second;
                best_cur_bboxIdx = track.first;
            }
        }
        if(max_score > 10) {
            bbBestMatches[prevFrame.boundingBoxes[tracks.first].boxID] = currFrame.boundingBoxes[best_cur_bboxIdx].boxID;
            currFrame.boundingBoxes[best_cur_bboxIdx].kptMatches = track_matches[tracks.first][best_cur_bboxIdx];
            for(auto const &match: currFrame.boundingBoxes[best_cur_bboxIdx].kptMatches){
                currFrame.boundingBoxes[best_cur_bboxIdx].keypoints.push_back(currFrame.keypoints[match.queryIdx]);
            }

            if(false) {
//                cout << max_score << endl;

                cv::Rect prevBB = prevFrame.boundingBoxes[tracks.first].roi, curBB = currFrame.boundingBoxes[best_cur_bboxIdx].roi;
                float iou = (prevBB & curBB).area() / (float) (prevBB | curBB).area();
                cv::Mat cur_copy = currFrame.cameraImg.clone(), prev_copy = prevFrame.cameraImg.clone();
                cv::rectangle(prev_copy, prevBB, cv::Scalar(255, 0, 0), 2);
                cv::rectangle(cur_copy, curBB, cv::Scalar(255, 0, 0), 2);

                cv::Mat match_vis;
                cv::drawMatches(cur_copy, currFrame.keypoints, prev_copy, prevFrame.keypoints,
                                track_matches[tracks.first][best_cur_bboxIdx],
                                match_vis, cv::Scalar::all(-1), cv::Scalar::all(-1), vector<char>(),
                                cv::DrawMatchesFlags::DEFAULT);

                displayImg(match_vis, "bestMatches");
            }
        }
    }
}
