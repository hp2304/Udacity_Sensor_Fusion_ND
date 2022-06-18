/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
      // Add inliers
      float scatter = 0.6;
      for(int i = -5; i < 5; i++)
      {
          double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
          double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
          pcl::PointXYZ point;
          point.x = i+scatter*rx;
          point.y = i+scatter*ry;
          point.z = 0;

          cloud->points.push_back(point);
      }
      // Add outliers
      int numOutliers = 10;
      while(numOutliers--)
      {
          double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
          double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
          pcl::PointXYZ point;
          point.x = 5*rx;
          point.y = 5*ry;
          point.z = 0;

          cloud->points.push_back(point);

      }
      cloud->width = cloud->points.size();
      cloud->height = 1;

      return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
    ProcessPointClouds<pcl::PointXYZ> pointProcessor;
    return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->initCameraParameters();
    viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
    viewer->addCoordinateSystem (1.0);
    return viewer;
}

std::unordered_set<int> randomSampleIndices(int arr_size, int nb_samples){
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> distrib(0, arr_size - 1);

    std::unordered_set<int> indices;
    for(int i=0; i<nb_samples; ++i){
        int random_idx = distrib(gen);
        while(indices.find(random_idx) != indices.end()){
            random_idx = distrib(gen)
        }
        indices.insert(random_idx);
    }
    return indices;
}

std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol){
    std::unordered_set<int> inliersResult;
    srand(time(NULL));
    
    // TODO: Fill in this function

    // For max iterations
    for(int i=0; i<maxIterations; ++i) {
        // Randomly sample subset and fit line
        
        auto random_indices = randomSampleIndices(cloud->points.size(), 3);

        auto it = random_indices.begin();
        auto p1 = cloud->points[*it];
        ++it;
        auto p2 = cloud->points[*it];
        ++it;
        auto p3 = cloud->points[*it];

        // v1 defined as a vector which goes to p2 from p1
        pcl::PointXYZ v1(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);

        // v2 defined as a vector which goes to p3 from p1
        pcl::PointXYZ v2(p3.x - p1.x, p3.y - p1.y, p3.z - p1.z);

        // ax + by + cz + d = 0
        float a = (v1.y * v2.z) - (v1.z * v2.y);
        float b = (v1.z * v2.x) - (v1.x * v2.z);
        float c = (v1.x * v2.y) - (v1.y * v2.x);
        float d = -((a * p1.x) + (b * p1.y) + (c * p1.z));
        
        float denom = sqrt((a*a) + (b*b) + (c*c));
        std::unordered_set<int> current_inliers_indices;
        for(int j=0; j<cloud->points.size(); ++j) {
            // Measure distance between every point and fitted line
            auto const p = cloud->points[j];
            float dist = fabs((a * p.x) + (b * p.y) + (c * p.z) + d) / denom;

            // If distance is smaller than threshold count it as inlier
            if(dist <= distanceTol){
                current_inliers_indices.insert(j);
            }
        }

        if(current_inliers_indices.size() > inliersResult.size()){
            inliersResult = current_inliers_indices;
        }
    }

    // Return indicies of inliers from fitted line with most inliers
    return inliersResult;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol){
    std::unordered_set<int> inliersResult;
    srand(time(NULL));
    
    // TODO: Fill in this function

    // For max iterations
    for(int i=0; i<maxIterations; ++i) {
        // Randomly sample subset and fit line
        int idx1 = rand() % cloud->points.size();
        int idx2 = rand() % cloud->points.size();
        while(idx2 == idx1){idx2 = rand() % cloud->points.size();}

        auto p1 = cloud->points[idx1];
        auto p2 = cloud->points[idx2];

        // y = m*x + y_intercept and ax + by + c = 0
        float a, b, c;
        if((p2.x - p1.x) == 0){
            // Vertical line -> slope is infinite
            // (1).x + c = 0 -> a = 1 and b = 0
            a = 1;
            b = 0;
            c = -p1.x;
        }
        else{
            // y - mx - y_intercept = 0
            float slope = (p2.y - p1.y) / (p2.x - p1.x);
            float y_intercept = p1.y - (slope * p1.x);

            b = 1;
            a = -slope;
            c = -y_intercept;
        }

        float denom = sqrt((a*a) + (b*b));
        std::unordered_set<int> current_inliers_indices;
        for(int j=0; j<cloud->points.size(); ++j) {
            // Measure distance between every point and fitted line
            auto const p = cloud->points[j];
            float dist = fabs((a * p.x) + (b * p.y) + c) / denom;

            // If distance is smaller than threshold count it as inlier
            if(dist <= distanceTol){
                current_inliers_indices.insert(j);
            }
        }

        if(current_inliers_indices.size() > inliersResult.size()){
            inliersResult = current_inliers_indices;
        }
    }

    // Return indicies of inliers from fitted line with most inliers
    return inliersResult;
}

int main ()
{

    // Create viewer
    pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

    // Create data
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
    

    // TODO: Change the max iteration and distance tolerance arguments for Ransac function
    // std::unordered_set<int> inliers = Ransac(cloud, 10, 1);
    std::unordered_set<int> inliers = RansacPlane(cloud, 50, 0.3);

    pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

    for(int index = 0; index < cloud->points.size(); index++)
    {
        pcl::PointXYZ point = cloud->points[index];
        if(inliers.count(index))
            cloudInliers->points.push_back(point);
        else
            cloudOutliers->points.push_back(point);
    }


    // Render 2D point cloud with inliers and outliers
    if(inliers.size())
    {
        renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
          renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
    }
      else
      {
          renderPointCloud(viewer,cloud,"data");
      }
    
      while (!viewer->wasStopped ())
      {
        viewer->spinOnce ();
      }
      
}
