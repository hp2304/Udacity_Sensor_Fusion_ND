// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include "cluster.h"


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{   
    typename pcl::PointCloud<PointT>::Ptr filtered_cloud(new typename pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr cropped_cloud(new typename pcl::PointCloud<PointT>);
    // Time filtering process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (filterRes, filterRes, filterRes);
    sor.filter(*filtered_cloud);

    pcl::CropBox<PointT> cb(true);
    cb.setInputCloud(filtered_cloud);
    cb.setMin(minPoint);
    cb.setMax(maxPoint);
    cb.filter(*cropped_cloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cropped_cloud;
}

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterRoofpoints(typename pcl::PointCloud<PointT>::Ptr cloud, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint) {
    typename pcl::PointCloud<PointT>::Ptr filtered_cloud(new typename pcl::PointCloud<PointT>);
    // Time rooftop filtering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<int> indices;
    pcl::CropBox<PointT> cb(true);
    cb.setInputCloud(cloud);
    cb.setMin(minPoint);
    cb.setMax(maxPoint);
    cb.filter(indices);

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    for (auto idx: indices)
        inliers->indices.push_back(idx);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*filtered_cloud);


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering roofpoints took " << elapsedTime.count() << " milliseconds" << std::endl;

    return filtered_cloud;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
    // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    pcl::ExtractIndices<PointT> extract;
    typename pcl::PointCloud<PointT>::Ptr road_pc(new pcl::PointCloud<PointT>), obstacle_pc(new pcl::PointCloud<PointT>);

    std::unordered_set<int> inlier_set(inliers->indices.begin(), inliers->indices.end());
    for(int i=0; i<cloud->points.size(); ++i){
        if(inlier_set.find(i) == inlier_set.end()){
            obstacle_pc->points.push_back(cloud->points[i]);
        }
        else{
            road_pc->points.push_back(cloud->points[i]);
        }
    }

    // Using PCL
//    extract.setInputCloud(cloud);
//    extract.setIndices(inliers);
//    extract.setNegative(false);
//    extract.filter(*road_pc);
//
//    extract.setNegative (true);
//    extract.filter(*obstacle_pc);


    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacle_pc, road_pc);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    // TODO:: Fill in this function to find inliers for the cloud.

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);

    pcl::SACSegmentation<PointT> segmenter;
    segmenter.setOptimizeCoefficients (true);

    segmenter.setModelType (pcl::SACMODEL_PLANE);
    segmenter.setMethodType (pcl::SAC_RANSAC);
    segmenter.setDistanceThreshold (distanceThreshold);
    segmenter.setMaxIterations(maxIterations);

    segmenter.setInputCloud (cloud);
    segmenter.segment (*inliers, *coefficients);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlaneCustom(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    std::vector<int> indices;

    // For max iterations
    for(int i=0; i<maxIterations; ++i) {
        // Randomly sample subset and fit line

        auto random_indices = sampleRandomIndices(0, cloud->points.size() - 1, 3);

        auto p1 = cloud->points[random_indices[0]];
        auto p2 = cloud->points[random_indices[1]];
        auto p3 = cloud->points[random_indices[2]];

        // v1 defined as a vector which goes to p2 from p1
        std::vector<float> v1{p2.x - p1.x, p2.y - p1.y, p2.z - p1.z};

        // v2 defined as a vector which goes to p3 from p1
        std::vector<float> v2{p3.x - p1.x, p3.y - p1.y, p3.z - p1.z};

        // ax + by + cz + d = 0
        float a = (v1[1] * v2[2]) - (v1[2] * v2[1]);
        float b = (v1[2] * v2[0]) - (v1[0] * v2[2]);
        float c = (v1[0] * v2[1]) - (v1[1] * v2[0]);
        float d = -((a * p1.x) + (b * p1.y) + (c * p1.z));

        float denom = sqrt((a*a) + (b*b) + (c*c));
        std::vector<int> current_inliers_indices;
        for(int j=0; j<cloud->points.size(); ++j) {
            // Measure distance between every point and fitted line
            auto const p = cloud->points[j];
            float dist = fabs((a * p.x) + (b * p.y) + (c * p.z) + d) / denom;

            // If distance is smaller than threshold count it as inlier
            if(dist <= distanceThreshold){
                current_inliers_indices.push_back(j);
            }
        }

        if(current_inliers_indices.size() > indices.size()){
            indices = current_inliers_indices;
        }
    }
    inliers->indices = indices;

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers, cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles

    typename pcl::search::KdTree<PointT>::Ptr tree(new typename pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    for(auto point_indices: cluster_indices){
        typename pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);
        for(auto point_idx: point_indices.indices)
            cluster->points.push_back(cloud->points[point_idx]);
        cluster->width = cluster->points.size();
        cluster->height = 1;
        cluster->is_dense = true;
        clusters.push_back(cluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::ClusteringCustom(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles

    std::vector<std::vector<float>> points;
    for(int i=0; i<cloud->points.size(); ++i) {
        auto p = cloud->points[i];
        std::vector<float> point{p.x, p.y, p.z, (float) i};
        points.push_back(point);
    }

//    KdTree *tree = new KdTree(points);
    KdTree *tree = new KdTree;
    for(int i=0; i<points.size(); ++i)
        tree->insert(points[i], i);

    std::vector<std::vector<int>> cluster_indices = euclideanCluster(points, tree, clusterTolerance, minSize, maxSize);

    for(auto point_indices: cluster_indices){
        typename pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);
        for(auto point_idx: point_indices)
            cluster->points.push_back(cloud->points[point_idx]);
        cluster->width = cluster->points.size();
        cluster->height = 1;
        cluster->is_dense = true;
        clusters.push_back(cluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}