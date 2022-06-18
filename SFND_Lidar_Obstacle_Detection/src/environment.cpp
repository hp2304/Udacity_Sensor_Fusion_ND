/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"
#include "config_parser.h"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr &viewer)
{

    Car egoCar(Vect3(0, 0, 0), Vect3(4, 2, 2), Color(0, 1, 0), "egoCar");
    Car car1(Vect3(15, 0, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car1");
    Car car2(Vect3(8, -4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car2");
    Car car3(Vect3(-12, 4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car3");

    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if (renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}

void simpleHighway(pcl::visualization::PCLVisualizer::Ptr &viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------

    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);

    // TODO:: Create lidar sensor
    auto *lidar = new Lidar(cars, 0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc = lidar->scan();
//     renderRays(viewer, lidar->position, pc);
//    renderPointCloud(viewer, pc, "PC");

    // TODO:: Create point processor
    auto *pc_processor = new ProcessPointClouds<pcl::PointXYZ>();
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmented_pc = pc_processor->SegmentPlane(pc, 10, 0.2);
    // renderPointCloud(viewer, segmented_pc.first, "Obstacle_PC", Color(1, 0, 0));
    renderPointCloud(viewer, segmented_pc.second, "Road_PC", Color(1, 1, 1));

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> obstacle_clusters = pc_processor->Clustering(segmented_pc.first, 1.0, 3, 30);

    int cluster_id = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};

    bool render_box = true;
    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : obstacle_clusters){
        std::cout << "Cluster size: ";
        pc_processor->numPoints(cluster);
        renderPointCloud(viewer, cluster, "Obstacle_PC_" + std::to_string(cluster_id), colors[cluster_id % colors.size()]);

        if(render_box){
            Box box = pc_processor->BoundingBox(cluster);
            renderBox(viewer, box, cluster_id);
        }
        ++cluster_id;
    }
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer,
               ProcessPointClouds<pcl::PointXYZI>* pc_processor,
               const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc,
               std::unordered_map<std::string, std::string> &args){
    

    // renderPointCloud(viewer, pc, "Real_PC");

    // Downsample data
    float filtRes = std::stof(args["FILTERING_FILT_RES"]);
    float min_x = std::stof(args["FILTERING_MIN_X"]);
    float min_y = std::stof(args["FILTERING_MIN_Y"]);
    float min_z = std::stof(args["FILTERING_MIN_Z"]);
    float max_x = std::stof(args["FILTERING_MAX_X"]);
    float max_y = std::stof(args["FILTERING_MAX_Y"]);
    float max_z = std::stof(args["FILTERING_MAX_Z"]);

    // Experiment with the ? values and find what works best
    auto filtered_pc = pc_processor->FilterCloud(pc, filtRes , Eigen::Vector4f (min_x, min_y, min_z, 1), Eigen::Vector4f (max_x, max_y, max_z, 1));

    float rooftop_min_x = std::stof(args["ROOFTOP_MIN_X"]);
    float rooftop_min_y = std::stof(args["ROOFTOP_MIN_Y"]);
    float rooftop_min_z = std::stof(args["ROOFTOP_MIN_Z"]);
    float rooftop_max_x = std::stof(args["ROOFTOP_MAX_X"]);
    float rooftop_max_y = std::stof(args["ROOFTOP_MAX_Y"]);
    float rooftop_max_z = std::stof(args["ROOFTOP_MAX_Z"]);

    Eigen::Vector4f rooftop_minpoint(rooftop_min_x, rooftop_min_y, rooftop_min_z, 1);
    Eigen::Vector4f rooftop_maxpoint(rooftop_max_x, rooftop_max_y, rooftop_max_z, 1);

    filtered_pc = pc_processor->FilterRoofpoints(filtered_pc, rooftop_minpoint, rooftop_maxpoint);
    renderPointCloud(viewer, filtered_pc, "Filtered_PC");

    // Segmentation (segregate road points from obstacles)
    int maxIters = std::stoi(args["SEGM_MAX_ITERS"]);
    float segm_dist_thr = std::stof(args["SEGM_DIST_THRESHOLD"]);

    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmented_pc = pc_processor->SegmentPlaneCustom(filtered_pc, maxIters, segm_dist_thr);
//    renderPointCloud(viewer, segmented_pc.first, "Obstacle_PC", Color(0, 1, 0));
    renderPointCloud(viewer, segmented_pc.second, "Road_PC", Color(1, 1, 1));


    // Cluster various objects from obstacle cloud
    int min_nb_points = std::stoi(args["CLUSTERING_MIN_POINTS"]);
    int max_nb_points = std::stoi(args["CLUSTERING_MAX_POINTS"]);
    float clust_dist_thr = std::stof(args["CLUSTERING_DIST_THRESHOLD"]);
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> obstacle_clusters = pc_processor->ClusteringCustom(segmented_pc.first, clust_dist_thr, min_nb_points, max_nb_points);

    int cluster_id = 0;
    std::vector<Color> colors = {Color(0.2,0.6,0),
                                  Color(0,0.3,0.4),
                                  Color(0,0,1),
                                  Color(0,1,1),
                                  Color(1,0,1),
                                  Color(1,1,0)};

    for(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cluster : obstacle_clusters){
        std::cout << "Cluster size: ";
        pc_processor->numPoints(cluster);
//        renderPointCloud(viewer, cluster, "Obstacle_PC_" + std::to_string(cluster_id), colors[cluster_id % colors.size()]);
        Box box = pc_processor->BoundingBox(cluster);
        renderBox(viewer, box, cluster_id);
        ++cluster_id;
    }
}

// setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr &viewer)
{

    viewer->setBackgroundColor(0, 0, 0);

    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;

    switch (setAngle)
    {
    case XY:
        viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0);
        break;
    case TopDown:
        viewer->setCameraPosition(0, 0, distance, 1, 0, 1);
        break;
    case Side:
        viewer->setCameraPosition(0, -distance, 0, 0, 0, 1);
        break;
    case FPS:
        viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if (setAngle != FPS)
        viewer->addCoordinateSystem(1.0);
}

int main(int argc, char **argv)
{
    std::cout << "starting enviroment" << std::endl;

    std::string config_path = "../src/configs.txt";
    std::unordered_map<std::string, std::string> args = parseConfigFile(config_path, "=");

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
//    simpleHighway(viewer);

    ProcessPointClouds<pcl::PointXYZI>* pc_processor = new ProcessPointClouds<pcl::PointXYZI>();

    std::vector<boost::filesystem::path> stream = pc_processor->streamPcd(args["PCD_FILE_PATH"]);
    auto streamIterator = stream.begin();

    pcl::PointCloud<pcl::PointXYZI>::Ptr pc;

    while (!viewer->wasStopped()){
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        pc = pc_processor->loadPcd((*streamIterator).string());
        cityBlock(viewer, pc_processor, pc, args);

        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();
        viewer->spinOnce();
    }
}