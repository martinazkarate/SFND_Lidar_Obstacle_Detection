/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

void renderCarBox(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
  Box box;
  box.x_min = -1.5;
  box.y_min = -0.9;
  box.z_min = -1;
  box.x_max = 2.6;
  box.y_max = 0.9;
  box.z_max = -0.4;
  renderBox(viewer,box,-1,Color(1,0,1),0.25);
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
  //* Filter input Point Cloud to reduce amount of data to process by voxeling and cropping box *//
  pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI->FilterCloud(inputCloud, 0.25 , Eigen::Vector4f (-15.0, -6.0, -3.0, 1), Eigen::Vector4f ( 30.0, 7.0, 0.0, 1));
  
  //* Render filtered Point Cloud and a Box representing your own car *//
  renderPointCloud(viewer,filterCloud,"filterCloud");
  renderCarBox(viewer);

  //* Segment road plane from Cloud and render it (option to render also the rest of separated obstacle cloud) *//
  std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlane(filterCloud,50,0.2);
  renderPointCloud(viewer,segmentCloud.second,"roadCloud",Color(0,1,0));
  //renderPointCloud(viewer,segmentCloud.first,"obstacleCloud",Color(1,0,0));
    
  //* Call clustering function to identify different cars and separate them in a vector of Point Clouds (option to call PCL library function) *//  
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->euclideanCluster(segmentCloud.first, 0.5, 5, 300);
  //std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->euclideanClusterPCL(segmentCloud.first, 0.5, 5, 300);

  int clusterId = 0;
  std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};

  //* Iterate through vector of Clusters and render each of them in sequential colors. Render a bounding box around each cluster car *//
    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
      std::cout << "cluster size "; pointProcessorI->numPoints(cluster);
      renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId%colors.size()]);
      Box box = pointProcessorI->BoundingBox(cluster);
      renderBox(viewer,box,clusterId);
      ++clusterId;
    }
}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    /*
    int num_args = argc;
    if (num_args!=5)
    {
        std::cerr << "Wrong number of arguments passed in line. Exiting now." << std::endl;
        return -1;
    }
    std::string exec_name = argv[0];
    std::string first_arg = argv[1];
    std::string second_arg = argv[2];
    std::string third_arg = argv[3];
    std::string fourth_arg = argv[4];
    int use_pcl=int(first_arg);
    float voxel_leaf_size=float(second_arg);
    int ransac_iter=int(third_arg);
    float cluster_dist=float(fourth_arg);
    */

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    
    //* Create Point Processor and prepare the PCD stream *//
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    
    //* Create the input Point Cloud variable that will contain the PCD at each stream cycle *//
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;
    
    while (!viewer->wasStopped ())
    {
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce ();
    }
}