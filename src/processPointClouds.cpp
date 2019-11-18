// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


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

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
	// Create the filtering object
	typename pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>());
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (filterRes, filterRes, filterRes);
    sor.filter (*cloud_filtered);

	typename pcl::PointCloud<PointT>::Ptr cloud_cropped (new pcl::PointCloud<PointT>());
	pcl::CropBox<PointT> roi;
	roi.setMin(minPoint);
	roi.setMax(maxPoint);
	roi.setInputCloud(cloud_filtered);
	roi.filter(*cloud_cropped);

	std::vector<int> indices;
	pcl::CropBox<PointT> roof;
	roof.setMin(Eigen::Vector4f (-1.5,-1.7,-1,1));
	roof.setMax(Eigen::Vector4f (2.6,1.7,-0.4,1));
	roof.setInputCloud(cloud_cropped);
	roof.filter(indices);

	pcl::PointIndices::Ptr inliers (new pcl::PointIndices());
	for (int index: indices)
		inliers->indices.push_back(index);

	pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud_cropped);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_cropped);    

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_cropped;

}

/*
std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	while (maxIterations--)
	{
		std::unordered_set<int> inliers;

		// Ensure getting 3 different random points
		while (inliers.size()<3)
			inliers.insert(rand()%(cloud->points.size()));

		auto itr = inliers.begin();
		float x1, y1, z1, x2, y2, z2, x3, y3, z3;

		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;
		z1 = cloud->points[*itr].z;
		itr++;
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;
		z2 = cloud->points[*itr].z;
		itr++;
		x3 = cloud->points[*itr].x;
		y3 = cloud->points[*itr].y;
		z3 = cloud->points[*itr].z;
		itr++;

		float A = (y2-y1)*(z3-z1)-(z2-z1)*(y3-y1);
		float B = (z2-z1)*(x3-x1)-(x2-x1)*(z3-z1);
		float C = (x2-x1)*(y3-y1)-(y2-y1)*(x3-x1);
		
		// Ensure cross product is not null (when 3 random points happen to be in line and do not define a plane)
		if (A==0 && B==0 && C==0)
			continue;

		float D = -(A*x1+B*y1+C*z1);

		for(int index = 0; index < cloud->points.size(); index++)
		{
			// Don't compute distance for the three originally selected random points
			if (inliers.count(index)>0)
				continue;

			pcl::PointXYZ point = cloud->points[index];
			float distance = fabs(A*point.x+B*point.y+C*point.z+D)/sqrt(A*A+B*B+C*C);
			if (distance < distanceTol)
			{
				inliers.insert(index);
			}
		}
		if (inliers.size()>inliersResult.size())
		{
			inliersResult=inliers;
		}
	}
	
	return inliersResult;
}
*/

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(std::unordered_set<int> inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstacleCloud (new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr roadCloud (new pcl::PointCloud<PointT>());

    /*
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*roadCloud);
    extract.setNegative(true);
    extract.filter(*obstacleCloud);
    */

    for(int index = 0; index < cloud->points.size(); index++)
	{
		PointT point = cloud->points[index];
		if(inliers.count(index))
			roadCloud->points.push_back(point);
		else
			obstacleCloud->points.push_back(point);
	}

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacleCloud, roadCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	
    // TODO:: Fill in this function to find inliers for the cloud.
    /*
    pcl::SACSegmentation<PointT> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices());
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients());
    
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    seg.setInputCloud(cloud);
    seg.segment(*inliers,*coefficients);
    */

    //std::unordered_set<int> inliers = RansacPlane(cloud, 20, 0.2);
    
    std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	while (maxIterations--)
	{
		std::unordered_set<int> inliers;

		// Ensure getting 3 different random points
		while (inliers.size()<3)
			inliers.insert(rand()%(cloud->points.size()));

		auto itr = inliers.begin();
		float x1, y1, z1, x2, y2, z2, x3, y3, z3;

		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;
		z1 = cloud->points[*itr].z;
		itr++;
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;
		z2 = cloud->points[*itr].z;
		itr++;
		x3 = cloud->points[*itr].x;
		y3 = cloud->points[*itr].y;
		z3 = cloud->points[*itr].z;
		itr++;

		float A = (y2-y1)*(z3-z1)-(z2-z1)*(y3-y1);
		float B = (z2-z1)*(x3-x1)-(x2-x1)*(z3-z1);
		float C = (x2-x1)*(y3-y1)-(y2-y1)*(x3-x1);
		
		// Ensure cross product is not null (when 3 random points happen to be in line and do not define a plane)
		if (A==0 && B==0 && C==0)
			continue;

		float D = -(A*x1+B*y1+C*z1);

		for(int index = 0; index < cloud->points.size(); index++)
		{
			// Don't compute distance for the three originally selected random points
			if (inliers.count(index)>0)
				continue;

			PointT point = cloud->points[index];
			float distance = fabs(A*point.x+B*point.y+C*point.z+D)/sqrt(A*A+B*B+C*C);
			if (distance < distanceThreshold)
			{
				inliers.insert(index);
			}
		}
		if (inliers.size()>inliersResult.size())
		{
			inliersResult=inliers;
		}
	}

    if (inliersResult.size() == 0)
    {
        std::cerr << "Could not estimate a planar model for the given dataset" << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliersResult,cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
	
	// Creating the KdTree object for the search method of the extraction
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (clusterTolerance); 
    ec.setMinClusterSize (minSize);
    ec.setMaxClusterSize (maxSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);

	int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
	  typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        cloud_cluster->points.push_back (cloud->points[*pit]);
      
	  cloud_cluster->width = cloud_cluster->points.size ();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;
	  clusters.push_back(cloud_cluster);
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