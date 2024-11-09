// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"

#include <unordered_set>
#include <functional>

#include <spdlog/spdlog.h>
#include <Eigen/Dense>


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

    // Do voxel grid point reduction and region based filtering
    typename pcl::PointCloud<PointT>::Ptr cloudSampled(new pcl::PointCloud<PointT>() );
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(filterRes, filterRes, filterRes);
    sor.filter(*cloudSampled);

    // region based filter using crop box filter
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered(new pcl::PointCloud<PointT>() );
    pcl::CropBox<PointT> cropbox(false);
    cropbox.setMin(minPoint);
    cropbox.setMax(maxPoint);
    cropbox.setInputCloud(cloudSampled);
    cropbox.filter(*cloudFiltered);

    // filter out roof points
    pcl::IndicesPtr cropIndices(new std::vector<int>);
    pcl::CropBox<PointT> cropbox2(true);
    cropbox2.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    cropbox2.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
    cropbox2.setInputCloud(cloudFiltered);
    cropbox2.filter(*cropIndices);
    
    typename pcl::PointCloud<PointT>::Ptr cloudClean(new pcl::PointCloud<PointT>() );
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloudFiltered);
    extract.setIndices(cropIndices);
    extract.setNegative(true);
    extract.filter(*cloudClean);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudClean;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
    // Create two new point clouds, one cloud with obstacles and other with segmented plane

    auto cloudIn = new pcl::PointCloud<PointT>();
    auto cloudOut = new pcl::PointCloud<PointT>();

    // Create the filtering object
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*cloudIn);
    extract.setNegative(true);
    extract.filter(*cloudOut);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloudIn, cloudOut);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    auto inliers = ransac3d(cloud, maxIterations, distanceThreshold);

	// pcl::PointIndices::Ptr inliers(new pcl::PointIndices ());  
    // // find inliers for the cloud.
    // pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    // // Create the segmentation object
    // pcl::SACSegmentation<PointT> seg;
    // // Optional
    // seg.setOptimizeCoefficients(true);
    // // Mandatory
    // seg.setModelType(pcl::SACMODEL_PLANE);
    // seg.setMethodType(pcl::SAC_RANSAC);
    // seg.setMaxIterations(maxIterations);
    // seg.setDistanceThreshold(distanceThreshold);

    // seg.setInputCloud (cloud);
    // seg.segment (*inliers, *coefficients);
    // if (inliers->indices.size () == 0)
    // {
    //     spdlog::error("Could not estimate a planar model for the given dataset.");
    // }
    
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

    // Perform euclidean clustering to group detected obstacles
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    // copy to return type
    for (const auto &cluster : cluster_indices)
    {
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT> );
        for (const auto &idx : cluster.indices)
        {
            cloud_cluster->push_back((*cloud)[idx]);
        } //*
        cloud_cluster->width = cloud_cluster->size();
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

template <typename PointT>
typename pcl::PointIndices::Ptr ProcessPointClouds<PointT>::ransac3d(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
	srand(time(NULL));

	const int SSET_SIZE = 3; // min 3 points to fit the plane in
	if (cloud->size()<= SSET_SIZE){
		spdlog::error("The cloud size={} is too small for RANSAC3.", cloud->size() );
		return {};
	}

	auto getRandomSet = [&]() 
	{
		std::unordered_set<int> subset;

		while(subset.size() < SSET_SIZE){
			int i = rand() % cloud->size();
			subset.insert(i);
		}

		auto it = subset.begin();
		int first = *it;
		++it;
		int second = *it;
		++it;
		int third = *it;

		auto p1 = cloud->at(first);
		auto p2 = cloud->at(second);	
		auto p3 = cloud->at(third);			

		return std::make_tuple(p1, p2, p3);
	};

	auto getPlaneEq = [&](const pcl::PointXYZ &point1, const pcl::PointXYZ &point2, const pcl::PointXYZ &point3)
	{
		Eigen::Vector3f p1(point1.x, point1.y, point1.z);
		Eigen::Vector3f p2(point2.x, point2.y, point2.z);
		Eigen::Vector3f p3(point3.x, point3.y, point3.z);

		Eigen::Vector3f v1 = p2 - p1;
		Eigen::Vector3f v2 = p3 - p1;

		Eigen::Vector3f normal = v1.cross(v2);
		normal.normalize();

		// Plane equation: Ax + By + Cz + D = 0
		float A = normal.x();
		float B = normal.y();
		float C = normal.z();
		float D = -(A * p1.x() + B * p1.y() + C * p1.z());

		return pcl::PointXYZI(A,B,C,D);
	};

	auto getDistance = [](const pcl::PointXYZI& plane, const pcl::PointXYZ& point)
	{
		float div = std::sqrt(plane.x*plane.x + plane.y*plane.y + plane.z*plane.z);

		if (std::fabs(div) < 1e-3){
			return std::numeric_limits<float>::max();
		}

		float d = std::fabs(plane.x * point.x + plane.y * point.y + plane.z * point.z + plane.intensity) / div;
		
		return d;
	};
	
	auto getNumInliers = [&](const pcl::PointXYZI& plane)
	{
		int numIn = 0;
		for (unsigned int i=0; i<cloud->size(); ++i){
			if ( getDistance(plane, cloud->at(i)) <= distanceThreshold ){
				numIn++;
			}
		}
		return numIn;
	};

	int numMaxIn = 0;
	pcl::PointXYZI bestPlane;
	while(maxIterations--){
		// randomly sample subset:
		auto [p1, p2, p3] = getRandomSet();
		
		// fit plane:
		auto plane = getPlaneEq(p1, p2, p3);

		auto numIn = getNumInliers(plane);
		numMaxIn = std::max(numIn, numMaxIn);

		if (numMaxIn == numIn){
			bestPlane = plane;
		}
	}

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices ());
	// Return indicies of inliers from fitted line with most inliers
	for (unsigned int i=0; i<cloud->size(); ++i){
		if (getDistance(bestPlane, cloud->at(i)) <= distanceThreshold ){
            inliers->indices.push_back(int(i));
		}
	}

    return inliers;
}
