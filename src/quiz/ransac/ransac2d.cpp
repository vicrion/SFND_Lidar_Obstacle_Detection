/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include <unordered_set>
#include <functional>

#include "../../render/render.h"
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

#include <spdlog/spdlog.h>
#include <Eigen/Dense>

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
	return pointProcessor.loadPcd("../../../../src/sensors/data/pcd/simpleHighway.pcd");
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

std::unordered_set<int> Ransac3D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
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
			if ( getDistance(plane, cloud->at(i)) <= distanceTol ){
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

	// Return indicies of inliers from fitted line with most inliers
	for (unsigned int i=0; i<cloud->size(); ++i){
		if (getDistance(bestPlane, cloud->at(i)) <= distanceTol ){
			inliersResult.insert(int(i));
		}
	}

	spdlog::info("Completed Ransac 3d with cloud of size={}, best in={}, output size={}.", cloud->size(), numMaxIn, inliersResult.size());

	return inliersResult;
}

std::unordered_set<int> Ransac2D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));

	const int subsetSz = 2; // min 2 points to fit the line in
	if (cloud->size()<= subsetSz){
		spdlog::error("The cloud size is too small for RANSAC.");
		return {};
	}

	auto getRandomSet = [&]() 
	{
		std::unordered_set<int> subset;
		
		int min = 0;
		int max = cloud->size()-1;

		while(subset.size() < subsetSz){
			int i = min + rand() % (( max + 1 ) - min);
			subset.insert(i);
		}

		assert(subset.size() == subsetSz);

		auto it = subset.begin();
		int first = *it;
		++it;
		int second = *it;

		auto p1 = cloud->at(first);
		auto p2 = cloud->at(second);				

		return std::make_tuple(p1, p2);
	};
	
	auto getLineEq = [&](const pcl::PointXYZ& p1, const pcl::PointXYZ& p2)
	{
		float A = p1.y - p2.y;
		float B = p2.x - p1.x;
		float C = p1.x*p2.y - p2.x*p1.y;

		pcl::PointXYZ line(A,B,C);

		return line;
	};

	auto getDistance = [](const pcl::PointXYZ& line, const pcl::PointXYZ& point)
	{
		float div = std::sqrt(line.x*line.x + line.y*line.y);
		if (std::fabs(div) < 1e-3){
			return std::numeric_limits<float>::max();
		}

		float d = std::fabs(line.x * point.x + line.y * point.y + line.z) / div;
		
		return d;
	};

	auto getNumInliers = [&](const pcl::PointXYZ& line)
	{
		int numIn = 0;
		for (unsigned int i=0; i<cloud->size(); ++i){
			if ( getDistance(line, cloud->at(i)) <= distanceTol ){
				numIn++;
			}
		}
		return numIn;
	};

	int numMaxIn = 0;
	pcl::PointXYZ bestLine;

	// For max iterations
	for (int i=0; i<maxIterations; ++i)
	{
		// Randomly sample subset and fit line
		auto [p1, p2] = getRandomSet(); // indices
		auto line = getLineEq(p1, p2);

		// Measure distance between every point and fitted line
		// If distance is smaller than threshold count it as inlier
		auto numIn = getNumInliers(line);
		numMaxIn = std::max(numIn, numMaxIn);

		if (numMaxIn == numIn){
			bestLine = line;
		}
	}
	spdlog::info("Num inliers: {}.", numMaxIn);

	// Return indicies of inliers from fitted line with most inliers
	for (unsigned int i=0; i<cloud->size(); ++i){
		if (getDistance(bestLine, cloud->at(i)) <= distanceTol ){
			inliersResult.insert(int(i));
		}
	}

	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	std::unordered_set<int> inliers = Ransac3D(cloud, 50, 0.5);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(unsigned int index = 0; index < cloud->points.size(); index++)
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
