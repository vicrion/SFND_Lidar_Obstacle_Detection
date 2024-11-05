/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include <unordered_set>
#include <functional>

#include "../../render/render.h"
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

#include <spdlog/spdlog.h>

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

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
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

			if (!subset.contains(i)){
				subset.insert(i);
			}
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
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	
	std::unordered_set<int> inliers = Ransac(cloud, 50, 0.5);

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
