/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include <memory>

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // Create lidar sensor - on heap since there can be many PCLs
    std::unique_ptr<Lidar> lidar = std::make_unique<Lidar>(cars, 0);
    auto cloud = lidar->scan();
    // renderRays(viewer, lidar->position, cloud);
    // renderPointCloud(viewer, cloud, "Simulated");

    // Create point processor - on stack is ok
    auto processor = ProcessPointClouds<pcl::PointXYZ>();
    auto [road, obstacles] = processor.SegmentPlane(cloud, 50, 0.2);
    // renderPointCloud(viewer, obstacles, "Obstacles", Color(1,0,0));
    // renderPointCloud(viewer, road, "Road", Color(0,0,1) );

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters = processor.Clustering(obstacles, 1.0, 3, 30);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1, 0, 0), Color(0, 1, 0), Color(0, 0, 1)};

    for (auto cluster : clusters)
    {
        spdlog::info("Cluster size={}.", cluster->size() );
        
        processor.numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId % colors.size() ]);
        
        Box box = processor.BoundingBox(cluster);
        renderBox(viewer, box, clusterId, colors[clusterId % colors.size() ]);
        
        clusterId++;
    }
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr &viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------

    auto pointProcessorI = std::make_unique<ProcessPointClouds<pcl::PointXYZI>>();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
    auto filteredCloud = pointProcessorI->FilterCloud(inputCloud, 0.25,
                                                      Eigen::Vector4f(-20, -10, -2, 1), 
                                                      Eigen::Vector4f(45, 10, 2, 1));
    renderPointCloud(viewer, filteredCloud, "inputCloud");
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

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = TopDown;
    initCamera(setAngle, viewer);
    // simpleHighway(viewer);
    cityBlock(viewer);

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce ();
    } 
}