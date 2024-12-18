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

void cityBlock(pcl::visualization::PCLVisualizer::Ptr &viewer,
               ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, 
               const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud
               )
{
    if (!pointProcessorI){
        spdlog::error("Point processor is null.");
        return;
    }

    auto filteredCloud = pointProcessorI->FilterCloud(inputCloud, 0.35,
                                                      Eigen::Vector4f(-18, -7, -2, 1), 
                                                      Eigen::Vector4f(30, 7, 1, 1));
    // renderPointCloud(viewer, filteredCloud, "inputCloud");

    // separate road plane from anything else using 3d ransac
    auto [road, obstacles] = pointProcessorI->SegmentPlane(filteredCloud, 50, 0.3);
    renderPointCloud(viewer, obstacles, "Obstacles", Color(1,1,1));
    renderPointCloud(viewer, road, "Road", Color(0.25,0.65,0.95) );

    // cluster the "obstacles"
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters = pointProcessorI->Clustering(obstacles, 0.85, 15, 500);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1, 1, 0), Color(0, 1, 1), Color(1, 0, 1)};

    for (auto cluster : clusters)
    {
        // spdlog::info("Cluster size={}.", cluster->size() );
        
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId % colors.size() ]);
        
        Box box = pointProcessorI->BoundingBox(cluster);
        renderBox(viewer, box, clusterId, Color(1, 0, 0));
        
        clusterId++;
    }

    // render EGO location
    Car egoCar( Vect3(0,0,-1.5), Vect3(4,2,2), Color(0.5, 0.8, 0.2), "egoCar");
    egoCar.render(viewer);
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
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    // simpleHighway(viewer);

    auto pointProcessorI = std::make_unique<ProcessPointClouds<pcl::PointXYZI>>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();

    while (!viewer->wasStopped ())
    {
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        auto inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI.get(), inputCloudI);

        streamIterator++;
        if (streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce();
    }
}