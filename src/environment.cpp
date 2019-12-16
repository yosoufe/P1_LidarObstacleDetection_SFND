/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
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

void cityBlockStream(pcl::visualization::PCLVisualizer::Ptr &viewer,
                     ProcessPointClouds<pcl::PointXYZI> *pointProcessorI,
                     const pcl::PointCloud<pcl::PointXYZI>::Ptr &inputCloud)
{
  // renderPointCloud(viewer, inputCloud, "ALL", Color(1.0, 0.0, 1.0));
  pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud = pointProcessorI->FilterCloud(inputCloud,
                                                                                    0.2,
                                                                                    Eigen::Vector4f(-15, -6, -3, 1),
                                                                                    Eigen::Vector4f(20, 6.5, 3, 1));

  auto segmentCloud = pointProcessorI->SegmentPlane(filteredCloud, 20, 0.25);
  renderPointCloud(viewer, segmentCloud.second, "Ground", Color(0.01, 0.01, 0.99));

  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->euclideanCluster(segmentCloud.first, 0.4, 10, 2000);

  int clusterId = 0;
  std::vector<Color> colors = {Color(0.99, 0.01, 0.01)};

  for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
  {
    std::cout << "cluster size ";
    pointProcessorI->numPoints(cluster);
    renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[0]);

    Box box = pointProcessorI->BoundingBox(cluster);
    renderBox(viewer, box, clusterId, colors[0]);

    ++clusterId;
  }
}

int main(int argc, char **argv)
{
  std::cout << "starting environment" << std::endl;

  pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
  CameraAngle setAngle = XY;
  initCamera(setAngle, viewer);
  // simpleHighway(viewer);
  // cityBlock(viewer);

  ProcessPointClouds<pcl::PointXYZI> *pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
  std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
  auto streamIterator = stream.begin();
  pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

  while (!viewer->wasStopped())
  {
    // Clear viewer
    viewer->removeAllPointClouds();
    viewer->removeAllShapes();

    // Load pcd and run obstacle detection process
    inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
    cityBlockStream(viewer, pointProcessorI, inputCloudI);

    streamIterator++;
    if (streamIterator == stream.end())
      streamIterator = stream.begin();

    viewer->spinOnce();
  }
}