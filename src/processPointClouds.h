// PCL lib Functions for processing point clouds

#pragma once

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <iostream>
#include <string>
#include <vector>
#include <ctime>
#include <chrono>
#include "render/box.h"
#include <unordered_set>
#include "kdtree.h"
#include <memory> // for std::shared_ptr

template <typename PointT>
class ProcessPointClouds
{
public:
  //constructor
  ProcessPointClouds();
  //deconstructor
  ~ProcessPointClouds();

  void
  numPoints(typename pcl::PointCloud<PointT>::Ptr cloud);

  typename pcl::PointCloud<PointT>::Ptr
  FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud,
              float filterRes,
              Eigen::Vector4f minPoint,
              Eigen::Vector4f maxPoint);

  std::unordered_set<int> ransac(typename pcl::PointCloud<PointT>::Ptr cloud,
                                 int maxIterations,
                                 float distanceTol);

  std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>
  SeparateClouds(pcl::PointIndices::Ptr inliers,
                 typename pcl::PointCloud<PointT>::Ptr cloud);

  std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>
  SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud,
               int maxIterations, float distanceThreshold);

  void
  proximity(size_t index,
            const typename pcl::PointCloud<PointT>::Ptr cloud,
            typename pcl::PointCloud<PointT>::Ptr cluster,
            typename KdTree<PointT>::Ptr tree,
            float distanceTol,
            std::unordered_set<int> &processed_indices);

  std::vector<typename pcl::PointCloud<PointT>::Ptr>
  euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud,
                   float clusterTolerance,
                   int minSize,
                   int maxSize);

  Box BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);

  void
  savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file);

  typename pcl::PointCloud<PointT>::Ptr
  loadPcd(std::string file);

  std::vector<boost::filesystem::path>
  streamPcd(std::string dataPath);
};