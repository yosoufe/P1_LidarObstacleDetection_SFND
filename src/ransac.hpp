#pragma once

#include <unordered_set>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>

template <typename PointT>
std::unordered_set<int> Ransac(typename pcl::PointCloud<PointT>::Ptr cloud,
                               int maxIterations,
                               float distanceTol);

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr,
          typename pcl::PointCloud<PointT>::Ptr>
SeparateClouds(pcl::PointIndices::Ptr inliers,
               typename pcl::PointCloud<PointT>::Ptr cloud);

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr,
          typename pcl::PointCloud<PointT>::Ptr>
segmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud,
             int maxIterations,
             float distanceThreshold);