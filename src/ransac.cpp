#include "ransac.hpp"

template <typename PointT>
std::unordered_set<int> Ransac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
  std::unordered_set<int> inliersResult;
  srand(time(NULL));

  // TODO: Fill in this function

  // For max iterations
  for (size_t iteration = 0; iteration < maxIterations; iteration++)
  {
    // Randomly sample subset and fit line
    PointT &p1 = (*cloud)[rand() % cloud->size()];
    PointT &p2 = (*cloud)[rand() % cloud->size()];
    PointT &p3 = (*cloud)[rand() % cloud->size()];

    /* for 2d Ransac
		// line parameters in form of Ax + By + C = 0
		float A = p1.y - p2.y,
					B = p2.x - p1.x,
					C = p1.x * p2.y - p2.x * p1.y;
		*/

    // plane parameters in form of Ax + By + Cz + D = 0
    float A = (p2.y - p1.y) * (p3.z - p1.z) - (p2.z - p1.z) * (p3.y - p1.y),
          B = (p2.z - p1.z) * (p3.x - p1.x) - (p2.x - p1.x) * (p3.z - p1.z),
          C = (p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x);
    float D = -1 * (A * p1.x + B * p1.y + C * p1.z);

    std::unordered_set<int> inliers;
    for (auto it = cloud->begin(); it != cloud->end(); it++)
    {
      PointT &p = *it;
      // For 2d Ransac
      // Measure distance between every point and fitted line
      // float distance  = std::fabs(A * p.x + B * p.y + C) / std::sqrt(A*A + B*B);

      // Measure distance between every point and fitted plane
      float distance = std::fabs(A * p.x + B * p.y + C * p.z + D) / std::sqrt(A * A + B * B + C * C);
      // If distance is smaller than threshold count it as inlier
      if (distance < distanceTol)
      {
        inliers.insert(it - cloud->begin());
      }
    }

    if (inliers.size() > inliersResult.size())
    {
      inliersResult.swap(inliers);
    }
  }
  // Return indicies of inliers from fitted line with most inliers

  return inliersResult;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr,
          typename pcl::PointCloud<PointT>::Ptr>
SeparateClouds(pcl::PointIndices::Ptr inliers,
               typename pcl::PointCloud<PointT>::Ptr cloud)
{
  // Create two new point clouds, one cloud with obstacles and other with segmented plane

  typename pcl::PointCloud<PointT>::Ptr obstCloud(new pcl::PointCloud<PointT>());
  typename pcl::PointCloud<PointT>::Ptr palaneCloud(new pcl::PointCloud<PointT>());

  // Extract the inliers
  typename pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*obstCloud);

  extract.setNegative(false);
  extract.filter(*palaneCloud);

  std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, palaneCloud);
  return segResult;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr,
          typename pcl::PointCloud<PointT>::Ptr>
segmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud,
             int maxIterations,
             float distanceThreshold)
{
  std::unordered_set<int> inliers_set = Ransac<PointT>(cloud, maxIterations, distanceThreshold);

  // copy the set into a pcl indicies object which make it easier to use with pcl
  pcl::PointIndices::Ptr inliers_indices(new pcl::PointIndices());
  for (int index : inliers_set)
  {
    inliers_indices->indices.push_back(index);
  }

  // return the pair of obstacle and plane clouds
  return SeparateClouds<PointT>(inliers_indices, cloud);
}

// instantiate template functions to avoid linking error (Undefined Reference error)
// for types pcl::PointXYZ and pcl::PointXYZI
template std::unordered_set<int>
Ransac<pcl::PointXYZ>(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                      int maxIterations,
                      float distanceTol);
template std::unordered_set<int>
Ransac<pcl::PointXYZI>(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                       int maxIterations,
                       float distanceTol);
template std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr,
                   pcl::PointCloud<pcl::PointXYZ>::Ptr>
SeparateClouds<pcl::PointXYZ>(pcl::PointIndices::Ptr inliers,
                              pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
template std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr,
                   pcl::PointCloud<pcl::PointXYZI>::Ptr>
SeparateClouds<pcl::PointXYZI>(pcl::PointIndices::Ptr inliers,
                               pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);

template std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr,
                   pcl::PointCloud<pcl::PointXYZ>::Ptr>
segmentPlane<pcl::PointXYZ>(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                            int maxIterations,
                            float distanceThreshold);
template std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr,
                   pcl::PointCloud<pcl::PointXYZI>::Ptr>
segmentPlane<pcl::PointXYZI>(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                             int maxIterations,
                             float distanceThreshold);