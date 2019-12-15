// PCL lib Functions for processing point clouds

#include "processPointClouds.h"

//constructor:
template <typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}

//de-constructor:
template <typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}

template <typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
  std::cout << cloud->points.size() << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud,
                                                                              float filterRes,
                                                                              Eigen::Vector4f minPoint,
                                                                              Eigen::Vector4f maxPoint)
{

  // Time segmentation process
  auto startTime = std::chrono::steady_clock::now();

  // TODO:: Fill in the function to do voxel grid point reduction and region based filtering

  // crop
  typename pcl::PointCloud<PointT>::Ptr cloud_cropped(new pcl::PointCloud<PointT>());
  typename pcl::CropBox<PointT> cropOp;
  cropOp.setInputCloud(cloud);
  cropOp.setMin(minPoint);
  cropOp.setMax(maxPoint);
  cropOp.filter(*cloud_cropped);

  // crop out the roof traces.
  typename pcl::PointCloud<PointT>::Ptr cloud_cropped2(new pcl::PointCloud<PointT>());
  cropOp.setInputCloud(cloud_cropped);
  cropOp.setMin(Eigen::Vector4f(-3, -1.5, -1, 1));
  cropOp.setMax(Eigen::Vector4f(3, 1.5, 1, 1));
  cropOp.setNegative(true);
  cropOp.filter(*cloud_cropped2);

  // Voxel filtering
  typename pcl::PointCloud<PointT>::Ptr cloud_voxel_filtered(new pcl::PointCloud<PointT>());
  typename pcl::VoxelGrid<PointT> sor;
  sor.setInputCloud(cloud_cropped2);
  sor.setLeafSize(filterRes, filterRes, filterRes);
  sor.filter(*cloud_voxel_filtered);

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

  return cloud_voxel_filtered;
}

template <typename PointT>
std::unordered_set<int>
ProcessPointClouds<PointT>::ransac(typename pcl::PointCloud<PointT>::Ptr cloud,
                                   int maxIterations,
                                   float distanceTol)
{
  std::unordered_set<int> inliersResult;
  srand(time(NULL));

  // For max iterations
  for (size_t iteration = 0; iteration < maxIterations; iteration++)
  {
    // Randomly sample subset and fit line
    PointT &p1 = (*cloud)[rand() % cloud->size()];
    PointT &p2 = (*cloud)[rand() % cloud->size()];
    PointT &p3 = (*cloud)[rand() % cloud->size()];

    /* for 2d ransac
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
      // For 2d ransac
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
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud)
{
  // Create two new point clouds, one cloud with obstacles and other with segmented plane
  typename pcl::PointCloud<PointT>::Ptr obstCloud(new pcl::PointCloud<PointT>());
  typename pcl::PointCloud<PointT>::Ptr palaneCloud(new pcl::PointCloud<PointT>());

  // Extract the inliers
  pcl::ExtractIndices<PointT> extract;
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
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud,
                                         int maxIterations,
                                         float distanceThreshold)
{
  // Time segmentation process
  auto startTime = std::chrono::steady_clock::now();
  std::unordered_set<int> inliers_set = ransac(cloud, maxIterations, distanceThreshold);
  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;
  // copy the set into a pcl indicies object which make it easier to use with pcl
  pcl::PointIndices::Ptr inliers_indices(new pcl::PointIndices());
  for (int index : inliers_set)
  {
    inliers_indices->indices.push_back(index);
  }
  return SeparateClouds(inliers_indices, cloud);
}

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud,
                                       float clusterTolerance,
                                       int minSize,
                                       int maxSize)
{

  // Time clustering process
  auto startTime = std::chrono::steady_clock::now();

  std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

  // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
  // Creating the KdTree object for the search method of the extraction
  typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
  tree->setInputCloud(cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance(clusterTolerance); // 2cm
  ec.setMinClusterSize(minSize);
  ec.setMaxClusterSize(maxSize);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.extract(cluster_indices);

  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
  {
    typename pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);
    for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
      cloud_cluster->points.push_back(cloud->points[*pit]); //*
    cloud_cluster->width = cloud_cluster->points.size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    clusters.push_back(cloud_cluster);
  }

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

  return clusters;
}

template <typename PointT>
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

template <typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
  pcl::io::savePCDFileASCII(file, *cloud);
  std::cerr << "Saved " << cloud->points.size() << " data points to " + file << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

  typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

  if (pcl::io::loadPCDFile<PointT>(file, *cloud) == -1) //* load the file
  {
    PCL_ERROR("Couldn't read file \n");
  }
  std::cerr << "Loaded " << cloud->points.size() << " data points from " + file << std::endl;

  return cloud;
}

template <typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

  std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

  // sort files in accending order so playback is chronological
  sort(paths.begin(), paths.end());

  return paths;
}