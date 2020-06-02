#pragma once

#include <iostream>
#include <string>
#include <chrono>
#include <thread>
#include <atomic>

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <boost/lockfree/queue.hpp>

#include <k4a/k4a.hpp>
#include <k4arecord/playback.hpp>

namespace azure
{
    using LockFreeQueue = boost::lockfree::queue<k4a_image_t, boost::lockfree::fixed_sized<true>>;
    using PointT = ::pcl::PointXYZRGB;
    using PointCloud = ::pcl::PointCloud<PointT>;
}