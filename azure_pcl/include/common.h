#pragma once

#include <iostream>
#include <string>
#include <chrono>
#include <thread>
#include <tuple>
#include <atomic>

#include <boost/lockfree/queue.hpp>
#include <k4a/k4a.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>


namespace azure
{
	using LockFreeQueue = boost::lockfree::queue<k4a_image_t, boost::lockfree::fixed_sized<true>>;

	using PointT = ::pcl::PointXYZRGB;
	using PointCloud = ::pcl::PointCloud<PointT>;
}