#pragma once

#include "pch.h"

namespace azure
{
    using LockFreeQueue = boost::lockfree::queue<k4a_image_t, boost::lockfree::fixed_sized<true>>;
    using PointT = ::pcl::PointXYZRGB;
    using PointCloud = ::pcl::PointCloud<PointT>;
}