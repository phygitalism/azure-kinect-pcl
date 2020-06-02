#pragma once

#include "pch.h"

namespace azure
{
    namespace pcl
    {
        // point_cloud is null if transformation failed
        void depth_image_to_point_cloud(k4a_transformation_t transformation, k4a_calibration_type_t calibration, const k4a_image_t depth_image, k4a_image_t * point_cloud);

        void convert_to_pcl(k4a_image_t point_cloud, PointCloud::Ptr pcl_point_cloud);

        void concatenate(PointCloud::ConstPtr cloud1, PointCloud::ConstPtr cloud2, PointCloud::Ptr concatenated);
    }
}