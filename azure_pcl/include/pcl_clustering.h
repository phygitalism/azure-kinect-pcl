#pragma once

#include "pch.h"
#include "colourmanager.hpp"

namespace azure
{
    namespace cluster
    {
        PointCloud::Ptr euclidean_clustering(PointCloud::ConstPtr pointCloud);
    }
}