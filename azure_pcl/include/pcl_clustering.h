#pragma once

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>


#include <algorithm>
#include "common.h"
#include "colourmanager.hpp"

namespace azure
{
	namespace cluster
	{
		PointCloud::Ptr euclidean_clustering(PointCloud::ConstPtr pointCloud);
	}
}