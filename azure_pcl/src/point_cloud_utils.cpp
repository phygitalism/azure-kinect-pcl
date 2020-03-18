#include "point_cloud_utils.h"


void azure::pcl::depth_image_to_point_cloud(k4a_transformation_t transformation, k4a_calibration_type_t calibration, const k4a_image_t depth_image, k4a_image_t* point_cloud)
{
	if (point_cloud == nullptr)
	{
		return;
	}

	int width{ k4a_image_get_width_pixels(depth_image) };
	int height{ k4a_image_get_height_pixels(depth_image) };

	if (width == 0 || height == 0)
	{
		*point_cloud = nullptr;
		return;
	}

	if (k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM, width, height, width * 3 * sizeof(int16_t), &*point_cloud) == K4A_RESULT_FAILED)
	{
		*point_cloud = nullptr;
	}

	if (k4a_transformation_depth_image_to_point_cloud(transformation, depth_image, calibration, *point_cloud) == K4A_RESULT_FAILED)
	{
		*point_cloud = nullptr;
	}
}

void azure::pcl::convert_to_pcl(k4a_image_t point_cloud, PointCloud::Ptr pcl_point_cloud)
{
	int width{ k4a_image_get_width_pixels(point_cloud) };
	int height{ k4a_image_get_height_pixels(point_cloud) };

	pcl_point_cloud->resize(static_cast<size_t>(height) * width);
	pcl_point_cloud->width = width;
	pcl_point_cloud->height = height;

	const int16_t* buffer = (const int16_t*)k4a_image_get_buffer(point_cloud);

	long i{ 0 };

	// Convert to mm to m
	for (auto& point : *pcl_point_cloud)
	{
		point.x = buffer[i++] / 1000.0f;
		point.y = buffer[i++] / 1000.0f;
		point.z = buffer[i++] / 1000.0f;
		point.r = point.g = point.b = 255;
	}
}

void azure::pcl::concatenate(PointCloud::ConstPtr cloud1, PointCloud::ConstPtr cloud2, PointCloud::Ptr concatenated)
{
	concatenated->resize(cloud1->size() + cloud2->size());

	size_t j{};

	for (size_t i = 0; i < cloud1->size(); i++)
	{
		concatenated->at(j++) = cloud1->at(i);
	}

	for (size_t i = 0; i < cloud2->size(); i++)
	{
		concatenated->at(j++) = cloud2->at(i);
	}
}