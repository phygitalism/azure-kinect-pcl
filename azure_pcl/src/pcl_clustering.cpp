#include "pcl_clustering.h"

azure::PointCloud::Ptr azure::cluster::euclidean_clustering(azure::PointCloud::ConstPtr input_point_cloud)
{
    using namespace azure;

    // Read in the cloud data
    PointCloud::Ptr cloud_f(new PointCloud);
    PointCloud::Ptr cloud_filtered(new PointCloud);

    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::VoxelGrid<PointCloud::PointType> vg;
    vg.setInputCloud(input_point_cloud);
    vg.setLeafSize(0.01f, 0.01f, 0.01f);
    vg.setFilterFieldName("z");
    vg.setFilterLimits(0, 1.6);
    vg.filter(*cloud_filtered);

    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<PointCloud::PointType> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setAxis(Eigen::Vector3f{ 0, 1, 0 });
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(50);
    seg.setDistanceThreshold(0.01);
    seg.setEpsAngle(DEG2RAD(5));

    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud(cloud_filtered);
    seg.segment(*inliers, *coefficients);

    PointCloud::Ptr cloud_cluster(new PointCloud);

    pcl::ExtractIndices<PointCloud::PointType> extract;
    extract.setInputCloud(cloud_filtered);
    extract.setIndices(inliers);
    extract.filter(*cloud_cluster);
    
    extract.setNegative(true);
    //Remove the planar inliers, extract the rest
    extract.filter(*cloud_f);

    *cloud_filtered = *cloud_f;

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<PointCloud::PointType>::Ptr tree(new pcl::search::KdTree<PointCloud::PointType>);
    tree->setInputCloud(cloud_filtered);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointCloud::PointType> ec;
    ec.setClusterTolerance(0.02); // 2cm
    ec.setMinClusterSize(90);
    ec.setMaxClusterSize(30000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_filtered);
    ec.extract(cluster_indices);

    color::ColourManager colour_manager(0, cluster_indices.size() - 1);

    int j{};

    for (auto it = cluster_indices.cbegin(); it != cluster_indices.cend(); ++it)
    {
            for (auto pit = it->indices.cbegin(); pit != it->indices.cend(); ++pit)
            {
                const auto& point = cloud_filtered->points[*pit];
                auto color = colour_manager.getInterpolatedColour(j);
                PointCloud::PointType color_point{ static_cast<uint8_t>(color.getIntR()), static_cast<uint8_t>(color.getIntG()), static_cast<uint8_t>(color.getIntB()) };

                color_point.x = point.x;
                color_point.y = point.y;
                color_point.z = point.z;

                cloud_cluster->push_back(color_point);
            }
        j++;
    }

    return cloud_cluster;

}
