#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/Dense>
#include <memory>
#include "ICPUtils.h"

// Structure to hold the result of ICP
struct ICPResult {
    Eigen::Matrix4f transformation;
    bool converged;
    double fitness_score;
};

// Class to perform ICP and multi-ICP operations
class ICP {
public:
    ICPResult PerformICP(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>& source_cloud, const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>& target_cloud, bool verbose = false);
    Eigen::Matrix4f PerformMultiICP(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>& source_cloud, const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>& target_cloud, int angle_count);
};

