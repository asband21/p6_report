#pragma once

#include <vector>
#include <array>
#include <memory>
#include <functional>
#include <optional> // Include for std::optional
#include <random> // Include for std::mt19937
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>

namespace ICPUtils {
    extern std::mt19937 random_generator; // Expose random generator

    Eigen::Matrix4f QuaternionToMatrix(const std::array<double, 4>& quaternion, bool normalize = true);
    std::vector<std::array<double, 4>> GenerateSuperFibonacciQuaternions(int count);
    void ApplyRandomTranslation(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>& cloud, float max_translation);
    void ApplyVoxelGridFilter(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>& cloud, double leaf_size = 0.005f);
    std::optional<std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>> LoadPointCloudFromCSV(const std::string& file_path);
    void PrintMatrixWithOperation(const Eigen::Matrix4f& matrix1, const Eigen::Matrix4f& matrix2, const std::function<double(double, double)>& operation);
    double ComputeRelativeDifference(double value1, double value2);
}
