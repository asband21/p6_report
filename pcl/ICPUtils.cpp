#include "ICPUtils.h"
#include <cmath>
#include <iostream>
#include <fstream>
#include <sstream>
#include <functional>
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <filesystem>
#include <optional>

namespace ICPUtils {
    std::mt19937 random_generator(std::random_device{}()); // Define random generator

    // Converts a quaternion to a 4x4 rotation matrix
    Eigen::Matrix4f QuaternionToMatrix(const std::array<double, 4>& quaternion, bool normalize) {
        double qx, qy, qz, qw, n;
        n = normalize ? 1.0f / std::sqrt(quaternion[0] * quaternion[0] + quaternion[1] * quaternion[1] + quaternion[2] * quaternion[2] + quaternion[3] * quaternion[3]) : 1;
        qw = n * quaternion[0]; qx = n * quaternion[1]; qy = n * quaternion[2]; qz = n * quaternion[3];

        Eigen::Matrix4f rotation_matrix = Eigen::Matrix4f::Identity();
        rotation_matrix(0, 0) = 1.0f - 2.0f * qy * qy - 2.0f * qz * qz;
        rotation_matrix(0, 1) = 2.0f * qx * qy - 2.0f * qz * qw;
        rotation_matrix(0, 2) = 2.0f * qx * qz + 2.0f * qy * qw;
        rotation_matrix(1, 0) = 2.0f * qx * qy + 2.0f * qz * qw;
        rotation_matrix(1, 1) = 1.0f - 2.0f * qx * qx - 2.0f * qz * qz;
        rotation_matrix(1, 2) = 2.0f * qy * qz - 2.0f * qx * qw;
        rotation_matrix(2, 0) = 2.0f * qx * qz - 2.0f * qy * qw;
        rotation_matrix(2, 1) = 2.0f * qy * qz + 2.0f * qx * qw;
        rotation_matrix(2, 2) = 1.0f - 2.0f * qx * qx - 2.0f * qy * qy;
        return rotation_matrix;
    }

    // Generates a list of unit quaternions using the super Fibonacci method
    std::vector<std::array<double, 4>> GenerateSuperFibonacciQuaternions(int count) {
        std::vector<std::array<double, 4>> quaternions(count);
        double dn = 1.0 / count;
        double mc0 = 1.0 / std::sqrt(2.0);
        double mc1 = 1.0 / 1.533751168755204288118041;

        for (int i = 0; i < count; ++i) {
            double s = i + 0.5;
            double ab = 2.0 * M_PI * s;
            double alpha = ab * mc0;
            double beta = ab * mc1;
            s *= dn;
            double r = std::sqrt(s);
            double R = std::sqrt(1.0 - s);
            quaternions[i] = { r * std::sin(alpha), r * std::cos(alpha), R * std::sin(beta), R * std::cos(beta) };
        }
        return quaternions;
    }

    // Applies a random translation to each point in the point cloud
    void ApplyRandomTranslation(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>& cloud, float max_translation) {
        std::uniform_real_distribution<float> distribution(-max_translation, max_translation);
        for (auto& point : *cloud) {
            point.x += distribution(random_generator);
            point.y += distribution(random_generator);
            point.z += distribution(random_generator);
        }
    }

    // Applies a voxel grid filter to downsample the point cloud
    void ApplyVoxelGridFilter(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>& cloud, double leaf_size) {
        pcl::PCLPointCloud2::Ptr pcl_cloud(new pcl::PCLPointCloud2());
        pcl::PCLPointCloud2::Ptr filtered_cloud(new pcl::PCLPointCloud2());

        pcl::toPCLPointCloud2(*cloud, *pcl_cloud);
        pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
        sor.setInputCloud(pcl_cloud);
        sor.setLeafSize(leaf_size, leaf_size, leaf_size);
        sor.filter(*filtered_cloud);
        pcl::fromPCLPointCloud2(*filtered_cloud, *cloud);
        std::cout << "Voxel grid filter applied with leaf size: " << leaf_size << std::endl;
    }

    // Loads a point cloud from a CSV file
    std::optional<std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>> LoadPointCloudFromCSV(const std::string& file_path) {
        if (!std::filesystem::exists(file_path)) {
            std::cerr << "Error: File could not be found: " << file_path << std::endl;
            return std::nullopt;
        }

        std::ifstream file(file_path);
        auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        std::string line;
        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string item;
            pcl::PointXYZ point;
            std::getline(ss, item, ','); point.x = std::stof(item);
            std::getline(ss, item, ','); point.y = std::stof(item);
            std::getline(ss, item, ','); point.z = std::stof(item);
            cloud->points.push_back(point);
        }
        cloud->width = cloud->points.size();
        cloud->height = 1;
        cloud->is_dense = false;

        std::cout << "Loaded " << cloud->width << " points from " << file_path << std::endl;
        return cloud;
    }

    // Applies a specified operation to two matrices and prints the result
    void PrintMatrixWithOperation(const Eigen::Matrix4f& matrix1, const Eigen::Matrix4f& matrix2, const std::function<double(double, double)>& operation) {
        for (int j = 0; j < 4; ++j) {
            for (int k = 0; k < 4; ++k) {
                std::cout << operation(matrix1(j, k), matrix2(j, k)) << ",";
            }
        }
        std::cout << "    ,";

        for (int j = 0; j < 4; ++j) {
            for (int k = 0; k < 4; ++k) {
                std::cout << matrix1(j, k) << ",";
            }
        }
        std::cout << "    ,";

        for (int j = 0; j < 4; ++j) {
            for (int k = 0; k < 4; ++k) {
                std::cout << matrix2(j, k) << ",";
            }
        }
        std::cout << "\n";
    }

    // Computes the relative difference between two values
    double ComputeRelativeDifference(double value1, double value2) {
        return std::abs(value1 - value2) / value1;
    }
}
