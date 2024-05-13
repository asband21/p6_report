#include "ICP.h"
#include <pcl/registration/icp.h>

// Perform Iterative Closest Point (ICP) on two point clouds
ICPResult ICP::PerformICP(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>& source_cloud, const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>& target_cloud, bool verbose) {
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(source_cloud);
    icp.setInputTarget(target_cloud);
    icp.setMaxCorrespondenceDistance(17.5);
    icp.setMaximumIterations(5000);
    icp.setTransformationEpsilon(1e-8);
    icp.setEuclideanFitnessEpsilon(0.001);

    pcl::PointCloud<pcl::PointXYZ> aligned_cloud;
    icp.align(aligned_cloud);

    if (verbose) {
        std::cout << "has converged: " << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
        std::cout << icp.getFinalTransformation() << std::endl;
    }

    return { icp.getFinalTransformation(), icp.hasConverged(), icp.getFitnessScore() };
}

// Perform multiple ICP operations with different initial rotations and return the best result
Eigen::Matrix4f ICP::PerformMultiICP(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>& source_cloud, const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>& target_cloud, int angle_count) {
    if (angle_count <= 0 || angle_count > 50) {
        throw std::invalid_argument("Number of angles must be between 1 and 50");
    }

    auto rotations = ICPUtils::GenerateSuperFibonacciQuaternions(angle_count);
    std::vector<ICPResult> icp_results(angle_count);
    auto transformed_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    double min_score = std::numeric_limits<double>::max();
    int best_index = 0;

    for (int i = 0; i < angle_count; ++i) {
        pcl::transformPointCloud(*source_cloud, *transformed_cloud, ICPUtils::QuaternionToMatrix(rotations[i]));
        icp_results[i] = PerformICP(transformed_cloud, target_cloud);

        if (icp_results[i].fitness_score < min_score) {
            best_index = i;
            min_score = icp_results[i].fitness_score;
        }
    }

    return icp_results[best_index].transformation * ICPUtils::QuaternionToMatrix(rotations[best_index]);
}
