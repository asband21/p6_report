#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h> // Include for transformPointCloud
#include <random> // Include for random number generation
#include "ICP.h"
#include "ICPUtils.h"

// Default values for leaf size and translation
constexpr float DEFAULT_LEAF_SIZE = 0.005f;
constexpr float MAX_LEAF_SIZE_VARIATION = 0.02f;
constexpr float MAX_TRANSLATION = 0.1f;

// Function to run ICP tests with varying parameters
void RunICPTests(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>& scan_cloud, int test_runs) {
    ICP icp;

    for (int i = 0; i < test_runs; ++i) {
        // Adjust point density with varying leaf size
        std::uniform_real_distribution<float> dist_leaf_size(0, MAX_LEAF_SIZE_VARIATION);
        float leaf_size = DEFAULT_LEAF_SIZE + dist_leaf_size(ICPUtils::random_generator);
        auto modified_scan = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(*scan_cloud);
        ICPUtils::ApplyVoxelGridFilter(modified_scan, leaf_size);

        // Apply random translation
        ICPUtils::ApplyRandomTranslation(modified_scan, MAX_TRANSLATION);

        // Generate random transformation
        Eigen::Matrix4f random_transformation = ICPUtils::QuaternionToMatrix(ICPUtils::GenerateSuperFibonacciQuaternions(1)[0]);
        std::uniform_real_distribution<float> dist_translation(0, 2);
        random_transformation(0, 3) = dist_translation(ICPUtils::random_generator);
        random_transformation(1, 3) = dist_translation(ICPUtils::random_generator);
        random_transformation(2, 3) = dist_translation(ICPUtils::random_generator);

        auto transformed_scan = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        pcl::transformPointCloud(*modified_scan, *transformed_scan, random_transformation);
        Eigen::Matrix4f icp_transformation = icp.PerformMultiICP(modified_scan, transformed_scan, 20);

        // Print matrices and differences using the function pointer
        ICPUtils::PrintMatrixWithOperation(random_transformation, icp_transformation, ICPUtils::ComputeRelativeDifference);
    }
}

int main(int argc, char* argv[]) {
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <path_to_csv> <test_runs>" << std::endl;
        return 1;
    }

    const std::string csv_file_path = argv[1];
    int test_runs = std::stoi(argv[2]);

    auto scan_cloud_opt = ICPUtils::LoadPointCloudFromCSV(csv_file_path);
    if (!scan_cloud_opt) {
        return 1;
    }

    auto scan_cloud = scan_cloud_opt.value();

    std::cout << "\n-----------\n";

    ICPUtils::ApplyVoxelGridFilter(scan_cloud, DEFAULT_LEAF_SIZE);
    RunICPTests(scan_cloud, test_runs);

    return 0;
}
