#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <thread>
#include <chrono>


pcl::PointCloud<pcl::PointXYZ> load_from_csv(char *file_pach)
{
        
        //open file pafh
        FILE *fs;
        if( !(fs = fopen(file_pach,"r")))
	{
		fprintf(stderr,"kunne ikke finde \"%s\"",file_pach);
                exit(2);
	}
		int point_coundt = 0;
        double x, y, z;
        while (EOF != fscanf(fs,"%lf,%lf,%lf",&x, &y, &z))
                point_coundt++;
        fclose(fs);     
        printf("%d antal pungter\n", point_coundt);

        pcl::PointCloud<pcl::PointXYZ> cloud;
        cloud.width    = point_coundt;
        cloud.height   = 1;
        cloud.is_dense = false;
        cloud.resize (cloud.width * cloud.height);
        
        if( !(fs = fopen(file_pach,"r")))
                exit(2);
        for (auto& point: cloud)
                if(EOF == fscanf(fs,"%f,%f,%f",&(point.x), &(point.y), &(point.z)))
                        exit(2);
        fclose(fs);     
        return cloud;

}

Eigen::Matrix4f performICP(const pcl::PointCloud<pcl::PointXYZ>::Ptr &source, const pcl::PointCloud<pcl::PointXYZ>::Ptr &target)
{
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setInputSource(source);
	icp.setInputTarget(target);
	// Set the max correspondence distance to 5cm (e.g., correspondences with higher
	// distances will be ignored)
	icp.setMaxCorrespondenceDistance (0.005);
	// Set the maximum number of iterations (criterion 1)
	icp.setMaximumIterations (5000);
	// Set the transformation epsilon (criterion 2)
	icp.setTransformationEpsilon (1e-8);
	// Set the euclidean distance difference epsilon (criterion 3)
	icp.setEuclideanFitnessEpsilon (1);

	pcl::PointCloud<pcl::PointXYZ> Final;
	icp.align(Final);
	std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
	std::cout << icp.getFinalTransformation() << std::endl;
	return icp.getFinalTransformation();
}

int main()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cad_model(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr scan(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr scan_2(new pcl::PointCloud<pcl::PointXYZ>);

	char sti_ref[] = "./pungsky.csv";
	char sti_scan[] = "./skan_RT_pungsky_med_dobbler.csv";
	*cad_model = load_from_csv(sti_ref);
	*scan = load_from_csv(sti_scan);
	printf("\n-----------\n");

	/*
	 */

	Eigen::Matrix4f transformation = performICP(scan, cad_model);
	
	pcl::transformPointCloud (*scan, *scan_2, transformation);

	// Optionally visualize the result
	pcl::visualization::PCLVisualizer viewer("ICP demo");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_color(scan, 255, 0, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_2_color(scan_2, 0, 0, 255);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color(cad_model, 0, 255, 0);
	viewer.addPointCloud(scan, source_color, "source");
	viewer.addPointCloud(scan_2, source_2_color, "source_2");
	viewer.addPointCloud(cad_model, target_color, "target");
	//pcl::visualization::PCLVisualizer viewer("3D Viewer");

	while (!viewer.wasStopped())
	{
		viewer.spinOnce(10000000);
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
	/*
	while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
	*/

	return 0;
}

