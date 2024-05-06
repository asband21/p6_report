#include <thread>
#include <chrono>

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

pcl::PointCloud<pcl::PointXYZ> lode_from_csv(char *file_pach)
{
	
	//open file pafh
	FILE *fs;
	if( !(fs = fopen(file_pach,"r")))
		exit(2);
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

void visualizePointCloud_4(pcl::PointCloud<pcl::PointXYZ>& cloud) {
	// Check if the cloud is empty
	if (cloud.empty())
	{
		std::cerr << "The point cloud is empty, cannot visualize." << std::endl;
		return;
	}

	// Check for minimum size
	if (cloud.size() < 3)
	{
		std::cerr << "Not enough points for meaningful visualization." << std::endl;
		return;
	}

	// Check for invalid data
	for (const auto& point : cloud.points)
	{
		if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
		{
			std::cerr << "Point cloud contains invalid points." << std::endl;
			return;
		}
	}

	// Proceed with visualization
	pcl::visualization::PCLVisualizer viewer("Point Cloud Viewer");
	viewer.setBackgroundColor(0, 0, 0);
	viewer.addPointCloud<pcl::PointXYZ>(cloud.makeShared(), "sample cloud");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
	viewer.addCoordinateSystem(1.0);

	while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
	}
}

void visualizePointCloud(pcl::PointCloud<pcl::PointXYZ>& cloud) {
    pcl::visualization::PCLVisualizer viewer("Point Cloud Viewer");
    viewer.setBackgroundColor(0, 0, 0);
    viewer.addPointCloud<pcl::PointXYZ>(cloud.makeShared(), "sample cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer.addCoordinateSystem(1.0);

    // Blocks here until the user closes the window
    viewer.spin();
}

void visualizePointCloud_5(pcl::PointCloud<pcl::PointXYZ>& cloud)
{
	pcl::visualization::PCLVisualizer viewer("Simple Point Cloud Viewer");
	viewer.addPointCloud<pcl::PointXYZ>(cloud.makeShared());
	//viewer.spinOnce(1000);
	while (!viewer.wasStopped())
	{
		viewer.spin(100);
	}
	/*
	*/
	int i = 0; scanf("%i",&i);
}

void visualizePointCloud_2(pcl::PointCloud<pcl::PointXYZ>& cloud)
{
	pcl::visualization::PCLVisualizer viewer("Point Cloud Viewer");

	viewer.setBackgroundColor(0, 0, 0);
	viewer.addPointCloud<pcl::PointXYZ>(cloud.makeShared(), "sample cloud");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
	viewer.addCoordinateSystem(1.0);
	
	int i = 0; scanf("%i",&i);

	while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
}

int main ()
{
	pcl::PointCloud<pcl::PointXYZ> cloud;
	// Fill in the cloud data
	cloud.width    = 5;
	cloud.height   = 1;
	cloud.is_dense = false;
	cloud.resize (cloud.width * cloud.height);
	for (auto& point: cloud)
	{
		point.x = 1024 * rand () / (RAND_MAX + 1.0f);
		point.y = 1024 * rand () / (RAND_MAX + 1.0f);
		point.z = 1024 * rand () / (RAND_MAX + 1.0f);
	}
	pcl::io::savePCDFileASCII ("test_pcd.pcd", cloud);
	std::cerr << "Saved " << cloud.size () << " data points to test_pcd.pcd." << std::endl;
	for (const auto& point: cloud)
		std::cerr << "    " << point.x << " " << point.y << " " << point.z << std::endl;
	
	char sti[] = "./pungsky.csv";
	cloud = lode_from_csv(sti);
	
	/*
	for (const auto& point: cloud)
		std::cerr << "    " << point.x << " " << point.y << " " << point.z << std::endl;
	*/
	visualizePointCloud(cloud);

	return (0);
}	
