#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

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
	
	for (const auto& point: cloud)
		std::cerr << "    " << point.x << " " << point.y << " " << point.z << std::endl;

	return (0);
}	
