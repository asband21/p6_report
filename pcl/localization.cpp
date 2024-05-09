#include <math.h>
#include <thread>
#include <chrono>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>

pcl::PointCloud<pcl::PointXYZ> point_on_shver(int number_cound)
{
        pcl::PointCloud<pcl::PointXYZ> cloud;
	cloud.width    = number_cound;
	cloud.height   = 1;
	cloud.is_dense = false;
	cloud.resize (cloud.width * cloud.height);

	double phi = M_PI * ( std::sqrt(5) -1);
	int i = 1;

	for (auto& point: cloud)
	{
		point.x = 1.0-(i/((double)number_cound - 1.0))*2;
		double radius = std::sqrt(1-point.x*point.x);
		point.y = std::cos(phi*i)*radius;
		point.z = std::sin(phi*i)*radius;
		i++;
	}
	return cloud;
}


std::vector<std::array<double,4>> super_fib_list(int n)
{
	std::vector<std::array<double,4>> Q(n);

	double dn = 1.0 / (double)n;
	double mc0 = 1.0 / sqrt(2.0);
	double mc1 = 1.0 / 1.533751168755204288118041;

	for (int i = 0; i < n; i++)
	{
		double s = (double)i+0.5;
		double ab = 2.0 * M_PI * s;
		double alpha = ab * mc0;
		double beta = ab * mc1;
		s *= dn;
		double r = sqrt(s);
		double R = sqrt(1.0-s);
		Q[i][0] = r*sin(alpha);
		Q[i][1] = r*cos(alpha);
		Q[i][2] = R*sin(beta);
		Q[i][3] = R*cos(beta);
	}
	return Q;
}

struct qrt_srt
{
	double x;
	double y;
	double z;
	double w;
};
//Generating n samples on SO(3) asunit quaternions
struct qrt_srt Super_Fibonacci(int i, int n)
{
	//constands
	double p = sqrt(2);
	double l = 1.533751168755204288118041;

	double s = i + 0.5;
	double t = s/n;
	double d = 2*M_PI*s;
	double r = std::sqrt(t);
	double R = std::sqrt(1-t);
	double a = d/p;
	double b = d/l;
	struct qrt_srt qur = {r*std::sin(a), r*std::cos(a), R*std::sin(b), R*std::cos(b)};
	return qur;
}

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

void VoxelGrid_fjerndubel(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double grid_size = 0.001f)
{
    pcl::PCLPointCloud2::Ptr cloud2(new pcl::PCLPointCloud2());
    pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());

    // Convert from PointXYZ to PCLPointCloud2
    pcl::toPCLPointCloud2(*cloud, *cloud2);

    // Create the voxel grid filter
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(cloud2);
    sor.setLeafSize(grid_size, grid_size, grid_size);
    sor.filter(*cloud_filtered);

    // Convert back from PCLPointCloud2 to PointXYZ
    pcl::fromPCLPointCloud2(*cloud_filtered, *cloud);
    std::cout << "Voxel grid filter applied with leaf size: " << grid_size << std::endl;
}






Eigen::Matrix4f performICP(const pcl::PointCloud<pcl::PointXYZ>::Ptr &source, const pcl::PointCloud<pcl::PointXYZ>::Ptr &target)
{
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setInputSource(source);
	icp.setInputTarget(target);
	
	// Set the max correspondence distance to 5cm (e.g., correspondences with higher
	// distances will be ignored)
	icp.setMaxCorrespondenceDistance (17.5);
	// Set the maximum number of iterations (criterion 1)
	icp.setMaximumIterations (50000);
	// Set the transformation epsilon (criterion 2)
	icp.setTransformationEpsilon (1e-8);
	// Set the euclidean distance difference epsilon (criterion 3)
	icp.setEuclideanFitnessEpsilon (0.01);
	/*
	*/
	
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

	pcl::PointCloud<pcl::PointXYZ>::Ptr rota(new pcl::PointCloud<pcl::PointXYZ>);
	*rota = point_on_shver(900);

	char sti_ref[] = "./pungsky.csv";
	char sti_scan[] = "./skan_RT_pungsky_med_dobbler.csv";
	*cad_model = load_from_csv(sti_ref);
	*scan = load_from_csv(sti_scan);
	printf("\n-----------\n");

	/*
	 */
	VoxelGrid_fjerndubel(scan, 0.005f);
	VoxelGrid_fjerndubel(cad_model, 0.005f);



	Eigen::Matrix4f transformation = performICP(scan, cad_model);
	
	pcl::transformPointCloud (*scan, *scan_2, transformation);

	// Optionally visualize the result
	pcl::visualization::PCLVisualizer viewer("ICP demo");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_color(scan, 255, 0, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_2_color(scan_2, 0, 0, 255);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rota_color(rota, 200, 200, 20);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color(cad_model, 0, 255, 0);
	viewer.addPointCloud(scan, source_color, "source");
	viewer.addPointCloud(scan_2, source_2_color, "source_2");
	viewer.addPointCloud(cad_model, target_color, "target");
	viewer.addPointCloud(rota, rota_color, "rota");

	while (!viewer.wasStopped())
	{
		viewer.spinOnce(10000000);
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
	return 0;
}

