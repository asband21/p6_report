#include <math.h>
#include <thread>
#include <chrono>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <boost/shared_ptr.hpp>
#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <Eigen/Core>
#include <vector>
#include <array>
#include <limits>
#include <iostream>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <cstring>

#ifdef USE_BOOST
#include <pcl/memory.h>
#else
#include <boost/make_shared.hpp>
#endif

Eigen::Matrix4f quaternions_to_matrix(std::array<double,4> q, bool normalise = true)
{
        //q is a unit quaternion
	double qx, qy, qz, qw, n;
	
	n = normalise ? 1.0f/std::sqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]) : 1;
	qw = n*q[0];
	qx = n*q[1];
	qy = n*q[2];
	qz = n*q[3];

	Eigen::Matrix4f rot = Eigen::Matrix4f::Identity();
	rot(0,0) = 1.0f - 2.0f*qy*qy - 2.0f*qz*qz;
	rot(0,1) = 2.0f*qx*qy - 2.0f*qz*qw;
	rot(0,2) = 2.0f*qx*qz + 2.0f*qy*qw;

	rot(1,0) = 2.0f*qx*qy + 2.0f*qz*qw;
	rot(1,1) = 1.0f - 2.0f*qx*qx - 2.0f*qz*qz;
	rot(1,2) = 2.0f*qy*qz - 2.0f*qx*qw;

	rot(2,0) = 2.0f*qx*qz - 2.0f*qy*qw;
	rot(2,1) = 2.0f*qy*qz + 2.0f*qx*qw;
	rot(2,2) = 1.0f - 2.0f*qx*qx - 2.0f*qy*qy;
	return rot;
}
pcl::PointCloud<pcl::PointXYZ> point_on_sphere(int number_count)
{
	//https://marcalexa.github.io/superfibonacci/
        pcl::PointCloud<pcl::PointXYZ> cloud;
	cloud.width    = number_count;
	cloud.height   = 1;
	cloud.is_dense = false;
	cloud.resize (cloud.width * cloud.height);

	double phi = M_PI * ( std::sqrt(5) -1);
	int i = 1;

	for (auto& point: cloud)
	{
		point.x = 1.0-(i/((double)number_count - 1.0))*2;
		double radius = std::sqrt(1-point.x*point.x);
		point.y = std::cos(phi*i)*radius;
		point.z = std::sin(phi*i)*radius;
		i++;
	}
	return cloud;
}


	// fib for Fibbonachi https://marcalexa.github.io/superfibonacci/
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


//Generating n samples on SO(3) as unit quaternions
struct qrt_srt Super_Fibonacci(int i, int n)
{
	//constants
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

pcl::PointCloud<pcl::PointXYZ> load_from_csv(char *file_path)
{
        //open file path
        FILE *fs;
        if( !(fs = fopen(file_path,"r")))
        {
                fprintf(stderr,"\"%s\" Could not be found",file_path);
                exit(2);
        }

		// The amount of data in the file is recorded
        int point_count = 0;
        double x, y, z;
        while (EOF != fscanf(fs,"%lf,%lf,%lf",&x, &y, &z))
                point_count++;
        fclose(fs);     
        printf("Amount of points: %d\n", point_count);

        pcl::PointCloud<pcl::PointXYZ> cloud;
        cloud.width    = point_count;
        cloud.height   = 1;
        cloud.is_dense = false;
        cloud.resize (cloud.width * cloud.height);
        
	if( !(fs = fopen(file_path,"r")))
		exit(2);
		// Data is read from the file
	for (auto& point: cloud)
		if(EOF == fscanf(fs,"%f,%f,%f",&(point.x), &(point.y), &(point.z)))
			exit(2);
	fclose(fs);     
	return cloud;
}

void VoxelGrid_homogenise(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double grid_size = 0.001f, bool verbos = false)
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
	if(verbos)
        	std::cout << "Voxel grid filter applied with leaf size: " << grid_size << std::endl;
}

struct icp_return
{
	Eigen::Matrix4f trans_matrix;
	bool converged;
	double FitnessScore;
};

struct icp_return performICP(const pcl::PointCloud<pcl::PointXYZ>::Ptr &source, const pcl::PointCloud<pcl::PointXYZ>::Ptr &target, bool verbos = false)
{
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setInputSource(source);
	icp.setInputTarget(target);
	
	// Set the max correspondence distance to 5cm (e.g., correspondences with higher
	// distances will be ignored)
	icp.setMaxCorrespondenceDistance (17.5);
	// Set the maximum number of iterations (criterion 1)
	icp.setMaximumIterations (50);
	// Set the transformation epsilon (criterion 2)
	icp.setTransformationEpsilon (1e-8);
	// Set the euclidean distance difference epsilon (criterion 3)
	icp.setEuclideanFitnessEpsilon (0.01);
	/*
	*/
	
	pcl::PointCloud<pcl::PointXYZ> Final;
	icp.align(Final);
	if(verbos)
	{
		std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
		std::cout << icp.getFinalTransformation() << std::endl;
	}
	struct icp_return g;
	g.trans_matrix = icp.getFinalTransformation();
	g.converged = icp.hasConverged();
	g.FitnessScore = icp.getFitnessScore();
	return g;
}

Eigen::Matrix4f multi_ICP(const pcl::PointCloud<pcl::PointXYZ>::Ptr &source, const pcl::PointCloud<pcl::PointXYZ>::Ptr &target, int angles)
{
	if(angles < 0){ fprintf(stderr,"Number of angles must be greater than 0\n"); exit(2);}
	if(angles > 50){fprintf(stderr,"Number of angles must be less than 50\n Smack the horse, buddy\n"); exit(2);}
	
	std::vector<std::array<double,4>> rotations_Q = super_fib_list(angles);
	
	std::vector<struct icp_return> icp_lis(angles);
	pcl::PointCloud<pcl::PointXYZ>::Ptr T(new pcl::PointCloud<pcl::PointXYZ>);

	double min = std::numeric_limits<double>::max();
	double max = 0;
	int index = 0;
	for(int i = 0; i < angles; i++)
	{
		pcl::transformPointCloud (*source, *T,	quaternions_to_matrix(rotations_Q[i]));
		icp_lis[i] = performICP(T, target);
		if((icp_lis[i]).FitnessScore < min)
		{
			index = i;
			min = (icp_lis[i]).FitnessScore;
			//max = (icp_lis[i]).FitnessScore;
		}

	}
	Eigen::Matrix4f complete_transformation =(icp_lis[index]).trans_matrix * quaternions_to_matrix(rotations_Q[index]) ;
	return complete_transformation;
}


Eigen::Matrix4f multi_ICP_multicore(const pcl::PointCloud<pcl::PointXYZ>::Ptr& source, const pcl::PointCloud<pcl::PointXYZ>::Ptr& target, int angles)
{
	if (angles <= 0) {
		fprintf(stderr, "Number of angles must be greater than 0\n");
		exit(2);
	}
	if (angles > 50) {
		fprintf(stderr, "Number of angles must be less than 50\n Smack the horse, buddy\n");
		exit(2);
	}

	std::vector<std::array<double, 4>> rotations_Q = super_fib_list(angles);
	std::vector<icp_return> icp_lis(angles);
	pcl::PointCloud<pcl::PointXYZ>::Ptr T(new pcl::PointCloud<pcl::PointXYZ>);

	double min = std::numeric_limits<double>::max();
	int index = 0;

	const char* shm_name = "/icp_shm";
	int shm_fd = shm_open(shm_name, O_CREAT | O_RDWR, 0666);
	if (shm_fd == -1) {
		perror("shm_open");
		exit(1);
	}

	size_t shm_size = angles * sizeof(icp_return);
	if (ftruncate(shm_fd, shm_size) == -1) {
		perror("ftruncate");
		exit(1);
	}

	void* shm_ptr = mmap(0, shm_size, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
	if (shm_ptr == MAP_FAILED) {
		perror("mmap");
		exit(1);
	}

	icp_return* shared_data = static_cast<icp_return*>(shm_ptr);
	std::vector<pid_t> pids;

	for (int i = 0; i < angles; ++i) {
		pid_t pid = fork();
		if (pid == 0) {
			// Child process
			pcl::PointCloud<pcl::PointXYZ>::Ptr T_local(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::transformPointCloud(*source, *T_local, quaternions_to_matrix(rotations_Q[i]));
			icp_return result = performICP(T_local, target);
			memcpy(&shared_data[i], &result, sizeof(icp_return));
			munmap(shm_ptr, shm_size);
			close(shm_fd);
			exit(0);
		} else if (pid > 0) {
			// Parent process
			pids.push_back(pid);
		} else {
			perror("fork");
			exit(1);
		}
	}

	for (pid_t pid : pids) {
		waitpid(pid, nullptr, 0);
	}

	for (int i = 0; i < angles; ++i) {
		icp_lis[i] = shared_data[i];
		if (icp_lis[i].FitnessScore < min) {
			index = i;
			min = icp_lis[i].FitnessScore;
		}
	}

	Eigen::Matrix4f complete_transformation = icp_lis[index].trans_matrix * quaternions_to_matrix(rotations_Q[index]);

	munmap(shm_ptr, shm_size);
	close(shm_fd);
	shm_unlink(shm_name);

	return complete_transformation;
}

pcl::PointCloud<pcl::PointXYZ> c_arey_to_pcl_pc(int point_count, double *points, bool debug = false)
{

	//auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        pcl::PointCloud<pcl::PointXYZ> cloud;
        cloud.width    = point_count;
        cloud.height   = 1;
        cloud.is_dense = false;
        cloud.resize(cloud.width * cloud.height);
        int i = 0;
	int h;
	printf("point count %d i %d \n",point_count, i);
	for (int i = 0; i < point_count; ++i)
	{
		cloud.points[i].x = points[i * 3];
		cloud.points[i].y = points[i * 3 + 1];
		cloud.points[i].z = points[i * 3 + 2];
	}
	/*
	for (auto& point: cloud)
        {
		point.x = *(points);// +i*3);
		point.y = *(points);//+i*3+1);
		point.z = *(points+i*3+2);
                i++;
        }
	*/
	if(debug)
		printf("point count %d i %d \n",point_count, i);
	scanf("%d",&h);
        return cloud;
}
extern "C"
{
        void lokailasiens(double* output,int scan_count, double *scan, int cad_count, double *cad)
        {
		pcl::PointCloud<pcl::PointXYZ> cad_model_pcl;
		pcl::PointCloud<pcl::PointXYZ> scan_pcl;
                scan_pcl = c_arey_to_pcl_pc(scan_count, scan, true);
                cad_model_pcl = c_arey_to_pcl_pc(cad_count, cad, true);
		int h;
		scanf("%d",&h);
		//skal roter lokalt
		//auto scan_pcl_sprt = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(scan_pcl);
        	//auto cad_pcl_sprt = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(cad_model_pcl);
		//auto scan_pcl_sprt = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>(scan_pcl);
		//auto cad_pcl_sprt = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>(cad_model_pcl);
		
		printf(" In order to entrain the operation, this is a debug.");
#ifdef USE_BOOST
		auto scan_pcl_sprt = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>(scan_pcl);
		auto cad_pcl_sprt = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>(cad_model_pcl);
#else
		auto scan_pcl_sprt = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>(scan_pcl);
		auto cad_pcl_sprt = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>(cad_model_pcl);
#endif

		printf(" In order to entrain the operation, this is a debug.");
		VoxelGrid_homogenise(scan_pcl_sprt, 0.005f);
		VoxelGrid_homogenise(cad_pcl_sprt, 0.005f);
	        Eigen::Matrix4f transformation = multi_ICP(scan_pcl_sprt, cad_pcl_sprt, 16);
	        //Eigen::Matrix4f transformation = multi_ICP_multicore(scan_pcl_sprt, cad_pcl_sprt, 16);
                for (int i = 0; i < 4; i++)
                {
                        output[4*i + 0] = transformation(i ,0);
                        output[4*i + 1] = transformation(i ,1);
                        output[4*i + 2] = transformation(i ,2);
                        output[4*i + 3] = transformation(i ,3);
                }
        }
}

int main()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cad_model(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr scan(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr scan_2(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::PointCloud<pcl::PointXYZ>::Ptr rota(new pcl::PointCloud<pcl::PointXYZ>);
	*rota = point_on_sphere(900);

	char sti_ref[] = "./reference_pointcloud.csv";
	char sti_scan[] = "./scan_RT_pointcloud_w_duplicates.csv";
	*cad_model = load_from_csv(sti_ref);
	*scan = load_from_csv(sti_scan);
	printf("\n-----------\n");

	
	VoxelGrid_homogenise(scan, 0.005f);
	VoxelGrid_homogenise(cad_model, 0.005f);

	//Eigen::Matrix4f transformation = performICP(scan, cad_model);
	Eigen::Matrix4f transformation = multi_ICP(scan, cad_model, 16);
	
	pcl::transformPointCloud (*scan, *scan_2, transformation);

	// Optionally visualise the result
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

