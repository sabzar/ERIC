
#include <scenario\LSNurbs.h>

#include "rda\io\io.h"
#include "rda\io\vizualization.h"
#include <rda\common\common.h>
#include "rda\filtering.h"
#include "rda\segmentation\euclidean.h"
#include <rda\rangefinder.h>

#include <rda\nurbs.h>

LSNurbs::LSNurbs(int argc, char* argv[]){

	std::cout << std::endl;

	std::vector<double> distances;	
	std::vector<double> med_distances;	
	std::vector<rda::RPoint> rob_points;

	rda::cloudPtr dist_cloud (new rda::cloud);
	rda::cloudPtr med_dist_cloud (new rda::cloud);
	rda::cloudPtr ls_dist_cloud (new rda::cloud);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> );	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2 (new pcl::PointCloud<pcl::PointXYZ> );	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ> );

	rda::cloudPtr filtered_dist_cloud (new rda::cloud);
	
	rda::Console::readArgs(argc, argv);	

	try{
		int sensor_id = 0;
		double rot_angle = atof(rda::Console::getParam("-rot_angle").c_str());
		//cloud = rda::readScene(rda::Console::getParam("-file"), distances);	
		rda::readScene(rda::Console::getParam("-file"), distances, rob_points, sensor_id);
		std::cout << "Input points number : " << rob_points.size() << std::endl;
		int sma_window_size = atoi(rda::Console::getParam("-sma_window_size").c_str());


		rda::computePointCloud(rob_points, distances, cloud, sensor_id);
		//rda::computePointCloud(rob_points, med_distances, cloud_2, 1);

		for(int i = 0; i < distances.size(); i++){
			dist_cloud->push_back(rda::Point(i, distances[i], 0));					
		}
		
		#pragma region Filteration
		
		int kN = atof(rda::Console::getParam("-filter_kN").c_str());
		double threshold = atof(rda::Console::getParam("-filter_threashold").c_str());

		clock_t filter_clock;
		filter_clock = clock();

		std::vector<int> filtered_dists_indexes;
		rda::statisticalDistanceFilter(distances, kN, threshold, filtered_dists_indexes); 

		rda::computePointCloud(rob_points, distances, filtered_dists_indexes, cloud_filtered, sensor_id);

		for(int i = 0; i < filtered_dists_indexes.size(); i++){
			filtered_dist_cloud->push_back(rda::Point(filtered_dists_indexes[i], distances[filtered_dists_indexes[i]], 0));
		}
		
		//rda::statisticalFilter(dist_cloud, *filtered_dist_cloud, kN, threshold);

		std::cout << "Filtration time :" <<  ((float)(clock() - filter_clock)) / CLOCKS_PER_SEC << "sec" << std::endl;
		std::cout << "\t k-Neighbours : " << kN << std::endl;
		std::cout << "\t threshold : " << threshold << std::endl;

		#pragma endregion

		#pragma region Clustering Euclidean
		
		clock_t segm_clock;
		segm_clock = clock();

		double eps = atof(rda::Console::getParam("-eps").c_str());
		int minPts = atoi(rda::Console::getParam("-minPts").c_str());

		std::vector<rda::cloudPtr> clusters;
		rda::euclideanClusterExctraction(cloud_filtered, clusters, eps, minPts, 999999);		

		std::cout << "Clustering Euclidean time :" <<  ((float)(clock() - segm_clock)) / CLOCKS_PER_SEC << "sec" << std::endl;
		std::cout << "\t clusters : " << clusters.size() <<  std::endl;
		
		#pragma endregion 

		

	}
	catch(rda::RdaException& e){
		std::cout << e.what() << std:: endl;
	}
}