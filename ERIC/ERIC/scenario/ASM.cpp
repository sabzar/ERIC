
#include <scenario\ASM.h>

#include <iostream>
#include <time.h>


#include "rda\io\io.h"
#include "rda\io\vizualization.h"
#include <rda\common\common.h>
#include "rda\filtering.h"
#include "rda\segmentation\euclidean.h"
#include <rda\rangefinder.h>

#include <rda\function\ApproximateMethod.h>
#include <rda\function\LeastSquares.h>

ASM::ASM(int argc, char* argv[]){

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
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_2 (new pcl::PointCloud<pcl::PointXYZ> );

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
		int reduce_median_window = atof(rda::Console::getParam("-reduce_median_window").c_str());

		clock_t filter_clock;
		filter_clock = clock();

		// statistical filter
		std::vector<int> filtered_dists_indexes;
		//rda::statisticalDistanceFilter(distances, kN, threshold, filtered_dists_indexes); 
		rda::reduce_median_filter(distances, reduce_median_window, filtered_dists_indexes);

		rda::computePointCloud(rob_points, distances, filtered_dists_indexes, cloud_filtered, sensor_id);

		for(int i = 0; i < filtered_dists_indexes.size(); i++){
			filtered_dist_cloud->push_back(rda::Point(filtered_dists_indexes[i], distances[filtered_dists_indexes[i]], 0));
		}
		
		//rda::statisticalFilter(cloud, *cloud_filtered_2, 3, 1);

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
		
		#pragma region Partitioning

		double max_dist = atof(rda::Console::getParam("-max_dist").c_str());

		clock_t partitioning_clock;
		partitioning_clock = clock();

		std::vector<rda::CloudPart> clusters_patrs;		

		std::vector<rda::Line> min_clusters_patrs;
		std::vector<rda::CloudPart> min_clusters_parts_lines;

		std::vector<std::vector<rda::ApproximiedCloudPart>> approximied_parts;
		std::vector<std::vector<rda::ApproximiedCloudPart>> merged_cluster;

		double rdp_eps = atof(rda::Console::getParam("-rdp_eps").c_str());
		std::cout << "\t rdd_eps = " << rdp_eps << std::endl;

		double merge_dist = atof(rda::Console::getParam("-merge_dist").c_str());
		double merge_angle = atof(rda::Console::getParam("-merge_angle").c_str());
		int min_part_size = atoi(rda::Console::getParam("-min_part_size").c_str());
		double min_rdp_eps = atof(rda::Console::getParam("-min_rdp_eps").c_str());
		
		for(int i = 0; i < clusters.size(); i++){					

			std::vector<rda::CloudPart> parts;			

			std::vector<rda::Line> min_partition_lines;
			std::vector<rda::CloudPart> min_parts;

			std::vector<rda::ApproximiedCloudPart> approximied;
			std::vector<rda::ApproximiedCloudPart> merged;

			rda::monotonePartitioning(clusters[i], max_dist, min_part_size, parts);

			//rda::lineSegmentation(parts, rdp_eps, min_parts);
			//Adaptive beta
			rda::adaptiveLineSegmentation(parts, 10, min_rdp_eps, min_parts); // make variables
			rda::lsLineApproximation(min_parts, approximied);
			rda::segmentsMerging(approximied, merge_dist, merge_angle, merged);

			for(int i = 0; i < parts.size();i++)
				clusters_patrs.push_back(parts[i]);

			for(int i = 0; i < min_parts.size();i++)
				min_clusters_patrs.push_back(rda::Line(min_parts[i].first_point(), min_parts[i].last_point()));

			approximied_parts.push_back(approximied);
			merged_cluster.push_back(merged);
		}


		std::cout << "Partitioning time :" <<  ((float)(clock() - partitioning_clock)) / CLOCKS_PER_SEC << "sec" << std::endl;

		std::cout << "Approximation : " << std::endl;
		std::cout << "\t Lengths : "  << std::endl;
		for(int i = 0; i < approximied_parts.size(); i++){
			std::cout << "\t cluster " << i  << " :" << std::endl;
			for(int j = 0; j < approximied_parts[i].size(); j++ ){
				std::cout << "\t " << approximied_parts[i][j].approx_line().length()  << " mm |";
				std::cout << " ( " << approximied_parts[i][j].approx_line().start().x  << " " << approximied_parts[i][j].approx_line().start().y << ")";
				std::cout << " ( " << approximied_parts[i][j].approx_line().end().x  << " " << approximied_parts[i][j].approx_line().end().y << ")" << std::endl;
			}
		}

		std::cout << "Merging : " << std::endl;
		std::cout << "\t Lengths : "  << std::endl;
		for(int i = 0; i < merged_cluster.size(); i++){
			std::cout << "\t cluster " << i  << " :" << std::endl;
			for(int j = 0; j < merged_cluster[i].size(); j++ ){
				std::cout << "\t " << merged_cluster[i][j].approx_line().length()  << " mm |";
				std::cout << " ( " << merged_cluster[i][j].approx_line().start().x  << " " << merged_cluster[i][j].approx_line().start().y << ")";
				std::cout << " ( " << merged_cluster[i][j].approx_line().end().x  << " " << merged_cluster[i][j].approx_line().end().y << ")" << std::endl;
			}
		}

		#pragma endregion		

		#pragma region Vizualization 

		rda::Vizualizer::init(&argc, argv);	
		rda::Vizualizer v;	
		rda::Vizualizer v2;	
		rda::Vizualizer v3;	
		rda::Vizualizer v4;	
		rda::Vizualizer v5;	
		rda::Vizualizer v6;
		rda::Vizualizer v7;
		rda::Vizualizer v8;

		rda::Vizualizer v9;

		v.createWindow("raw", 700, 700, 20, 20);		
		v2.createWindow("minimized_ramer", 700, 700, 640, 20);
		v3.createWindow("approximied_rumer", 700, 700, 640, 20);
		v4.createWindow("merged", 700, 700, 20, 20);
		v5.createWindow("distances", 700, 700, 20, 20);
		v6.createWindow("filtered", 700, 700, 640, 20);
		v7.createWindow("euclidean_segmentation", 700, 700, 640, 20);
		v8.createWindow("monotone_segmentation", 700, 700, 640, 20);		
		v9.createWindow("statistical filter test", 700, 700, 640, 20);		

		v.addCloud(cloud, rda::CIRCLES, 0.0f, 0.0f, 0.0f, 0.5f);	

		
		//v5.addCloud(med_dist_cloud, rda::LINE_STRIP, 0.0, 0.0, 1.0, 1.0);
		//v5.addCloud(dist_cloud, rda::POINTS, 0.0f, 0.0f, 1.0f, 1.0f);						
		//v5.addCloud(ls_dist_cloud, rda::POINTS, 0.0, 1.0, 0.0, 1.0);

		/*for(int i = 0; i < dist_lines.size(); i++)
			v5.addCloud(dist_lines[i], rda::LINE_STRIP, 0.0f, 0.0f, 1.0f, 1.0f);*/		
		
			
		
		v2.addClouds(min_clusters_patrs, rda::LINE_STRIP, 1.0f, 15.0f);
		v2.addCloud(cloud_filtered, rda::CIRCLES, 0.3f, 0.3f, 0.3f, 0.2f);	
		//v2.addCloud(cloud, rda::POINTS, 1.0f, 1.0f, 1.0f, 0.5f);		

		v3.addCloud(cloud, rda::CIRCLES, 0.3f, 0.3f, 0.3f, 0.2f);	
		for(int i = 0; i < approximied_parts.size(); i++)
			v3.addClouds(approximied_parts.at(i), rda::LINES, 1.0f, 10.0f);

		v4.addCloud(cloud, rda::CIRCLES, 0.3f, 0.3f, 0.3f, 0.3f);		
		for(int i = 0; i < merged_cluster.size(); i++)
			v4.addClouds(merged_cluster.at(i), rda::LINES, 1.0f);

		v5.addCloud(dist_cloud, rda::LINE_STRIP, 0.3f, 0.3f, 1.0f, 1.0f);
		v5.addCloud(dist_cloud, rda::POINTS, 0.0f, 0.0f, 0.0f, 1.0f, 4.0f);	
		v5.addCloud(filtered_dist_cloud, rda::CIRCLES, 0.0f, 1.0f, 0.0f, 1.0f);	

		v6.addCloud(cloud_filtered, rda::CIRCLES, 0.0f, 0.0f, 0.0f, 0.5f);	

		v7.addClouds(clusters, rda::CIRCLES, 1.0f);

		v8.addClouds(clusters_patrs, rda::CIRCLES, 0.5f);
		v8.addClouds(clusters_patrs, rda::LINE_STRIP, 0.5f);

		v9.addCloud(cloud_filtered_2, rda::POINTS, 0, 0, 1, 1);

		rda::Vizualizer::start();	
		
#pragma endregion 

	}
	catch(rda::RdaException& e){

		std::cout << e.what() << std:: endl;
	}
}