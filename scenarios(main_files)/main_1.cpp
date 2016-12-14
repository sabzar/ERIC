
/*
 * Ramer on distanses -> LeastSquares on parts (4th order) -> Recount into 2D
*/

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



int main(int argc, char* argv[]){

	std::cout << std::endl;

	std::vector<double> distances;	
	std::vector<rda::RPoint> rob_points;

	rda::cloudPtr dist_cloud (new rda::cloud);
	rda::cloudPtr ls_dist_cloud (new rda::cloud);
	rda::cloudPtr rdp_up_ls_dist_cloud (new rda::cloud); // y + some value 
	rda::cloudPtr rdp_down_ls_dist_cloud (new rda::cloud); // y - some value 

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> );	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2 (new pcl::PointCloud<pcl::PointXYZ> );	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ> );
	
	rda::Console::readArgs(argc, argv);	

	try{
		
		double rot_angle = atof(rda::Console::getParam("-rot_angle").c_str());
		//cloud = rda::readScene(rda::Console::getParam("-file"), distances);	
		rda::readScene(rda::Console::getParam("-file"), distances, rob_points);
		std::cout << "Input points number : " << rob_points.size() << std::endl;
		int sma_window_size = atoi(rda::Console::getParam("-sma_window_size").c_str());


		rda::computePointCloud(rob_points, distances, cloud);

#pragma region Distances		
		
		for(int i = 0; i < distances.size(); i++)
			dist_cloud->push_back(rda::Point(i, distances[i], 0));		

		double rdp_eps = atof(rda::Console::getParam("-rdp_eps").c_str());

		clock_t ls_dist_clock;
		ls_dist_clock = clock();

		std::vector<rda::CloudPart> dist_line_parts;
		dist_line_parts.push_back(rda::CloudPart(dist_cloud, rda::Range(1,2))); // end 

		rda::rdp_minimization(rda::CloudPart(dist_cloud, rda::Range(0, dist_cloud->size()-1)),0, dist_cloud->size() - 1, rdp_eps, dist_line_parts);  

		std::vector<double> ls_dists;

		for(int i = 0; i < dist_line_parts.size(); i++){

			function::LeastSquares ls(sma_window_size);
			ls.init(dist_line_parts[i].cloud(), dist_line_parts[i].begin(), dist_line_parts[i].end());
			ls.approximate();

			for(int j = dist_line_parts[i].begin(); j < dist_line_parts[i].end(); j++ ){
				ls_dist_cloud->push_back(rda::Point(j, ls.value(j), 0));
				ls_dists.push_back(ls_dist_cloud->back().y);
			}
		}		
		
		rda::computePointCloud(rob_points, ls_dists, cloud_2);

		//rda::simple_moving_avarage(dist_cloud, sma_window_size, ls_dist_cloud);		

		std::cout << "Distances approximation time :" <<  ((float)(clock() - ls_dist_clock)) / CLOCKS_PER_SEC << "sec" << std::endl;

		std::vector<rda::Line> dist_lines;
		for(int i=0; i < dist_line_parts.size(); i++)
			dist_lines.push_back(rda::Line(dist_line_parts[i].first_point(), dist_line_parts[i].last_point()));
		/*
#pragma endregion
		
		#pragma region Filteration
		
		clock_t filter_clock;
		filter_clock = clock();

		rda::statisticalFilter(cloud, *cloud_filtered, 4, 0.1);

		std::cout << "Filtration time :" <<  ((float)(clock() - filter_clock)) / CLOCKS_PER_SEC << "sec" << std::endl;
		
		#pragma endregion

		#pragma region Segmentation Euclidean
		
		clock_t segm_clock;
		segm_clock = clock();

		double eps = atof(rda::Console::getParam("-eps").c_str());
		int minPts = atoi(rda::Console::getParam("-minPts").c_str());

		std::vector<rda::cloudPtr> clusters;
		rda::euclideanClusterExctraction(cloud_filtered, clusters, eps, minPts, 999999);		

		std::cout << "Segmentation Euclidean time :" <<  ((float)(clock() - segm_clock)) / CLOCKS_PER_SEC << "sec" << std::endl;
		
		#pragma endregion 
		
		#pragma region Partitioning

		double max_dist = atof(rda::Console::getParam("-max_dist").c_str());

		clock_t partitioning_clock;
		partitioning_clock = clock();

		std::vector<rda::CloudPart> clusters_patrs;		

		std::vector<rda::Line> min_clusters_patrs;
		std::vector<rda::CloudPart> min_clusters_parts_lines;

		std::vector<std::vector<rda::Line>> approximied_parts;
		std::vector<std::vector<rda::Line>> merged_cluster;

		double rdp_eps = atof(rda::Console::getParam("-rdp_eps").c_str());

		//rdp_eps = rda::compute_rdp_threashold(dist_cloud, ls_dist_cloud);
		std::cout << "\t rdd_eps = " << rdp_eps << std::endl;

		for(auto i = ls_dist_cloud->begin(); i != ls_dist_cloud->end(); i++){
			rdp_up_ls_dist_cloud->push_back(rda::Point(i->x, i->y + rdp_eps, i->z));
			rdp_down_ls_dist_cloud->push_back(rda::Point(i->x, i->y - rdp_eps, i->z));
		}

		double merge_dist = atof(rda::Console::getParam("-merge_dist").c_str());
		double merge_angle = atof(rda::Console::getParam("-merge_angle").c_str());
		int min_part_size = atoi(rda::Console::getParam("-min_part_size").c_str());
		
		for(int i = 0; i < clusters.size(); i++){					

			std::vector<rda::CloudPart> parts;			

			std::vector<rda::Line> min_partition_lines;
			std::vector<rda::CloudPart> min_parts;

			std::vector<rda::Line> approximied;
			std::vector<rda::Line> merged;

			rda::monotonePartitioning(clusters[i], max_dist, min_part_size, parts);
			//Here need to compute rdp_eps
			rda::lineSegmentation(parts, rdp_eps, min_parts);
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

		#pragma endregion
		*/
		

		rda::Vizualizer::init(&argc, argv);	
		rda::Vizualizer v;	
		rda::Vizualizer v2;	
		rda::Vizualizer v3;	
		rda::Vizualizer v4;	
		rda::Vizualizer v5;	

		v.createWindow("partitioning", 600, 600, 20, 20);
		v2.createWindow("minimized_ramer", 600, 600, 640, 20);
		v3.createWindow("approximied_rumer", 600, 600, 640, 20);
		v4.createWindow("merged", 600, 600, 20, 20);
		v5.createWindow("distances", 600, 600, 20, 20);


		v5.addCloud(dist_cloud, rda::LINE_STRIP, 1.0, 0.0, 1.0, 1.0);
		v5.addCloud(dist_cloud, rda::POINTS, 1.0, 1.0, 1.0, 1.0, 1.0f);
		v5.addCloud(ls_dist_cloud, rda::POINTS, 0.0, 1.0, 0.0, 1.0);
		v5.addCloud(rdp_up_ls_dist_cloud, rda::LINE_STRIP, 1.0, 1.0, 1.0, 1.0);
		v5.addCloud(rdp_down_ls_dist_cloud, rda::LINE_STRIP, 1.0, 1.0, 1.0, 1.0);

		for(int i = 0; i < dist_lines.size(); i++)
			v5.addCloud(dist_lines[i], rda::LINE_STRIP, 0.0f, 0.0f, 1.0f, 1.0f);

		v4.addCloud(cloud, rda::POINTS, 1.0f, 1.0f, 1.0f, 0.5f);		
		v.addCloud(cloud, rda::POINTS, 1.0f, 1.0f, 1.0f, 0.5f);					
		v.addCloud(cloud_2, rda::POINTS, 0.0f, 1.0f, 0.0f, 0.5f);	

		//v.addClouds(clusters, rda::POINTS, 1.0f);

		//v.addClouds(clusters_patrs, rda::LINE_STRIP, 0.5f);	
		
		//v2.addClouds(min_clusters_patrs, rda::LINE_STRIP, 1.0f, 4.0f);	

		v2.addCloud(cloud, rda::POINTS, 1.0f, 1.0f, 1.0f, 0.5f);		
		v3.addCloud(cloud, rda::POINTS, 1.0f, 1.0f, 1.0f, 0.5f);		

		/*for(int i = 0; i < merged_cluster.size(); i++)
			v4.addClouds(merged_cluster.at(i), rda::LINES, 1.0f);
		for(int i = 0; i < approximied_parts.size(); i++)
			v3.addClouds(approximied_parts.at(i), rda::LINES, 1.0f);*/


		rda::Vizualizer::start();	
	}
	catch(rda::RdaException& e){

		std::cout << e.what() << std:: endl;
	}	

	return 0;
}