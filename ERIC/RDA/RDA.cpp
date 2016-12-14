// RDA.cpp: определяет экспортированные функции для приложения DLL.
//



#include <vector>
#include <string>

#include <rda\common\common.h>
#include <rda\common\cloud_part.h>
#include <rda\approximied_cloud_part.h>
#include <rda\segmentation\euclidean.h>
#include <rda\filtering.h>
#include <rda\rangefinder.h>
#include <rda\curve.h>

#include "RDA.h"

const int  POINT_SIZE = 5;

/* ---- Functions to work with points in X,Y ----- */

void convertRawArrayToPointCloud(double* input, rda::cloudPtr output_cloud){
		
	int size = input[0] - 1;

	if( ( size % POINT_SIZE) == 0){		 
	
		for(int i = 0; i < size ; i+= POINT_SIZE){
			output_cloud->points.push_back(rda::Point(input[i+1], input[i+2], input[i+3]));
		}
	}
	else{		

		throw rda::RdaException("Wrong size ." );
	}

}

void convertPointCloudToFormatArray(std::vector<rda::cloudPtr>& clusters, double**& output, int& clusters_size){

	clusters_size = clusters.size();

	output = new double*[clusters_size];	


	for(int i=0; i < clusters_size; i++){

		output[i] = new double[POINT_SIZE * clusters[i]->size() + 1]; // +1 for size element
		output[i][0] = POINT_SIZE * clusters[i]->size() + 1;

		int k = 0;

		for(int j=0; j < clusters[i]->size(); j++){

			output[i][k+1] = clusters[i]->points[j].x;
			output[i][k+2] = clusters[i]->points[j].y;
			output[i][k+3] = clusters[i]->points[j].z;
			output[i][k+4] = 1;
			output[i][k+5] = 1;

			k+= POINT_SIZE;
		}

	}
}

/* ---- Functions to work with distances and robots positions ----- */

void convertRawArrayToDistancesPointCloud(double* input, std::vector<double>& distances, std::vector<rda::RPoint>& rob_points){

	int size = input[0];

	if( ( (size - 1) % POINT_SIZE) == 0 ){

		for(int i = 4; i < size; i += POINT_SIZE){
			distances.push_back(input[i]);
		}

		for(int i = 1; i < size - (POINT_SIZE - 1); i += POINT_SIZE){
			rob_points.push_back(rda::RPoint(input[i], input[i+1], input[i+2]));
		}

	}
	else{
		throw rda::RdaException("Wrong size." );
	}
}

void convertLinesToFormatArray(std::vector<std::vector<rda::Line>>& lines, double**& output, int& clusters_size){

	clusters_size = lines.size();

	output = new double*[clusters_size];	


	for(int i=0; i < clusters_size; i++){

		output[i] = new double[2 * POINT_SIZE * lines[i].size() + 1]; // +1 for size element
		output[i][0] = 2 * POINT_SIZE * lines[i].size() + 1; // line is 2 points

		int k = 0;

		for(int j=0; j < lines[i].size(); j++){

			output[i][k+1] = lines[i][j].start().x;
			output[i][k+2] = lines[i][j].start().y;
			output[i][k+3] = 1;
			output[i][k+4] = 1;
			output[i][k+5] = 1;

			output[i][k+6] = lines[i][j].end().x;
			output[i][k+7] = lines[i][j].end().y;
			output[i][k+8] = 1;
			output[i][k+9] = 1;
			output[i][k+10] = 1;

			k+= POINT_SIZE * 2;
		}

	}

}

void convertApproxCloudPartsToFormatArray(std::vector<std::vector<rda::ApproximiedCloudPart>>& lines, double**& output, int& clusters_size){
	
	clusters_size = lines.size();

	output = new double*[clusters_size];	


	for(int i=0; i < clusters_size; i++){

		output[i] = new double[2 * POINT_SIZE * lines[i].size() + 1]; // +1 for size element
		output[i][0] = 2 * POINT_SIZE * lines[i].size() + 1; // line is 2 points

		int k = 0;

		for(int j=0; j < lines[i].size(); j++){

			output[i][k+1] = lines[i][j].approx_line().start().x;
			output[i][k+2] = lines[i][j].approx_line().start().y;
			output[i][k+3] = 1;
			output[i][k+4] = 1;
			output[i][k+5] = 1;

			output[i][k+6] = lines[i][j].approx_line().end().x;
			output[i][k+7] = lines[i][j].approx_line().end().y;
			output[i][k+8] = 1;
			output[i][k+9] = 1;
			output[i][k+10] = 1;

			k+= POINT_SIZE * 2;
		}

	}
}





 /*  ------- RDA.h implementation ------- */

void extractLines(double* input, double clustering_eps, int clustering_minPts, double min_rdp_eps, double max_dist, int min_part_size, double merge_dist, double merge_angle, int filter_kN, double filter_treshold, int sensor_id, double**& output, int& clusters_size){
	
	std::vector<double> distances;
	std::vector<rda::RPoint> rob_points;

	convertRawArrayToDistancesPointCloud(input, distances, rob_points);

	rda::cloudPtr dist_cloud (new rda::cloud); // cloud to present distances in 2D. X - i, Y - distance ( it is stupid, of course)

	for(int i = 0; i < distances.size(); i++){
			dist_cloud->push_back(rda::Point(i, distances[i], 0));				
	}

	#pragma region Filtering

	rda::cloudPtr filtered_dist_cloud (new rda::cloud);
	//rda::statisticalFilter(dist_cloud, *filtered_dist_cloud, filter_kN, filter_treshold);
	std::vector<int> filtered_dists_indexes;
	rda::statisticalDistanceFilter(distances, filter_kN, filter_treshold, filtered_dists_indexes);

	#pragma endregion

	#pragma region Count into world coordinate system filtered values

	rda::cloudPtr cloud_filtered (new rda::cloud);
	rda::computePointCloud(rob_points, distances, filtered_dists_indexes, cloud_filtered, sensor_id);	

	#pragma endregion

	#pragma region Clustering Euclidean
		
	std::vector<rda::cloudPtr> clusters;
	rda::euclideanClusterExctraction(cloud_filtered, clusters, clustering_eps, clustering_minPts, 999999);			
	
	#pragma endregion 

	#pragma region Partitionong

	std::vector<rda::CloudPart> clusters_patrs;		
	
	std::vector<rda::Line> min_clusters_patrs;
	std::vector<rda::CloudPart> min_clusters_parts_lines;
	
	std::vector<std::vector<rda::ApproximiedCloudPart>> approximied_parts;
	std::vector<std::vector<rda::ApproximiedCloudPart>> merged_cluster;
	
	for(int i = 0; i < clusters.size(); i++){					
	
		std::vector<rda::CloudPart> parts;			
	
		std::vector<rda::Line> min_partition_lines;
		std::vector<rda::CloudPart> min_parts;
	
		std::vector<rda::ApproximiedCloudPart> approximied;
		std::vector<rda::ApproximiedCloudPart> merged;
	
		rda::monotonePartitioning(clusters[i], max_dist, min_part_size, parts);
	
		//rda::lineSegmentation(parts, rdp_eps, min_parts);
		//Adaptive
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

	#pragma endregion

	convertApproxCloudPartsToFormatArray(merged_cluster, output, clusters_size);
}

int pointSize(){

	return POINT_SIZE;
}

/*void clearMemory(double*& ptr){
	delete[] ptr;
	ptr = 0;
}*/

void clearMemory(double**& ptr, int size){
	for(int i = 0; i < size; i++){
		delete[] ptr[i];
		ptr[i] = 0;
	}
	delete[] ptr;

}
