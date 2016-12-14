/*
 *
 * RDAClient_2 is a project which uses only RDA.dll for analysing rangefinder data.
 * The main idea is to create an application without any references to pcl or rda headers and source code.
 * Namespace "client " is an "emulation" of environment whithout installed pcl library (Only dll).
 * 
 * This approach is used because users cant operate pcl data structures ( client is written on Delphi ).
 * So, the only way to communicate is to use native types like double, int ect.
 *
 * RDA_Client_2 works with measurements of sensor and positions of robot.
 * 
*/

#include "stdafx.h"
#include <iostream>
#include <iomanip>

#include <RDA.h>

#include "client\common.h"
#include "client\io.h"
#include "client\console.h"
#include "client\point_cloud.h"
#include "client\vizualizer.h"

using namespace client;


void printOutput(double* output){

	std::cout << "Size : " << output[0] << std:: endl;

	for(int i = 1; i < output[0]; i ++){
		std::cout << std::setprecision(5) << output[i] << " ";
		if(i % pointSize() == 0){
			std::cout << std::endl;
		}
	}
	std::cout << std::endl;
}

int main(int argc, char* argv[])
{

	Console::readArgs(argc, argv);
	std::string file = Console::getParam("-file");
	double clustering_eps = atof(client::Console::getParam("-clustering_eps").c_str());
	int clustering_minPts = atoi(client::Console::getParam("-clustering_minPts").c_str());
	double min_rdp_eps = atof(client::Console::getParam("-min_rdp_eps").c_str());
	double max_dist = atof(client::Console::getParam("-max_dist").c_str());
	double min_part_size = atof(client::Console::getParam("-min_part_size").c_str());
	double merge_dist = atof(client::Console::getParam("-merge_dist").c_str());
	double merge_angle = atof(client::Console::getParam("-merge_angle").c_str());
	int filter_kN = atoi(client::Console::getParam("-filter_kN").c_str());
	double filter_threshold = atof(client::Console::getParam("-filter_threshold").c_str());
	int sensor_id = 0;

	std::vector<double> data;

	
	try {
		readScene(file, data, sensor_id);
	}
	catch(RDAException& e){
		std::cout << e.what() << std::endl; 
		return -1;
	}

	double **output;
	int clusters_number;

	double amount_time = 0;
	clock_t amount_clock;
	amount_clock = clock();

	extractLines(&data[0], clustering_eps, clustering_minPts, min_rdp_eps, max_dist, min_part_size, merge_dist, merge_angle, filter_kN, filter_threshold, sensor_id, output, clusters_number);

	amount_time = ((float)(clock() - amount_clock)) / CLOCKS_PER_SEC;
	std::cout << "Amount time :" << amount_time  << "sec" << std::endl;

	printOutput(output[0]);

	//clearMemory(output, clusters_number);

	std::vector<PointCloud> lines_cluster;

	for(int i=0; i < clusters_number; i++){
		PointCloud lc(output[i], pointSize());
		lines_cluster.push_back(lc);
	}	

	Vizualizer::init(&argc, argv);
	Vizualizer v1;

	v1.addClouds(lines_cluster, client::LINES, 1.0f);	
	v1.createWindow("Lines", 600, 600, 2, 2);	

	Vizualizer::start();		

	return 0;
}

