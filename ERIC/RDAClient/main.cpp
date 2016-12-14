
/*
 *
 * RDAClient is a project which uses only RDA.dll for analysing rangefinder data.
 * The main idea is to create an application without any references to pcl or rda headers and source code.
 * Namespace "client " is an "emulation" of environment whithout installed pcl library (Only dll).
 * 
 * This approach is used because users cant operate pcl data structures ( client is written on Delphi ).
 * So, the only way to communicate is to use native types like double, int ect.
 *
*/

#include <iostream>
#include <iomanip>

#include <RDA.h>

#include "client\common.h"
#include "client\io.h"
#include "client\console.h"
#include "client\point_cloud.h"
#include "client\vizualizer.h"

#include <time.h>

using namespace client;

void printOutput(double* output){

	std::cout << "Size : " << output[0] << std:: endl;

	for(int i = 1; i < output[0]; i ++){
		std::cout << std::setprecision(5) << output[i] << " ";
		if(i % 5 == 0){
			std::cout << std::endl;
		}
	}
	std::cout << std::endl;
}

int main(int argc, char* argv[] ){

	Console::readArgs(argc, argv);
	std::string file = Console::getParam("-file");
	double tolerance = atof(client::Console::getParam("-tolerance").c_str());
	int minPts = atoi(client::Console::getParam("-minPts").c_str());
	unsigned order = atoi(client::Console::getParam("-order").c_str());
	unsigned n_control_points = atoi(client::Console::getParam("-control_points").c_str());
	double smoothness = atof(client::Console::getParam("-smothness").c_str());
	double rScale = atof(client::Console::getParam("-rScale").c_str());
	double threashold = atof(client::Console::getParam("-threashold").c_str());
	unsigned int resolution = atoi(client::Console::getParam("-res").c_str());
	bool writeToFile = atoi(client::Console::getParam("-write").c_str());

	std::vector<double> cloud;	

	try {
		readScene(file, cloud);
	}
	catch(RDAException& e){
		std::cout << e.what() << std::endl; 
		return -1;
	}
	
	std::cout << std::endl << std::endl;
	clock_t amount_clock;
	amount_clock = clock();

	PointCloud pc(&cloud[0], pointSize());

	double** clusters;
	double** curves;

	int clusters_number=0;
	int curves_number=0;

	std::vector<PointCloud> clusters_clouds;
	std::vector<PointCloud> curves_clouds;

	clock_t clustering_clock;
	clustering_clock = clock();

	extractClusters(&cloud[0], tolerance, minPts, clusters, clusters_number);

	std::cout << "Clusteringt time :" <<  ((float)(clock() - clustering_clock)) / CLOCKS_PER_SEC << "sec" << std::endl;

	
	clock_t fitting_clock;
	fitting_clock = clock();

	fitCurve(clusters, clusters_number, order, n_control_points, smoothness, rScale, resolution, threashold, curves, curves_number); 

	std::cout << "Fitiing time :" <<  ((float)(clock() - fitting_clock)) / CLOCKS_PER_SEC << "sec" << std::endl;

	std::cout << " -------------------" << std::endl;
	std::cout << "Amount time :" <<  ((float)(clock() - amount_clock)) / CLOCKS_PER_SEC << "sec" << std::endl;
	std::cout << std::endl;
	std::cout << "Clusters : " << clusters_number << std::endl;

	if(writeToFile){		
		std::string output_file = file.append("_result.txt");
		std::cout << "Output file : " << output_file << std::endl;
		writeScene(output_file, curves, curves_number, pointSize());
	}

	for(int i=0; i < curves_number; i++){
		PointCloud cc(curves[i], pointSize());
		curves_clouds.push_back(cc);
	}

	for(int i=0; i < clusters_number; i++){
		PointCloud cc(clusters[i], pointSize());
		clusters_clouds.push_back(cc);
	}

	/*clearMemory(clusters, clusters_number);
	clearMemory(curves, curves_number);*/

	Vizualizer::init(&argc, argv);
	Vizualizer v1, v2;

	//v1.addCloud(pc, client::POINTS, 1.0f, 1.0f, 1.0f, 1.0f);	
	v2.addClouds(clusters_clouds, POINTS, 0.5f);
	v2.addClouds(curves_clouds, LINE_STRIP, 1.0f);
	v2.addClouds(curves_clouds, POINTS, 1.0f, 6.0f);

	//v1.createWindow("Clusters", 600, 600, 2, 2);
	v2.createWindow("Curves", 600, 600, 2, 2);

	Vizualizer::start();		

	return 0;
}