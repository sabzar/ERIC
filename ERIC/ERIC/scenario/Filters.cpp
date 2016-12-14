
#include <scenario\Filters.h>

#include <rda\io\io.h>
#include <rda\io\console.h>
#include <rda\io\vizualization.h>
#include <rda\common\common.h>
#include <rda\rangefinder.h>
#include <rda\filtering.h>
#include <rda\rdaException.h>


Filters::Filters(int argc, char* argv[]){

	try{				
		std::vector<double> distances;			
		std::vector<rda::RPoint> rob_points;		

		rda::cloudPtr cloud (new rda::cloud);
		rda::cloudPtr dist_cloud (new rda::cloud);
		rda::cloudPtr med_cloud (new rda::cloud);
		rda::cloudPtr med_dist_cloud (new rda::cloud);	
		rda::cloudPtr sma_dist_cloud (new rda::cloud);	
				
		rda::Console::readArgs(argc, argv);	

		int sensor_id = 0;
		rda::readScene(rda::Console::getParam("-file"), distances, rob_points, sensor_id);
		rda::computePointCloud(rob_points, distances, cloud, sensor_id);

		// median filter		
		int median_window = atof(rda::Console::getParam("-median_window").c_str());
		std::cout << median_window << std::endl;
		std::vector<double> median_distances;
		rda::medianFilter(distances, median_window, median_distances);

		rda::computePointCloud(rob_points, median_distances, med_cloud, sensor_id);


		for(int i = 0; i < distances.size(); i++)
			dist_cloud->push_back(rda::Point(i, distances[i], 0));					

		for(int i = 0; i < median_distances.size(); i++)
			med_dist_cloud->push_back(rda::Point(i, median_distances[i], 0));					
		



		#pragma region Vizualization

		rda::Vizualizer::init(&argc, argv);	
		rda::Vizualizer v;
		rda::Vizualizer v_1;
		rda::Vizualizer v_2;

		v.createWindow("raw", 700, 700, 20, 20);
		v_1.createWindow("distances", 700, 700, 20, 20);
		//v_2.createWindow("median filtered", 700, 700, 20, 20);

		v.addCloud(cloud, rda::CIRCLES, 0.0f, 0.0f, 0.0f, 0.5f);
		v_1.addCloud(dist_cloud, rda::LINE_STRIP, 0.3f, 0.3f, 1.0f, 1.0f);
		v_1.addCloud(dist_cloud, rda::POINTS, 0.0f, 0.0f, 0.0f, 1.0f, 4.0f);
		v_1.addCloud(med_dist_cloud, rda::LINE_STRIP, 0.0f, 1.0f, 0.0f, 1.0f, 4.0f);

		v.addCloud(cloud, rda::LINE_STRIP, 1.0f, 0.0f, 0.0f, 1.0f, 4.0f);
		v.addCloud(med_cloud, rda::LINE_STRIP, 0.0f, 1.0f, 0.0f, 1.0f, 4.0f);

		rda::Vizualizer::start();

		#pragma endregion

	}
	catch(rda::RdaException& e){
		std::cout << e.what() << std::endl;
	}
}