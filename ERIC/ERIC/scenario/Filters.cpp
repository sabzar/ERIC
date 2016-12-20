
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
		rda::cloudPtr rmed_cloud (new rda::cloud);
		rda::cloudPtr kuw_cloud (new rda::cloud);
		rda::cloudPtr stat_cloud (new rda::cloud);

		rda::cloudPtr med_dist_cloud (new rda::cloud);	
		rda::cloudPtr kuw_dist_cloud (new rda::cloud);	
		rda::cloudPtr stat_dist_cloud (new rda::cloud);
		rda::cloudPtr rmed_dist_cloud (new rda::cloud);	
				
		rda::Console::readArgs(argc, argv);	

		int sensor_id = 0;
		rda::readScene(rda::Console::getParam("-file"), distances, rob_points, sensor_id);
		rda::computePointCloud(rob_points, distances, cloud, sensor_id);

		// median filter		
		int median_window = atof(rda::Console::getParam("-median_window").c_str());		
		std::vector<double> median_distances;
		rda::medianFilter(distances, median_window, median_distances);

		// reduce median filter		
		int reduce_median_window = atof(rda::Console::getParam("-reduce_median_window").c_str());		
		std::vector<int> reduce_median_indexes;
		rda::reduce_median_filter(distances, rda::Range(0, distances.size() - 1), reduce_median_window, reduce_median_indexes);

		//kuwahara filter
		std::vector<double> kuwahara_distances;
		int kuwahara_window = atof(rda::Console::getParam("-kuwahara_window").c_str());						
		rda::kuwahara_filter(distances, kuwahara_window, kuwahara_distances);

		//statistical filter
		int kN = atof(rda::Console::getParam("-filter_kN").c_str());
		double threshold = atof(rda::Console::getParam("-filter_threashold").c_str());

		std::vector<int> filtered_dists_indexes;
		rda::statisticalDistanceFilter(distances, kN, threshold, filtered_dists_indexes); 
		
		rda::computePointCloud(rob_points, median_distances, med_cloud, sensor_id);
		rda::computePointCloud(rob_points, distances, reduce_median_indexes, rmed_cloud, sensor_id);
		rda::computePointCloud(rob_points, kuwahara_distances, kuw_cloud, sensor_id);
		rda::computePointCloud(rob_points, distances, filtered_dists_indexes, stat_cloud, sensor_id);


		for(auto i = 0; i < distances.size(); i++)
			dist_cloud->push_back(rda::Point(i, distances[i], 0));					

		for(auto i = 0; i < median_distances.size(); i++)
			med_dist_cloud->push_back(rda::Point(i, median_distances[i], 0));					

		for(auto i = 0; i < median_distances.size(); i++)
			kuw_dist_cloud->push_back(rda::Point(i, kuwahara_distances[i], 0));					

		for(auto i = 0; i < filtered_dists_indexes.size(); i++){
			stat_dist_cloud->push_back(rda::Point(filtered_dists_indexes[i], distances[filtered_dists_indexes[i]], 0));
		}

		for(auto i = 0; i < reduce_median_indexes.size(); i++)
			rmed_dist_cloud->push_back(rda::Point(reduce_median_indexes[i], distances[reduce_median_indexes[i]], 0));					


		#pragma region Vizualization

		std::cout << "Input: " << dist_cloud->size() << std::endl;
		std::cout << "Statistical: " << stat_dist_cloud->size() << std::endl;
		std::cout << "Reduce median: " << rmed_dist_cloud->size() << std::endl;

		rda::Vizualizer::init(&argc, argv);	
		rda::Vizualizer v;
		rda::Vizualizer v_1;
		rda::Vizualizer v_2;

		v.createWindow("raw", 700, 700, 20, 20);
		v_1.createWindow("distances", 700, 700, 20, 20);
		//v_2.createWindow("median filtered", 700, 700, 20, 20);

		v.addCloud(cloud, rda::CIRCLES, 0.0f, 0.0f, 0.0f, 0.5f);
		v_1.addCloud(dist_cloud, rda::LINE_STRIP, 1.0f, 0.0f, 0.0f, 1.0f);
		v_1.addCloud(dist_cloud, rda::POINTS, 0.0f, 0.0f, 0.0f, 1.0f, 4.0f);
		v_1.addCloud(stat_dist_cloud, rda::LINE_STRIP, 0.0f, 0.0f, 1.0f, 1.0f, 4.0f);
		v_1.addCloud(med_dist_cloud, rda::LINE_STRIP, 0.0f, 1.0f, 0.0f, 1.0f, 4.0f);		
		v_1.addCloud(rmed_dist_cloud, rda::LINE_STRIP, 1.0f, 0.0f, 1.0f, 1.0f, 4.0f);
		//v_1.addCloud(kuw_dist_cloud, rda::LINE_STRIP, 0.0f, 0.0f, 1.0f, 1.0f, 4.0f);

		v.addCloud(cloud, rda::LINE_STRIP, 1.0f, 0.0f, 0.0f, 1.0f, 4.0f);
		v.addCloud(stat_cloud, rda::LINE_STRIP, 0.0f, 0.0f, 1.0f, 1.0f, 4.0f);
		v.addCloud(med_cloud, rda::LINE_STRIP, 0.0f, 1.0f, 0.0f, 1.0f, 4.0f);		
		v.addCloud(rmed_cloud, rda::LINE_STRIP, 1.0f, 0.0f, 1.0f, 1.0f, 4.0f);
		//v.addCloud(kuw_cloud, rda::LINE_STRIP, 0.0f, 0.0f, 1.0f, 1.0f, 4.0f);

		rda::Vizualizer::start();

		#pragma endregion

	}
	catch(rda::RdaException& e){
		std::cout << e.what() << std::endl;
	}
}