
#include <scenario\ASM_F.h>

#include <rda\io\io.h>
#include <rda\io\console.h>
#include <rda\io\vizualization.h>
#include <rda\common\common.h>
#include <rda\rangefinder.h>
#include <rda\filtering.h>
#include <rda\rdaException.h>


ASM_F::ASM_F(int argc, char* argv[]){

	try{				
		std::vector<double> distances;			
		std::vector<rda::RPoint> rob_points;		

		rda::cloudPtr cloud (new rda::cloud);
		std::vector<rda::cloudPtr> clouds;

		rda::cloudPtr dist_cloud (new rda::cloud);
		std::vector<rda::cloudPtr> dist_clouds;
		rda::cloudPtr rmed_dist_cloud (new rda::cloud);	

		rda::cloudPtr rmed_cloud (new rda::cloud);				
		
				
		rda::Console::readArgs(argc, argv);	

		int sensor_id = 0;
		rda::readScene(rda::Console::getParam("-file"), distances, rob_points, sensor_id);
		rda::computePointCloud(rob_points, distances, cloud, sensor_id);

		//naive breakpoint detector (distances)
		double max_dist_diff = atof(rda::Console::getParam("-max_dist_diff").c_str());
		int min_segm_points = atof(rda::Console::getParam("-min_segm_points").c_str());
		std::vector<rda::Range> brp_indexes;
		rda::naive_beakpoint_detector(distances, max_dist_diff, min_segm_points, brp_indexes); 

		// reduce median filter		
		int reduce_median_window = atof(rda::Console::getParam("-reduce_median_window").c_str());		
		std::vector<int> reduce_median_indexes;
		//rda::reduce_median_filter(distances, reduce_median_window, reduce_median_indexes);		
				
		//rda::computePointCloud(rob_points, distances, reduce_median_indexes, rmed_cloud, sensor_id);		

		// add result to pointcloud for vizualization
		for(auto it = brp_indexes.begin(); it != brp_indexes.end(); ++it){
			rda::cloudPtr n_cloud(new rda::cloud);
			rda::cloudPtr n_dist_cloud(new rda::cloud);
			for(auto ind = it->start; ind <= it->end; ind++){
				n_cloud->push_back(rda::computePoint(rob_points[ind], distances[ind], sensor_id));
				n_dist_cloud->push_back(rda::Point(ind, distances[ind], 0));
			}
			clouds.push_back(n_cloud);
			dist_clouds.push_back(n_dist_cloud);
		}

		for(auto i = 0; i < distances.size(); i++)
			dist_cloud->push_back(rda::Point(i, distances[i], 0));					

		/*for(auto i = 0; i < reduce_median_indexes.size(); i++)
			rmed_dist_cloud->push_back(rda::Point(reduce_median_indexes[i], distances[reduce_median_indexes[i]], 0));	*/		

		#pragma region Vizualization				

		std::cout << "Input: " << dist_cloud->size() << std::endl;
		std::cout << "Breakpoint segments: " << brp_indexes.size() << std::endl;
		int brp_points = 0;
		for(auto i = 0; i < brp_indexes.size(); i++){
			brp_points += brp_indexes[i].size();
			std::cout << "\t segmenment_: " << i << " size: " << brp_indexes[i].size() + 1 << std::endl;
		}
		std::cout << "Points after breakpoint detector: " << brp_points << std::endl;
		std::cout << "Reduce median: " << rmed_dist_cloud->size() << std::endl;

		rda::Vizualizer::init(&argc, argv);	
		rda::Vizualizer v;
		rda::Vizualizer v_1;
		rda::Vizualizer v_2;

		v.createWindow("raw&breakpoint", 700, 700, 20, 20);
		v_1.createWindow("distances", 700, 700, 20, 20);		

		v.addClouds(clouds, rda::LINE_STRIP, 1.0f, 4.0f);
		v.addCloud(cloud, rda::CIRCLES, 0.0f, 0.0f, 0.0f, 0.5f);
		//v.addCloud(rmed_cloud, rda::CIRCLES, 1.0f, 0.0f, 1.0f, 1.0f, 4.0f);				
		//v.addCloud(rmed_cloud, rda::LINE_STRIP, 1.0f, 0.0f, 1.0f, 1.0f, 4.0f);
		
		v_1.addCloud(dist_cloud, rda::POINTS, 0.0f, 0.0f, 0.0f, 1.0f, 4.0f);
		v_1.addCloud(dist_cloud, rda::LINE_STRIP, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f);
		v_1.addClouds(dist_clouds, rda::LINE_STRIP, 1.0f, 4.0f);
		//v_1.addCloud(rmed_dist_cloud, rda::LINE_STRIP, 1.0f, 0.0f, 1.0f, 1.0f, 4.0f);		
		

		rda::Vizualizer::start();

		#pragma endregion

	}
	catch(rda::RdaException& e){
		std::cout << e.what() << std::endl;
	}
}

