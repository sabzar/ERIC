
#include <scenario\ASM_F.h>

#include <rda\io\io.h>
#include <rda\io\console.h>
#include <rda\io\vizualization.h>
#include <rda\common\common.h>
#include <rda\common\cloud_part.h>
#include <rda\rangefinder.h>
#include <rda\filtering.h>
#include <rda\rdaException.h>


void ASM_F::general(int argc, char* argv[]){

	try{				
		std::vector<double> distances;			
		std::vector<rda::RPoint> rob_points;		

		rda::cloudPtr cloud (new rda::cloud);
		std::vector<rda::cloudPtr> clouds;
		std::vector<rda::cloudPtr> rmed_clouds;

		rda::cloudPtr dist_cloud (new rda::cloud);
		std::vector<rda::cloudPtr> dist_clouds;
		std::vector<rda::cloudPtr> rmed_dist_clouds;	

		rda::cloudPtr rmed_cloud (new rda::cloud);				
		
				
		rda::Console::readArgs(argc, argv);	

		int sensor_id = 0;
		rda::readScene(rda::Console::getParam("-file"), distances, rob_points, sensor_id);
		rda::computePointCloud(rob_points, distances, cloud, sensor_id);

		//naive breakpoint detector (distances)
		std::vector<std::pair<double, double>> dist_errors;
		dist_errors.push_back(std::make_pair(280, 10));
		dist_errors.push_back(std::make_pair(500, 20));
		dist_errors.push_back(std::make_pair(630, 30));
		dist_errors.push_back(std::make_pair(930, 80));
		dist_errors.push_back(std::make_pair(1000, 100));

		double max_dist_diff = atof(rda::Console::getParam("-max_dist_diff").c_str());
		int min_segm_points = atof(rda::Console::getParam("-min_segm_points").c_str());
		std::vector<rda::Range> brp_indexes;
		//rda::naive_beakpoint_detector(distances, max_dist_diff, min_segm_points, brp_indexes); 
		rda::adaptiveNaiveDetector(distances, dist_errors, min_segm_points, brp_indexes);

		// reduce median filter	
		int reduce_median_window = atof(rda::Console::getParam("-reduce_median_window").c_str());		
		std::vector<std::vector<int>> reduce_median_indexes;		
		for(auto i = 0; i < brp_indexes.size(); i++){
			std::vector<int> n_indexes;			
			rda::reduce_median_filter(distances, brp_indexes[i], reduce_median_window, n_indexes);
			if(n_indexes.size() > 0){
				reduce_median_indexes.push_back(n_indexes);
				rda::cloudPtr f_cloud (new rda::cloud);
				rda::computePointCloud(rob_points, distances, n_indexes, f_cloud, sensor_id);
				rmed_clouds.push_back(f_cloud);
			}
		}

		// Adaptive Split&Merge
		double min_rdp_eps = atof(rda::Console::getParam("-min_rdp_eps").c_str());
		int min_rdp_size = atof(rda::Console::getParam("-min_rdp_size").c_str());

		std::vector<std::vector<rda::CloudPart>> min_cloud_parts;
		for(auto it = rmed_clouds.begin(); it != rmed_clouds.end(); ++it){
			std::vector<rda::CloudPart> min_parts;
			rda::adaptive_rdp(rda::CloudPart(*it), min_rdp_size, min_rdp_eps, min_parts);
			min_cloud_parts.push_back(min_parts);
		}					

		// Leas Squares
		std::vector<std::vector<rda::ApproximiedCloudPart>> approx_clouds;
		for(auto it = min_cloud_parts.begin(); it != min_cloud_parts.end(); ++it){
			std::vector<rda::ApproximiedCloudPart> n_aprox;
			rda::lsLineApproximation(*it, n_aprox);
			approx_clouds.push_back(n_aprox);
		}

		#pragma region Vizualization				

		// Add result to pointcloud for vizualization
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

		int rmed_points = 0;
		for(auto it = reduce_median_indexes.begin(); it != reduce_median_indexes.end(); ++it){
			rda::cloudPtr n_rmed(new rda::cloud);				
			for(auto j = 0; j < it->size(); j++){
				n_rmed->push_back(rda::Point(it->at(j), distances[it->at(j)], 0));				
			}	
			rmed_points += n_rmed->size();
			rmed_dist_clouds.push_back(n_rmed);
		}

		std::vector<rda::Line> min_cloud_parts_lines;
		for(auto it = min_cloud_parts.begin(); it != min_cloud_parts.end(); ++it){
			for(auto jt = it->begin(); jt != it->end(); ++jt){
				min_cloud_parts_lines.push_back(jt->line());
			}
		}

		std::vector<rda::Line> approx_cloud_lines;
		for(auto it = approx_clouds.begin(); it != approx_clouds.end(); ++it){
			for(auto jt = it->begin(); jt != it->end(); ++jt){
				approx_cloud_lines.push_back(jt->approx_line());
			}
		}

		// Vizualization

		std::cout << "Input: " << dist_cloud->size() << std::endl;
		std::cout << "Breakpoint segments: " << brp_indexes.size() << std::endl;
		int brp_points = 0;
		for(auto i = 0; i < brp_indexes.size(); i++){
			brp_points += brp_indexes[i].size();
			std::cout << "\t segmenment_: " << i << " size: " << brp_indexes[i].size() + 1 << std::endl;
		}
		std::cout << "Points after breakpoint detector: " << brp_points << std::endl;
		std::cout << "Reduce median: " << rmed_points << std::endl;

		rda::Vizualizer::init(&argc, argv);	
		rda::Vizualizer v;
		rda::Vizualizer v_1;
		rda::Vizualizer v_2;
		rda::Vizualizer v_3;
		rda::Vizualizer v_4;

		v.createWindow("raw&breakpoint", 700, 700, 20, 20);
		v_1.createWindow("distances", 700, 700, 700, 20);
		v_2.createWindow("median filter", 700, 700, 20, 20);
		v_3.createWindow("split&merge", 700, 700, 700, 20);
		v_4.createWindow("least squares", 700, 700, 20, 20);


		v.addClouds(clouds, rda::LINE_STRIP, 1.0f, 4.0f);
		v.addClouds(clouds, rda::CIRCLES, 1.0f, 4.0f);
		v.addCloud(cloud, rda::CIRCLES, 0.0f, 0.0f, 0.0f, 0.5f);		
		
		v_1.addCloud(dist_cloud, rda::POINTS, 0.0f, 0.0f, 0.0f, 1.0f, 4.0f);
		v_1.addCloud(dist_cloud, rda::LINE_STRIP, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f);
		v_1.addCloud(dist_cloud, rda::CIRCLES, 0.0f, 0.0f, 0.0f, 0.3f, 1.0f);
		v_1.addClouds(dist_clouds, rda::LINE_STRIP, 1.0f, 4.0f);		
		for(auto i = 0; i < rmed_dist_clouds.size(); i++)
			v_1.addCloud(rmed_dist_clouds[i], rda::LINE_STRIP, 1.0f, 0.0f, 1.0f, 1.0f, 4.0f);	
		
		v_2.addCloud(cloud, rda::CIRCLES, 0.0f, 0.0f, 0.0f, 0.3f, 4.0f);
		v_2.addClouds(rmed_clouds, rda::CIRCLES, 1.0f, 4.0f);
		v_2.addClouds(rmed_clouds, rda::LINE_STRIP, 1.0f, 2.0f);
		
		v_3.addClouds(min_cloud_parts_lines, rda::LINES, 1.0f);
		v_3.addClouds(rmed_clouds, rda::CIRCLES, 0.3f, 4.0f);

		v_4.addCloud(cloud, rda::CIRCLES, 0.0f, 0.0f, 0.0f, 0.1f, 4.0f);
		v_4.addClouds(approx_cloud_lines, rda::LINES, 1.0f);

		rda::Vizualizer::start();

		#pragma endregion

	}
	catch(rda::RdaException& e){
		std::cout << e.what() << std::endl;
	}
}

void ASM_F::sectorScanning(int argc, char* argv[])
{
	try{
		std::vector<double> distances;
		std::vector<rda::RPoint> rob_points;
		std::vector<rda::Range> part_ranges;

		std::vector<rda::cloudPtr> raw_clouds;
		std::vector<rda::cloudPtr> reduce_median_clouds;

		rda::Console::readArgs(argc, argv);

		int sensor_id = 0;
		rda::readScene(rda::Console::getParam("-file"), distances, rob_points, part_ranges, sensor_id);				

		//statistical filter
		int statistacal_kN = atof(rda::Console::getParam("-statistacal_kN").c_str());
		double statistacal_threashold = atof(rda::Console::getParam("-statistacal_threashold").c_str());
		std::vector<std::vector<int>> stat_filtered_indexes(part_ranges.size());
		for(auto i = 0; i < part_ranges.size(); i++){			
			rda::statisticalDistanceFilter(distances, part_ranges[i], statistacal_kN, statistacal_threashold, stat_filtered_indexes[i]);
		}		

		// reduce median filter	
		int reduce_median_window = atof(rda::Console::getParam("-reduce_median_window").c_str());
		std::vector<std::vector<int>> reduce_median_indexes(stat_filtered_indexes.size());
		for(auto i = 0; i < stat_filtered_indexes.size(); i++){			
			rda::reduce_median_filter(distances, stat_filtered_indexes[i], reduce_median_window, reduce_median_indexes[i]);						

			// count filtered into world coordinate system)
			rda::cloudPtr rm_cloud(new rda::cloud);
			rda::computePointCloud(rob_points, distances, reduce_median_indexes[i], rm_cloud, sensor_id);
			reduce_median_clouds.push_back(rm_cloud);
		}

		// Adaptive Split&Merge
		double min_rdp_eps = atof(rda::Console::getParam("-min_rdp_eps").c_str());
		int min_rdp_size = atof(rda::Console::getParam("-min_rdp_size").c_str());

		std::vector<std::vector<rda::CloudPart>> min_cloud_parts;
		for(auto it = reduce_median_clouds.begin(); it != reduce_median_clouds.end(); ++it){
			std::vector<rda::CloudPart> min_parts;
			rda::adaptive_rdp(rda::CloudPart(*it), min_rdp_size, min_rdp_eps, min_parts);
			min_cloud_parts.push_back(min_parts);
		}


		#pragma region Vizualization
		
		//raw clouds
		for(auto i = 0; i < part_ranges.size(); i++){
			rda::cloudPtr pc(new rda::cloud);			
			for(auto j = part_ranges[i].start; j <= part_ranges[i].end; j++){
				pc->push_back(rda::computePoint(rob_points[j], distances[j], sensor_id));
			}
			raw_clouds.push_back(pc);
		}

		//dists clouds
		std::vector<rda::cloudPtr> dists_clouds;
		int dists_number = 0;
		for(auto it = part_ranges.begin(); it != part_ranges.end(); ++it){
			dists_clouds.push_back(rda::cloudPtr(new rda::cloud));
			for(auto i = it->start; i <= it->end; i++){
				dists_clouds.back()->push_back(rda::Point(dists_number++, distances[i], 1));
			}
		}

		// statistical filtering dist cloud and 2d cloud
		std::vector<rda::cloudPtr> sf_dists_clouds(stat_filtered_indexes.size());
		std::vector<rda::cloudPtr> sf_clouds(stat_filtered_indexes.size());
		for(auto i = 0; i < stat_filtered_indexes.size(); i++){
			sf_dists_clouds[i] = rda::cloudPtr(new rda::cloud);
			sf_clouds[i] = rda::cloudPtr(new rda::cloud);
			for(auto j = 0; j < stat_filtered_indexes[i].size(); j++){
				sf_dists_clouds[i]->push_back(rda::Point(stat_filtered_indexes[i][j], distances[stat_filtered_indexes[i][j]], 1));
				sf_clouds[i]->push_back(rda::computePoint(rob_points[stat_filtered_indexes[i][j]], distances[stat_filtered_indexes[i][j]], sensor_id));
			}
		}

		//reduce median filtered dists clouds
		int filtered_dist_number = 0;
		std::vector<rda::cloudPtr> filtered_dists_clouds;
		for(auto it = reduce_median_indexes.begin(); it != reduce_median_indexes.end(); ++it){
			filtered_dists_clouds.push_back(rda::cloudPtr(new rda::cloud));
			for(auto i = it->begin(); i != it->end(); ++i){
				filtered_dists_clouds.back()->push_back(rda::Point(*i, distances[*i], 1));
			}
		}

		std::vector<rda::Line> min_cloud_parts_lines;
		for(auto it = min_cloud_parts.begin(); it != min_cloud_parts.end(); ++it){
			for(auto jt = it->begin(); jt != it->end(); ++jt){
				min_cloud_parts_lines.push_back(jt->line());
			}
		}			


		// Vizualizer
		rda::Vizualizer::init(&argc, argv);	
		rda::Vizualizer v;
		rda::Vizualizer v_1;
		rda::Vizualizer v_2;
		rda::Vizualizer v_3;
		rda::Vizualizer v_4;

		v.createWindow("distances", 700, 700, 20, 20);
		v_1.createWindow("raw", 700, 700, 720, 20);
		v_2.createWindow("reduce median filter", 700, 700, 720, 20);
		v_3.createWindow("split&merge", 700, 700, 720, 20);
		v_4.createWindow("statistical filter", 700, 700, 20, 20);

		
		for(auto i = 0; i < dists_clouds.size(); i++){
			v.addCloud(dists_clouds[i], rda::CIRCLES, 0.0f, 0.0f, 0.0f, 1.0f);
			v.addCloud(dists_clouds[i], rda::LINE_STRIP, 0.0f, 0.0f, 0.0f, 1.0f);			
		}

		for(auto i = 0; i < sf_dists_clouds.size(); i++){
			v.addCloud(sf_dists_clouds[i], rda::CIRCLES, 0.0f, 1.0f, 0.0f, 1.0f);
		}

		for(auto i = 0; i < raw_clouds.size(); i++){
			v_1.addCloud(raw_clouds[i], rda::CIRCLES, 0.0f, 0.0f, 0.0f, 1.0f);
			v_4.addCloud(raw_clouds[i], rda::CIRCLES, 0.0f, 0.0f, 0.0f, 0.3f);
		}

		v_2.addClouds(reduce_median_clouds, rda::CIRCLES, 1.0f);
		v_2.addClouds(raw_clouds, rda::CIRCLES, 0.2f);

		v_3.addClouds(min_cloud_parts_lines, rda::LINES, 1.0f);
		v_3.addClouds(reduce_median_clouds, rda::CIRCLES, 0.3f, 4.0f);
		
		for(auto i = 0; i < sf_clouds.size(); i++){
			v_4.addCloud(sf_clouds[i], rda::CIRCLES, 1.0f, 1.0f, 0.0f, 1.0f);			
		}

		rda::Vizualizer::start();

		#pragma endregion
	}
	catch(rda::RdaException& e){
		std::cout << e.what() << std::endl;
	}
}

