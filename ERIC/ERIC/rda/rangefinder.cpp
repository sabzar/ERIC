
#include <list>
#include <iostream>
#include <map>

#include <rda\rangefinder.h>
#include <rda\vector.h>
#include <rda\function\ApproximateMethod.h>
#include <rda\function\LeastSquares.h>
#include <rda\io\io.h> // SensorNum
#include <rda\approximied_cloud_part.h>

#define PI 3.14159265358979323846

double left_matrix[4][4] = {{ 0, -1, 0, -120 }, 
							{ 1, 0, 0, 241.0 },
							{ 0, 0, 1, 229.5 },
							{ 0, 0, 0, 1 }};

double front_matrix[4][4] = {{ 1, 0, 0, 119 }, 
							 { 0, 1, 0, 0.0 },
							 { 0, 0, 1, 181.0 },
							 { 0, 0, 0, 1 }};

double right_matrix[4][4] = {{ 0, 1, 0, -65.0 }, 
							 { -1, 1, 0, -223.0 },
							 { 0, 0, 1, 265.0 },
							 { 0, 0, 0, 1 }};

void rda::computePointCloud(std::vector<rda::RPoint> rob_point, std::vector<double> distances, rda::cloudPtr cloud, int sensor){
	for(int i = 0; i < distances.size(); i++){
		cloud->push_back(rda::computePoint(rob_point[i], distances[i], sensor));
	}	
}

void rda::computePointCloud(std::vector<rda::RPoint> rob_point, std::vector<double> distances, std::vector<int>& indexes, rda::cloudPtr cloud, int sensor){

	for(int i = 0; i < indexes.size(); i++){
		cloud->push_back(rda::computePoint(rob_point[indexes[i]], distances[indexes[i]], sensor));
	}	
}

rda::Point rda::computePoint(RPoint& rob_point, double distance, int sensor_id){
	
		const int size = 4;

		double sensor_vec[4] = {distance, 0.0, 0.0, 1.0 };
		std::vector<double> sensor_vector(&sensor_vec[0], &sensor_vec[size]);
		// count coordinates in robot cordinates from sensor coordinates
		std::vector<double> robot_vector; 
		switch(sensor_id){
			case FrontSensor: 
				robot_vector = rda::mulMatrixOnVector<size>(front_matrix, sensor_vector); break;
			case LeftSensor: 
				robot_vector = rda::mulMatrixOnVector<size>(left_matrix, sensor_vector); break;
			case RightSensor: 
				robot_vector = rda::mulMatrixOnVector<size>(right_matrix, sensor_vector); break;
		}
		
		double robot_matrix[size][size] = 
		{{ std::cos(rob_point.angle_robot * PI/180.0), -std::sin(rob_point.angle_robot * PI/180.0), 0, rob_point.x_robot }, 
		 { std::sin(rob_point.angle_robot * PI/180.0), std::cos(rob_point.angle_robot * PI/180.0), 0,  rob_point.y_robot },
		 { 0, 0, 1, 0 },
		 { 0, 0, 0, 1 }};

		// count coordinates in global cordinates from robot coordinates
		std::vector<double> gcs_vector = rda::mulMatrixOnVector<size>(robot_matrix, robot_vector);

		return rda::Point(gcs_vector[0], gcs_vector[1], 0.0/*gcs_vector[2]*/);
}

void rda::distances(rda::cloudPtr cloud, std::vector<double>& dists){

	for(int i = 0; i < cloud->size()-1; i++){
		dists.push_back(rda::distancePointToPoint(cloud->at(i), cloud->at(i+1)));		
	}
}

void rda::monotonePartitioning(rda::cloudPtr cloud, double max_dist, int min_part_size, std::vector<rda::CloudPart>& parts){	

	std::vector<double> dists;
	distances(cloud, dists);	

	int start = 0;	

	for(int i = 0; i < dists.size(); i++){

		if( (max_dist < dists[i]) || i == (dists.size() - 1) ){

			if(i - start >= min_part_size){
				rda::CloudPart part(cloud, Range(start, i));	
				parts.push_back(part);				
			}
			start = i + 1;
		}
	}	
}

void rda::naive_beakpoint_detector(std::vector<double>& distances, double max_diff, int min_points, std::vector<rda::Range>& indexes)
{
	int last = 0;	

	for(auto i = 0; i < distances.size() - 1; i++){		
		if( std::abs( distances[i] - distances[i+1]) > max_diff ){
			if( (i - last + 1) >= min_points)
				indexes.push_back(rda::Range(last, i));
			last = i + 1;
		}
	}

	if(distances.size() - last >= min_points) // close last segment
		indexes.push_back(rda::Range(last, distances.size() - 1));
}

void rda::naive_beakpoint_detector(std::vector<double>& distances, std::vector<int>& v_indexes, double max_diff, int min_points, std::vector<std::vector<int>>& indexes)
{
	std::vector<int> tmp;
	
	for(auto i = 0; i < v_indexes.size() - 1; i++){

		tmp.push_back(v_indexes[i]);

		if( std::abs( distances[v_indexes[i]] - distances[v_indexes[i+1]]) > max_diff ){
			if( tmp.size() >= min_points)
				indexes.push_back(tmp);	
			tmp.clear();
		}		
	}

	tmp.push_back(v_indexes.back());
	if(tmp.size() >= min_points)
		indexes.push_back(tmp);	
}

void rda::naiveBreakpointDetector(rda::cloudPtr cloud, std::vector<int>& v_indexes, double max_diff, int min_points, std::vector<std::vector<int>>& indexes)
{
	std::vector<int> tmp;
	
	for(auto i = 0; i < v_indexes.size() - 1; i++){

		tmp.push_back(v_indexes[i]);

		if( std::abs( rda::distancePointToPoint(cloud->at(v_indexes[i]), cloud->at(v_indexes[i+1])) ) > max_diff ){
			if( tmp.size() >= min_points)
				indexes.push_back(tmp);	
			tmp.clear();
		}		
	}

	tmp.push_back(v_indexes.back());
	if(tmp.size() >= min_points)
		indexes.push_back(tmp);	
}

double maxDistError(double distance, std::vector<std::pair<double, double>> errors)
{
	if(distance < errors.front().first)
		return errors.front().second;

	if(distance > errors.back().first)
		return errors.back().second;

	for(auto i = 0; i < errors.size() - 1; i++){
		if(distance >= errors[i].first && distance <= errors[i+1].first)
			return errors[i].second + (distance - errors[i].first) * (errors[i+1].second - errors[i].second) / (errors[i+1].first - errors[i].first);
	}
}

void rda::adaptiveNaiveDetector(std::vector<double>& distances, std::vector<std::pair<double, double>>& errors, int min_points, std::vector<rda::Range>& indexes)
{
	int last = 0;	
	std::cout << "Breakpoints:" << std::endl;
	for(auto i = 0; i < distances.size() - 1; i++){		
		if( std::abs( distances[i] - distances[i+1]) > maxDistError(std::min(distances[i],distances[i+1]) , errors) ){
			if( (i - last + 1) >= min_points){
				indexes.push_back(rda::Range(last, i));
				std::cout << distances[i] << " " << distances[i+1] << " error ";
				std::cout << std::abs( distances[i] - distances[i+1]) << " maxError " << maxDistError(std::min(distances[i],distances[i+1]) , errors) << std::endl;
			}
			last = i + 1;
		}
	}

	if(distances.size() - last >= min_points) // close last segment
		indexes.push_back(rda::Range(last, distances.size() - 1));
}

void rda::lineSegmentation(std::vector<rda::CloudPart>& parts, double threshold, std::vector<rda::CloudPart>& line_parts){	

	for(int i=0; i < parts.size(); i++){		
		rda::rdp_minimization(parts[i], parts[i].range().start, parts[i].range().end, threshold, line_parts);
	}
}

void rda::adaptiveLineSegmentation(std::vector<rda::CloudPart>& parts, int min_part_size, double min_error,std::vector<rda::CloudPart>& line_parts){

	for(int i=0; i < parts.size(); i++){		
		rda::adaptive_rdp(parts[i], min_part_size, min_error, line_parts);
	}
}

void rda::lsLineApproximation(std::vector<rda::CloudPart>& parts, std::vector<rda::ApproximiedCloudPart>& line_approx){

	for(int i = 0; i < parts.size(); i++){

		rda::Line corr_line(parts[i].first_point(), parts[i].last_point());
		rda::Vector oX(1, 0);
		
		double angle = rda::Vector::angle(corr_line.directionVector(), oX) * PI / 180.0;

		if(corr_line.k() > 0)
			angle *= -1;

		rda::cloudPtr rotated_cloud (new rda::cloud);
		rda::rotateCloud(parts[i].cloud(), parts[i].range().start, parts[i].range().end, -angle , rotated_cloud);		

		function::LeastSquares ls(1);
		ls.init(rotated_cloud, 0, rotated_cloud->points.size() - 1);
		ls.approximate();

		rda::cloudPtr appr_line_cloud (new rda::cloud);
		appr_line_cloud->push_back(rda::Point(rotated_cloud->at(0).x, ls.value(rotated_cloud->at(0).x), rotated_cloud->at(0).z)); 
		appr_line_cloud->push_back(rda::Point(rotated_cloud->back().x, 
											  ls.value(rotated_cloud->back().x), 
											  rotated_cloud->back().z));

		rda::cloudPtr unrotated_cloud (new rda::cloud);
		rda::rotateCloud(appr_line_cloud,  angle, unrotated_cloud);

		rda::Line line(unrotated_cloud->front(), unrotated_cloud->back());
		//rda::Line line(appr_line_cloud->front(), appr_line_cloud->back());

		line_approx.push_back(rda::ApproximiedCloudPart(parts[i], line));
	}
}

rda::Line rda::lsLineApproximation(rda::CloudPart& part){

	rda::Line corr_line(part.first_point(), part.last_point());
	rda::Vector oX(1, 0);
	
	double angle = rda::Vector::angle(corr_line.directionVector(), oX) * PI / 180.0;
	
	if(corr_line.k() > 0)
		angle *= -1;
	
	rda::cloudPtr rotated_cloud (new rda::cloud);
	rda::rotateCloud(part.cloud(), part.range().start, part.range().end, -angle , rotated_cloud);		
	
	function::LeastSquares ls(1);
	ls.init(rotated_cloud, 0, rotated_cloud->points.size() - 1);
	ls.approximate();
	
	rda::cloudPtr appr_line_cloud (new rda::cloud);
	appr_line_cloud->push_back(rda::Point(rotated_cloud->at(0).x, ls.value(rotated_cloud->at(0).x), rotated_cloud->at(0).z)); 
	appr_line_cloud->push_back(rda::Point(rotated_cloud->back().x, 
										  ls.value(rotated_cloud->back().x), 
										  rotated_cloud->back().z));
	
	rda::cloudPtr unrotated_cloud (new rda::cloud);
	rda::rotateCloud(appr_line_cloud,  angle, unrotated_cloud);
	
	rda::Line line(unrotated_cloud->front(), unrotated_cloud->back());	
	
	return line;
}


bool x_comp(rda::Point& p1, rda::Point& p2) {return p1.x < p2.x;}
bool y_comp(rda::Point& p1, rda::Point& p2) {return p1.y < p2.y;}

rda::Line extebdLine(rda::ApproximiedCloudPart p1, rda::ApproximiedCloudPart p2){
	rda::Line pl = rda::projectionLineToLine(p2.approx_line(), p1.approx_line());
	return rda::maxDiagonal(p1.approx_line(), pl);	
}

rda::Line merge_lines(rda::ApproximiedCloudPart l1, rda::ApproximiedCloudPart l2){
	rda::cloudPtr cloud ( new rda::cloud);
	double overlapping = 0.0;
	return rda::middle_line(l1, l2, &overlapping);	
}

// copy of vector
void rda::segmentsMerging(std::vector<rda::ApproximiedCloudPart> segments, double dist_threashold, double angle_threashold, std::vector<rda::ApproximiedCloudPart>& merged_segments){

	if(segments.size() < 1) 
		return;

	for(std::size_t i = 0; i < segments.size(); i++ ){

		for(std::size_t j = 0; j < segments.size(); j++){

			if(i != j){
				if(rda::is_similar(segments[i].approx_line(), segments[j].approx_line(), dist_threashold, angle_threashold)){
					double d = segments[j].approx_line().length() / segments[i].approx_line().length();					
					double m = segments[j].size() / (double)segments[i].size();
					if(m <= 1.0){ // j length is smaller than i length
						double overlapping = 0.0;
						Line mid_line = rda::middle_line(segments[i], segments[j], &overlapping);
						if( overlapping/mid_line.length() > 0.3) {							
							if(m > 0.25){
								segments[i].set_approx_line(mid_line); //merge
							}
							else{
								segments[i].set_approx_line(extebdLine(segments[i], segments[j])); //merge
							}
						}
						else{
							segments[i].set_approx_line(extebdLine(segments[i], segments[j])); //merge						
						}

						segments.erase(segments.begin() + j );
						if(j < i)
							i--;
						j--;
					}
				}
			}
		}
	}

	for(std::size_t i = 0; i < segments.size(); i++)
		merged_segments.push_back(segments[i]);
}

bool rda::is_similar(Line& line_1, Line& line_2, double dist_threashold, double angle_threashold){
	if( (rda::distanceSegmentToSegment(line_1, line_2) <= dist_threashold ) &&
		( rda::Vector::angle(line_1.directionVector(), line_2.directionVector()) <= angle_threashold) )
		return true;	
	return false;
}

double rda::compute_rdp_threashold(rda::cloudPtr dists, rda::cloudPtr ls_dists){

	std::vector<double> diffs;

	for(int i = 0; i < dists->size(); i++){
		diffs.push_back( std::fabs(dists->at(i).y - ls_dists->at(i).y)) ;
	}

	double av = 0;
	double st_d = rda::standardDeviation(diffs, av); // call avarage again !!!

	printf(" avarage = %f; standart dev. = %f \n", av, st_d);

	double max_diff = 0.0;

	for(auto it = diffs.begin(); it != diffs.end(); it++){
		//if( (*it) <= av + st_d)
			if(max_diff < (*it) ){
				max_diff = *it;
			}		
	}

	return 1.0 * (av + 2.0 * st_d);
}

void rda::statisticalDistanceFilter(std::vector<double>& distances, int k, double coef, std::vector<int>& indexes)
{
	rda::statisticalDistanceFilter(distances, rda::Range(0, distances.size() - 1), k, coef, indexes);

}

void rda::statisticalDistanceFilter(std::vector<double>& distances, rda::Range range, int k, double coef, std::vector<int>& indexes)
{
	std::vector<double> dists_sum; //values
	std::vector<int> dist_indexes; //values
 
	int half = k/2;

	// Begin
	for(int i = range.start; i < range.start + half ; i++){
		double sumDist = 0;
		for(int j = range.start; j < range.start + k ; j++){
			if(i != j){
				sumDist += std::abs(distances[i] - distances[j]);
			}
		}
		dist_indexes.push_back(i);
		dists_sum.push_back(sumDist);
	}

	//Middle
	for(int i = range.start + half; i <= range.end - half; i++){
		double sumDist = 0;
		for(int j = i - half; j <= i + half; j++){
			if(i != j){
				sumDist += std::abs(distances[i] - distances[j]);
			}
		}

		dist_indexes.push_back(i);
		dists_sum.push_back(sumDist);
	}

	//End
	for(int i = range.end - half + 1; i <= range.end ; i++){
		double sumDist = 0;
		for(int j = range.end - k + 1; j <= range.end; j++){
			if(i != j){
				sumDist += std::abs(distances[i] - distances[j]);
			}
		}
		dist_indexes.push_back(i);
		dists_sum.push_back(sumDist);
	}

	double av = 0;
	double st_d = rda::standardDeviation(dists_sum, av);
	double threshold = st_d * coef;

	for(int i = 0; i < dists_sum.size(); i++){
		if(dists_sum[i] <= av + threshold)
			indexes.push_back(dist_indexes[i]);
	}
}

void rda::statisticalDistanceFilterDebug(std::vector<double>& distances, rda::Range range, int k, double coef, std::vector<int>& indexes, std::vector<double>& dists_sum)
{	
	std::vector<int> dist_indexes; //values
 
	int half = k/2;

	// Begin
	for(int i = range.start; i < range.start + half ; i++){
		double sumDist = 0;
		for(int j = range.start; j < range.start + k ; j++){
			if(i != j){
				sumDist += std::abs(distances[i] - distances[j]);
			}
		}
		dist_indexes.push_back(i);
		dists_sum.push_back(sumDist);
	}

	//Middle
	for(int i = range.start + half; i <= range.end - half; i++){
		double sumDist = 0;
		for(int j = i - half; j <= i + half; j++){
			if(i != j){
				sumDist += std::abs(distances[i] - distances[j]);
			}
		}

		dist_indexes.push_back(i);
		dists_sum.push_back(sumDist);
	}

	//End
	for(int i = range.end - half + 1; i <= range.end ; i++){
		double sumDist = 0;
		for(int j = range.end - k + 1; j <= range.end; j++){
			if(i != j){
				sumDist += std::abs(distances[i] - distances[j]);
			}
		}
		dist_indexes.push_back(i);
		dists_sum.push_back(sumDist);
	}

	double av = 0;
	double st_d = rda::standardDeviation(dists_sum, av);
	double threshold = st_d * coef;

	for(int i = 0; i < dists_sum.size(); i++){
		if(dists_sum[i] <= av + threshold)
			indexes.push_back(dist_indexes[i]);
	}
}