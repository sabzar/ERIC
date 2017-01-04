
/*
 * Functions only for sorted point clouds 
*/

#ifndef RDA_RANGEFINDER_H_
#define RDA_RANGEFINDER_H_

#include <vector>

#include <rda\common\common.h>
#include <rda\common\cloud_part.h>
#include <rda\approximied_cloud_part.h>
#include <rda\curve.h>

namespace rda {	

	struct RPoint {
		double x_robot;
		double y_robot;
		double angle_robot;

		RPoint(double x_rob, double y_rob, double angle_rob){
			this->x_robot = x_rob;
			this->y_robot = y_rob;
			this->angle_robot = angle_rob;			
		}
	};

	void computePointCloud(std::vector<rda::RPoint> rob_point, std::vector<double> distances, rda::cloudPtr cloud, int sensor);

	void computePointCloud(std::vector<rda::RPoint> rob_point, std::vector<double> distances, std::vector<int>& indexes, rda::cloudPtr cloud, int sensor);

	rda::Point computePoint(RPoint& rob_point, double distance, int sensor_id);

	//Finds distances from i to i+1 point 
	void distances(rda::cloudPtr cloud, std::vector<double>& dists);

	void monotonePartitioning(rda::cloudPtr cloud, double max_dist, int min_part_size, std::vector<rda::CloudPart>& parts);

	void naive_beakpoint_detector(std::vector<double>& distances, double max_diff, int min_points, std::vector<rda::Range>& indexes);

	void adaptiveNaiveDetector(std::vector<double>& distances, std::vector<std::pair<double, double>>& errors, int min_points, std::vector<rda::Range>& indexes);

	void lineSegmentation(std::vector<rda::CloudPart>& parts, double threshold, std::vector<rda::CloudPart>& line_parts);	

	void adaptiveLineSegmentation(std::vector<rda::CloudPart>& parts, int min_part_size, double min_error, std::vector<rda::CloudPart>& line_parts);

	//Least Squares Line Approximation
	void lsLineApproximation(std::vector<rda::CloudPart>& parts, std::vector<rda::ApproximiedCloudPart>& line_approx);

	rda::Line lsLineApproximation(rda::CloudPart& part);

	void segmentsMerging(std::vector<rda::ApproximiedCloudPart> segments, double dist_threashold, double angle_threashold, std::vector<rda::ApproximiedCloudPart>& merged_segments);

	bool is_similar(Line& line_1, Line& line_2, double dist_threashold, double angle_threashold);

	double compute_rdp_threashold(rda::cloudPtr dists, rda::cloudPtr ls_dists);

	void statisticalDistanceFilter(std::vector<double>& distances, int neighbours_number, double threshold, std::vector<int>& indexes);
}

#endif
