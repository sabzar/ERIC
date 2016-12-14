
#ifndef RDA_CURVE_H
#define RDA_CURVE_H

#include <pcl\point_types.h>
#include <pcl\point_cloud.h>

#include <rda\common\common.h>
#include <rda\common\cloud_part.h>
#include <rda\approximied_cloud_part.h>
#include <rda\vector.h>
#include <rda\common\line.h>

namespace rda {
	
	//Ramer Douglas Peucker algorithm
	void rdp_minimization(rda::CloudPart cloud_part, int start, int end, double threshold, std::vector<rda::CloudPart>& lines);

	void adaptive_rdp(rda::CloudPart cloud_part, int min_part_size, double min_error, std::vector<rda::CloudPart>& lines);

	void simple_moving_avarage(rda::cloudPtr cloud, int window_size, rda::cloudPtr sma_cloud);

	// value of overlaping of projections of line_1 anf line_2 undo middle_line
	rda::Line middle_line(rda::ApproximiedCloudPart acp_1, rda::ApproximiedCloudPart acp_2, double* overlaping);

	rda::Point projectionPointToLine(rda::Line line, rda::Point& point);

	rda::Point projectionPointToLine(double line_angle, Point& line_mid_point, Point& point);

	//projects line_1 to line_2 
	rda::Line projectionLineToLine(rda::Line line_1, rda::Line line_2);

	rda::Line maxDiagonal(rda::Line& line_1, rda::Line& line_2);

}

#endif