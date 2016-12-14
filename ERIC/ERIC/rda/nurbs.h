
#ifndef RDA_NURBS_H
#define RDA_NURBS_H

#include <pcl\point_types.h>
#include <pcl\point_cloud.h>

#include <rda\common\common.h>


namespace rda {
	
	void nurbsFitClosedCurve2dPDM(rda::cloudPtr input_cloud, unsigned order, unsigned n_control_points, double smoothness, double rScale, unsigned  int resolution,  rda::cloudPtr curve_cloud);

	void nurbsFitCurve2d(rda::cloudPtr input_cloud, unsigned order, unsigned n_control_points, double smoothness, double rScale, unsigned  int resolution,  rda::cloudPtr curve_cloud);

		// true - nurbsFitClosedCurve2dPDM better for approximation false - nurbsFitCurve2d
	bool isClosed(rda::cloudPtr curve_cloud);

}

#endif