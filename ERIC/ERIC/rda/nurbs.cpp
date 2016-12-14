
#include <pcl\surface\concave_hull.h> // for pcl::copyPointCloud 

#include <pcl\surface\on_nurbs\fitting_curve_2d.h>
#include <pcl\surface\on_nurbs\fitting_curve_2d_tdm.h>
#include <pcl\surface\on_nurbs\fitting_curve_2d_sdm.h>
#include <pcl\surface\on_nurbs\fitting_curve_2d_pdm.h>
#include <pcl\surface\on_nurbs\triangulation.h>

#include <rda\nurbs.h>

void PointCloud2Vector3d (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::on_nurbs::vector_vec2d &data) {
  for (unsigned i = 0; i < cloud->size (); i++) {
    pcl::PointXYZ &p = cloud->at (i);
    if (!pcl_isnan (p.x) && !pcl_isnan (p.y) && !pcl_isnan (p.z)){
		//data.push_back (Eigen::Vector3d (p.x, p.y, p.z));
		data.push_back(Eigen::Vector2d(p.x, p.y));
	}
  }
}

void rda::nurbsFitClosedCurve2dPDM(rda::cloudPtr input_cloud, unsigned order, unsigned n_control_points, double smoothness, double rScale, unsigned int resolution, rda::cloudPtr curve_cloud_result){

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr curve_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

		pcl::PCLPointCloud2 cloud2;

		//pcl::fromPCLPointCloud2(cloud2, *cloud);		
		pcl::on_nurbs::NurbsDataCurve2d data;

		PointCloud2Vector3d(input_cloud, data.interior);

		pcl::on_nurbs::FittingCurve2dPDM::Parameter curve_params;
		curve_params.smoothness = smoothness;
		curve_params.rScale = rScale;		

		ON_NurbsCurve curve = pcl::on_nurbs::FittingCurve2dPDM::initNurbsCurve2D(order, data.interior, n_control_points);

		pcl::on_nurbs::FittingCurve2dPDM fit(&data, curve);
		fit.assemble(curve_params);
		fit.solve();

		pcl::on_nurbs::Triangulation::convertCurve2PointCloud(fit.m_nurbs, curve_cloud, resolution);
		
		pcl::copyPointCloud(*curve_cloud, *curve_cloud_result);

}

void rda::nurbsFitCurve2d(rda::cloudPtr input_cloud, unsigned order, unsigned n_control_points, double smoothness, double rScale, unsigned  int resolution,  rda::cloudPtr curve_cloud_result){

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr curve_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

		pcl::PCLPointCloud2 cloud2;

		//pcl::fromPCLPointCloud2(cloud2, *cloud);		
		pcl::on_nurbs::NurbsDataCurve2d data;

		PointCloud2Vector3d(input_cloud, data.interior);

		pcl::on_nurbs::FittingCurve2d::Parameter curve_params;
		curve_params.smoothness = smoothness;
		curve_params.rScale = rScale;		

		ON_NurbsCurve curve = pcl::on_nurbs::FittingCurve2d::initNurbsPCA(order, &data, n_control_points);

		pcl::on_nurbs::FittingCurve2d fit(&data, curve);
		fit.assemble(curve_params);
		fit.solve();

		pcl::on_nurbs::Triangulation::convertCurve2PointCloud(fit.m_nurbs, curve_cloud, resolution);
		
		pcl::copyPointCloud(*curve_cloud, *curve_cloud_result);
}


bool rda::isClosed(rda::cloudPtr input_cloud){

	double closed_error = 0;
	double open_error = 0;

	rda::cloudPtr closed_curve (new rda::cloud );
	rda::cloudPtr open_curve (new rda::cloud );

	nurbsFitClosedCurve2dPDM(input_cloud, 2, 5, 1.0, 1.0, 8, closed_curve);
	nurbsFitCurve2d(input_cloud, 2, 5, 1.0, 1.0, 8, open_curve);

	for(int i=0; i < input_cloud->size(); i++){

		double closed_err = 9999999;
		double open_err = 9999999;

		// min for closed 
		for(int j = 0; j < closed_curve->size() - 1; j++){
			double curr_err = rda::distancePointToSegment(closed_curve->at(j), closed_curve->at(j+1), input_cloud->at(i));
			if(curr_err < closed_err){
				closed_err = curr_err;
			}
		}

		for(int j = 0; j < open_curve->size() - 1; j++){
			double curr_err = rda::distancePointToSegment(open_curve->at(j), open_curve->at(j+1), input_cloud->at(i));
			if(curr_err < open_err){
				open_err = curr_err;
			}
		}


		closed_error += closed_err;
		open_error += open_err;
	}
	
	//std::cout << "Closed error : " << closed_error << std::endl;  
	//std::cout << "Open error : " << open_error << std::endl;

	return closed_error < open_error;
}
