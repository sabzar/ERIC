
#ifndef RDA_FILTERING_H
#define RDA_FILTERING_H

#include <pcl\point_types.h>
#include <pcl\point_cloud.h>
#include <pcl\filters\statistical_outlier_removal.h>

namespace rda {

	void statisticalFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ> &filtered_cloud, int neighbour, double thresh );

	void medianFilter(std::vector<double>& values, int wsize, std::vector<double>& output);

	void reduce_median_filter(std::vector<double>& values, int wsize, std::vector<int>& indexes);

	// final window size is 2*window_size + 1
	void kuwahara_filter(const std::vector<double>& values, int window_size, std::vector<double>& output);
}

#endif