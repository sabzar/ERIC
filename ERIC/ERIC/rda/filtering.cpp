
#include "filtering.h"
#include <rda\common\common.h>
#include <list>
#include <algorithm>

void rda::statisticalFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ> &filtered_cloud, int k, double thr ){
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud);
	sor.setMeanK(k);
	sor.setStddevMulThresh(thr);
	sor.filter(filtered_cloud);	
}

void rda::medianFilter(std::vector<double>& values, int wsize, std::vector<double>& output){

	output.resize(values.size());

	std::vector<double> buf(wsize);
	int n = values.size(); // to int 
	for(int i = 0; i < values.size(); i++){
		for(int j = 0; j < buf.size(); j++){
			int k = i + j - wsize/2;
			//if(k < 0)
				buf[j] = values.front();
			if(k >= n)
				buf[j] = values.back();
			if( k >= 0 && k < values.size())
				buf[j] = values[k];
		}
		std::sort(buf.begin(), buf.end());
		output[i] = buf[wsize/2];
	}
}

void rda::reduce_median_filter(std::vector<double>& values, int wsize, std::vector<int>& indexes)
{
	std::vector<double> v(values.begin(), values.end());

	std::vector<double> buf(wsize);
	int n = v.size(); // to int 
	for(auto i = 0; i < v.size(); i++){
		for(auto j = 0; j < buf.size(); j++){
			int k = i + j - wsize/2;
			//if(k < 0)
				buf[j] = v.front();
			if(k >= n)
				buf[j] = v.back();
			if( k >= 0 && k < v.size())
				buf[j] = v[k];
		}

		std::sort(buf.begin(), buf.end());
		if(v[i] == buf[wsize/2])
			indexes.push_back(i);

		//v[i] = buf[wsize/2];

	}		
}

void rda::kuwahara_filter(const std::vector<double>& values, int w_size, std::vector<double>& output){

	output.resize(values.size());
	std::copy(values.begin(), values.end(), output.begin());

	if(values.size() > w_size){
		for(auto i = w_size; i < output.size() - w_size - 1; i++){
			double mean_left = 0.0;
			double mean_right = 0.0;
			double st_dev_left = rda::standart_deviation(&output[i-w_size], &output[i], mean_left);
			double st_dev_right = rda::standart_deviation(&output[i+1], &output[i+w_size+1], mean_right);			

			if(st_dev_left < st_dev_right)
				output[i] = mean_left;
			else
				output[i] = mean_right;
		}
	}		
}