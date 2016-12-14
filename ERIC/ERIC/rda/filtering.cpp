
#include "filtering.h"


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

	for(int i = 0; i < values.size(); i++){
		for(int j = 0; j < buf.size(); j++){
			int k = i + j - wsize/2;
			//if(k < 0)
				buf[j] = values.front();
			if(k >= values.size())
				buf[j] = values.back();
			if( k >= 0 && k < values.size())
				buf[j] = values[k];
		}
		std::sort(buf.begin(), buf.end());
		output[i] = buf[wsize/2];
	}
}