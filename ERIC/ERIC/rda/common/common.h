
#ifndef RDA_COMMON
#define RDA_COMMON

#include <vector>
#include <string>

#include <pcl\point_cloud.h>
#include <pcl\point_types.h>

#include <rda\rdaException.h>

namespace rda {

	typedef pcl::PointXYZ Point;

	typedef pcl::PointCloud<pcl::PointXYZ> cloud;

	typedef pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr;	
	
	struct Range {
		int start;
		int end;

		Range(){
			this->start = 0;
			this->end = 0;
		}

		Range(int start, int end){
			this->start = start;
			this->end = end;
		}

		int size() { return std::abs(end - start); }
	};

	typedef struct { Point bottom; double height; double width; } BoundingBox;


	void split(std::string str, char* delimiter, std::vector<std::string>& parts);

	double distancePointToLine(pcl::PointXYZ& line_start, pcl::PointXYZ& line_end, pcl::PointXYZ& point);

	double distancePointToPoint(rda::Point& point_1, rda::Point& point_2);

	double distancePointToSegment(rda::Point& segment_start, rda::Point& segment_end, rda::Point& point);	

	double avarage(std::vector<double>& values);

	double standardDeviation(std::vector<double>& values, double& avarage);

	template<typename T, class InputIterator>
	T mean_value(InputIterator begin, InputIterator end)
	{
		T mean = T(0);
		auto size = end - begin;

		while(begin != end){
			mean += *begin;
			++begin;
		}
		return mean / size;
	}

	template<typename T, class InputIterator>
	T standart_deviation(InputIterator begin, InputIterator end, T& mean)
	{
		mean = mean_value<T>(begin, end);
		T sum = 0;
		auto size = end - begin;

		while(begin != end){
			sum += pow(*begin - mean, 2);
			++begin;
		}

		return sqrt(sum / size);
	}

	BoundingBox boundingBox(rda::cloudPtr cloud);

	//returns clouPtr with vertexes of bounding box
	rda::cloudPtr boundingBoxVertices(rda::cloudPtr cloud);

	void rotateCloud(rda::cloudPtr cloud, double angle_rad, rda::cloudPtr ratated_cloud);

	void rotateCloud(rda::cloudPtr cloud, int start, int end, double angle_rad, rda::cloudPtr ratated_cloud);

	// matrix is size * size ; vecot is size
	template <size_t N>
	std::vector<double> mulMatrixOnVector(double (&matrix)[N][N], std::vector<double>& vector){
		std::vector<double> result;
		for(int i = 0; i < N; i++){
			double el = 0.0;
			for(int j = 0; j < N; j++){
				el += matrix[i][j] * vector[j]; 
			}
			result.push_back(el);
		}
		return result;
	}
}


#endif