
/*
 * Cluster.cpp
*/

#include "Cluster.h"


using namespace rda;

Cluster::Cluster(Point* p){

	points.push_back(p);
	center = 0;
}

Cluster::Cluster(){

	center = 0;
}

Cluster::Cluster(std::vector<Point*>& p){
	this->points = p;
	center = 0;
}

std::vector<Point*>& Cluster::getPoints(){
	return points;
}

void Cluster::addPoint(Point* cell){
	points.push_back(cell);
}

Point Cluster::centerPoint(){
	if(center != 0)
		return *center;

	if(points.size() < 1)
		return Point(0,0,0,-1,0,0);
	Point max = *points[0];
	Point min = *points[0];

	for(int i=0; i < points.size(); i++){
		if ((*points[i]).x > max.x)
			max.x = (*points[i]).x;
		if ((*points[i]).y > max.y)
			max.y = (*points[i]).y;
		if ((*points[i]).x < min.x)
			min.x = (*points[i]).x;
		if ((*points[i]).y < min.y)
			min.y = (*points[i]).y;
	}
	center = new Point();
	(*center).x  = min.x + (max.x - min.x )/3;
	(*center).y  = min.y + (max.y - min.y )/2;

	return *center;
}

Cluster::~Cluster(){
	points.clear();
}