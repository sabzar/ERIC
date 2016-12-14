
/*
 * Cell.cpp
*/

#include "Cell.h"

using namespace rda;

Cell::Cell(){
	cellValue = 0;
}

std::vector<Point*>& Cell::getPoints(){
	return points;
}


Point* Cell::getPoint(int i){
	return points[i];
}

int Cell::getValue(){
	return cellValue;
}

void Cell::setValue(int nValue){
	cellValue = nValue;
}

void Cell::incValue(){
	cellValue++;
}

void Cell::pushPoint(Point* p){
	points.push_back(p);
}

Cell::~Cell(){
	points.clear();
}