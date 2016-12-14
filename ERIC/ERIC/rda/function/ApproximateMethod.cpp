/*
 * ApproximateMethod.cpp
*/

#include "ApproximateMethod.h"

using namespace function;

void ApproximateMethod::init(rda::cloudPtr cloud, int begin_index, int end_index){

	this->cloud = cloud;
	this->begin = begin_index;
	this->end = end_index;
}

ApproximateMethod::~ApproximateMethod(){

}