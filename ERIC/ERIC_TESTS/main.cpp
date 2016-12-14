

#define BOOST_TEST_MODULE MyTest

#include <rda\common\common.h>
#include <rda\common\line.h>
#include <rda\vector.h>
#include <rda\curve.h>

#include <math.h>
#include <iostream>
	
#include <boost\test\unit_test.hpp>

BOOST_AUTO_TEST_CASE(my_test){

	//BOOST_CHECK( rda::distancePointToSegment(rda::Point(-1, 0, 0),  rda::Point(1, 0, 0), rda::Point(1, 0, 0))  == 0.0);	
	
	rda::Line l1(rda::Point(0,-1,0), rda::Point(0,2,0));
	rda::Line l2(rda::Point(1,0,0), rda::Point(1,1,0));
	rda::Line l3(rda::Point(1,0,0), rda::Point(1.1,1,0));

	std::cout << rda::Vector::isParallel(l1.directionVector(), l2.directionVector()) << std::endl;
	std::cout << rda::Vector::isParallel(l1.directionVector(), l3.directionVector()) << std::endl;

	std::cout << l1.directionVector().x << " " << l1.directionVector().y << std::endl;
	std::cout << l3.directionVector().x << " " << l3.directionVector().y << std::endl;

	std::cout << "-----Fin---------" << std::endl;
	std::cin.get();
}
