
// Interface ApproximateMethod

#ifndef APPROXIMATE_MATHOD_H
#define APPROXIMATE_MATHOD_H


#include <rda\function\Function.h>
#include <rda\common\common.h>


namespace function {

class ApproximateMethod : public Function
{
public:

	void init(rda::cloudPtr cloud, int begin_index, int end_index);

	// creates approximate function
	virtual int approximate() = 0;

 	virtual ~ApproximateMethod();

protected:

	rda::cloudPtr cloud;
	int begin;
	int end;

};

}

#endif