#ifndef RDA_CLOUD_PART_H
#define RDA_CLOUD_PART_H

#include <rda\common\common.h>
#include <rda\common\line.h>

namespace rda {

	class CloudPart {

	private:

		//CloudPart();

	protected:

		rda::cloudPtr cloud_;
		rda::Range range_;

	public: 		

		CloudPart(rda::cloudPtr cloud, rda::Range range);

		rda::cloudPtr cloud();

		rda::Point& at(int index);

		rda::Point& first_point();

		rda::Point& last_point();

		rda::Range& range();

		int begin();

		int end();

		int size();

		// Line from first to last point
		rda::Line line(); 
	};
}

#endif
