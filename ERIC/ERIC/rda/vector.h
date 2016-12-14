

#ifndef RDA_VECTOR_H
#define RDA_VECTOR_H

#include <rda\common\common.h>
#include <math.h>

#define PI 3.14159265


namespace rda {

	struct Vector {

		double x;
		double y;
		
	//public: 

		Vector(double x, double y){
			this->x = x;
			this->y = y;
		}

		Vector(rda::Point& start, rda::Point& end){
			x = end.x - start.x;
			y = end.y - start.y;
		}

		double operator*(const Vector& v){
			return (this->x*v.x + this->y*v.y);
		}

		double length(){
			return std::sqrt( x*x  + y*y );
		}

		Vector unit(){					
			return Vector(this->x / this->length(), this->y / this->length());
		}

		static bool isParallel(Vector& vec_1, Vector& vec_2){
			double kx = 0;
			double ky = 0;

			vec_1.x == vec_2.x ? kx = 1 : kx = vec_1.x/vec_2.x;
			vec_1.y == vec_2.y ? ky = 1 : ky = vec_1.y/vec_2.y;				
			
			return ( fabs(kx - ky) <= 0.0000001); // ?
		}

		static double angle(Vector& vec_1, Vector& vec_2){
			double fi = acos( vec_1*vec_2/(vec_1.length() * vec_2.length()) ) * 180.0/PI ;
			if(fi > 90.0)
				return 180 - fi;
			return fi;
		}
	};

}


#endif