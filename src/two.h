#ifndef __MYC_TWO__
#define __MYC_TWO__

#include <boost/math/quaternion.hpp>


class TWO {
public:
	TWO();
	TWO(double x, double y);

	TWO& operator=(const TWO& a);
	TWO operator+(const TWO& a);
	TWO operator-(const TWO& a);
	TWO operator*(double a);
	bool operator==(const TWO& a);
	double get_size();
	double get_size_sqr();
	double dot(const TWO& a);
	TWO normalize();
	void print(const char* tag);

public:
	double x_;
	double y_;
};

#endif
