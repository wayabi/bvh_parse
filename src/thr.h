#ifndef __MYC_THR__
#define __MYC_THR__

#include <boost/math/quaternion.hpp>

typedef boost::math::quaternion<double> Q;

class THR {
public:
	THR();
	THR(double x, double y, double z);
	THR(const Q& q);
	THR(double w, double x, double y, double z);

	THR& operator=(const THR& a);
	THR operator+(const THR& a) const;
	THR operator-(const THR& a) const;
	THR operator*(double a) const;
	THR operator/(double a) const;
	bool operator==(const THR& a) const;
	bool nearly_equal(const THR& a) const;
	double get_size() const;
	double get_size_sqr() const;
	THR rotate(double theta, const char* axis);
	double dot(const THR& a);
	THR cross(const THR& a);
	THR normalize();
	THR rotate(Q q);
	THR convert_coordinate(THR& rotate);
	THR matrix(double mat[9]);
	static THR get_rand();
	void print(const char* name = NULL);
	Q q();

public:
	double w_;
	double x_;
	double y_;
	double z_;
};

#endif
