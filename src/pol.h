#ifndef __MYC_POL__
#define __MYC_POL__

#include "thr.h"
#include <boost/math/quaternion.hpp>

typedef boost::math::quaternion<double> Q;

class POL {
public:
	POL(){}
	POL(THR p1, THR p2, THR p3);
	bool check_hit_half_line(THR pos_start, THR dir);
	void z_sort(std::vector<POL>& pol);
	THR norm();

public:
	THR p1_;
	THR p2_;
	THR p3_;
};

class PolColor {
public:
	PolColor(){}
	PolColor(POL pol, int r, int g, int b);
public:
	POL pol_;
	int r_;
	int g_;
	int b_;
};

#endif
