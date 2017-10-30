#include "pol.h"
#include <algorithm>

POL::POL(THR p1, THR p2, THR p3) :
	p1_(p1),
	p2_(p2),
	p3_(p3)
{
}

bool POL::check_hit_half_line(THR pos_start, THR dir)
{
	THR a1 = p1_-pos_start;
	THR a2 = p2_-pos_start;
	THR a3 = p3_-pos_start;
	
	THR b1 = a1.cross(a2);
	THR b2 = a2.cross(a3);
	THR b3 = a3.cross(a1);

	double c1 = b1.dot(dir);
	double c2 = b2.dot(dir);
	double c3 = b3.dot(dir);
	if(c1 <= 0 && c2 <= 0 && c3 <= 0) return true;
	if(c1 >= 0 && c2 >= 0 && c3 >= 0) return true;
	return false;
}

void z_sort(std::vector<POL>& pol)
{
	std::sort(pol.begin(), pol.end(), [](const POL& a, const POL& b) ->
		int {
			return (a.p1_.z_+a.p2_.z_+a.p3_.z_ < b.p1_.z_+b.p2_.z_+b.p3_.z_);
		}
	);
}

THR POL::norm()
{
	THR v1 = p2_ - p1_;
	THR v2 = p3_ - p1_;
	return v1.cross(v2).normalize();
}

PolColor::PolColor(POL pol, int r, int g, int b) :
	pol_(pol), r_(r), g_(g), b_(b)
{
}
