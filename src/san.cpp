#include "san.h"
#include "qua.h"
#include <boost/math/quaternion.hpp>
#include "thr.h"

using namespace std;
typedef boost::math::quaternion<double> Q;

int ccc = 30;

SAN::SAN() :
	img_(w_img_, h_img_)
{
	len_cube_ = 1.0;
	img_.sla(0, w_img_/2, w_img_, w_img_/2, ccc, ccc, ccc);
	img_.sla(w_img_/2, 0, w_img_/2, w_img_, ccc, ccc, ccc);
}

void SAN::plot(std::vector<std::vector<double> >& p, int r, int g, int b)
{
	for(auto ite = p.begin();ite != p.end();++ite){
		plot(*ite, r, g, b);
	}
}

void SAN::point(int x, int y, int r, int g, int b)
{
	img_.cir(x, y, 2, r, g, b, true);
}

void SAN::plot(std::vector<double>& p, int r, int g, int b)
{
	int len = w_img_/2;
	double x = p.at(0);
	double y = p.at(1);
	double z = p.at(2);
	int ix = len*x/(2*len_cube_)+len/2;
	int iy = -len*y/(2*len_cube_)+len/2;
	int iz1 = len*z/(2*len_cube_)+len/2;
	int iz2 = -len*z/(2*len_cube_)+len/2;
	int px = 0;
	int py = 0;
	point(px+ix, py+iz2, r, g, b);
	px = 0;
	py = w_img_/2;
	point(px+ix, py+iy, r, g, b);
	px = w_img_/2;
	py = w_img_/2;
	point(px+iz1, py+iy, r, g, b);

	px = w_img_/2;
	py = 0;
	THR from(1, 0, 0);
	THR to(-1, 1, 1);
	to = to.normalize();
	Q q = qua::get_quaternion_from_vector(to.x_, to.y_, to.z_, from.x_, from.y_, from.z_);
	THR a(x, y, z);
	THR bb(q*a.q()/q);
	ix = len*bb.x_/(2*len_cube_)+len/2;
	iy = -len*bb.y_/(2*len_cube_)+len/2;
	iz1 = len*bb.z_/(2*len_cube_)+len/2;
	point(px+ix, py+iz1, r, g, b);

}

void SAN::sla(std::vector<double>& p1, std::vector<double>& p2, int r, int g, int b)
{
	int len = w_img_/2;
	int ix1 = len*p1.at(0)/(2*len_cube_)+len/2;
	int iy1 = -len*p1.at(1)/(2*len_cube_)+len/2;
	int iz11 = len*p1.at(2)/(2*len_cube_)+len/2;
	int iz12 = -len*p1.at(2)/(2*len_cube_)+len/2;
	int ix2 = len*p2.at(0)/(2*len_cube_)+len/2;
	int iy2 = -len*p2.at(1)/(2*len_cube_)+len/2;
	int iz21 = len*p2.at(2)/(2*len_cube_)+len/2;
	int iz22 = -len*p2.at(2)/(2*len_cube_)+len/2;
	int px = 0;
	int py = 0;
	img_.sla(px+ix1, py+iz12, px+ix2, py+iz22, r, g, b);
	px = 0;
	py = w_img_/2;
	img_.sla(px+ix1, py+iy1, px+ix2, py+iy2, r, g, b);
	px = w_img_/2;
	py = w_img_/2;
	img_.sla(px+iz11, py+iy1, px+iz21, py+iy2, r, g, b);

	px = w_img_/2;
	py = 0;
	THR from(1, 0, 0);
	THR to(-1, 1, 1);
	to = to.normalize();
	Q q = qua::get_quaternion_from_vector(to.x_, to.y_, to.z_, from.x_, from.y_, from.z_);
	THR af(p1.at(0), p1.at(1), p1.at(2));
	THR at(q*af.q()/q);
	THR bf(p2.at(0), p2.at(1), p2.at(2));
	THR bt(q*bf.q()/q);
	ix1 = len*at.x_/(2*len_cube_)+len/2;
	iy1 = -len*at.y_/(2*len_cube_)+len/2;
	iz11 = len*at.z_/(2*len_cube_)+len/2;
	ix2 = len*bt.x_/(2*len_cube_)+len/2;
	iy2 = -len*bt.y_/(2*len_cube_)+len/2;
	iz21 = len*bt.z_/(2*len_cube_)+len/2;
	img_.sla(px+ix1, py+iz11, px+ix2, py+iz21, r, g, b);
}

