#include "thr.h"
#include <math.h>
#include <stdio.h>

#include <boost/random.hpp>

THR::THR()
{
	x_ = 0.0;
	y_ = 0.0;
	z_ = 0.0;
	w_ = 0.0;
}

THR::THR(double x, double y, double z)
{
	x_ = x;
	y_ = y;
	z_ = z;
	w_ = 0.0;
}

THR::THR(const Q& q)
{
	x_ = q.R_component_2();
	y_ = q.R_component_3();
	z_ = q.R_component_4();
	w_ = q.R_component_1();
}

THR::THR(double w, double x, double y, double z)
{
	w_ = w;
	x_ = x;
	y_ = y;
	z_ = z;
}

bool THR::operator==(const THR& a) const
{
	return (x_ == a.x_ && y_ == a.y_ && z_ == a.z_);
}

THR& THR::operator=(const THR& a)
{
	x_ = a.x_;
	y_ = a.y_;
	z_ = a.z_;
	w_ = a.w_;
	return *this;
}

THR THR::operator+(const THR& a) const
{
	THR ret;
	ret.x_ = x_+a.x_;
	ret.y_ = y_+a.y_;
	ret.z_ = z_+a.z_;
	ret.w_ = w_+a.w_;
	return ret;
}

THR THR::operator-(const THR& a) const
{
	THR ret;
	ret.x_ = x_-a.x_;
	ret.y_ = y_-a.y_;
	ret.z_ = z_-a.z_;
	ret.w_ = w_-a.w_;
	return ret;
}

THR THR::operator*(double a) const
{
	THR ret;
	ret.x_ = x_*a;
	ret.y_ = y_*a;
	ret.z_ = z_*a;
	ret.w_ = w_*a;
	return ret;
}

THR THR::rotate(double theta, const char* axis)
{
	double x = x_;
	double y = y_;
	double z = z_;
	if(*(axis+0) == 'x'){
		y = y_*cos(theta) - z_*sin(theta);
		z = y_*sin(theta) + z_*cos(theta);
	}else if(*(axis+0) == 'y'){
		z = z_*cos(theta) - x_*sin(theta);
		x = z_*sin(theta) + x_*cos(theta);
	}else if(*(axis+0) == 'z'){
		x = x_*cos(theta) - y_*sin(theta);
		y = x_*sin(theta) + y_*cos(theta);
	}
	return THR(x, y, z);
}

void THR::print(const char* c)
{
	if(c == NULL){
		printf("%f, %f, %f, w=%f\n", x_, y_, z_, w_);
	}else{
		printf("%s %f, %f, %f w=%f\n", c, x_, y_, z_, w_);
	}
}

double THR::dot(const THR& a)
{
	return x_*a.x_+y_*a.y_+z_*a.z_;
}

THR THR::cross(const THR& a)
{
	return THR(
		y_*a.z_ - z_*a.y_,
		z_*a.x_ - x_*a.z_,
		x_*a.y_ - y_*a.x_
	);
}

THR THR::normalize()
{
	double sum = sqrt(x_*x_+y_*y_+z_*z_);
	if(sum == 0.0) return THR(0, 0, 0);
	return THR(x_/sum, y_/sum, z_/sum);
}

THR THR::rotate(Q q)
{
	Q t(0, x_, y_, z_);
	Q a = q*t/q;
	return THR(0, a.R_component_2(), a.R_component_3(), a.R_component_4());
}

THR THR::matrix(double mat[9])
{
	double x = mat[0]*x_+mat[1]*y_+mat[2]*z_;
	double y = mat[3]*x_+mat[4]*y_+mat[5]*z_;
	double z = mat[6]*x_+mat[7]*y_+mat[8]*z_;
	return THR(x, y, z);
}

THR THR::convert_coordinate(THR& r)
{
//1 0 0  c 0-s  c s 0
//0 c s  0 1 0 -s c 0
//0-s c  s 0 c  0 0 1

	double mat_x[] = {
		1, 0, 0,
		0, cos(r.x_), sin(r.x_),
		0, -sin(r.x_), cos(r.x_)
	};
	double mat_y[] = {
		cos(r.y_), 0, -sin(r.y_),
		0, 1, 0,
		sin(r.y_), 0, cos(r.y_),
	};
	double mat_z[] = {
		cos(r.z_), sin(r.z_), 0,
		-sin(r.z_), cos(r.z_), 0,
		0, 0, 1
	};

	//return matrix(mat_z).matrix(mat_y).matrix(mat_x);
	return matrix(mat_x).matrix(mat_y).matrix(mat_z);
}

double THR::get_size() const
{
	double sum = sqrt(x_*x_+y_*y_+z_*z_);
	return sum;
}

double THR::get_size_sqr() const
{
	return x_*x_+y_*y_+z_*z_;
}

Q THR::q()
{
	return Q(w_, x_, y_, z_);
}

THR THR::get_rand()
{
	using namespace boost;
	static mt19937 gen(static_cast<unsigned long>(time(0)));
	static normal_distribution<> dst(0, 1);
	static variate_generator<mt19937&, normal_distribution<> > rand_g(gen, dst);
	double x = rand_g();
	double y = rand_g();
	double z = rand_g();
	THR t = THR(x, y, z).normalize();
	return t;
}

bool THR::nearly_equal(const THR& a) const
{
	double E = 0.00001;
	return ((x_-a.x_)*(x_-a.x_)+(y_-a.y_)*(y_-a.y_)+(z_-a.z_)*(z_-a.z_)+(w_-a.w_)*(w_-a.w_) < E);
}
