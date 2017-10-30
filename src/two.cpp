#include "two.h"

TWO::TWO() :
	x_(0),
	y_(0)
{
}

TWO::TWO(double x, double y) :
	x_(x),
	y_(y)
{
}

TWO& TWO::operator=(const TWO& a)
{
	x_ = a.x_;
	y_ = a.y_;
	return *this;
}

TWO TWO::operator+(const TWO& a)
{
	TWO ret;
	ret.x_ = x_+a.x_;
	ret.y_ = y_+a.y_;
	return ret;
}

TWO TWO::operator-(const TWO& a)
{
	TWO ret;
	ret.x_ = x_-a.x_;
	ret.y_ = y_-a.y_;
	return ret;
}

TWO TWO::operator*(double a)
{
	TWO ret;
	ret.x_ = x_*a;
	ret.y_ = y_*a;
	return ret;
}

bool TWO::operator==(const TWO& a)
{
	return (x_ == a.x_ && y_ == a.y_);
}

double TWO::get_size()
{
	return sqrt(x_*x_+y_*y_);
}

double TWO::get_size_sqr()
{
	return x_*x_+y_*y_;
}

double TWO::dot(const TWO& a)
{
	return x_*a.x_+y_*a.y_;
}

TWO TWO::normalize()
{
	double s = get_size();
	if(s == 0.0) return TWO(1, 0);
	return TWO(x_/s, y_/s);
}

void TWO::print(const char* tag)
{
	if(tag){
		printf("%s (%f, %f)\n", tag, x_, y_);
	}else{
		printf("(%f, %f)\n", x_, y_);
	}
}
