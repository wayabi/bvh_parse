#include "pol_reach_constraint.h"
#include <boost/make_shared.hpp>
#include <math.h>
#include "qua.h"

boost::shared_ptr<PolReachConstraint> PolReachConstraint::create_by_regular_polygon(THR v_local, double max_theta, int num_pol, double twist_theta)
{
	auto ret = boost::make_shared<PolReachConstraint>();
	ret->v_local_ = v_local;
	ret->twist_theta_ = twist_theta;
	//tan(89.999999) == 57295779.1
	double MAX_THETA = 89.999999*M_PI/180;
	if(max_theta > MAX_THETA) max_theta = MAX_THETA;
	double len = tan(max_theta);
	for(int i=0;i<num_pol;++i){
		double t1 = (i+0)*2*M_PI/num_pol;
		double t2 = (i+1)*2*M_PI/num_pol;
		ret->pol_.push_back(TWO(0, 0));
		ret->pol_.push_back(TWO(cos(t1), sin(t1))*len);
		ret->pol_.push_back(TWO(cos(t2), sin(t2))*len);
	}
	return ret;
}

boost::shared_ptr<PolReachConstraint> PolReachConstraint::create_by_line(THR pos_a, THR pos_b, double twist_theta)
{
	// check 3 points on a line
	{
		const double E = 0.00000000001;
		THR va = pos_a - THR(0, 0, 0);
		THR vb = pos_b - THR(0, 0, 0);
		if(fabs(va.cross(vb).get_size_sqr()) < E){
			return boost::shared_ptr<PolReachConstraint>();
		}
	}

	auto ret = boost::make_shared<PolReachConstraint>();
	ret->twist_theta_ = twist_theta;
	THR pos_center = (pos_a+pos_b)*0.5;
	ret->v_local_ = pos_center.normalize();
	auto a = ret->onPlain(pos_a);
	auto b = ret->onPlain(pos_b);
	if(a.get() == NULL || b.get() == NULL){
		return boost::shared_ptr<PolReachConstraint>();
	}
	ret->pol_.push_back(*a.get());
	ret->pol_.push_back(*a.get());
	ret->pol_.push_back(*b.get());
	
	return ret;
}

PolReachConstraint::PolReachConstraint()
{
}

PolReachConstraint::PolReachConstraint(const PolReachConstraint& p)
{
	v_local_ = p.v_local_;
	twist_theta_ = p.twist_theta_;
	for(auto ite = p.pol_.begin();ite != p.pol_.end();++ite){
		pol_.push_back(*ite);
	}
}

bool PolReachConstraint::isIn(THR v)
{
	v.print("isIn v");
	auto pp = onPlain(v);
	if(pp.get() == NULL) return false;
	TWO p = *pp.get();
	for(int i=0;i<(int)pol_.size()/3;++i){
		TWO p1 = pol_.at(i*3+0);
		TWO p2 = pol_.at(i*3+1);
		TWO p3 = pol_.at(i*3+2);
		TWO v1 = p2 - p1;
		TWO v2 = p3 - p2;
		TWO v3 = p1 - p3;
		TWO vp1 = p - p1;	
		TWO vp2 = p - p2;	
		TWO vp3 = p - p3;	
		// z of cross product
		double z1 = v1.x_*vp1.y_-v1.y_*vp1.x_;
		double z2 = v2.x_*vp2.y_-v2.y_*vp2.x_;
		double z3 = v3.x_*vp3.y_-v3.y_*vp3.x_;
		if(z1 >= 0 && z2 >= 0 && z3 >= 0) return true;
		if(z1 <= 0 && z2 <= 0 && z3 <= 0) return true;
	}
	v.print("isIn(v)=false");
	return false;
}

TWO get_min_pos_line_point(TWO line1, TWO line2, TWO point)
{
	if((line2-line1).dot(point-line1) < 0) return line1;
	if((line1-line2).dot(point-line2) < 0) return line2;
	TWO a(line2-line1);
	TWO b(point-line1);
	if(a.get_size_sqr() == 0.0 || b.get_size_sqr() == 0.0){
		return line1;
	}
	// dot = cos(t)|a||b|
	// cos(t)|a|
	double c = a.dot(b)/a.get_size();
	return (a.normalize()*c)+line1;
}
 
THR PolReachConstraint::toIn(THR v)
{
	v.print("toIn(v)");
	auto pp = onPlain(v);
	if(pp.get() == NULL){
		printf("error@PolReachConstraint::toIn()\n");
		return THR();
	}
	TWO p = *pp.get();
	double min_sqr = 99999999999999;
	TWO p_min;
	for(int i=0;i<(int)pol_.size()/3;++i){
		TWO p1 = pol_.at(i*3+0);
		TWO p2 = pol_.at(i*3+1);
		TWO p3 = pol_.at(i*3+2);
		TWO a = get_min_pos_line_point(p1, p2, p);
		double b = (a-p).get_size_sqr();
		if(b < min_sqr){
			min_sqr = b;
			p_min = a;
		}
		a = get_min_pos_line_point(p2, p3, p);
		b = (a-p).get_size_sqr();
		if(b < min_sqr){
			min_sqr = b;
			p_min = a;
		}
		a = get_min_pos_line_point(p3, p1, p);
		b = (a-p).get_size_sqr();
		if(b < min_sqr){
			min_sqr = b;
			p_min = a;
		}
	}
//tentative
//p_min = p;
	p.print("toIn p");
	p_min.print("toIn p_min");
	THR a(1, p_min.y_, p_min.x_);
	a = a.normalize();
	THR v_n = v_local_.normalize();
	a.print("toIn a");
	v_n.print("toIn v_n");
	Q q = qua::get_quaternion_from_vector(v_n.x_, v_n.y_, v_n.z_, 1, 0, 0);
	THR ret(q*a.q()/q);
	v.print("toIn before");
	ret.print("toIn return");
	return ret;
}

boost::shared_ptr<TWO> PolReachConstraint::onPlain(THR v)
{
	v = v.normalize();
	v.print("onPlain v_arg");
	THR v_n = v_local_.normalize();
	v_n.print("onPlain v_n");
	Q q = qua::get_quaternion_from_vector(1, 0, 0, v_n.x_, v_n.y_, v_n.z_);
	THR b(q*v.q()/q);
	//v.print("onPlain v");
	b.print("onPlain b");
	if(b.x_ <= 0.0000001){
		THR ra = v_n.cross(v).normalize();
		double theta = acos(v_n.dot(v));
		theta = theta - M_PI/2 + M_PI/1000;
		theta = -theta;
		Q qq = qua::get_quaternion_from_axis(ra.x_, ra.y_, ra.z_, theta);
		v = qq*v.q()/qq;
		return onPlain(v);

		/*
		//THR ra = v.cross(v_n).normalize();
		THR ra = THR(1, 0, 0).cross(b).normalize();
		ra.print("onPlain ra");
		double theta = acos(THR(1, 0, 0).dot(b));
		theta = theta - M_PI/2 + M_PI/1000;
		theta = -theta;
		printf("onPlain theta = %f\n", theta);	
		Q qq = get_quaternion_from_axis(ra.x_, ra.y_, ra.z_, theta);
		//qq = conj(q)*qq*q;
		b = THR(qq*b.q()/qq);
		b.print("onPlain b_second");
		v.print("onv1");
		v = conj(q)*b.q()*q;
		//v = (qq)*v.q()/(qq);
		v.print("onv2");

		return onPlain(v);
		*/
	}
	double c = 1.0/b.x_;
	printf("onPlain ret(%f, %f)\n", b.z_*c, b.y_*c);
	return boost::make_shared<TWO>(b.z_*c, b.y_*c);
}
