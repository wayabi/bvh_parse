#include <stdio.h>
#include <math.h>
#include <iostream>
#include "qua.h"
#include "thr.h"

using namespace std;
using namespace boost;
using namespace boost::math;
typedef quaternion<double> Q;

qua::RotSeq qua::get_rot_seq(int index1, int index2, int index3)
{
	switch(index1*100+index2*10+index3){
	case 121:
		return RotSeq::xyx;
	case 123:
		return RotSeq::xyz;
	case 131:
		return RotSeq::xzx;
	case 132:
		return RotSeq::xzy;
	case 212:
		return RotSeq::yxy;
	case 213:
		return RotSeq::yxz;
	case 231:
		return RotSeq::yzx;
	case 232:
		return RotSeq::yzy;
	case 312:
		return RotSeq::zxy;
	case 313:
		return RotSeq::zxz;
	case 321:
		return RotSeq::zyx;
	case 323:
		return RotSeq::zyz;
	default:
		return RotSeq::none;
	}
	return RotSeq::none;
}

double qua::convert_single_pi(double theta)
{
	while(theta > M_PI) theta -= 2*M_PI;
	while(theta < -M_PI) theta += 2*M_PI;
	if(theta == M_PI) theta = M_PI-0.0000001;
	if(theta == -M_PI) theta = -M_PI+0.0000001;
	return theta;
}

//Q qua::from_euler(double bank, double heading, double attitude)
Q qua::e2q(double v1, double v2, double v3, RotSeq rs)
{
	v1 = convert_single_pi(v1);
	v2 = convert_single_pi(v2);
	v3 = convert_single_pi(v3);

	double c1 = cos(v1/2);
	double c2 = cos(v2/2);
	double c3 = cos(v3/2);
	double s1 = sin(v1/2);
	double s2 = sin(v2/2);
	double s3 = sin(v3/2);
	
	double x = 0.0;
	double y = 0.0;
	double z = 0.0;
	double w = 0.0;

	switch(rs){
	case RotSeq::xyx:
		w = c2*cos((v1+v3)/2);
		x = c2*sin((v1+v3)/2);
		y = c2*cos((v1-v3)/2);
		z = c2*sin((v1-v3)/2);
		break;
	case RotSeq::xyz:
		w = -s1*s2*s3+c1*c2*c3;
		x =  s1*c2*c3+s2*s3*c1;
		y = -s1*s3*c2+s2*c1*c3;
		z =  s1*s2*c3+s3*c1*c2;
		break;
	case RotSeq::xzx:
		w = c2*cos((v1+v3)/2);
		x = c2*sin((v1+v3)/2);
		y = -s2*sin((v1-v3)/2);
		z = s2*cos((v1-v3)/2);
		break;
	case RotSeq::xzy:
		w =  s1*s2*s3+c1*c2*c3;
		x =  s1*c2*c3-s2*s3*c1;
		y = -s1*s2*c3+s3*c1*c2;
		z =  s1*s3*c2+s2*c1*c3;
		break;
	case RotSeq::yxz:
		w =  s1*s2*s3+c1*c2*c3;
		x =  s1*s3*c2+s2*c1*c3;
		y =  s1*c2*c3-s2*s3*c1;
		z = -s1*s2*c3+s3*c1*c2;
		break;
	case RotSeq::yxy:
		w = c2*cos((v1+v3)/2);
		x = s2*cos((v1-v3)/2);
		y = c2*sin((v1+v3)/2);
		z = -s2*sin((v1-v3)/2);
		break;
	case RotSeq::yzx:
		w = -s1*s2*s3+c1*c2*c3;
		x =  s1*s2*c3+s3*c1*c2;
		y =  s1*c2*c3+s2*s3*c1;
		z = -s1*s3*c2+s2*c1*c3;
		break;
	case RotSeq::yzy:
		w = c2*cos((v1+v3)/2);
		x = s2*sin((v1-v3)/2);
		y = c2*sin((v1+v3)/2);
		z = s2*cos((v1-v3)/2);
		break;
	case RotSeq::zxy:
		w = -s1*s2*s3+c1*c2*c3;
		x = -s1*s3*c2+s2*c1*c3;
		y =  s1*s2*c3+s3*c1*c2;
		z =  s1*c2*c3+s2*s3*c1;
		break;
	case RotSeq::zxz:
		w = c2*cos((v1+v3)/2);
		x = s2*cos((v1-v3)/2);
		y = s2*sin((v1-v3)/2);
		z = c2*sin((v1+v3)/2);
		break;
	case RotSeq::zyx:
		w =  s1*s2*s3+c1*c2*c3;
		x = -s1*s2*c3+s3*c1*c2;
		y =  s1*s3*c2+s2*c1*c3;
		z =  s1*c2*c3-s2*s3*c1;
		break;
	case RotSeq::zyz:
		w = c2*cos((v1+v3)/2);
		x = -c2*sin((v1-v3)/2);
		y = s2*cos((v1-v3)/2);
		z = c2*sin((v1+v3)/2);
		break;
	default:
		break;
	}
	return Q(w, x, y, z);
}

Q qua::rotate_by_axis(double x_v_normal, double y_v_normal, double z_v_normal, double theta, Q target)
{
	Q before(cos(theta/2), x_v_normal*sin(theta/2), y_v_normal*sin(theta/2), z_v_normal*sin(theta/2));
	return before*target/before;
}

Q qua::rotate_by_euler(double v1, double v2, double v3, RotSeq rs, Q target)
{
	Q before = e2q(v1, v2, v3, rs);
	return before*target/before;
}

Q qua::lerp(Q q1, Q q2, double t) 
{
	Q q = q1*(1-t) + q2*t;
	double size = sqrt(q.R_component_1()*q.R_component_1()+q.R_component_2()*q.R_component_2()+q.R_component_3()*q.R_component_3()+q.R_component_4()*q.R_component_4());
	if(fabs(size) < 0.000000001){
		return Q(1, 0, 0, 0);
	}
	return q/size;
}

//! spherical linear interpolation
Q qua::slerp(Q q1, Q q2, float t) 
{
	Q q3;
	//float dot = dot(q1, q2);
	double dot = q1.R_component_1()*q2.R_component_1()+q1.R_component_2()*q2.R_component_2()+q1.R_component_3()*q2.R_component_3()+q1.R_component_4()*q2.R_component_4();
	//dot = 1*1*cos(theta)
	//if (dot < 0), q1 and q2 are more than 90 degrees apart,
	//so we can invert one to reduce spinning	*/
	if (dot < 0) {
		dot = -dot;
		q3 = -q2;
	} else {
		q3 = q2;
	}

	if (dot < 0.95f) {
		double angle = acos(dot);
		return (q1*sin(angle*(1-t)) + q2*sin(angle*t))/sin(angle);
	} else {
		// if the angle is small, use linear interpolation								
		return lerp(q1,q3,t);	
	}
}

///////////////////////////////
// Quaternion to Euler
///////////////////////////////

void twoaxisrot(double r11, double r12, double r21, double r31, double r32, double res[]){
	double E = 0.000000001;
	if(fabs(r11) < E && fabs(r12) < E){
		res[0] = 0.0;
	}else{
  	res[0] = atan2( r11, r12 );
	}
	if(r21 > 1.0){
		res[1] = 0.0;
	}else if(r21 < -1.0){
		res[1] = M_PI;
	}else{
  	res[1] = acos ( r21 );
	}
	if(fabs(r31) < E && fabs(r32) < E){
		res[2] = 0.0;
	}else{
  	res[2] = atan2( r31, r32 );
	}
}

void threeaxisrot(double r11, double r12, double r21, double r31, double r32, double res[]){
	double E = 0.000000001;
	if(fabs(r31) < E && fabs(r32) < E){
		res[0] = 0.0;
	}else{
  	res[0] = atan2( r31, r32 );
	}
	if(r21 > 1.0){
		res[1] = M_PI/2;
	}else if(r21 < -1.0){
		res[1] = -M_PI/2;
	}else{
  	res[1] = asin ( r21 );
	}
	if(fabs(r11) < E && fabs(r12) < E){
		res[2] = 0.0;
	}else{
  	res[2] = atan2( r11, r12 );
	}
}

void qua::q2e(const Q& q, double& v1, double& v2, double& v3, RotSeq rotSeq)
{
	double res[3];
	switch(rotSeq){
	case RotSeq::zyx:
		threeaxisrot( 2*(q.R_component_2()*q.R_component_3() + q.R_component_1()*q.R_component_4()),
									 q.R_component_1()*q.R_component_1() + q.R_component_2()*q.R_component_2() - q.R_component_3()*q.R_component_3() - q.R_component_4()*q.R_component_4(),
									-2*(q.R_component_2()*q.R_component_4() - q.R_component_1()*q.R_component_3()),
									 2*(q.R_component_3()*q.R_component_4() + q.R_component_1()*q.R_component_2()),
									 q.R_component_1()*q.R_component_1() - q.R_component_2()*q.R_component_2() - q.R_component_3()*q.R_component_3() + q.R_component_4()*q.R_component_4(),
									 res);
		break;

	case RotSeq::zyz:
		twoaxisrot( 2*(q.R_component_3()*q.R_component_4() - q.R_component_1()*q.R_component_2()),
								 2*(q.R_component_2()*q.R_component_4() + q.R_component_1()*q.R_component_3()),
								 q.R_component_1()*q.R_component_1() - q.R_component_2()*q.R_component_2() - q.R_component_3()*q.R_component_3() + q.R_component_4()*q.R_component_4(),
								 2*(q.R_component_3()*q.R_component_4() + q.R_component_1()*q.R_component_2()),
								-2*(q.R_component_2()*q.R_component_4() - q.R_component_1()*q.R_component_3()),
								res);
		break;

	case RotSeq::zxy:
		threeaxisrot( -2*(q.R_component_2()*q.R_component_3() - q.R_component_1()*q.R_component_4()),
										q.R_component_1()*q.R_component_1() - q.R_component_2()*q.R_component_2() + q.R_component_3()*q.R_component_3() - q.R_component_4()*q.R_component_4(),
										2*(q.R_component_3()*q.R_component_4() + q.R_component_1()*q.R_component_2()),
									 -2*(q.R_component_2()*q.R_component_4() - q.R_component_1()*q.R_component_3()),
										q.R_component_1()*q.R_component_1() - q.R_component_2()*q.R_component_2() - q.R_component_3()*q.R_component_3() + q.R_component_4()*q.R_component_4(),
										res);
		break;

	case RotSeq::zxz:
		twoaxisrot( 2*(q.R_component_2()*q.R_component_4() + q.R_component_1()*q.R_component_3()),
								-2*(q.R_component_3()*q.R_component_4() - q.R_component_1()*q.R_component_2()),
								 q.R_component_1()*q.R_component_1() - q.R_component_2()*q.R_component_2() - q.R_component_3()*q.R_component_3() + q.R_component_4()*q.R_component_4(),
								 2*(q.R_component_2()*q.R_component_4() - q.R_component_1()*q.R_component_3()),
								 2*(q.R_component_3()*q.R_component_4() + q.R_component_1()*q.R_component_2()),
								 res);
		break;

	case RotSeq::yxz:
		threeaxisrot( 2*(q.R_component_2()*q.R_component_4() + q.R_component_1()*q.R_component_3()),
									 q.R_component_1()*q.R_component_1() - q.R_component_2()*q.R_component_2() - q.R_component_3()*q.R_component_3() + q.R_component_4()*q.R_component_4(),
									-2*(q.R_component_3()*q.R_component_4() - q.R_component_1()*q.R_component_2()),
									 2*(q.R_component_2()*q.R_component_3() + q.R_component_1()*q.R_component_4()),
									 q.R_component_1()*q.R_component_1() - q.R_component_2()*q.R_component_2() + q.R_component_3()*q.R_component_3() - q.R_component_4()*q.R_component_4(),
									 res);
		break;

	case RotSeq::yxy:
		twoaxisrot( 2*(q.R_component_2()*q.R_component_3() - q.R_component_1()*q.R_component_4()),
								 2*(q.R_component_3()*q.R_component_4() + q.R_component_1()*q.R_component_2()),
								 q.R_component_1()*q.R_component_1() - q.R_component_2()*q.R_component_2() + q.R_component_3()*q.R_component_3() - q.R_component_4()*q.R_component_4(),
								 2*(q.R_component_2()*q.R_component_3() + q.R_component_1()*q.R_component_4()),
								-2*(q.R_component_3()*q.R_component_4() - q.R_component_1()*q.R_component_2()),
								res);
		break;

	case RotSeq::yzx:
		threeaxisrot( -2*(q.R_component_2()*q.R_component_4() - q.R_component_1()*q.R_component_3()),
										q.R_component_1()*q.R_component_1() + q.R_component_2()*q.R_component_2() - q.R_component_3()*q.R_component_3() - q.R_component_4()*q.R_component_4(),
										2*(q.R_component_2()*q.R_component_3() + q.R_component_1()*q.R_component_4()),
									 -2*(q.R_component_3()*q.R_component_4() - q.R_component_1()*q.R_component_2()),
										q.R_component_1()*q.R_component_1() - q.R_component_2()*q.R_component_2() + q.R_component_3()*q.R_component_3() - q.R_component_4()*q.R_component_4(),
										res);
		break;

	case RotSeq::yzy:
		twoaxisrot( 2*(q.R_component_3()*q.R_component_4() + q.R_component_1()*q.R_component_2()),
								-2*(q.R_component_2()*q.R_component_3() - q.R_component_1()*q.R_component_4()),
								 q.R_component_1()*q.R_component_1() - q.R_component_2()*q.R_component_2() + q.R_component_3()*q.R_component_3() - q.R_component_4()*q.R_component_4(),
								 2*(q.R_component_3()*q.R_component_4() - q.R_component_1()*q.R_component_2()),
								 2*(q.R_component_2()*q.R_component_3() + q.R_component_1()*q.R_component_4()),
								 res);
		break;

	case RotSeq::xyz:
		threeaxisrot( -2*(q.R_component_3()*q.R_component_4() - q.R_component_1()*q.R_component_2()),
									q.R_component_1()*q.R_component_1() - q.R_component_2()*q.R_component_2() - q.R_component_3()*q.R_component_3() + q.R_component_4()*q.R_component_4(),
									2*(q.R_component_2()*q.R_component_4() + q.R_component_1()*q.R_component_3()),
								 -2*(q.R_component_2()*q.R_component_3() - q.R_component_1()*q.R_component_4()),
									q.R_component_1()*q.R_component_1() + q.R_component_2()*q.R_component_2() - q.R_component_3()*q.R_component_3() - q.R_component_4()*q.R_component_4(),
									res);
		break;

	case RotSeq::xyx:
		twoaxisrot( 2*(q.R_component_2()*q.R_component_3() + q.R_component_1()*q.R_component_4()),
								-2*(q.R_component_2()*q.R_component_4() - q.R_component_1()*q.R_component_3()),
								 q.R_component_1()*q.R_component_1() + q.R_component_2()*q.R_component_2() - q.R_component_3()*q.R_component_3() - q.R_component_4()*q.R_component_4(),
								 2*(q.R_component_2()*q.R_component_3() - q.R_component_1()*q.R_component_4()),
								 2*(q.R_component_2()*q.R_component_4() + q.R_component_1()*q.R_component_3()),
								 res);
		break;

	case RotSeq::xzy:
		threeaxisrot( 2*(q.R_component_3()*q.R_component_4() + q.R_component_1()*q.R_component_2()),
									 q.R_component_1()*q.R_component_1() - q.R_component_2()*q.R_component_2() + q.R_component_3()*q.R_component_3() - q.R_component_4()*q.R_component_4(),
									-2*(q.R_component_2()*q.R_component_3() - q.R_component_1()*q.R_component_4()),
									 2*(q.R_component_2()*q.R_component_4() + q.R_component_1()*q.R_component_3()),
									 q.R_component_1()*q.R_component_1() + q.R_component_2()*q.R_component_2() - q.R_component_3()*q.R_component_3() - q.R_component_4()*q.R_component_4(),
									 res);
		break;

	case RotSeq::xzx:
		twoaxisrot( 2*(q.R_component_2()*q.R_component_4() - q.R_component_1()*q.R_component_3()),
								 2*(q.R_component_2()*q.R_component_3() + q.R_component_1()*q.R_component_4()),
								 q.R_component_1()*q.R_component_1() + q.R_component_2()*q.R_component_2() - q.R_component_3()*q.R_component_3() - q.R_component_4()*q.R_component_4(),
								 2*(q.R_component_2()*q.R_component_4() + q.R_component_1()*q.R_component_3()),
								-2*(q.R_component_2()*q.R_component_3() - q.R_component_1()*q.R_component_4()),
								res);
		break;
	default:
		std::cout << "Unknown rotation sequence" << std::endl;
		break;
	}
	v1 = convert_single_pi(res[2]);
	v2 = convert_single_pi(res[1]);
	v3 = convert_single_pi(res[0]);
}

Q qua::get_quaternion_from_axis(double x_normalized, double y_normalized, double z_normalized, double angle)
{
	return Q (cos(angle/2), x_normalized*sin(angle/2), y_normalized*sin(angle/2), z_normalized*sin(angle/2));
}

Q qua::get_quaternion_from_vector(THR to, THR from)
{
	return get_quaternion_from_vector(to.x_, to.y_, to.z_, from.x_, from.y_, from.z_);
}

Q qua::get_quaternion_from_vector(double x_to, double y_to, double z_to, double x_from, double y_from, double z_from)
{
	THR f(x_from, y_from, z_from);
	THR t(x_to, y_to, z_to);
	if(f == t) return Q(1, 0, 0, 0);
	f = f.normalize();
	t = t.normalize();
	if(t == THR(-1, 0, 0)  && f == THR(1, 0, 0)){
		return get_quaternion_from_axis(0, 1, 0, M_PI);
	}
	THR q = f.cross( t);
	q.w_ = sqrt(f.get_size_sqr() * t.get_size_sqr()) + f.dot( t);
	double sum = sqrt(q.x_*q.x_+q.y_*q.y_+q.z_*q.z_+q.w_*q.w_);
	Q ret(q.w_/sum, q.x_/sum, q.y_/sum, q.z_/sum);
	return ret;
}

void qua::print(const Q& q)
{
	printf("%f, %f, %f, %f\n", q.R_component_1(), q.R_component_2(), q.R_component_3(), q.R_component_4());

}

THR qua::get_rotation_axis(const Q& q)
{
	double theta = q.R_component_1();
	theta = convert_single_pi(acos(theta)*2);
	THR ret;
	double s = sin(theta/2);
	ret.x_ = q.R_component_2()/s;
	ret.y_ = q.R_component_3()/s;
	ret.z_ = q.R_component_4()/s;
	ret = ret.normalize();
	ret.w_ = theta;
	return ret;
}

THR qua::q2em(const Q& q)
{
	THR t = qua::get_rotation_axis(q);
	t.x_ = t.x_*t.w_/2;
	t.y_ = t.y_*t.w_/2;
	t.z_ = t.z_*t.w_/2;
	t.w_ = 0.0;
	return t;
}

Q qua::em2q(const THR& t)
{
	double half_theta = sqrt(t.x_*t.x_+t.y_*t.y_+t.z_*t.z_);
	return Q(cos(half_theta), t.x_/half_theta*sin(half_theta), t.y_/half_theta*sin(half_theta), t.z_/half_theta*sin(half_theta));
}
