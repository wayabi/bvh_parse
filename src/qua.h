#ifndef __H_QUA__
#define __H_QUA__

#include <stdio.h>
#include <boost/math/quaternion.hpp>
#include "thr.h"

using namespace std;
using namespace boost;
using namespace boost::math;

class qua {
public:
typedef quaternion<double> Q;
enum RotSeq{xyx, xyz, xzx, xzy, yxy, yxz, yzx, yzy, zxy, zxz, zyx, zyz, none};

static RotSeq get_rot_seq(int index1, int index2, int index3);
static double convert_single_pi(double theta);
static Q e2q(double v1, double v2, double v3, RotSeq rs);
static void q2e(const Q& q, double& v1, double& v2, double& v3, RotSeq rs);
static Q rotate_by_axis(double x_v_normal, double y_v_normal, double z_v_normal, double theta, Q target);
static Q rotate_by_euler(double v1, double v2, double v3, RotSeq rs, Q target);
static Q lerp(Q q1, Q q2, double t) ;
static Q slerp(Q q1, Q q2, float t) ;
static Q get_quaternion_from_axis(double x_normalized, double y_normalized, double z_normalized, double angle);
static Q get_quaternion_from_vector(double x_to, double y_to, double z_to, double x_from, double y_from, double z_from);
static Q get_quaternion_from_vector(THR to, THR from);
static void print(const Q& q);
static THR get_rotation_axis(const Q& q);
static THR q2em(const Q& q);
static Q em2q(const THR& t);
};
#endif
