#ifndef __MYC_CAM_POL__
#define __MYC_CAM_POL__

#include "thr.h"
#include "pol.h"
#include "rot2.h"

#include <vector>
#include <boost/math/quaternion.hpp>

typedef boost::math::quaternion<double> Q;

class CamPol {
public:
	CamPol();
	void draw(std::vector<PolColor>& a, const char* name);
	std::vector<PolColor> transform2camera_screen(std::vector<PolColor>& a);
	std::vector<PolColor> make_bone(ROT2* r);
	static void simple_draw(ROT2* r, const char* path_img_out, THR pos, THR dir);

public:
	static void z_sort(std::vector<PolColor>& a);
	static void adhoc_lighting(THR dir_light, std::vector<PolColor>& a);
	static std::vector<PolColor> load(const char* name);

private:
	bool check_see_obverse(const PolColor& p);
	bool check_in_screen(const PolColor& p);
	bool check_draw(const PolColor& p);
	THR get_pos_in_camera_coordinate(THR p);
	THR get_pos_on_screen(THR p);
	Q get_q_camera();
	void init_base_bone();
	void _make_bone(std::vector<PolColor>& d, ROT2* r);

private:
	std::vector<PolColor> base_bone_;

public:
	double angle_x_;
	double angle_y_;
	double width_screen_;
	double height_screen_;
	THR pos_cam_;
	THR dir_cam_;
};

#endif
