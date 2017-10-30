#include <algorithm>
#include <stdio.h>
#include "campol.h"
#include "qua.h"
#include "a.h"
#include "utiuti.h"

CamPol::CamPol()
{
	init_base_bone();
}

void CamPol::z_sort(std::vector<PolColor>& a)
{
	std::sort(a.begin(), a.end(), [](const PolColor& a1, const PolColor& a2) ->
		int {
			return (
				a1.pol_.p1_.z_+
				a1.pol_.p2_.z_+
				a1.pol_.p3_.z_ >
				a2.pol_.p1_.z_+
				a2.pol_.p2_.z_+
				a2.pol_.p3_.z_);
		}
	);
}

std::vector<PolColor> CamPol::transform2camera_screen(std::vector<PolColor>& a)
{
	std::vector<PolColor> ret;
	for(auto ite = a.begin();ite != a.end();++ite){
		PolColor pc = *ite;
		pc.pol_.p1_ = get_pos_on_screen(ite->pol_.p1_);
		pc.pol_.p2_ = get_pos_on_screen(ite->pol_.p2_);
		pc.pol_.p3_ = get_pos_on_screen(ite->pol_.p3_);
		ret.push_back(pc);
	}
	return ret;
}

THR CamPol::get_pos_in_camera_coordinate(THR p)
{
	Q q = get_q_camera();
	p = p - pos_cam_;
	THR a(conj(q)*p.q()*q);
	return a;
}

THR CamPol::get_pos_on_screen(THR p)
{
	THR p1 = get_pos_in_camera_coordinate(p);
	p1.x_ = (width_screen_/2)*(p1.x_/(p1.z_*tan(angle_x_/2))) + width_screen_/2;
	p1.y_ = (height_screen_/2)*(-1)*(p1.y_/(p1.z_*tan(angle_y_/2))) + height_screen_/2;
	return p1;
}

Q CamPol::get_q_camera()
{
	double a_axis_y = atan2(dir_cam_.x_, dir_cam_.z_);
	double a_axis_horizontal = atan2(dir_cam_.y_, sqrt(dir_cam_.x_*dir_cam_.x_+dir_cam_.z_*dir_cam_.z_));
	THR dir_horizontal = THR(0, 1, 0).cross(dir_cam_);
	Q q_axis_horizontal = qua::get_quaternion_from_axis(dir_horizontal.x_, dir_horizontal.y_, dir_horizontal.z_, -a_axis_horizontal);
	Q q_axis_y = qua::e2q(0, a_axis_y, 0, qua::RotSeq::xyz);
	return q_axis_horizontal*q_axis_y;
}

bool CamPol::check_see_obverse(const PolColor& p)
{
	THR v1 = p.pol_.p2_-p.pol_.p1_;
	THR v2 = p.pol_.p3_-p.pol_.p1_;
	return (v1.cross(v2).z_ <= 0);
}

bool _in_screen(THR p, double w, double h)
{
	// return always true to draw large triangle
	return true;
	// no check a triangle larger than screen, the three points out of the screen.
	if(p.x_ >= 0 && p.x_ <= w && p.y_ >= 0 && p.y_ <= h) return true;
	return false;
}

bool CamPol::check_in_screen(const PolColor& p)
{
	if(p.pol_.p1_.z_ < 0 && p.pol_.p2_.z_ < 0 && p.pol_.p3_.z_ < 0) return false;
	if(
		_in_screen(p.pol_.p1_, width_screen_, height_screen_) ||
		_in_screen(p.pol_.p2_, width_screen_, height_screen_) ||
		_in_screen(p.pol_.p3_, width_screen_, height_screen_)
	){
		return true;
	}
	return false;
}

bool CamPol::check_draw(const PolColor& p)
{
	return (check_see_obverse(p) && check_in_screen(p));
}

void CamPol::draw(std::vector<PolColor>& d, const char* name)
{
	int num_pol_draw = 0;
	sm im((int)width_screen_, (int)height_screen_);
	for(auto ite = d.begin();ite != d.end();++ite){
		if(!check_draw(*ite)) continue;
		bool flag_wire = false;
		++num_pol_draw;
		if(!flag_wire){	
			im.tri(
				ite->pol_.p1_.x_,
				ite->pol_.p1_.y_,
				ite->pol_.p2_.x_,
				ite->pol_.p2_.y_,
				ite->pol_.p3_.x_,
				ite->pol_.p3_.y_,
				ite->r_, ite->g_, ite->b_);
		}else{
			im.sla(
				ite->pol_.p1_.x_,
				ite->pol_.p1_.y_,
				ite->pol_.p2_.x_,
				ite->pol_.p2_.y_,
				ite->r_, ite->g_, ite->b_);
			im.sla(
				ite->pol_.p1_.x_,
				ite->pol_.p1_.y_,
				ite->pol_.p3_.x_,
				ite->pol_.p3_.y_,
				ite->r_, ite->g_, ite->b_);
			im.sla(
				ite->pol_.p3_.x_,
				ite->pol_.p3_.y_,
				ite->pol_.p2_.x_,
				ite->pol_.p2_.y_,
				ite->r_, ite->g_, ite->b_);
		}
	}
	//printf("num_pol_draw:%d\n", num_pol_draw);
	im.go(name);
}

std::vector<PolColor> CamPol::load(const char* name)
{
	vector<PolColor> ret;
	FILE* f;
	if((f = fopen(name, "r")) == NULL) return ret;
	vector<char> buf;
	unsigned int size_buf = 1024*4;
	buf.resize(size_buf);
	while(fgets(&buf[0], size_buf, f) != NULL){
		auto ss = Utiuti::split(&buf[0]);
		if(ss.size() != 12) continue;
		PolColor pc;
		pc.pol_.p1_.x_ = atof(ss.at(0).c_str());
		pc.pol_.p1_.y_ = atof(ss.at(1).c_str());
		pc.pol_.p1_.z_ = atof(ss.at(2).c_str());
		pc.pol_.p2_.x_ = atof(ss.at(3).c_str());
		pc.pol_.p2_.y_ = atof(ss.at(4).c_str());
		pc.pol_.p2_.z_ = atof(ss.at(5).c_str());
		pc.pol_.p3_.x_ = atof(ss.at(6).c_str());
		pc.pol_.p3_.y_ = atof(ss.at(7).c_str());
		pc.pol_.p3_.z_ = atof(ss.at(8).c_str());
		pc.r_ = atoi(ss.at(9).c_str());
		pc.g_ = atoi(ss.at(10).c_str());
		pc.b_ = atoi(ss.at(11).c_str());
		ret.push_back(pc);
	}
	fclose(f);
	return ret;
}

void CamPol::adhoc_lighting(THR dir_light, std::vector<PolColor>& a)
{
	dir_light = dir_light.normalize();
	for(auto ite = a.begin();ite != a.end();++ite){	
		double dot = ite->pol_.norm().dot(dir_light);
		double brightness = 0.6+0.4*(1-dot)/2;
		ite->r_ *= brightness;
		ite->g_ *= brightness;
		ite->g_ *= brightness;
	}
}

void CamPol::_make_bone(std::vector<PolColor>& d, ROT2* r)
{
	const bool flag_xyz_expand = true;
	if(r->len_ != 0.0){
		for(auto ite = base_bone_.begin();ite != base_bone_.end();++ite){
			if(flag_xyz_expand){
				THR p1 = THR(r->q_aa_cw_.q()*ite->pol_.p1_.q()/r->q_aa_cw_.q())*r->len_ + r->p_;
				THR p2 = THR(r->q_aa_cw_.q()*ite->pol_.p2_.q()/r->q_aa_cw_.q())*r->len_ + r->p_;
				THR p3 = THR(r->q_aa_cw_.q()*ite->pol_.p3_.q()/r->q_aa_cw_.q())*r->len_ + r->p_;
				d.push_back(PolColor(POL(p1, p2, p3), ite->r_, ite->g_, ite->b_));
			}else{
				THR pb1 = ite->pol_.p1_;
				THR pb2 = ite->pol_.p2_;
				THR pb3 = ite->pol_.p3_;
				pb1.x_ *= r->len_;
				pb2.x_ *= r->len_;
				pb3.x_ *= r->len_;
				THR p1 = THR(r->q_aa_cw_.q()*pb1.q()/r->q_aa_cw_.q()) + r->p_;
				THR p2 = THR(r->q_aa_cw_.q()*pb2.q()/r->q_aa_cw_.q()) + r->p_;
				THR p3 = THR(r->q_aa_cw_.q()*pb3.q()/r->q_aa_cw_.q()) + r->p_;
				d.push_back(PolColor(POL(p1, p2, p3), ite->r_, ite->g_, ite->b_));
			}
		}
	}
	for(auto ite = r->children_.begin();ite != r->children_.end();++ite){
		_make_bone(d, *ite);
	}
}

std::vector<PolColor> CamPol::make_bone(ROT2* r)
{
	std::vector<PolColor> ret;
	_make_bone(ret, r);
	return ret;
}

void CamPol::init_base_bone()
{
	int cr1 = 200; int cg1 = 0; int cb1 = 0;
	int cr2 = 200; int cg2 = 200; int cb2 = 0;
	int cr3 = 0; int cg3 = 200; int cb3 = 0;
	int cr4 = 0; int cg4 = 0; int cb4 = 200;
	double len_a = 0.3;
	double r = 0.1;
	THR o1(0, 0, 0);
	THR o2(1, 0, 0);
	THR a1(len_a, 0, -r);
	THR a2 = a1.rotate(2*M_PI/4, "x");
	THR a3 = a2.rotate(2*M_PI/4, "x");
	THR a4 = a3.rotate(2*M_PI/4, "x");

	base_bone_.push_back(PolColor(POL(o1, a1, a2), cr1, cg1, cb1));
	base_bone_.push_back(PolColor(POL(a1, o2, a2), cr1, cg1, cb1));

	base_bone_.push_back(PolColor(POL(o1, a2, a3), cr2, cg2, cb2));
	base_bone_.push_back(PolColor(POL(a2, o2, a3), cr2, cg2, cb2));

	base_bone_.push_back(PolColor(POL(o1, a3, a4), cr3, cg3, cb3));
	base_bone_.push_back(PolColor(POL(a3, o2, a4), cr3, cg3, cb3));

	base_bone_.push_back(PolColor(POL(o1, a4, a1), cr4, cg4, cb4));
	base_bone_.push_back(PolColor(POL(a4, o2, a1), cr4, cg4, cb4));
}

void CamPol::simple_draw(ROT2* r, const char* path_img_out, THR pos, THR dir)
{
	CamPol cp;
	cp.angle_x_ = 60*M_PI/180;
	cp.angle_y_ = 60*M_PI/180;
	cp.width_screen_ = 800;
	cp.height_screen_ = 800;
	cp.pos_cam_ = pos;
	cp.dir_cam_ = dir;

	std::vector<PolColor> pc = cp.make_bone(r);
	cp.adhoc_lighting(THR(-1, -1, 1), pc);
	
	double len_axis = 100;
	pc.push_back(PolColor(POL(THR(0, 0, 0), THR(0, 0, 0), THR(len_axis, 0, 0)), 255, 0, 0));
	pc.push_back(PolColor(POL(THR(0, 0, 0), THR(0, 0, 0), THR(0, len_axis, 0)), 0, 255, 0));
	pc.push_back(PolColor(POL(THR(0, 0, 0), THR(0, 0, 0), THR(0, 0, len_axis)), 0, 0, 255));

	int num_h_grid = 10;
	double span_h_grid = 10.0;

	for(int x=0;x<num_h_grid+1;++x){
		THR p1((x-num_h_grid/2)*span_h_grid, 0, -num_h_grid/2*span_h_grid);
		THR p2((x-num_h_grid/2)*span_h_grid, 0, +num_h_grid/2*span_h_grid);
		pc.push_back(PolColor(POL(p1, p1+THR(0, 0, 0), p2), 255, 255, 255));
	}
	for(int z=0;z<num_h_grid+1;++z){
		THR p1(-num_h_grid/2*span_h_grid, 0, (z-num_h_grid/2)*span_h_grid);
		THR p2(+num_h_grid/2*span_h_grid, 0, (z-num_h_grid/2)*span_h_grid);
		pc.push_back(PolColor(POL(p1, p1+THR(0, 0, 0), p2), 255, 255, 255));
	}
	std::vector<PolColor> pc2 = cp.transform2camera_screen(pc);
	cp.z_sort(pc2);
	cp.draw(pc2, path_img_out);
}


