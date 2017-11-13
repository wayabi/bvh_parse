#include <stdio.h>
#include <memory.h>
#include <boost/make_shared.hpp>
#include <boost/algorithm/string.hpp>
#include "rot2.h"
#include "qua.h"
#include "utiuti.h"

typedef boost::math::quaternion<double> Q;
using namespace boost;

ROT2::ROT2()
{
	p_ = THR(0, 0, 0);
	q_al_cl_ = THR(1, 0, 0, 0);
	q_al_cw_ = THR(1, 0, 0, 0);
	q_base_al_cl_ = THR(1, 0, 0, 0);
	q_aa_cw_ = THR(1, 0, 0, 0);
	q_parent_aa_cw_ = THR(1, 0, 0, 0);
	q_parent_move_only_aa_cw_ = THR(1, 0, 0, 0);
	len_ = 1.0;
	parent_ = NULL;
	weight_ik_ = 1.0;
}

ROT2::~ROT2()
{
	for(auto ite = children_.begin();ite != children_.end();++ite){
		delete(*ite);
	}
}

ROT2* ROT2::add()
{
	ROT2* a = new ROT2();
	children_.push_back(a);
	a->parent_ = this;
	return a;
}

// cw Coordinate World
// cl Coordinate Local
// aa Angle All
// al Angle Local
void _update_pos(THR parent_aa_cw, THR parent_move_only_aa_cw, ROT2& r)
{
	r.q_parent_aa_cw_ = parent_aa_cw;
	r.q_parent_move_only_aa_cw_ = parent_move_only_aa_cw.q();
	THR al_cl = r.q_base_al_cl_;
	THR al_cw_base = THR(parent_aa_cw.q()*al_cl.q()/parent_aa_cw.q());
	r.q_aa_cw_ = THR(al_cw_base.q()*parent_aa_cw.q());
	
	al_cl = r.q_al_cl_;
	r.q_al_cw_ = THR(parent_move_only_aa_cw.q()*al_cl.q()/parent_move_only_aa_cw.q());
	r.q_aa_cw_ = THR(r.q_al_cw_.q()*r.q_aa_cw_.q());
	parent_move_only_aa_cw = THR(r.q_al_cw_.q()*parent_move_only_aa_cw.q());

	THR v(r.q_aa_cw_.q()*THR(1, 0, 0).q()/r.q_aa_cw_.q());
	for(auto ite = r.children_.begin();ite != r.children_.end();++ite){
		(*ite)->p_ = r.p_+(v*r.len_);
		_update_pos(r.q_aa_cw_, parent_move_only_aa_cw, **ite);
	}
}

void ROT2::update_pos()
{
	THR parent_aa_cw(1, 0, 0, 0);
	THR parent_base_aa_cw(1, 0, 0, 0);
	if(parent_){
		printf("run update_pos() in root node");
		return;
	}
	_update_pos(parent_aa_cw, parent_base_aa_cw, *this);
}

ROT2* _make_bone(BVH::Joint* j, ROT2* r, THR parent_aa_cw)
{
	if(j == NULL) return NULL;
	if(r == NULL){
		r = new ROT2();
		r->p_ = THR(j->offset_[0], j->offset_[1], j->offset_[2]);
		r->q_base_al_cl_ = THR(1, 0, 0, 0);
		r->len_ = 0;
	}else{
		r = r->add();
		THR offset(j->offset_[0], j->offset_[1], j->offset_[2]);
		r->len_ = offset.get_size();
		offset = offset.normalize();
		THR v_aa(parent_aa_cw.q()*THR(1, 0, 0).q()/parent_aa_cw.q());
		THR al_cw(qua::get_quaternion_from_vector(offset.x_, offset.y_, offset.z_, v_aa.x_, v_aa.y_, v_aa.z_));
		//offset.print("offset");
		//v_aa.print("v_aa");
		THR al_cl = conj(parent_aa_cw.q())*al_cw.q()*parent_aa_cw.q();
		r->q_base_al_cl_ = al_cl;
		parent_aa_cw = al_cw.q()*parent_aa_cw.q();
	}
	r->name_ = j->name_;
	if(j->has_site_){
		ROT2* rs1 = r->add();
		rs1->name_ = r->name_+string("_");
		THR offset(j->site_[0], j->site_[1], j->site_[2]);
		rs1->len_ = offset.get_size();
		offset = offset.normalize();
		THR v_aa(parent_aa_cw.q()*THR(1, 0, 0).q()/parent_aa_cw.q());
		THR al_cw(qua::get_quaternion_from_vector(offset.x_, offset.y_, offset.z_, v_aa.x_, v_aa.y_, v_aa.z_));
		//offset.print("offset");
		//v_aa.print("v_aa");
		//al_cw.print("al_cw");
		//parent_aa_cw.print("parent");
		THR al_cl = conj(parent_aa_cw.q())*al_cw.q()*parent_aa_cw.q();
		rs1->q_base_al_cl_ = al_cl;

		ROT2* rs2 = rs1->add();
		rs2->name_ = rs1->name_+string("_");
		rs2->len_ = 0;
		rs2->q_base_al_cl_ = THR(1, 0, 0, 0);
	}
	for(auto ite = j->children_.begin();ite != j->children_.end();++ite){
		_make_bone(*ite, r, parent_aa_cw);
	}
	return r;
}

ROT2* ROT2::make_bone(BVH* bvh)
{
	BVH::Joint* j = bvh->joint_root_;
	THR parent_aa_cw(1, 0, 0, 0);
	ROT2* ret = _make_bone(j, NULL, parent_aa_cw);
	return ret;
}

void _refer_parent_angle(ROT2* r)
{
	for(auto ite = r->children_.begin();ite != r->children_.end();++ite){
		_refer_parent_angle(*ite);
	}
	if(r->parent_){
		r->q_al_cl_ = r->parent_->q_al_cl_;
	}else{
		;
	}
}

void _refer_child_angle(ROT2* r)
{
	if(r->children_.size() > 0){
		r->q_al_cl_ = r->children_.at(0)->q_al_cl_;
	}else{
		;
	}
	for(auto ite = r->children_.begin();ite != r->children_.end();++ite){
		_refer_child_angle(*ite);
	}
}

void _set_serialized_angle(ROT2* rot, std::vector<THR>::iterator& ite_data)
{
	if(rot->name_.at(rot->name_.size()-1) != '_'){
		rot->q_al_cl_ = *ite_data;
		++ite_data;
	}else{
		rot->q_al_cl_ = THR(1, 0, 0, 0);
	}
	for(auto ite = rot->children_.begin();ite != rot->children_.end();++ite){
		_set_serialized_angle(*ite, ite_data);
	}
}

void ROT2::set_serialized_angle(std::vector<THR>& d)
{
	auto ite = d.begin();
	p_ = *ite;
	++ite;
	_set_serialized_angle(this, ite);
	THR root_angle = q_al_cl_;
	q_al_cl_ = THR(1, 0, 0, 0);
	_refer_parent_angle(this);
	q_al_cl_ = root_angle;
}

void _get_serialized_angle(ROT2* rot, std::vector<THR>& d){
	if(rot->name_.at(rot->name_.size()-1) != '_'){
  	d.push_back(rot->q_al_cl_);
	}else{
		return;
	}
  for(auto ite = rot->children_.begin();ite != rot->children_.end();++ite){
    _get_serialized_angle(*ite, d);
  }
}

std::vector<THR> ROT2::get_serialized_angle()
{
	std::vector<THR> ret;
	ret.push_back(p_);
	ROT2* r = copy(NULL);
	THR root_angle = r->q_al_cl_;
	_refer_child_angle(r);
	r->q_al_cl_ = root_angle;
	_get_serialized_angle(r, ret);
	delete(r);
	return ret;
}

std::vector<THR> ROT2::get_serialized_angle_al_cw()
{
	vector<THR> ret;
	ret.push_back(p_);
	make_serialized_pointer();
	for(auto ite = serialized_pointer_.begin();ite != serialized_pointer_.end();++ite){
		if((*ite)->name_.at((*ite)->name_.size()-1) == '_'){
			continue;
		}
		ret.push_back((*ite)->q_al_cw_);
	}
	return ret;
}

ROT2* ROT2::copy(ROT2* parent)
{
	ROT2* ret = new ROT2();
	ret->p_ = p_;
	ret->q_al_cl_ = q_al_cl_;
	ret->q_al_cw_ = q_al_cw_;
	ret->q_base_al_cl_ = q_base_al_cl_;
	ret->q_aa_cw_ = q_aa_cw_;
	ret->q_parent_aa_cw_ = q_parent_aa_cw_;
	ret->q_parent_move_only_aa_cw_ = q_parent_move_only_aa_cw_;
	ret->len_ = len_;
	ret->parent_ = parent;
	ret->name_ = name_;
	ret->weight_ik_ = weight_ik_;

	for(auto ite = children_.begin();ite != children_.end();++ite){
		if(parent && (*ite)->name_ == parent->name_) continue;
		ret->children_.push_back((*ite)->copy(ret));
		//ret->children_.push_back((*ite)->copy(this));
	}

	if(constraint_.get()){
		ret->constraint_ = boost::make_shared<PolReachConstraint>(*(constraint_.get()));
	}
	ret->name_constraint_ = name_constraint_;
	return ret;
}

std::vector<THR> ROT2::get_frame(BVH& bvh, int frame)
{
	vector<THR> ret;
	vector<double> data = bvh.motion_.at(frame);
	for(int i=0;i<data.size()/3;++i){
		if(i==0){
			double xyz[3] = {0, 0, 0};
			for(int j=0;j<3;++j){
				xyz[bvh.channels_.at(i+j)->type_ - BVH::ChannelEnum::X_POSITION] = data.at(i+j);
			}
			ret.push_back(THR(xyz[0], xyz[1], xyz[2]));
		}else{
			if(bvh.channels_.at(i*3+0)->type_ == BVH::ChannelEnum::X_POSITION ||
				bvh.channels_.at(i*3+0)->type_ == BVH::ChannelEnum::Y_POSITION ||
				bvh.channels_.at(i*3+0)->type_ == BVH::ChannelEnum::Z_POSITION){
				//skip position
				continue;
			}
			int index_rs1 = bvh.channels_.at(i*3+0)->type_ - BVH::ChannelEnum::X_ROTATION+1;
			int index_rs2 = bvh.channels_.at(i*3+1)->type_ - BVH::ChannelEnum::X_ROTATION+1;
			int index_rs3 = bvh.channels_.at(i*3+2)->type_ - BVH::ChannelEnum::X_ROTATION+1;
			qua::RotSeq rs = qua::get_rot_seq(index_rs1, index_rs2, index_rs3);
			THR q(qua::e2q(data.at(i*3+0)*M_PI/180, data.at(i*3+1)*M_PI/180, data.at(i*3+2)*M_PI/180, rs));
			ret.push_back(q);
		}
	}
	return ret;
}

void _print_pos(ROT2* a, int index)
{
	for(int i=0;i<index;++i){
		printf("  ");
	}
	printf("%s:\n", a->name_.c_str());
	for(int i=0;i<index;++i){
		printf("  ");
	}
	a->q_al_cl_.print("q_al_cl");
	for(int i=0;i<index;++i){
		printf("  ");
	}
	a->q_base_al_cl_.print("q_base_al_cl");
	for(int i=0;i<index;++i){
		printf("  ");
	}
	a->p_.print("pos");
	for(int i=0;i<index;++i){
		printf("  ");
	}
	printf("len = %f\n", a->len_);

	for(auto ite = a->children_.begin();ite != a->children_.end();++ite){
		_print_pos(*ite, index+1);
	}
}

void ROT2::print()
{
	int index = 0;
	_print_pos(this, index);
}

void ROT2::set_global_vector(THR v, float weight)
{
	THR v_source(q_aa_cw_.q()*THR(1, 0, 0).q()/q_aa_cw_.q());
	Q q = qua::get_quaternion_from_vector(v, v_source);
	q = conj(q_parent_move_only_aa_cw_.q())*q*q_parent_move_only_aa_cw_.q();
	Q to = q*q_al_cl_.q();
	q_al_cl_ = THR(qua::slerp(q_al_cl_.q(), to, weight));
}

ROT2* ROT2::search(const char* name)
{
	if(string(name) == name_) return this;
	for(auto ite = children_.begin();ite != children_.end();++ite){
		ROT2* r = (*ite)->search(name);
		if(r != NULL) return r;
	}
	return NULL;
}

void _set_q_ik_base(ROT2* r)
{
	r->q_ik_base_ = r->q_al_cl_;
	for(auto ite = r->children_.begin();ite != r->children_.end();++ite){
		_set_q_ik_base(*ite);
	}
}

void ROT2::set_q_ik_base()
{
	_set_q_ik_base(this);
}

double calc_ik_weight(std::vector<ROT2::IK>& iks)
{
	double sum = 0.0;
	double max = 0.0;
	double min = 99999;
	for(auto ite = iks.begin();ite != iks.end();++ite){
		ROT2* start = ite->get<0>();
		ROT2* end = ite->get<1>();
		ROT2* r = start;

		while(r != NULL){
			sum += r->weight_ik_;
			if(max < r->weight_ik_) max = r->weight_ik_;
			if(min > r->weight_ik_) min = r->weight_ik_;
			if(r == end) break;
			r = r->parent_;
		}
	}

	return min;
}

void ROT2::iks(std::vector<ROT2::IK>& iks)
{
	update_pos();
	set_q_ik_base();
	const int num_repeat = 10;
	double a_weight = calc_ik_weight(iks);
	for(int i=0;i<num_repeat;++i){
		for(auto ite = iks.begin();ite != iks.end();++ite){
			ROT2* start = ite->get<0>();
			ROT2* end = ite->get<1>();
			THR pos = ite->get<2>();
			ROT2* r = start;
			while(true){
					
				THR dir_t = (pos - r->p_).normalize();
				THR dir_s = (start->p_ - r->p_).normalize();
				const double min = 0.000000001;
				if(dir_t.get_size_sqr() < min || dir_s.get_size_sqr() < min){
					printf("dir_t or dir_s is 0. skip processing iks\n");
				}else{
					bool angle_set = false;
					Q q = qua::get_quaternion_from_vector(dir_t.x_, dir_t.y_, dir_t.z_, dir_s.x_, dir_s.y_, dir_s.z_);
					// to local coordinate
					q = conj(r->q_parent_move_only_aa_cw_.q())*q*r->q_parent_move_only_aa_cw_.q();
					THR tq(q);
					double b_weight = a_weight/r->weight_ik_;
					q = qua::slerp(Q(1, 0, 0, 0), q, b_weight);
					if(!angle_set){
						r->q_al_cl_ = THR(q*r->q_al_cl_.q());
					}
					update_pos();
				}
				if(r == end) break;
				r = r->parent_;
				if(r == NULL) break;

				bool flag_break = false;
				while(r->parent_ && r->parent_->children_.size() >= 2){
					// skip r which has brother. bvh format lock brothers relative angle.
					if(r == end){
						flag_break = true;
						break;
					}
					r = r->parent_;
				}
				if(flag_break) break;
			}
		}
	}
}

std::vector<double> ROT2::to_bvh_frame(std::vector<THR>& d)
{
	vector<double> ret;
	auto ite = d.begin();
	ret.push_back(ite->x_);
	ret.push_back(ite->y_);
	ret.push_back(ite->z_);
	++ite;
	for(;ite != d.end();++ite){
		double x, y, z;
		qua::q2e(ite->q(), x, y, z, qua::RotSeq::yzx);
		x *= 180/M_PI;
		y *= 180/M_PI;
		z *= 180/M_PI;
		ret.push_back(y);
		ret.push_back(z);
		ret.push_back(x);
	}
	return ret;
}

ROT2::IK ROT2::get_single_ik()
{
	ROT2* p = this;
	if(parent_) p = parent_;
	return boost::make_tuple(this, p, p_);
}

void _get_descendant(std::vector<ROT2*>& ret, ROT2* r){
	ret.push_back(r);
	for(auto ite = r->children_.begin();ite != r->children_.end();++ite){
		_get_descendant(ret, *ite);
	}
}

std::vector<ROT2*> ROT2::get_descendant()
{
	vector<ROT2*> ret;
	_get_descendant(ret, this);
	return ret;
}

double ROT2::normalize_height()
{
	update_pos();
	make_serialized_pointer();
	double min_y = DBL_MAX;
	double max_y = DBL_MIN;
	for(auto ite = serialized_pointer_.begin();ite != serialized_pointer_.end();++ite){
		if((*ite)->p_.y_ < min_y) min_y = (*ite)->p_.y_;
		if((*ite)->p_.y_ > max_y) max_y = (*ite)->p_.y_;
	}
	double size = max_y-min_y;
	for(auto ite = serialized_pointer_.begin();ite != serialized_pointer_.end();++ite){
		(*ite)->len_ /= size;
		//printf("len[%s] = %f\n", (*ite)->name_, (*ite)->len_);
	}
	return 1/size;
	
}

void _make_serialized_pointer(ROT2* r, std::vector<ROT2*>& p)
{
	p.push_back(r);
	for(auto ite = r->children_.begin();ite != r->children_.end();++ite){
		_make_serialized_pointer(*ite, p);
	}
}
void ROT2::make_serialized_pointer()
{
	serialized_pointer_.clear();
	_make_serialized_pointer(this, serialized_pointer_);
}

void ROT2::multiply_len(double a)
{
	make_serialized_pointer();
	for(auto ite = serialized_pointer_.begin();ite != serialized_pointer_.end();++ite){
		(*ite)->len_ *= a;
	}
}

double ROT2::except_y_rotation()
{
	/*
  THR origin(1, 0, 0);
  THR dir_swing = THR(q_al_cl_.q()*origin.q()/q_al_cl_.q());
  THR swing = THR(qua::get_quaternion_from_vector(dir_swing, origin));
  double v1 = 0;
  double v2 = 0;
  double v3 = 0;
  qua::q2e(swing.q(), v1, v2, v3, qua::RotSeq::yxz);
  THR qy = THR(qua::e2q(v1, 0, 0, qua::RotSeq::yxz));

  THR q_except_y = q_al_cl_.q()/qy.q();
	q_al_cl_ = q_except_y;
	return v3;
	*/
	double v1 = 0;
	double v2 = 0;
	double v3 = 0;
	qua::q2e(q_al_cl_.q(), v1, v2, v3, qua::RotSeq::yxz);
	q_al_cl_ = qua::e2q(0, v2, v3, qua::RotSeq::yxz);
	return v1;
	
}

void ROT2::multiply_pos(double a)
{
	make_serialized_pointer();
	for(auto ite = serialized_pointer_.begin();ite != serialized_pointer_.end();++ite){
		(*ite)->p_ = (*ite)->p_*a;
	}
}

void ROT2::normalize_motion(std::vector<ROT2*>& rots)
{
}

std::vector<double> ROT2::get_minmax_pos()
{
	double x_min = DBL_MAX;
	double x_max = DBL_MIN;
	double y_min = DBL_MAX;
	double y_max = DBL_MIN;
	double z_min = DBL_MAX;
	double z_max = DBL_MIN;

	make_serialized_pointer();
	for(auto ite = serialized_pointer_.begin();ite != serialized_pointer_.end();++ite){
		THR p = (*ite)->p_;
		if(p.x_ > x_max) x_max = p.x_;
		if(p.x_ < x_min) x_min = p.x_;
		if(p.y_ > y_max) y_max = p.y_;
		if(p.y_ < y_min) y_min = p.y_;
		if(p.z_ > z_max) z_max = p.z_;
		if(p.z_ < z_min) z_min = p.z_;
	}
	vector<double> ret;
	ret.push_back(x_min);
	ret.push_back(x_max);
	ret.push_back(y_min);
	ret.push_back(y_max);
	ret.push_back(z_min);
	ret.push_back(z_max);
	return ret;
}
