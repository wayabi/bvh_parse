#ifndef __MYC_ROT2__
#define __MYC_ROT2__

#include "bvh.h"
#include "thr.h"
#include "pol_reach_constraint.h"
#include <vector>
#include <string>
#include <boost/tuple/tuple.hpp>
#include <boost/math/quaternion.hpp>

class ROT2 {
public:
	typedef boost::tuple<ROT2*, ROT2*, THR> IK;

	ROT2();
	void set_parents(ROT2* r);
	~ROT2();
	void update_pos();
	ROT2* add();
	void set_serialized_angle(std::vector<THR>& d);
	std::vector<THR> get_serialized_angle();
	ROT2* copy(ROT2* parent);
	ROT2* search(const char* name);
	void set_global_vector(THR v, float weight);
	void ik(ROT2* end, THR pos);
	void iks(std::vector<boost::tuple<ROT2*, ROT2*, THR> >& iks);
	void init_rotate_pol_reach_constraint_v();
	bool write_extension(const char* path);
	void print();
	static ROT2* make_bone(BVH* bvh);
	static std::vector<THR> get_frame(BVH& bvh, int frame);
	static std::vector<double> to_bvh_frame(std::vector<THR>& d);
	ROT2::IK get_single_ik();
	std::vector<ROT2*> get_descendant();

private:
	void set_q_ik_base();

public:
	THR p_;
	THR q_al_cl_;
	THR q_base_al_cl_;
	THR q_aa_cw_;
	THR q_parent_aa_cw_;
	THR q_parent_move_only_aa_cw_;

	THR q_ik_base_;

	double len_;
	std::vector<ROT2*> children_;
	ROT2* parent_;
	std::string name_;
	double weight_ik_;
	boost::shared_ptr<PolReachConstraint> constraint_;
	std::string name_constraint_;
	void normalize_height();
	void make_serialized_pointer();
	std::vector<ROT2*> serialized_pointer_;
	void multiply_len(double a);
};
#endif

