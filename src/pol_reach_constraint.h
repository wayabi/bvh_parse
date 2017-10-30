#ifndef __MYC_POL_REACH_CONSTRAINT__
#define __MYC_POL_REACH_CONSTRAINT__

#include <vector>
#include <utility>
#include <boost/shared_ptr.hpp>
#include "thr.h"
#include "two.h"
#include "pol.h"

class PolReachConstraint {
public:
	static boost::shared_ptr<PolReachConstraint> create_by_regular_polygon(THR v_local, double max_theta, int num_pol, double twist_theta);
	static boost::shared_ptr<PolReachConstraint> create_by_line(THR pos_a, THR pos_b, double twist_theta);

	PolReachConstraint();
	PolReachConstraint(const PolReachConstraint& p);
	bool isIn(THR v);
	THR toIn(THR v);
	boost::shared_ptr<TWO> onPlain(THR v);

public:
	std::vector<TWO> pol_;
	THR v_local_;
	double twist_theta_;//premised plus/minus equivalent
};

#endif
