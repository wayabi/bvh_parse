#ifndef __MYC_SAN__
#define __MYC_SAN__
#include "a.h"
#include "pol.h"

#include <vector>

class SAN {
public:
	SAN();
	void plot(std::vector<std::vector<double> >& p, int r, int g, int b);
	void plot(std::vector<double>& p, int r, int g, int b);
	void sla(std::vector<double>& p1, std::vector<double>& p2, int r, int g, int b);

private:
	void point(int x, int y, int r, int g, int b);
	const int w_img_ = 400;
	const int h_img_ = 400;

public:
	sm img_;
	double len_cube_;
};

#endif
