#include "utiuti.h"

using namespace std;

std::vector<std::string> Utiuti::split(const char* c)
{
	vector<string> ret;
	int i = 0;
	while(*(c+i) == '\t' || *(c+i) == ' ') ++i;
	int start = i;
	while(*(c+i) != '\0' && *(c+i) != '\n'){
		while(*(c+i) != ',' && *(c+i) != '\0' && *(c+i) != '\n') ++i;
		string s(c+start, 0, i-start);
		ret.push_back(s);
		if(*(c+i) != '\0'){
			++i;
			if(*(c+i) == '\0' || *(c+i) == '\n'){
				if(*(c+i-1) == ','){
					ret.push_back("");
				}
			}
		}
		start = i;
	}
	return ret;
}

