#include <fstream>
#include <string.h>
#include <sstream>
#include <iostream>

#include "bvh.h"
#include "qua.h"

using namespace std;
using namespace boost::math;

typedef quaternion<double> Q;

BVH::BVH()
{
	clear();
}

bool BVH::load(const char* bvh_file_name)
{
	clear();

	// ファイルのオープン
	ifstream  file;
	file.open(bvh_file_name, ios::in);
	if ( file.is_open() == 0 ){
		printf("can't open %s\n", bvh_file_name);
		return false;
	}

	const int size_buf = 1024*4;
	vector<char> buf;
	buf.resize(size_buf);
	const char* separater = " :,\t";
	vector<Joint*> joint_stack;
	Joint* joint = NULL;
	Joint* joint_new = NULL;
	bool is_site = false;

	while (!file.eof()) {
		if ( file.eof() ){
			file.close();
			return false;
		}
		file.getline(&buf[0], size_buf);
		char* token = strtok(&buf[0], separater);

		// empty line
		if (token == NULL) continue;
		if (strncmp(token, "{", 1) == 0){
			// 現在の関節をスタックに積む
			joint_stack.push_back(joint);
			joint = joint_new;
			continue;
		}

		// 関節ブロックの終了
		if (strncmp(token, "}",1) == 0){
			// 現在の関節をスタックから取り出す
			joint = joint_stack.back();
			joint_stack.pop_back();
			is_site = false;
			continue;
		}

		// 関節情報の開始
		if(
			strncmp(token, "ROOT", 4) == 0 ||
			strncmp(token, "JOINT", 5) == 0
		){
			// 関節データの作成
			joint_new = new Joint();
			joint_new->index_ = joints_.size();
			joint_new->parent_ = joint;
			joint_new->has_site_ = false;
			joint_new->offset_[0] = 0.0;
			joint_new->offset_[1] = 0.0;
			joint_new->offset_[2] = 0.0;
			joint_new->site_[0] = 0.0;
			joint_new->site_[1] = 0.0;
			joint_new->site_[2] = 0.0;
			joints_.push_back(joint_new);

			if(joint){
				joint->children_.push_back(joint_new);
			}else{
				joint_root_ = joint_new;
			}

			// 関節名の読み込み
			token = strtok(NULL, "");
			while (*token == ' ') ++token;
			joint_new->name_ = token;
			if(*(joint_new->name_.c_str() + joint_new->name_.size()-1) == '\r'){
				joint_new->name_ = string(joint_new->name_.c_str(), joint_new->name_.size()-1);
			}
			//add to map
			joint_index_[joint_new->name_] = joint_new;
			continue;
		}

		// 末端情報の開始
		if((strncmp(token, "End", 3) == 0)){
			joint_new = joint;
			is_site = true;
			continue;
		}

		// 関節のオフセット or 末端位置の情報
		if(strncmp(token, "OFFSET", 6) == 0){
			// 座標値を読み込み
			token = strtok(NULL, separater);
			double x = token ? atof(token) : 0.0;
			token = strtok(NULL, separater);
			double y = token ? atof(token) : 0.0;
			token = strtok(NULL, separater);
			double z = token ? atof(token) : 0.0;

			// 関節のオフセットに座標値を設定
			if(is_site){
				joint->has_site_ = true;
				joint->site_[0] = x;
				joint->site_[1] = y;
				joint->site_[2] = z;
				is_site = false;
			}
			else
			// 末端位置に座標値を設定
			{
				joint->offset_[0] = x;
				joint->offset_[1] = y;
				joint->offset_[2] = z;
			}
			continue;
		}

		// 関節のチャンネル情報
		if(strncmp(token, "CHANNELS", 7) == 0){
			// チャンネル数を読み込み
			token = strtok(NULL, separater);
			joint->channels_.resize(token ? atoi(token) : 0);

			// チャンネル情報を読み込み
			for(int i=0;i<joint->channels_.size();++i){
				// チャンネルの作成
				Channel*  channel = new Channel();
				channel->joint_ = joint;
				channel->index_ = channels_.size();
				channels_.push_back(channel);
				joint->channels_[i] = channel;

				// チャンネルの種類の判定
				token = strtok(NULL, separater);
				if      (strncmp(token, "Xrotation",9) == 0){ channel->type_ = X_ROTATION; }
				else if (strncmp(token, "Yrotation",9) == 0){ channel->type_ = Y_ROTATION; }
				else if (strncmp(token, "Zrotation",9) == 0){ channel->type_ = Z_ROTATION; }
				else if (strncmp(token, "Xposition",9) == 0){ channel->type_ = X_POSITION; }
				else if (strncmp(token, "Yposition",9) == 0){ channel->type_ = Y_POSITION; }
				else if (strncmp(token, "Zposition",9) == 0){ channel->type_ = Z_POSITION; }
			}
			continue;
		}

		// Motionデータのセクションへ移る
		if (strncmp(token, "MOTION", 6) == 0) break;
	}

	// モーション情報の読み込み
	file.getline(&buf[0], size_buf);
	char* token = strtok(&buf[0], separater);

	while(token == NULL){
		file.getline(&buf[0], size_buf);
		token = strtok(&buf[0], separater);
	}
	if(strcmp(token, "Frames") != 0){
			file.close();
			return false;
	}

	token = strtok(NULL, separater);

	if(token == NULL){
			file.close();
			return false;
	}

	int num_frame = atoi(token);

	file.getline(&buf[0], size_buf);
	token = strtok(&buf[0], ":");
	if(strcmp(token, "Frame Time") != 0){
			file.close();
			return false;
	}

	token = strtok(NULL, separater);
	if (token == NULL){
			file.close();
			return false;
	}

	interval_ = atof(token);

	// モーションデータの読み込み
	for (int i=0;i<num_frame;++i){
		file.getline(&buf[0], size_buf);
		token = strtok(&buf[0], separater);

		vector<double> m;
		for (int j=0;j<channels_.size();++j){
			if (token == NULL){
				file.close();
				return false;
			}

			m.push_back(atof(token));
			token = strtok(NULL, separater);
		}
		motion_.push_back(m);
	}

	file.close();
	return true;
}

string get_s_indent(int num){
	string s;
	for(int i=0;i<num;++i){
		s += "  ";
	}
	return s;
}

void BVH::write_joint(FILE* f, Joint* j, int* indent)
{
	//ROOT JOINT
	string s_indent = get_s_indent(*indent);
	string s_command = "JOINT";
	if(j == joint_root_){
		s_command = "ROOT";
	}
	vector<char> buf;
	int size_buf = 1024*4;
	buf.resize(size_buf);
	sprintf(&buf[0], "%s%s %s\n", s_indent.c_str(), s_command.c_str(), j->name_.c_str());
	fwrite(&buf[0], strlen(&buf[0]), sizeof(char), f);
	//{
	sprintf(&buf[0], "%s{\n", s_indent.c_str());
	fwrite(&buf[0], strlen(&buf[0]), sizeof(char), f);
	s_indent = get_s_indent(++(*indent));
	//OFFSET
	sprintf(&buf[0], "%sOFFSET %f %f %f\n", s_indent.c_str(), j->offset_[0], j->offset_[1], j->offset_[2]);
	fwrite(&buf[0], strlen(&buf[0]), sizeof(char), f);
	//CHANNELS
	stringstream ss;
	ss << s_indent << "CHANNELS " << j->channels_.size();
	for(auto ite = j->channels_.begin();ite != j->channels_.end();++ite){
		string c(S_CHANNEL[(int)((*ite)->type_)], 9);
		ss << " " << c;
	}
	ss << "\n";
	fwrite(ss.str().c_str(), ss.str().size(), sizeof(char), f);
	//End Site
	if(j->has_site_){
		sprintf(&buf[0], "%sEnd Site\n", s_indent.c_str());
		fwrite(&buf[0], strlen(&buf[0]), sizeof(char), f);
		sprintf(&buf[0], "%s{\n", s_indent.c_str());
		fwrite(&buf[0], strlen(&buf[0]), sizeof(char), f);
		s_indent = get_s_indent(++(*indent));
		sprintf(&buf[0], "%sOFFSET %f %f %f\n", s_indent.c_str(), j->site_[0], j->site_[1], j->site_[2]);
		fwrite(&buf[0], strlen(&buf[0]), sizeof(char), f);
		s_indent = get_s_indent(--(*indent));
		sprintf(&buf[0], "%s}\n", s_indent.c_str());
		fwrite(&buf[0], strlen(&buf[0]), sizeof(char), f);
	}
	for(auto ite = j->children_.begin();ite != j->children_.end();++ite){
		write_joint(f, *ite, indent);
	}
	s_indent = get_s_indent(--(*indent));
	sprintf(&buf[0], "%s}\n", s_indent.c_str());
	fwrite(&buf[0], strlen(&buf[0]), sizeof(char), f);
}

bool BVH::save(const char* file)
{
	Joint* j = joint_root_;
	FILE* f;
	if((f = fopen(file, "w")) == NULL){
		return false;
	}
	vector<char> buf;
	int size_buf = 1024*10;
	buf.resize(size_buf);
	sprintf(&buf[0], "HIERARCHY\n");
	fwrite(&buf[0], strlen(&buf[0]), sizeof(char), f);

	int indent = 0;
	write_joint(f, j, &indent);

	sprintf(&buf[0], "MOTION\n");
	fwrite(&buf[0], strlen(&buf[0]), sizeof(char), f);
	sprintf(&buf[0], "Frames: %d\n", motion_.size());
	fwrite(&buf[0], strlen(&buf[0]), sizeof(char), f);
	sprintf(&buf[0], "Frame Time: %f\n", interval_);
	fwrite(&buf[0], strlen(&buf[0]), sizeof(char), f);
	
	for(auto ite = motion_.begin();ite != motion_.end();++ite){
		stringstream ss;
		for(int j=0;j<channels_.size();++j){
			if(j == 0){
				ss << ite->at(j);
			}else{
				ss << " " << ite->at(j);
			}
		}
		ss << "\n";
		string s = ss.str();
		fwrite(s.c_str(), s.size(), sizeof(char), f);
	}

	fclose(f);
	return true;
}

// デストラクタ
BVH::~BVH()
{
	for(int i=0;i<channels_.size();++i){
		delete channels_[i];
	}

	channels_.clear();
	joints_.clear();
	joint_index_.clear();
}

void BVH::clear()
{
	for (int i=0;i<channels_.size();++i){
		delete channels_[i];
	}
	for(int i=0;i<joints_.size();++i){
		delete joints_[i];
		joints_[i] = NULL;
	}
	motion_.clear();

	channels_.clear();
	joints_.clear();
	joint_index_.clear();
	interval_= 0.0;
}

vector<quaternion<double> > BVH::toQuaternion(const vector<double>& m){
	vector<quaternion<double> > ret;
	int i=0;
	vector<double> v_tmp;
	vector<int> index_type;
	for(auto ite = m.begin();ite != m.end();++ite, ++i){
		v_tmp.push_back(*ite);
		index_type.push_back((int)(channels_.at(i)->type_) - BVH::ChannelEnum::X_ROTATION+1);
		if(v_tmp.size() >= 3){
			bool flag_rotation = true;
			if(channels_.at(i)->type_ >= BVH::ChannelEnum::X_POSITION){
				flag_rotation = false;
			}
			if(flag_rotation){
				qua::RotSeq rs = qua::get_rot_seq(index_type.at(0), index_type.at(1), index_type.at(2));
				quaternion<double> q = qua::e2q(v_tmp.at(0)*M_PI/180, v_tmp.at(1)*M_PI/180, v_tmp.at(2)*M_PI/180, rs);
				ret.push_back(q);
			}else{
				quaternion<double> q(0.0, v_tmp.at(0), v_tmp.at(1), v_tmp.at(2));
				ret.push_back(q);
			}
			index_type.clear();
			v_tmp.clear();
		}
	}
	return ret;
}

vector<double> BVH::toEuler(vector<quaternion<double> >& q)
{
	vector<double> ret;
	int i=0;
	for(auto ite = q.begin();ite != q.end();++ite){
		bool flag_rotation = true;
		if(channels_.at(i)->type_ >= BVH::ChannelEnum::X_POSITION){
			flag_rotation = false;
		}

		if(flag_rotation){
			qua::RotSeq rs = get_rot_seq(i);
			double x = 0;
			double y = 0;
			double z = 0;
			qua::q2e(*ite, x, y, z, rs);
			ret.push_back(x*180/M_PI);
			ret.push_back(y*180/M_PI);
			ret.push_back(z*180/M_PI);
		}else{
			ret.push_back(ite->R_component_2());
			ret.push_back(ite->R_component_3());
			ret.push_back(ite->R_component_4());
		}
		i += 3;
	}
	return ret;
}

vector<boost::math::quaternion<double> > BVH::averageQuaternion(const vector<vector<boost::math::quaternion<double> > > sum)
{
	print(sum.at(0));
	print(sum.at(1));
	print(sum.at(2));
	vector<boost::math::quaternion<double> > ret;
	int num = sum.at(0).size();
	for(int i=0;i<num;++i){
		quaternion<double> q_unit(1.0, 0.0, 0.0, 0.0);
		bool flag_rotation = true;
		if(channels_.at(i*3)->type_ >= ChannelEnum::X_POSITION){
			flag_rotation = false;
		}
		for(int j=0;j<sum.size();++j){
			if(flag_rotation){
				quaternion<double> q_a = sum.at(j).at(i);
				q_unit = q_a*q_unit;
				//q_unit = sum.at(j).at(i);
				//cout << "size(" << j << ", " << i << "):" << sqrt(q_unit.R_component_1()*q_unit.R_component_1()+q_unit.R_component_2()*q_unit.R_component_2()+q_unit.R_component_3()*q_unit.R_component_3()+q_unit.R_component_4()*q_unit.R_component_4()) << endl;
				//cout << q_unit.R_component_1() << ", " << q_unit.R_component_2() << ", " << q_unit.R_component_3() << ", " << q_unit.R_component_4() << endl;
			}else{
				q_unit += sum.at(j).at(i);
			}
		}
		if(flag_rotation){
			;
		}else{
			q_unit /= sum.size();
		}
		ret.push_back(q_unit);
		//cout << q_unit.R_component_1() << ", " << q_unit.R_component_2() << ", " << q_unit.R_component_3() << ", " << q_unit.R_component_4() << endl;
	}

	for(int i=0;i<num;++i){
		if(channels_.at(i*3)->type_ < ChannelEnum::X_POSITION){
			ret.at(i) = quaternion<double>(ret.at(i).R_component_1(), ret.at(i).R_component_2(), ret.at(i).R_component_3(), ret.at(i).R_component_4());
			//ret.at(i) = norm(ret.at(i)/(double)(sum.size()));
			//ret.at(i) = norm(ret.at(i));
		}
	}
	return ret;
}

void BVH::print(const std::vector<boost::math::quaternion<double> >& q)
{
	printf("print size = %d\n", q.size());
	for(int i=0;i<q.size();++i){
		if(channels_.at(i*3)->type_ >= ChannelEnum::X_POSITION){
			printf("position(%.15f, %.15f, %.15f)\n", q.at(i).R_component_2(), q.at(i).R_component_3(), q.at(i).R_component_4());
		}else{
			printf("rotation(%.15f, %.15f, %.15f, %.15f)%f\n", q.at(i).R_component_1(), q.at(i).R_component_2(), q.at(i).R_component_3(), q.at(i).R_component_4(), boost::math::abs(q.at(i)));
		}
	}
}

boost::math::quaternion<double> BVH::normalize_root_rotation()
{
	int index_root_rot = 3;//skip first position
	qua::RotSeq rs = get_rot_seq(index_root_rot);
	Q q = qua::e2q(
		motion_.at(0).at(index_root_rot+0),
		motion_.at(0).at(index_root_rot+1),
		motion_.at(0).at(index_root_rot+2), rs);
	
}

qua::RotSeq BVH::get_rot_seq(int index_data)
{
	return qua::get_rot_seq(
		channels_.at(index_data+0)->type_ - BVH::ChannelEnum::X_ROTATION + 1,
		channels_.at(index_data+1)->type_ - BVH::ChannelEnum::X_ROTATION + 1,
		channels_.at(index_data+2)->type_ - BVH::ChannelEnum::X_ROTATION + 1);
}
