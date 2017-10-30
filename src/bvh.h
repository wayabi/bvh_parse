#ifndef _BVH_H_
#define _BVH_H_

#include <vector>
#include <list>
#include <map>
#include <string>
#include <string.h>
#include <boost/math/quaternion.hpp>
#include "qua.h"

class BVH
{
 public:
	/* 内部用構造体 */
	// チャンネルの種類
	enum ChannelEnum
	{
		X_ROTATION, Y_ROTATION, Z_ROTATION,
		X_POSITION, Y_POSITION, Z_POSITION
	};
	char S_CHANNEL[6][10] = {
		"Xrotation", "Yrotation", "Zrotation",
		"Xposition", "Yposition", "Zposition"
	};
	struct Joint;

	struct Channel
	{
		Joint* joint_;
		ChannelEnum type_;
		int index_;
	};

	struct Joint
	{
		std::string name_;
		int index_;
		Joint* parent_;
		std::vector<Joint*> children_;
		double offset_[3];
		bool has_site_;
		double site_[3];
		std::vector<Channel*> channels_;
	};

 private:
	bool is_load_success_;
	std::map<std::string, Joint*> joint_index_; // 関節名から関節情報へのインデックス

public:
	double interval_; // フレーム間の時間間隔
	std::vector<std::vector<double> > motion_;  // [フレーム番号][チャンネル番号]
	std::vector<Channel*>  channels_; // チャンネル情報 [チャンネル番号]
	std::vector<Joint*> joints_;  // 関節情報 [パーツ番号]
	Joint* joint_root_;

 public:
	BVH();
	~BVH();

	void clear();

	// BVHファイルのロード
	bool load(const char* bvh_file_name);
	bool save(const char* file_name);
	void write_joint(FILE* f, Joint* j, int* indent);

 public:
	Joint* GetJoint(const std::string& j) const
	{
		std::map<std::string, Joint*>::const_iterator i = joint_index_.find(j);
		return (i != joint_index_.end() ) ? (*i).second : NULL;
	}

	Joint* GetJoint(const char* name) const
	{
		//map< string, Joint * >::const_iterator i = joint_index.find( j );
		//return ( i != joint_index.end() ) ? (*i).second : NULL; }
		std::map<std::string, Joint*>::const_iterator ite = joint_index_.begin();

		while(ite != joint_index_.end())
		{
			if(strncmp(name,(*ite).first.c_str(),strlen(name)) == 0)
			{
				return (*ite).second;
			}
			++ite;
		}
		return NULL;
	}

public:
	std::vector<boost::math::quaternion<double> > toQuaternion(const std::vector<double>& m);
	std::vector<double> toEuler(std::vector<boost::math::quaternion<double> >& q);
	std::vector<boost::math::quaternion<double> > averageQuaternion(const std::vector<std::vector<boost::math::quaternion<double> > > sum);
	int get_channel_type(int index){
		return channels_.at(index)->type_;
	}
	void print(const std::vector<boost::math::quaternion<double> >& q);
	//ret Q of initial root rotation
	boost::math::quaternion<double> normalize_root_rotation();
	qua::RotSeq get_rot_seq(int index_data);
};

#endif // _BVH_H_
