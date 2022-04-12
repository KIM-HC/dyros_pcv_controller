#include "mobile_controller.h"
#include "yaml-cpp/yaml.h"
#include <ros/package.h>
#include <fstream>
#include <cstdio>

/*
this code makes odometry with
recorded q&q_dot data and kinematic parameter(pcv_parameter.yaml)
*/

int main(int argc, char **argv)
{
	const double hz = 300.0 ;
	std::string pkg_path = ros::package::getPath("dyros_pcv_controller");
	bool is_first = true;
	VectorQd q_, q_dot_, tau_, tmp_q, tmp_v;
	tau_.setZero();

	YAML::Node yam_ = YAML::LoadFile(pkg_path + "/setting/pcv_parameter.yaml");
	int param_num = yam_["pcv_index"].as<int>();
	double trash;

	int size_file = yam_["odom_maker_file_list"].size();
	std::cout << "num_file: " << size_file << std::endl;

	for (int f_idx=0; f_idx < size_file; f_idx++) {
		int file_idx = yam_["odom_maker_file_list"][f_idx].as<int>();
		std::string q_inp = pkg_path + "/data/odom_maker/joint_" + std::to_string(file_idx) + ".csv";
		std::string v_inp = pkg_path + "/data/odom_maker/joint_vel_" + std::to_string(file_idx) + ".csv";
		std::ifstream qf, vf;
		qf.open(q_inp);
		vf.open(v_inp);
		MobileController mc(hz, pkg_path, false);
		while (!qf.eof())
		{
			// read one tick
			for (int i=0; i<9; i++) {
				if (i == 0) {
					qf >> trash; vf >> trash;
				}
				else {
					qf >> tmp_q(i-1);
					vf >> tmp_v(i-1);
				}
			}

			// change order
			// (canopen order and controller order is different)
			for (int i=0; i<4; i++) {
				int controller_s = i * 2;
				int controller_r = i * 2 + 1;
				int canopen_s = i * 2 + 1;
				int canopen_r = i * 2;
				q_(controller_s) = tmp_q(canopen_s);
				q_dot_(controller_s) = tmp_v(canopen_s);
				q_(controller_r) = tmp_q(canopen_r);
				q_dot_(controller_r) = tmp_v(canopen_r);
			}

			// std::cout << "       q: " << q_.transpose() << std::endl;
			// std::cout << "   tmp_q: " << tmp_q.transpose() << std::endl;
			// std::cout << "   q_dot: " << q_dot_.transpose() << std::endl;

			mc.readJoint(q_, q_dot_, tau_);
			if (is_first)
			{
				mc.setInitialJoint(q_, q_dot_);
				is_first = false;
			}
			mc.compute();
		}
		std::cout << "file number:" << file_idx << " param number:" << param_num << std::endl;
		// p: param_index, f: file_index
		std::string from_path = pkg_path + "/data/raw/pcv_gx.txt";
		std::string to_path = pkg_path + "/data/odom_maker/odom_out/gx_p" + std::to_string(param_num) +
							"_f" + std::to_string(file_idx) + ".txt";
		std::rename(from_path.c_str(), to_path.c_str());
	}
	return 0;
}
