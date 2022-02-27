#include "mobile_controller.h"
#include "ros_node.h"

#include "linux_terminal_tool.h"
#define MODE(X,Y) case X: mc.setMode(Y); break;

int main(int argc, char **argv)
{
	const double hz = 300.0 ;
	std::string pkg_path = ros::package::getPath("dyros_pcv_controller");
	bool is_simulation_run = true;
	bool exit_flag = false;
	bool is_first = true;

	ros::init(argc, argv, "pcv_controller");
	ros::NodeHandle nh;
	ros::Rate loop_rate(hz);
	RosNode rn(nh);
	while(!rn.updated_first) {
		loop_rate.sleep();
		if (ros::isShuttingDown()) {
			break;
			rn.stopPublisher();
			is_simulation_run = false;
			exit_flag = true;
		}
	}
	MobileController mc(hz, pkg_path);

	while (!ros::isShuttingDown() && !exit_flag)
	{
		rn.read();
		mc.readJoint(rn.getCurrentJointAngle(), rn.getCurrentJointVelocity(), rn.getCurrentJointTorque());
		mc.readJoy(rn.getJoyInput());
		if (is_first)
		{
			mc.setInitialJoint(rn.getCurrentJointAngle(), rn.getCurrentJointVelocity());
			mc.setMode("none");
			is_first = false;
		}

		if (kbhit())
		{
			int key = getchar();
			switch (key)
			{
				MODE('t', "joy_control_test")
				MODE('j', "joy_control")
				// MODE('s', "steer_control")
				// MODE('i', "steer_init")
				// MODE('w', "wheel_control")
				MODE('n', "none")
				case 'o':
					mc.is_target_1 = true;
					mc.setMode("op_control");
					break;
				case 'p':
					mc.is_target_1 = false;
					mc.setMode("op_control");
					break;
				case 'h':
					mc.setMode("none");
					rn.homingPublisher();
					break;
				case 'q':
					mc.setMode("none");
					rn.stopPublisher();
					is_simulation_run = false;
					exit_flag = true;
					break;
				case '\t':
					if (is_simulation_run)
					{
						std::cout << "Simulation Pause" << std::endl;
						is_simulation_run = false;
					}
					else
					{
						std::cout << "Simulation Run" << std::endl;
						is_simulation_run = true;
					}
					break;
				default:
					break;
			}
		}

		if (is_simulation_run) {
			mc.compute();
			rn.jointTorquePublisher(mc.setDesiredJointTorque());
		}
		loop_rate.sleep();
		
	}
		
	return 0;
}
