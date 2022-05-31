#ifndef _ROS_NODE_H_
#define _ROS_NODE_H_

#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/String.h>
#include "pcv_mtx_utils.h"


class RosNode
{
    public:
        RosNode(ros::NodeHandle &nh);
        ~RosNode(){};

        void read();
        void homingPublisher();
        void stopPublisher();
        void jointPublisher(const VectorQd& tau, const VectorQd& vel);

        VectorQd getCurrentJointAngle();
        VectorQd getCurrentJointVelocity();
        VectorQd getCurrentJointTorque();
        Eigen::Vector3d getJoyInput();
        bool updated_first{false};

    private:
        void jointCallback(const sensor_msgs::JointState::ConstPtr& msg);
        void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);
        VectorQd q_, q_dot_, tau_;
        ros::NodeHandle &nh_;
        ros::Subscriber sub_, sub_joy_;
        ros::Publisher pub_;
        ros::Time time_checker;
        sensor_msgs::JointState msg_;
        Eigen::Vector3d joy_input_;

};


#endif  // _ROS_NODE_H_