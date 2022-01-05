#include "ros_node.h"

RosNode::RosNode(ros::NodeHandle &nh) : nh_(nh) {
    pub_ = nh_.advertise<sensor_msgs::JointState>("/dyros_mobile/desired_joint", 10);
    sub_ = nh_.subscribe("/dyros_mobile/joint_state", 10, &RosNode::jointCallback, this);
    sub_joy_ = nh_.subscribe("/joy", 10, &RosNode::joyCallback, this);
    for (int i=0; i<N_CASTERS*2; i++) {
        msg_.name.push_back("m"+std::to_string(i+1));
        msg_.position.push_back(0);
        msg_.velocity.push_back(0);
        msg_.effort.push_back(0);
    }
    q_.setZero();
    q_dot_.setZero();
    tau_.setZero();
    joy_input_.setZero();
    time_checker = ros::Time::now();
    ros::Duration(0.3).sleep();
    ros::spinOnce();
    ros::spinOnce();
}

void RosNode::jointCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    for (int i=0; i<N_CASTERS; i++) {
        int controller_s = i * 2;
        int controller_r = i * 2 + 1;
        int canopen_s = i * 2 + 1;
        int canopen_r = i * 2;

        q_(controller_s) = msg->position[canopen_s];
        q_dot_(controller_s) = msg->velocity[canopen_s];
        tau_(controller_s) = msg->effort[canopen_s];

        q_(controller_r) = msg->position[canopen_r];
        q_dot_(controller_r) = msg->velocity[canopen_r];
        tau_(controller_r) = msg->effort[canopen_r];
    }
}

void RosNode::joyCallback(const sensor_msgs::Joy::ConstPtr& msg) {
    time_checker = ros::Time::now();
    if (msg->buttons[7] == 1) {  // changes with what dongle you use
        joy_input_(0) = msg->axes[1];
        joy_input_(1) = msg->axes[0];
        joy_input_(2) = msg->axes[2];
    } else {
        joy_input_.setZero();
    }
}

void RosNode::jointTorquePublisher(const VectorQd& tau) {
    msg_.header.stamp = ros::Time::now();
    for (int i=0; i<N_CASTERS; i++) {
        int controller_s = i * 2;
        int controller_r = i * 2 + 1;
        int canopen_s = i * 2 + 1;
        int canopen_r = i * 2;

        msg_.effort[canopen_s] = tau[controller_s];

        msg_.effort[canopen_r] = tau[controller_r];
    }
    pub_.publish(msg_);
}

void RosNode::homingPublisher() {
    msg_.header.stamp = ros::Time::now();
    for (int i=0; i<2*N_CASTERS; i++) {
        msg_.effort[i] = 0.0;
    }
    msg_.position[0] = 3.0;
    pub_.publish(msg_);
    ros::Duration(0.1).sleep();
    pub_.publish(msg_);
    msg_.position[0] = 0.0;
}

void RosNode::stopPublisher() {
    msg_.header.stamp = ros::Time::now();
    for (int i=0; i<2*N_CASTERS; i++) {
        msg_.effort[i] = 0.0;
    }
    msg_.position[0] = 9.0;
    pub_.publish(msg_);
    ros::Duration(0.1).sleep();
    pub_.publish(msg_);
    msg_.position[0] = 0.0;
}

void RosNode::read() {
    ros::spinOnce();
}

Eigen::Vector3d RosNode::getJoyInput() {
    if ((ros::Time::now() - time_checker).toSec() > 0.07) {
        joy_input_.setZero();
    }
    return joy_input_;
}

VectorQd RosNode::getCurrentJointAngle() {
    return q_;
}

VectorQd RosNode::getCurrentJointVelocity() {
    return q_dot_;
}

VectorQd RosNode::getCurrentJointTorque() {
    return tau_;
}

