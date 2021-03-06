#pragma once

#include <iostream>
#include <iomanip>
#include <Eigen/Dense>
#include <memory>
#include <fstream>
#include "yaml-cpp/yaml.h"
#include "math_type_define.h"
#include "pcv_mtx_utils.h"
#include "vehicle.h"

class MobileController
{
    public:
        MobileController(const double hz, const std::string pkg_path, const bool print_option=true, const int param_index=-1); 
        ~MobileController(){};
        
        // get the raw data from the robot or simulator
        void readJoint(const VectorQd &q, const VectorQd &q_dot, const VectorQd &tau);
        void readJoy(const Eigen::Vector3d &joy_input);
        void setInitialJoint(const VectorQd &q, const VectorQd &q_dot);
        void initClass();
        void initMode();
        void compute();
        void setMode(const std::string & mode);
        void updateTime(double dt);
        void resetOpSpace();
        void startFollowTarget();

        VectorQd setDesiredJointTorque();
        VectorQd setDesiredJointVelocity();

        Vehicle veh_;

        // JT-SPACE PARAMETERS(i.e., steer/wheel joint)
        //                      pos               vel        acc
        VectorQd                  q_,           q_dot_,    q_ddot_;     // phi, rho
        VectorQd                 qd_,          qd_dot_,   qd_ddot_;     // desired
        VectorQd             q_init_,      q_dot_init_;                 // init values
        VectorQd           q_filter_,    q_dot_filter_;                 // for low pass filter
        VectorQd             q_prev_,      q_dot_prev_;                 // for low pass filter
        VectorQd           q_target_;
        VectorQd            q_error_;
        VectorQd                            q_dot_hat_;                 // computed by Jacobian
        VectorQd                           q_dot_null_;
        VectorQd                                              tau_;
        VectorQd                                         tau_null_;
        VectorQd                                             rtqS_;
        VectorQd                                              tqS_;
        VectorQd                                              tqE_;
        VectorQd                              cmd_vel_,   cmd_tau_;     // commands for joint

        // OP-SPACE PARAMETERS(i.e., mobile base (X,Y,THETA))
        //                      pos               vel         acc
        Eigen::Vector3d    target_op;
        Eigen::Vector3d     target_1;
        Eigen::Vector3d     target_2;
        Eigen::Vector3d    gtarget_1;
        Eigen::Vector3d    gtarget_2;
        Eigen::Vector3d           x_,           x_dot_,    x_ddot_;     // local coordinate
        Eigen::Vector3d          gx_,          gx_dot_,   gx_ddot_;     // global coordinate
        Eigen::Vector3d          xd_,          xd_dot_,   xd_ddot_;     // desired
        Eigen::Vector3d      x_init_,      x_dot_init_;
        Eigen::Vector3d      x_prev_,      x_dot_prev_;
        Eigen::Vector3d     gx_init_,     gx_dot_init_;
        Eigen::Vector3d     gx_prev_,     gx_dot_prev_;
        Eigen::Vector3d    x_filter_,    x_dot_filter_;
        Eigen::Vector3d   gx_filter_,   gx_dot_filter_;
        Eigen::Vector3d                        rx_dot_;
        Eigen::Vector3d                       rgx_dot_;
        Eigen::Vector3d                                         f_;
        Eigen::Vector3d                                        fd_;
        Eigen::Vector3d                                   fd_star_;
        Eigen::Vector3d                                  gfd_star_;     // global frame
        Eigen::Vector3d     x_delta_,     x_dot_delta_;
        Eigen::Vector3d    x_target_,    x_dot_target_;
        Eigen::Matrix3d         rot_;
        double              heading_;

        Eigen::Vector3d   joy_input_;
        Eigen::Vector3d                     joy_speed_;
        Eigen::Vector2d                  op_max_speed_;

        // VIRTUAL LINKAGE MODEL
        double Kp_E_;
        Eigen::MatrixXd E_;
        Eigen::VectorXd tE_;
        Eigen::VectorXd rtE_;
        Eigen::VectorXd ctE_;

        // DYNAMICS PARAMETERS
        MatrixQtd   Jcp_,   J_;
        MatrixQd    Jcpt_,  Jt_, Jcp_inv_;
        MatrixQd    C_;             // wheel constraint matrix
        Eigen::Matrix3d Lambda_;
        Eigen::Vector3d Mu_;

        // GAINS
        VectorQd        Kp_joint,   Kv_joint;
        VectorQd        weight_;
        Eigen::Vector3d Kp_task,    Kv_task;
        Eigen::Vector3d Kp_joy,     Kv_joy;

        unsigned long tick_;
        int print_tick_;
        double play_time_;
        double hz_, dt_, duration_;
        double control_start_time_;
        double additional_mass_;
        double multiplier_, damping1_;
        double steer_weight_, q_dot_gain, print_option_;

        std::string control_mode_, package_path_;
        bool is_mode_changed_, is_op_ctrl, is_plan_global;
        bool is_follow_target_global, is_target_1, is_follow_target{false};
        bool is_torque_control{true};

    private:        
        void printState();
        void saveState();
        void prev();
        void doLowPassFilter();
        int current_target;
        std::vector<Eigen::Vector3d> targets;

        // TODO: make it to add date

        // for debugging
        std::ofstream pcv_q;
        std::ofstream pcv_q_dot;

        std::ofstream pcv_qd;
        std::ofstream pcv_qd_dot;
        std::ofstream pcv_taud;
        std::ofstream pcv_tqs;
        std::ofstream pcv_tqe;

        std::ofstream pcv_x;
        std::ofstream pcv_x_dot;

        std::ofstream pcv_gx;
        std::ofstream pcv_gx_dot;

        std::ofstream pcv_xd;
        std::ofstream pcv_xd_dot;
        std::ofstream pcv_fd_star;
        std::ofstream pcv_fd;

        std::ofstream pcv_debug;


};
