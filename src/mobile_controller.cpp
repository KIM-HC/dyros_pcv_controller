#include "mobile_controller.h"


MobileController::MobileController(const double hz, const std::string pkg_path) : 
tick_(0), play_time_(0.0), hz_(hz), control_mode_("none"), is_mode_changed_(false), dt_(1/hz), package_path_(pkg_path)
{
  veh_.Add_Solid(  0.000,  0.000,  XR_Mv, XR_Iv);
  // // angle info is not used --> use home_offset for canopen
  // veh_.Add_Caster(0,  XR_l,  XR_w, 0.5*M_PI_2); // front left 
  // veh_.Add_Caster(1,  XR_l, -XR_w, 3.5*M_PI_2); // front right
  // veh_.Add_Caster(2, -XR_l, -XR_w, 2.5*M_PI_2); // rear right
  // veh_.Add_Caster(3, -XR_l,  XR_w, 1.5*M_PI_2); // rear left

  veh_.Add_Caster(0,  XR_l,  XR_w, 0.0); // front left 
  veh_.Add_Caster(1,  XR_l, -XR_w, 0.0); // front right
  veh_.Add_Caster(2, -XR_l, -XR_w, 0.0); // rear right
  veh_.Add_Caster(3, -XR_l,  XR_w, 0.0); // rear left

  initClass();

  print_tick_ = int(hz_ * 0.5);

  pcv_q.open(       package_path_ + "/save_data/pcv_q.txt");
  pcv_q_dot.open(   package_path_ + "/save_data/pcv_q_dot.txt");

  pcv_qd.open(      package_path_ + "/save_data/pcv_qd.txt");
  pcv_qd_dot.open(  package_path_ + "/save_data/pcv_qd_dot.txt");
  pcv_taud.open(    package_path_ + "/save_data/pcv_taud.txt");
  pcv_tqs.open(     package_path_ + "/save_data/pcv_tqs.txt");
  pcv_tqe.open(     package_path_ + "/save_data/pcv_tqe.txt");

  pcv_x.open(       package_path_ + "/save_data/pcv_x.txt");
  pcv_x_dot.open(   package_path_ + "/save_data/pcv_x_dot.txt");

  pcv_xd.open(      package_path_ + "/save_data/pcv_xd.txt");
  pcv_xd_dot.open(  package_path_ + "/save_data/pcv_xd_dot.txt");
  pcv_fd_star.open( package_path_ + "/save_data/pcv_fd_star.txt");
  pcv_fd.open(      package_path_ + "/save_data/pcv_fd.txt");

  pcv_debug.open(   package_path_ + "/save_data/pcv_debug.txt");

  std::cout<<"Load Mobile Controller"<<std::endl;
}

void MobileController::compute()
{
  // update parameters
  veh_.JointRad(q_);
  veh_.Fill_C(C_);
  veh_.Fill_J(J_);
  veh_.Fill_Jcp(Jcp_);          // NOTE: VIA CONTACT POINTS - for MIN CONTACT FORCES
  Jt_ = J_.transpose();
  Jcpt_ = Jcp_.transpose();

  // BEGIN ODOMETRY SECTION
  q_dot_filter_ = DyrosMath::lowPassFilter(q_dot_, q_dot_prev_, 1/hz_, 10.0);
  q_dot_ = q_dot_filter_;

  // FIND LOCAL OPERATIONAL SPEEDS
  rx_dot_ = Jcp_ * q_dot_;      // RAW LOCAL OP SPEEDS
  x_ += rx_dot_ * dt_;      // "LOCAL" COORDS
  x_dot_filter_ = DyrosMath::lowPassFilter(rx_dot_, x_dot_prev_, dt_, 10.0);
  x_dot_ = x_dot_filter_;       // LOCAL SPEED (ALSO FOR DYN)

  // DELTA: MAP LOCAL --> GLOBAL COORDS
  heading_ = gx_(2) + 0.5 * rx_dot_(2) * dt_;  // USE raw x dot
  rot_(0,0) =  cos(heading_);
  rot_(0,1) = -sin(heading_);
  rot_(1,0) =  sin(heading_);
  rot_(1,1) =  cos(heading_);
  rgx_dot_ = rot_ * rx_dot_;    // RAW GLOBAL OP SPEEDS

  // INTEGRATION TO GLOBAL COORDS
  gx_ += rgx_dot_ * dt_;
  gx_dot_filter_ = DyrosMath::lowPassFilter(rgx_dot_, gx_dot_prev_, dt_, 10.0);
  gx_dot_ = gx_dot_filter_;       // GLOBAL SPEED (ALSO FOR DYN)

  // END ODOMETRY SECTION

  q_dot_hat_ = C_ * x_dot_; // ignore any slip for dynamics
  veh_.Dyn(q_dot_hat_, x_dot_(2)); // veh_.Dyn(q_dot_, x_dot_(2));
  Lambda_ = veh_.Lambda_;
  Mu_ = veh_.Mu_;

  if (is_mode_changed_) {
    is_mode_changed_ = false;
    control_start_time_ = play_time_;

    initMode();
  }

  if(control_mode_ == "op_control") {
    // x_target_ << 1.0, 0.0, 0.0;
    x_target_ << 0.0, -0.5, 0.0;
    // x_target_ << 0.0, 0.0, M_PI_2;
    duration_ = 40.0 * (x_init_ - x_target_).norm();

    for(int i = 0; i < 3; i ++) {
      Eigen::Vector3d quintic_temp;
      quintic_temp = DyrosMath::quinticSpline(play_time_, control_start_time_, control_start_time_ + duration_, x_init_(i), x_dot_init_(i), 0.0, x_target_(i), 0.0, 0.0);     

      xd_(i)      = quintic_temp(0);
      xd_dot_(i)  = quintic_temp(1);
      xd_ddot_(i) = quintic_temp(2);
    }

    x_delta_ = xd_ - x_;
    x_dot_delta_ = xd_dot_ - x_dot_;

    fd_star_ = xd_ddot_
                 + Kp_task.asDiagonal() * x_delta_
                 + Kv_task.asDiagonal() * x_dot_delta_;

    taud_ = Jcpt_ * (Lambda_ * fd_star_ + Mu_);  // with Mu
    // taud_ = Jcpt_ *  Lambda_ * fd_star_ ;        // without Mu
    taud_ = weight_.asDiagonal() * taud_;
  }

  else if(control_mode_ == "joy_control_test") {
    if (joy_input_.head<2>().norm() > 1.0) {
      joy_input_.head<2>() = joy_input_.head<2>().normalized();
    }
    xd_dot_ = joy_speed_.asDiagonal() * joy_input_;
    xd_ = xd_ + xd_dot_ * dt_;

    x_delta_ = xd_ - x_;
    x_dot_delta_ = xd_dot_ - x_dot_;

    fd_star_ = Kp_task.asDiagonal() * x_delta_
             + Kv_task.asDiagonal() * x_dot_delta_;

    taud_ = Jcpt_ * (Lambda_ * fd_star_ + Mu_);  // with Mu
    // taud_ = Jcpt_ *  Lambda_ * fd_star_ ;        // without Mu
    taud_ = weight_.asDiagonal() * taud_;
  }

  else if(control_mode_ == "joy_control") {
    if (joy_input_.head<2>().norm() > 1.0) {
      joy_input_.head<2>() = joy_input_.head<2>().normalized();
    }
    Eigen::Vector3d tmp_kp{15000, 15000, 15000};
    Eigen::Vector3d imp_sp{0.15, 0.15, 0.4};

    x_delta_ = imp_sp.asDiagonal() * joy_input_;

    fd_star_ = tmp_kp.asDiagonal() * x_delta_;

    taud_ = Jcpt_ * (Lambda_ * fd_star_ + Mu_);  // with Mu
    // taud_ = Jcpt_ *  Lambda_ * fd_star_ ;        // without Mu
    taud_ = weight_.asDiagonal() * taud_;
  }

  else if(control_mode_  == "steer_control") {
    // q_target_ << 1.5*M_PI_2, q_init_(1), -1.5*M_PI_2, q_init_(3), -0.5*M_PI_2, q_init_(5), 0.5*M_PI_2, q_init_(7);
    // q_target_ << M_PI, q_init_(1), -M_PI, q_init_(3), -M_PI, q_init_(5), M_PI, q_init_(7);
    q_target_ << 0.0, q_init_(1), 0.0, q_init_(3), 0.0, q_init_(5), 0.0, q_init_(7);
    duration_ = 1.5 * (q_init_ - q_target_).norm();

    for(int  i = 0; i < 2*N_CASTERS; i++) {
      Eigen::Vector3d q_temp;
      q_temp = DyrosMath::quinticSpline(play_time_, control_start_time_, control_start_time_ + duration_,  q_init_(i), 0.0, 0.0, q_target_(i), 0.0, 0.0);     
      qd_(i)      = q_temp(0);
      qd_dot_(i)  = q_temp(1);
      qd_ddot_(i) = q_temp(2);
    }

    taud_ = qd_ddot_ + Kp_joint.asDiagonal()*(qd_ - q_) + Kv_joint.asDiagonal()*(qd_dot_ -  q_dot_);
  }

  else if(control_mode_  == "steer_init") {
    // q_target_ << 0.0, q_init_(1), 0.0, q_init_(3), 0.0, q_init_(5), 0.0, q_init_(7);
    q_target_ << -1.5*M_PI_2, q_init_(1), 1.5*M_PI_2, q_init_(3), 0.5*M_PI_2, q_init_(5), -0.5*M_PI_2, q_init_(7);
    duration_ = 1.5 * (q_init_ - q_target_).norm();

    for(int  i = 0; i < 2*N_CASTERS; i++) {
      Eigen::Vector3d q_temp;
      q_temp = DyrosMath::quinticSpline(play_time_, control_start_time_, control_start_time_ + duration_,  q_init_(i), 0.0, 0.0, q_target_(i), 0.0, 0.0);     
      qd_(i)      = q_temp(0);
      qd_dot_(i)  = q_temp(1);
      qd_ddot_(i) = q_temp(2);
    }

    taud_ = qd_ddot_ + Kp_joint.asDiagonal()*(qd_ - q_) + Kv_joint.asDiagonal()*(qd_dot_ -  q_dot_);
  }

  else if (control_mode_ == "wheel_control") {
    double wheel_vel, wheel_ang, time_duration;
    time_duration = 6.0;
    wheel_vel = 0.5;
    wheel_ang = wheel_vel*time_duration;
    q_target_ = q_init_;
    for (int i=0; i<N_CASTERS; i++) {
      q_target_(i*2 + 1) += wheel_ang;
    }

    for(int  i = 0; i < 2*N_CASTERS; i++) {
      Eigen::Vector3d q_temp;
      q_temp = DyrosMath::quinticSpline(play_time_, control_start_time_, control_start_time_ + time_duration,  q_init_(i), 0.0, 0.0, q_target_(i), 0.0, 0.0);     
      qd_(i)      = q_temp(0);
      qd_dot_(i)  = q_temp(1);
      qd_ddot_(i) = q_temp(2);
    }

    taud_ = qd_ddot_ + Kp_joint.asDiagonal()*(qd_ - q_) + Kv_joint.asDiagonal()*(qd_dot_ -  q_dot_);
  }

  else {
    // tau_null_ = 5.0*(q_init_ - q_) + 0.1*(q_dot_init_ - q_dot_init_);
    // tau_null_ = (Eigen::Matrix8d::Identity() - Jcpt_*Jcp_)*tau_null_;

    // fd_star_ = 20*(x_init_ - x_) + 10*(x_dot_init_ - x_dot_);
    // taud_ = Jcpt_*(Lambda_*fd_star_ + Mu_) + tau_null_;

    taud_.setZero();
  }

  // STEERING FRICTION COMPENSATION
  // TODO: I dont understand this Fill_tqS function
  veh_.Fill_tqS(q_dot_, taud_, rtqS_);
  tqS_ = DyrosMath::lowPassFilter(rtqS_, tqS_, 1/hz_, 42.0);
  taud_ += tqS_;

  // // INTERNAL FORCE COMPUTATION
  // // TODO: CHECK ORIGINAL CODE AGAIN!
  // if(true) {
  //   veh_.Fill_E_q( E_ );
  //   // PROJECT TO 'E'-SPACE AND THEN BACK TO JT-SPACE
  //   q_dot_null_ = q_dot_ - q_dot_hat_;  // NULL SPACE WHEEL SPEEDS
  //   rtE_ = E_ * q_dot_null_  ;    // RAW SLIP (INTERNAL VELs)
  //   tE_ = DyrosMath::lowPassFilter(rtE_, tE_, 1/hz_, 5.0); //
  //   ctE_ = Kp_E_ * tE_; // CONTROL FORCES to RESIST SLIP
  //   tqE_ * E_.transpose() * ctE_;   // MAP TO JOINT TORQUES
  //   taud_ += tqE_;
  // }

  prev();
  printState();
  saveState();

  tick_++;
  play_time_ = tick_ * dt_;	// second // TODO: need to use real time
}

void MobileController::initClass()
{
  E_.resize(NUM_TRUSS_LINKS, 2*N_CASTERS);

  tE_.resize(NUM_TRUSS_LINKS);
  ctE_.resize(NUM_TRUSS_LINKS);
  rtE_.resize(NUM_TRUSS_LINKS);

  heading_ = 0.0;
  additional_mass_ = 0.0;
  is_op_ctrl = false;

  x_.setZero();
  x_dot_.setZero();
  x_ddot_.setZero();

  gx_.setZero();
  gx_dot_.setZero();
  gx_ddot_.setZero();

  xd_.setZero();
  xd_dot_.setZero();
  xd_ddot_.setZero();

  gx_prev_.setZero();
  gx_dot_prev_.setZero();

  Kp_joint.setZero();
  Kv_joint.setZero();
  Kp_task.setZero();
  Kv_task.setZero();
  weight_.setZero();

  joy_speed_.setZero();

  initMode();
}

void MobileController::initMode()
{
  taud_.setZero();
  rtqS_.setZero();
  tqS_.setZero();
  tqE_.setZero();

  rot_.setIdentity();

  xd_ddot_.setZero();

  x_delta_.setZero();
  x_dot_delta_.setZero();

  joy_input_.setZero();
  x_target_.setZero();
  x_dot_target_.setZero();

  q_target_.setZero();

  x_init_ = x_;
  x_dot_init_ = x_dot_;

  xd_ = x_init_;
  xd_dot_ = x_dot_init_;

  x_prev_ = x_;
  x_dot_prev_ = x_dot_;

  q_init_ = q_;
  q_dot_init_ = q_dot_;

  q_prev_ = q_;
  q_dot_prev_ = q_dot_;
}

void MobileController::setMode(const std::string & mode)
{
  is_mode_changed_ = true;
  control_mode_ = mode;
  std::cout << "Current mode (changed) : " << mode << std::endl;

  std::ifstream mode_setter(package_path_ + "/setting/basic_setting.txt");
  std::string dummy;
  mode_setter >> dummy;
  for (int i = 0; i < N_CASTERS*2; i++) {mode_setter >> Kp_joint(i);}

  mode_setter >> dummy;
  for (int i = 0; i < N_CASTERS*2; i++) {mode_setter >> Kv_joint(i);}

  mode_setter >> dummy;
  for (int i = 0; i < 3; i++) {mode_setter >> Kp_task(i);}

  mode_setter >> dummy;
  for (int i = 0; i < 3; i++) {mode_setter >> Kv_task(i);}

  mode_setter >> dummy; mode_setter >> Kp_E_;

  mode_setter >> dummy;
  for (int i = 0; i < N_CASTERS*2; i++) {mode_setter >> weight_(i);}

  mode_setter >> dummy;
  for (int i = 0; i < 3; i++) {mode_setter >> joy_speed_(i);}

  veh_.Add_Solid(0.0, 0.0, -additional_mass_, 0.0);
  mode_setter >> dummy;
  mode_setter >> additional_mass_;
  veh_.Add_Solid(0.0, 0.0, additional_mass_, 0.0);

  mode_setter.close();

  if (control_mode_ == "wheel_control" || control_mode_ == "steer_init" ||
      control_mode_ == "steer_control" || control_mode_ == "none") {
        is_op_ctrl = false;
      }
  else {
    is_op_ctrl = true;
  }

  std::cout << " Kp_joint: " << Kp_joint.transpose() << std::endl;
  std::cout << " Kv_joint: " << Kv_joint.transpose() << std::endl;
  std::cout << "  Kp_task: " << Kp_task.transpose() << std::endl;
  std::cout << "  Kv_task: " << Kv_task.transpose() << std::endl;
  std::cout << "     Kp_E: " << Kp_E_ << std::endl;
  std::cout << "   weight: " << weight_.transpose() << std::endl;
  std::cout << "joy_speed: " << joy_speed_.transpose() << std::endl;
  std::cout << "     mass: " << additional_mass_ << std::endl;
}

VectorQd MobileController::setDesiredJointTorque(){ return taud_; }

void MobileController::readJoint(const VectorQd &q, const VectorQd &q_dot, const VectorQd &tau)
{
    q_ = q;
    q_dot_ = q_dot;
    tau_ = tau;
}

void MobileController::readJoy(const Eigen::Vector3d &joy_input)
{
    joy_input_ = joy_input;
}

void MobileController::setInitialJoint(const VectorQd &q, const VectorQd &q_dot)
{
  q_init_ = q;
  q_dot_init_ = q_dot;
}

void MobileController::printState()
{
  if (tick_ % print_tick_ == 0)
  {
    std::cout <<"---------------------------------------------------------------------"<<std::endl;
    std::cout <<"---------------------------------------------------------------------"<<std::endl;
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "    mode: " << control_mode_ << std::endl;
    std::cout << "run time: " << play_time_- control_start_time_ << std::endl;
    std::cout << "duration: " << duration_ << std::endl;
    if (is_op_ctrl) {
      std::cout << "  x_init: " << x_init_.transpose() << std::endl;
      std::cout << "       x: " << x_.transpose() << std::endl;
      std::cout << "   x_dot: " << x_dot_.transpose() << std::endl;
      std::cout << "      xd: " << xd_.transpose() << std::endl;
      std::cout << "  xd_dot: " << xd_dot_.transpose() << std::endl;
      std::cout << " x_delta: " << x_delta_.transpose() << std::endl;
      std::cout << "x_ddelta: " << x_dot_delta_.transpose() << std::endl;
      std::cout << "      Mu: " << Mu_.transpose() << std::endl;
    }
    else {
      std::cout << "  q_init: " << q_init_.transpose() << std::endl;
      std::cout << "       q: " << q_.transpose() << std::endl;
      std::cout << "   q_dot: " << q_dot_.transpose() << std::endl;
      std::cout << "      qd: " << qd_.transpose() << std::endl;
      std::cout << "  qd_dot: " << qd_dot_.transpose() << std::endl;
      std::cout << " qd_ddot: " << qd_ddot_.transpose() << std::endl;
      std::cout << " q_delta: " << (qd_ - q_).transpose() << std::endl;
    }
    std::cout << "    taud: " << taud_.transpose() << std::endl;
    std::cout << "    tqS_: " << tqS_.transpose() << std::endl;
    std::cout << "    tqE_: " << tqE_.transpose() << std::endl;
    // std::cout << "t_n : " << tau_null_.transpose() << std::endl;
    std::cout << " fd_star: " << fd_star_.transpose() << std::endl;
    std::cout << "      Fd: " << (Lambda_ * fd_star_).transpose() << std::endl;
    // std::cout << "L:\n" << Lambda_ << std::endl;      
    // std::cout << "Jcpt  :\n" << Jcpt_ << std::endl;
    // std::cout << "Jt:\n" << Jt_ << std::endl;
    // std::cout << "C:\n" << C_ << std::endl;
  }
}

void MobileController::saveState()
{
  Eigen::IOFormat tab_format(Eigen::FullPrecision, 0, "\t", "\n");

  pcv_q       << q_.transpose().format(tab_format)                    << std::endl;
  pcv_q_dot   << q_dot_.transpose().format(tab_format)                << std::endl;

  pcv_qd      << qd_.transpose().format(tab_format)                   << std::endl;
  pcv_qd_dot  << qd_dot_.transpose().format(tab_format)               << std::endl;
  pcv_taud    << taud_.transpose().format(tab_format)                 << std::endl;
  pcv_tqs     << tqS_.transpose().format(tab_format)                  << std::endl;
  pcv_tqe     << tqE_.transpose().format(tab_format)                  << std::endl;

  pcv_x       << x_.transpose().format(tab_format)                    << std::endl;
  pcv_x_dot   << x_dot_.transpose().format(tab_format)                << std::endl;

  pcv_xd      << xd_.transpose().format(tab_format)                   << std::endl;
  pcv_xd_dot  << xd_dot_.transpose().format(tab_format)               << std::endl;
  pcv_fd_star << fd_star_.transpose().format(tab_format)              << std::endl;
  pcv_fd      << (Lambda_ * fd_star_).transpose().format(tab_format)  << std::endl;

}

void MobileController::prev()
{
  q_prev_ = q_;
  q_dot_prev_ = q_dot_filter_;

  x_prev_ = x_;
  x_dot_prev_ = x_dot_filter_;

  gx_dot_prev_ = gx_dot_filter_; 

}

void MobileController::doLowPassFilter()
{
  // q_ = DyrosMath::lowPassFilter(q_, q_prev_, 1/hz_, 50.0);
  q_dot_filter_ = DyrosMath::lowPassFilter(q_dot_, q_dot_prev_, 1/hz_, 10.0);
  // x_ = DyrosMath::lowPassFilter(x_, x_prev_, 1/hz_, 50.0);
  x_dot_filter_ = DyrosMath::lowPassFilter(x_dot_, x_dot_prev_, 1/hz_, 10.0);

  q_dot_ = q_dot_filter_;
  x_dot_ = x_dot_filter_;
}

// ----------------------------------------------------

