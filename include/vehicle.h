/*======================================================================
Copyright 2019 Google LLC

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    https://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
======================================================================*/

#ifndef _Vehicle_h_
#define _Vehicle_h_
#include <cmath>
#include <Eigen/Dense>
#include <iostream>
#include "caster.h"
#include "pcv_mtx_utils.h"

#define NUM_TRUSS_LINKS 6

class Vehicle
{
public:
  Vehicle();
  ~Vehicle();

  // void Home(void);
  void JointRad(VectorQd &q);
  // void JointRad(VectorQd &jRad);
  // void MotorTq(VectorQd const &mTq);
  // void JointTq(VectorQd const &jTq);

  // void JtTq2MotAmp(VectorQd const &jtq, VectorQd &motAmp);

  // ADD PARTS THAT ARE FIXED TO VEHICLE
  // USE x,y LOCATION OF CENTER OF MASS OF EACH PIECE
  // USE INERTIA ABOUT PIECE'S OWN CENTER OF MASS
  void Add_Solid(double _x, double _y, double _M, double _I);

  // FILL Lambda AND Mu FOR VEHICLE (THIS OBJECT)
  void Dyn(VectorQd &qd, double w);

  void Add_Caster(int idx, double Kx, double Ky, double ang,
                  double _b = XR_b, double _r = XR_r,
                  double _f = XR_f, double _Mf = XR_Mf, double _If = XR_If,
                  double _Ih = XR_Ih, double _Ii = XR_Ii,
                  double _Is = XR_Is, double _It = XR_It, double _Ij = XR_Ij,
                  double _Ns = XR_Ns, double _Nt = XR_Nt, double _Nw = XR_Nw,
                  double _px = XR_px, double _py = XR_py,
                  double _Mp = XR_Mp, double _Ip = XR_Ip);

  void Fill_C(MatrixQd &C);
  void Fill_J(MatrixQtd &J);
  void Fill_Jcp(MatrixQtd &J);  
  // void Fill_Jt_gamma(MatrixQd &Jt);
  void Fill_E_p(MatrixXd &E);
  void Fill_E_q(MatrixXd &E);

  Vector3d Fill_tqS(VectorQd const &qd, VectorQd const &tq, VectorQd &tqS);

  Matrix3d Lambda_;
  Vector3d Mu_;

  double M_gross_;          // Total Mass of vehicle + casters
  double I_gross_;          // Approximate total Inertia of vehicle + casters
  double X_gross_, Y_gross_; // CoM of approx total mass

private:

  void Add_Gross(double _x, double _y, double _M, double _I);
  void Fill_Mu_veh(double w);

  double X_veh_, Y_veh_, M_veh_, I_veh_;

  Caster cstr_[4] = {Caster(0.0,0.0,0.0), Caster(0.0,0.0,0.0), Caster(0.0,0.0,0.0), Caster(0.0,0.0,0.0)};

  Matrix3d L_veh_;
  Vector3d Mu_veh_;
};

#endif // _Vehicle_h_
