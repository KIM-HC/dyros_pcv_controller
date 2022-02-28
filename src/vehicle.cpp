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
#include "vehicle.h"

// // KHC Try
// #define N_2X2_MATRIX  \
//     double N_00 =  cstr_[i].Ns; \
//     double N_10 =  cstr_[i].Nt; \
//     double N_11 = -cstr_[i].Nt * cstr_[i].Nw;

// #define Ni_2X2_MATRIX  \
//     double Ni_00 =  1.0/cstr_[i].Ns; \
//     double Ni_10 =  1.0/(cstr_[i].Ns * cstr_[i].Nw); \
//     double Ni_11 = -1.0/(cstr_[i].Nt * cstr_[i].Nw);

#define N_2X2_MATRIX  \
    double N_00 = -cstr_[i].Ns; \
    double N_10 = -cstr_[i].Nt; \
    double N_11 =  cstr_[i].Nt * cstr_[i].Nw;

#define Ni_2X2_MATRIX  \
    double Ni_00 = -1.0/cstr_[i].Ns; \
    double Ni_10 = -1.0/(cstr_[i].Ns * cstr_[i].Nw); \
    double Ni_11 =  1.0/(cstr_[i].Nt * cstr_[i].Nw);


Vehicle::Vehicle()
:Lambda_(M3Z),Mu_(V3Z),
X_veh_(0),Y_veh_(0),M_veh_(0),I_veh_(0),
L_veh_(M3Z),Mu_veh_(V3Z),M_gross_(0.0),I_gross_(0.0),X_gross_(0.0),Y_gross_(0.0)
{}

Vehicle::~Vehicle()
{}


// void Vehicle::Home()
// {
//   ::Home( cstr[0]->enc_offset,
//           cstr[1]->enc_offset,
//           cstr[2]->enc_offset,
//           cstr[3]->enc_offset);
// }

void Vehicle::Add_Caster(int idx, double _Kx, double _Ky, double _ang,
                         double _b, double _r,
                         double _f, double _Mf, double _If,
                         double _Ih, double _Ii, double _Is, double _It, double _Ij,
                         double _Ns, double _Nt, double _Nw,
                         double _px, double _py, double _Mp, double _Ip)
{
  cstr_[idx] = Caster(_Kx, _Ky, _ang, _b, _r);

  std::cout<<"================"<<std::endl;
  std::cout<<"Add "<<idx<<"th caster"<<std::endl;
  std::cout<<"Kx    : "<<cstr_[idx].Kx<<std::endl;
  std::cout<<"Ky    : "<<cstr_[idx].Ky<<std::endl;
  std::cout<<"ang   : "<<cstr_[idx].ang<<std::endl;
  std::cout<<"offset: "<<cstr_[idx].b<<std::endl;
  std::cout<<"radius: "<<cstr_[idx].r<<std::endl;
  Add_Gross(_Kx, _Ky, _Mf + _Mp, _Ih + _Ii + _Is + _It + _Ij); // approximate only
}

void Vehicle::JointRad(VectorQd &q)
{
  for(int i = 0; i < N_CASTERS; i ++)
  {
    double cali_q = q(2*i) - cstr_[i].ang;
    cstr_[i].a = cali_q;
    cstr_[i].s = sin(cali_q);
    cstr_[i].c = cos(cali_q);
  }
}

// void Vehicle::JointRad(VectorQd &jRad)
// {
//   jRad.setZero();
//   return;
//   ENC_REGISTER e[8];
//   hwi_->EncReadAll(e); // LATCHed by HWInterface

//   for( int i=0; i<num_casters; i++ )
//   {
//     Ni_2X2_MATRIX;

//     int j = 2*i;

//     jRad[ j ] = -(Ni_00*e[j]               )*ENC_CNT2RAD;
//     jRad[j+1] = -(Ni_10*e[j] + Ni_11*e[j+1])*ENC_CNT2RAD;
// // NOTE: MINUS SIGN DUE TO DIRECTION OF ENCODERS
// // NOTE: ENCODERS GIVE OPPOSITE SIGN OF MOTOR ROTATION

//     cstr[i]->a = jRad[j];
//     cstr[i]->c = cos(jRad[j]);
//     cstr[i]->s = sin(jRad[j]);
//   }
// }


// void Vehicle::MotorTq(VectorQd const &mtq)
// /*** Each motor will produce the specified torque ***/
// /*** tq should be a vector of 8 entries in Nm ***/
// {
//   long counts;

//   for( unsigned char j=0; j<2*num_casters; j++ )
//   {
//     counts =(long)( mtq[j] * (1.0/Nm_PER_AMP) * (1.0/AMPS_PER_VOLT) * COUNTS_PER_VOLT );
//     hwi_->RawDAC(j,counts); // RawDAC will check MAX/MIN values
//   }
// }


// void Vehicle::JointTq(VectorQd const &jtq)
// /*** tq should be a vector of 8 entries in Nm ***/
// {
//   static VectorQd mtq;

//   for(int i=0; i<num_casters; i++ )
//   {
//     Ni_2X2_MATRIX;

//     int j = 2*i;
//     mtq[ j ] = Ni_00 * jtq[ j ] + Ni_10 * jtq[j+1];
//     mtq[j+1] =                    Ni_11 * jtq[j+1];
//   }
//   MotorTq(mtq);
// }


// // FOR DATA COLLECTION CONVENIENCE ONLY
// void Vehicle::JtTq2MotAmp(VectorQd const &jtq, VectorQd &motAmp)
// {
//   static VectorQd mtq;
//   for(int i=0; i<num_casters; i++ )
//   {
//     Ni_2X2_MATRIX;
//     int j = 2*i;
//     mtq[ j ] = Ni_00 * jtq[ j ] + Ni_10 * jtq[j+1];
//     mtq[j+1] =                    Ni_11 * jtq[j+1];
//   }
//   motAmp = mtq * (1.0/Nm_PER_AMP);
// }


// CONSTRAINT MATRIX
void Vehicle::Fill_C(MatrixQd &C)
{
  for(int i = 0; i < N_CASTERS; i++ )
  {
    double bi = cstr_[i].bi;
    double ri = cstr_[i].ri;
    double c  = cstr_[i].c;
    double s  = cstr_[i].s;
    double Kx = cstr_[i].Kx;
    double Ky = cstr_[i].Ky;

    int j = 2*i;

    C( j ,0) =   bi*s;
    C( j ,1) =  -bi*c;
    C( j ,2) =  -bi*(Kx*c+Ky*s) - 1.0;

    C(j+1,0) =   ri*c;
    C(j+1,1) =   ri*s;
    C(j+1,2) =   ri*(Kx*s-Ky*c);

    // C( j ,0) =  -bi*s;
    // C( j ,1) =  bi*c;
    // C( j ,2) =  bi*(Kx*c+Ky*s) - 1.0;

    // C(j+1,0) =   -ri*c;
    // C(j+1,1) =   -ri*s;
    // C(j+1,2) =   -ri*(Kx*s-Ky*c);
  }
}

// compute psedo inverse of the Jacobian
void Vehicle::Fill_J(MatrixQtd &J)
{
  MatrixQd C;
  MatrixQtd Ct;
  
  Fill_C(C);
  Ct = C.transpose();

  J = (Ct*C).inverse()*Ct;
}

// JACOBIAN VIA C.P.'s to minimize slip in odometry
// USE Jt_cp to minimize contact forces

void Vehicle::Fill_Jcp(MatrixQtd &J)
{  
  MatrixQd   Cp;
  MatrixQtd  CptCli;
  MatrixQtd  Cpt;
  Matrix3d   CptCp_i;
  // static Matrix3d   J;
  
  for(int i = 0; i < N_CASTERS; i++ )
  {   // Cli IS 2x2 BLOCK DIAGONAL, SO MULTIPLY
      // IN PIECES TO AVOID SLOW NxN MATRIX OPERATIONS
    double b = cstr_[i].b;
    double r = cstr_[i].r;
    double c = cstr_[i].c;
    double s = cstr_[i].s;
    double Kx= cstr_[i].Kx;
    double Ky= cstr_[i].Ky;

    int j = 2*i;

    // Note: sign of p-dot is: wheel as viewed from ground

    CptCli(0, j ) =  b*s;
    CptCli(0,j+1) =  r*c;
    CptCli(1, j ) = -b*c;
    CptCli(1,j+1) =  r*s;
    CptCli(2, j ) = -b*((Kx*c+Ky*s)+b);
    CptCli(2,j+1) =  r* (Kx*s-Ky*c);

    Cp( j ,0) =  1.0;
    Cp( j ,1) =  0.0;
    Cp( j ,2) = -b*s - Ky;

    Cp(j+1,0) =  0.0;
    Cp(j+1,1) =  1.0;
    Cp(j+1,2) =  b*c + Kx;
  }


  // J = inv(Cpt*Cp)*Cpt * inv(Cl)
  // J = inv(Cpt*Cp)*CptCli
  // J = (Cp.transpose()*Cp).llt().solve(CptCli);

  Cpt = Cp.transpose();
  CptCp_i = (Cpt*Cp).inverse();
  J = CptCp_i*CptCli;

  
}


// JACOBIAN to Minimize Motor Torques

// void Vehicle::Fill_Jt_gamma(MatrixQd &Jt)
// {
//   int i;
//   static MatrixQd C     = MatrixQd::Zero();
//   static MatrixQd NC    = MatrixQd::Zero();
//   static MatrixQtd NCtN = MatrixQtd::Zero();

//   Fill_C( C );

//   for( i=0; i<num_casters ; i++ )
//   {   // N IS 2x2 BLOCK DIAGONAL, SO MULTIPLY
//       // IN PIECES TO AVOID SLOW NxN MATRIX OPERATIONS
//     N_2X2_MATRIX;

//     int j = 2*i;

//     NC( j ,0)   = N_00* C(j,0);
//     NC( j ,1)   = N_00* C(j,1);
//     NC( j ,2)   = N_00* C(j,2);
//     NC(j+1,0)   = N_10* C(j,0) + N_11* C(j+1,0);
//     NC(j+1,1)   = N_10* C(j,1) + N_11* C(j+1,1);
//     NC(j+1,2)   = N_10* C(j,2) + N_11* C(j+1,2);

//     NCtN(0, j ) = NC(j,0)*N_00 + NC(j+1,0)*N_10;
//     NCtN(1, j ) = NC(j,1)*N_00 + NC(j+1,1)*N_10;
//     NCtN(2, j ) = NC(j,2)*N_00 + NC(j+1,2)*N_10;
//     NCtN(0,j+1) =                NC(j+1,0)*N_11;
//     NCtN(1,j+1) =                NC(j+1,1)*N_11;
//     NCtN(2,j+1) =                NC(j+1,2)*N_11;
//   }

//   // wdot = Cn xdot ; wdot = N qdot ; qdot = C xdot
//   // J_gamma = Cn# N   ; Cn = N C
//   // J_gamma  = [(NC)t * NC]i * (NC)t N
//   // Jt_gamma = J_gamma.transpose()

//   Jt = ( (NC.transpose()*NC).llt().solve(NCtN) ).transpose()

// }


// JACOBIAN FOR VIRTUAL LINKAGE AT CONTACT POINTS
void Vehicle::Fill_E_p(MatrixXd &E)
{
  int i,j;
  double ex, ey, mag_inv;
  double pix, pjx, piy, pjy;

  // DESCRIBE CONNECTIONS OF VIRTUAL TRUSS

//   int pi[NUM_TRUSS_LINKS]={0,1,2,3,1};
//   int pj[NUM_TRUSS_LINKS]={1,2,3,0,3};
  int pi[NUM_TRUSS_LINKS]={0,3,1,2,1,2};
  int pj[NUM_TRUSS_LINKS]={3,1,2,0,0,3};

  for( int k=0; k<NUM_TRUSS_LINKS; k++ )
  {
    i = pi[k];
    j = pj[k];

    pix = cstr_[i].Kx + cstr_[i].b*cstr_[i].c;
    pjx = cstr_[j].Kx + cstr_[j].b*cstr_[j].c;

    piy = cstr_[i].Ky + cstr_[i].b*cstr_[i].s;
    pjy = cstr_[j].Ky + cstr_[j].b*cstr_[j].s;

    ex = pix - pjx;
    ey = piy - pjy;
    mag_inv = 1.0/hypot(ex,ey);
    E(k, 2*i  ) = ex * mag_inv;
    E(k, 2*i+1) = ey * mag_inv;
    E(k, 2*j  ) =  -E(k, 2*i  );
    E(k, 2*j+1) =  -E(k, 2*i+1);
  }

}


void Vehicle::Fill_E_q(MatrixXd &E)
{
  int j = 2*N_CASTERS;

  MatrixXd Ep(NUM_TRUSS_LINKS,j);
  MatrixXd Cli(j,j);
  // q_dot = Cl * p_dot ==> p_dot = Cli * q_dot
  // eps_dot = Ep * p_dot
  // eps_dot = Ep * Cli * q_dot ==> E_q = Ep * Cli

  Fill_E_p( Ep );
  for(int i=0; i<N_CASTERS; i++ )
  {
    j = 2*i;
    Cli( j , j ) =  cstr_[i].b*cstr_[i].s;
    Cli( j ,j+1) =  cstr_[i].r*cstr_[i].c;

    Cli(j+1, j ) = -cstr_[i].b*cstr_[i].c;
    Cli(j+1,j+1) =  cstr_[i].r*cstr_[i].s;
  }

  E = Ep * Cli;

}


void Vehicle::Add_Solid(double _x, double _y, double _M, double _I)
{
  // COMPUTE NEW veh QUANTITIES w/ ARGS AND PREVIOUS M & I
  X_veh_  = (M_veh_ * X_veh_ + _M * _x) / (M_veh_ + _M);
  Y_veh_  = (M_veh_ * Y_veh_ + _M * _y) / (M_veh_ + _M);
  M_veh_ += _M;  // TOTAL veh MASS
  I_veh_ += _I +_M*(pow(_x,2) + pow(_y,2));  // new I at (0,0) local coords

  // COMPUTE NEW VEHICLE MASS MATRIX L_veh_
  // (ie Lambda_Vehicle for fixed parts not including Casters)
  L_veh_(0,0) =  M_veh_;
  L_veh_(0,1) =  0.0;
  L_veh_(0,2) = -M_veh_ * Y_veh_;

  L_veh_(1,0) =  0.0;
  L_veh_(1,1) =  M_veh_;
  L_veh_(1,2) =  M_veh_ * X_veh_;

  L_veh_(2,0) =  L_veh_(0,2);
  L_veh_(2,1) =  L_veh_(1,2);
  L_veh_(2,2) =  I_veh_;

  Add_Gross(_x,_y,_M,_I);
}


void Vehicle::Add_Gross(double _x, double _y, double _M, double _I)
{
  // COMPUTE NEW ESTIMATES FOR GROSS MASS AND INERTIA (includes Casters)
  X_gross_  = (M_gross_ * X_gross_ + _M * _x) / (M_gross_ + _M);
  Y_gross_  = (M_gross_ * Y_gross_ + _M * _y) / (M_gross_ + _M);
  M_gross_ += _M;
  I_gross_ += _I +_M*(pow(_x,2) + pow(_y,2));  // new I at (0,0) local coords
}


void Vehicle::Fill_Mu_veh( double w )
{
  double w2 = pow(w,2);

  // (ie Mu_Vehicle for fixed parts not including Casters)
  Mu_veh_(0)  = -M_veh_ * X_veh_ * w2;
  Mu_veh_(1)  = -M_veh_ * Y_veh_ * w2;
  Mu_veh_(2)  =  0.0;
}

void Vehicle::Dyn(VectorQd &qd, double w)
{
  // INITIALIZE TO VEHICLE PROPERTIES
  Lambda_ = L_veh_;
  Fill_Mu_veh(w);
  Mu_ = Mu_veh_;

  // ADD DYNAMICS FROM THE CASTERS
  for (int i = 0; i < N_CASTERS; i++)
  {
    int j = 2 * i;
    cstr_[i].Fill_LM(qd[j], qd[j + 1], w);
    Lambda_ += cstr_[i].Lambda;
    Mu_ += cstr_[i].Mu;
  }
}

//                                      q_dot              torque   steer_torque
Vector3d Vehicle::Fill_tqS(VectorQd const &qd, VectorQd const &tq, VectorQd &tqS)
{
  // Experimental function to compute operational force to compensate for
  // rotational friction of the wheel contact patches.
  // qd: q dot              tq: torque
  // atq: absolute torque   frd: 
  // SUFFIX M: multiplication factor (?)

  int i,j;
  double m,b,frd,atq,tqX;
//   double bM[4] = {0.35, 0.57, 0.35, 0.57};
//   double mM[4] = {0.21, 0.17, 0.21, 0.17};
//   double tqM   = 0.35;
  // double rdM   = 25.0;
  // double bM[4] = {0.20, 0.50, 0.20, 0.50};
  // double mM[4] = {0.10, 0.10, 0.10, 0.10};
  // double tqM   = 0.60;
  double rdM   = 25.0;
  double bM[4] = {0.20, 0.20, 0.20, 0.20};
  double mM[4] = {0.10, 0.10, 0.10, 0.10};
  double tqM   = 0.60;

  double tqA = 0.0;
  static Vector3d fS = Vector3d::Zero();
    
  for(i=0; i<N_CASTERS; i++)
  { j = 2*i;
    frd = 1.0 - fabs(qd[j+1])/rdM;
    b = bM[i]*frd;
    m = mM[i]*frd;
    atq = fabs(tq[j]);
    tqX = atq<tqM ? atq/tqM : 1.0;  // if (atq<tqM) {atq/tqM}; else {1.0;};
    tqS[j] = (m*qd[j] + (qd[j]>0?b:-b)) * tqX;  // if (qd[j]>0) {b}; else {-b};
    tqA += tqS[j];
  }

  fS[2] = tqA;
  return fS;
}
