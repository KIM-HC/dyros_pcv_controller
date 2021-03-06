/*======================================================================
Copyright 2019 Google LLC

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You mplaay obtain a copy of the License at

    https://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
======================================================================*/

#include <cmath>
#include <Eigen/Dense>
#include "filter.h"


// New zero-size filter to be Allocated by creation of specific filter type
Filter::Filter()
  : size_(0),order_(0),a_(NULL),b_(NULL),f_(NULL),r_(NULL)
{ }

Filter::~Filter()
// Destructor
{
  Purge();
}

void Filter::Purge()
{
  delete a_;
  delete b_;
  for(int i=0; i<order_+1 ; i++ )
  { delete f_[i];
    delete r_[i];
  }
  delete [] f_;
  delete [] r_;

  size_ = 0;
  order_ = 0;
}

void Filter::Allocate( int size,  int order)
// Allocates memory for filter constants and aging the data
{
  size_ = size;
  order_ = order;

  a_ = new Eigen::VectorXd(order_+1);
  a_->setZero(order+1);
  b_ = new Eigen::VectorXd(order_+1);
  b_->setZero(order+1);

  f_ = new Eigen::VectorXd* [order_+1];
  r_ = new Eigen::VectorXd* [order_+1];
  for(int i=0; i<order_+1 ; i++ )
  { f_[i] = new Eigen::VectorXd(size_);
    f_[i]->setZero(size_);
    r_[i] = new Eigen::VectorXd(size_);
    r_[i]->setZero(size_);
  }
}

Eigen::VectorXd Filter::Filt(Eigen::VectorXd const &in)
// Use this after assigning a filter type to filter the data
{
  int i;

  if( order_ == 0 )  // COPY AND BAIL OUT
  { return in;
  }

  *r_[0] = in;  // NEED THIS ASSIGNMENT FOR AGEING TOO
  *f_[0] = *r_[0] * (*b_)[0];

  for( i=1; i<order_+1 ; i++) // NOTE: a_[0] NOT USED. (NORMALIZED)
  {
    *f_[0] +=   *r_[i] * (*b_)[i] - *f_[i] * (*a_)[i];
  }
  // AGE THE VALUES
  for( i=0; i<order_ ; i++)
  { *f_[i+1] = *f_[i];
    *r_[i+1] = *r_[i];
  }
  return *f_[0];
}

void Filter::Z_NumDen(Eigen::VectorXd const &r0,
                      Eigen::VectorXd const &z_num,
                      Eigen::VectorXd const &z_den,
                      Eigen::VectorXd const &f0 )
// general digital transfer function: z_num/z_den
{
  Allocate( r0.size(), z_den.size()-1 );

  // COPY COEFF VECTORS
  for(int i=0; i<z_num.size(); i++)
    (*b_)[i] = z_num[i];  // FOR order(num) <= order(den)
  *a_ = z_den; // COPY DENOMINATOR

  for(int i=1; i<order_+1; i++)
  { *r_[i] = r0; //Store initial value
    *f_[i] = f0; //Store initial value
  }
}


void Filter::LowPass(Eigen::VectorXd const &r0, float sampFreq, float cutOffFreq)
// w/(s+w)
{
  Allocate( r0.size(), 1 );
  float T = 1.0/sampFreq;

  float w = M_2_PI*cutOffFreq;
  w = (2/T)*tan(w*T/2);  // WARPING TO MATCH CUTOFF FREQ

  float p = (T*w/2)+1;
  float m = (T*w/2)-1;

  (*b_)[0] =  (T*w/2)/p;
  (*b_)[1] =  (*b_)[0];
  (*a_)[0] =  1;
  (*a_)[1] =  m/p;

  *r_[1] = r0; //Store initial value
  *f_[1] = r0; //Store initial value
}


void Filter::D_LowPass(Eigen::VectorXd const &r0, float sampFreq, float cutOffFreq)
// (s*w)/(s+w)
{
  Allocate( r0.size(), 1 );
  float T = 1.0/sampFreq;

  float w = M_2_PI*cutOffFreq;
  w = (2/T)*tan(w*T/2);  // WARPING TO MATCH CUTOFF FREQ

  (*b_)[0] =  2*w/(w*T+2);
  (*b_)[1] = -(*b_)[0];
  (*a_)[0] =  1;
  (*a_)[1] =  (w*T-2)/(w*T+2);

  *r_[1] = r0;       //Store initial value
   f_[1]->setZero(); //Store initial value
}
