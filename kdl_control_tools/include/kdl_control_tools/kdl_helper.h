// Copyright (C) 2020  CRISP, Queen Mary University of London
// This code is licensed under the BSD 3-Clause license (see LICENSE for details)

#include <vector>
#include <deque>

#include <kdl/jntarray.hpp>

#include <eigen3/Eigen/Core>

using namespace std;
using namespace KDL;

namespace kdl_control_tools
{

  // convert std to KDL --------------------------------------------------
  void vectorStdToKdl(const vector<double>& data_in, JntArray& data_out);
  JntArray vectorStdToKdl(const vector<double>& data_in);
  void vectorStdToKdl(const vector<double>& data_in, KDL::Vector& data_out);

  // convert KDL to std --------------------------------------------------
  void vectorKdlToStd(const JntArray& data_in, vector<double>& data_out);
  vector<double> vectorKdlToStd(const JntArray& data_in);
  void vectorKdlToStd(const KDL::Vector& data_in, vector<double>& data_out);

  // convert KDL to Eigen --------------------------------------------------
  // this is left out since JntArray::data provides it
  // void arrayKdlToEigen(const JntArray& data_in, Eigen::VectorXd& data_out);

  // convert Eigen to KDL  --------------------------------------------------
  void arrayEigenToKdl(const Eigen::VectorXd& data_in, JntArray& data_out);

  // smoothing
  void smoothVector(KDL::Vector& vec_now, deque<KDL::Vector>& vec_past, int past_len);
  void smoothArray(KDL::JntArray& arr_now, deque<KDL::JntArray>& arr_past, int past_len);

}
