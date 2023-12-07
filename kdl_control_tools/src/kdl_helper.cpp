// Copyright (C) 2020  CRISP, Queen Mary University of London
// This code is licensed under the BSD 3-Clause license (see LICENSE for details)

#include "kdl_control_tools/kdl_helper.h"

using namespace std;
using namespace KDL;

namespace kdl_control_tools
{
  // convert std to KDL --------------------------------------------------
  void vectorStdToKdl(const vector<double>& data_in, JntArray& data_out){
    int size = data_in.size();
    data_out.resize(size);

    for (int i=0; i<size; i++)
      data_out(i) = data_in[i];
  }
  JntArray vectorStdToKdl(const vector<double>& data_in){
    JntArray data_out;
    vectorStdToKdl(data_in, data_out);
    return data_out;
  }
  void vectorStdToKdl(const vector<double>& data_in, Vector& data_out){
    int size = data_in.size();
    // KDL vectors are strictly 3 dimensional
    if(size!=3){
      return;
    }

    for (int i=0; i<size; i++)
      data_out(i) = data_in[i];
  }


  // convert KDL to std --------------------------------------------------
  void vectorKdlToStd(const JntArray& data_in, vector<double>& data_out){
    int size = data_in.rows();
    data_out.resize(size);

    for (int i=0; i<size; i++)
      data_out[i] = data_in(i);
  }
  vector<double> vectorKdlToStd(const JntArray& data_in){
    vector<double> data_out;
    vectorKdlToStd(data_in, data_out);
    return data_out;
  }
  void vectorKdlToStd(const Vector& data_in, vector<double>& data_out){
    // KDL vectors are strictly 3 dimensional
    int size = 3;

    data_out.resize(size);

    for (int i=0; i<size; i++)
      data_out[i] = data_in(i);
  }

  // convert Eigen to KDL  --------------------------------------------------
  void arrayEigenToKdl(const Eigen::VectorXd& data_in, JntArray& data_out){
    int size = data_in.size();

    data_out.resize(size);

    for (int i=0; i<size; i++)
      data_out(i) = data_in(i);

  }

  /*********************************************************************
  * Smooth the given vector with past vectors
  *********************************************************************/
  void smoothVector(KDL::Vector& vec_now, deque<KDL::Vector>& vec_past, int past_len){
    for(int vi=0; vi<vec_past.size(); vi++){
      vec_now = vec_now + vec_past[vi];
    }
    vec_now = vec_now / (double)(vec_past.size()+1);

    // remove extra history
    if(vec_past.size()>past_len){
      vec_past.pop_back();
    }

    // add the final vec
    vec_past.push_front(vec_now);

  }
  // takes JntArray
  void smoothArray(KDL::JntArray& arr_now, deque<KDL::JntArray>& arr_past, int past_len){
    for(int vi=0; vi<arr_past.size(); vi++){
      arr_now.data = arr_now.data + arr_past[vi].data;
    }
    arr_now.data = arr_now.data / (double)(arr_past.size()+1);

    // remove extra history
    if(arr_past.size()>past_len){
      arr_past.pop_back();
    }

    // add the final vec
    arr_past.push_front(arr_now);

  }

}
