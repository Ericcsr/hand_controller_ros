/**
* allegro_hand_kdl is a ROS package to control Allegro Hand using the KDL library.
* Copyright (C) 2020  Gokhan Solak, CRISP, Queen Mary University of London
*
* This library is free software; you can redistribute it and/or
* modify it under the terms of the GNU Lesser General Public
* License as published by the Free Software Foundation; either
* version 2.1 of the License, or (at your option) any later version.
*
* This library is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
* Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public
* License along with this library; if not, write to the Free Software
* Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301
* USA
*/

#include "allegro_hand_kdl/cartesian_force.h"

namespace allegro_hand_kdl
{
/*********************************************************************
* Creates solvers for each finger
*********************************************************************/
CartesianForce::CartesianForce(AllegroKdlConfig& kdl_config)
{
  // Initialize each finger's reference joint angles manually
  q_ref_vec_.clear();
  float pi = 3.14159265359;
  Eigen::VectorXd finger_ref(4);
  finger_ref << 0.0, pi/6, pi/6, pi/6;
  q_ref_vec_.push_back(finger_ref);
  q_ref_vec_.push_back(finger_ref);
  q_ref_vec_.push_back(finger_ref);
  Eigen::VectorXd thumb_ref(4);
  thumb_ref << pi/3, 0.0, pi/6, pi/6;
  q_ref_vec_.push_back(thumb_ref);
  if (!kdl_config.isReady())
  {
    ROS_ERROR("CartesianForce: null kdl config");
    ros::shutdown();
    return;
  }

  createFingerSolvers_(kdl_config);
}
CartesianForce::~CartesianForce()
{
  for (ChainJntToJacSolver* obj : finger_solver_vec_)
  {
    delete obj;
  }
  finger_solver_vec_.clear();
}

/*********************************************************************
* Creates solvers for each finger
*********************************************************************/
void CartesianForce::createFingerSolvers_(AllegroKdlConfig& kdl_config)
{
  // Create inverse dynamics solvers for each finger
  for (int fi=0; fi < FINGER_COUNT; fi++)
  {
    // Extract the finger chain from whole hand tree
    Chain *finger_chain = kdl_config.getFingerChain(fi);

    // Solver requires the chain and the gravity info
    ChainJntToJacSolver* solver = new ChainJntToJacSolver(*finger_chain);

    // Fill vectors
    finger_solver_vec_.push_back(solver);
  }
}

/*********************************************************************
* Calculates the joint torques which achieve the desired cartesian forces.
*********************************************************************/
KDL::JntArray CartesianForce::computeTorquesNullSpace(const KDL::JntArray& q, const vector<KDL::JntArray>& w_vec)
{
  // make sure that vector sizes are correct
  unsigned int jnt_count = FINGER_COUNT * FINGER_LENGTH;
  if (w_vec.size() != FINGER_COUNT || w_vec[0].rows() != 6 || q.rows() != jnt_count)
  {
    ROS_ERROR("CartesianForce: input lengths are wrong.");
    return KDL::JntArray();
  }

  // empty torque vector
  KDL::JntArray t_joint;

  // Repeat for each finger
  for (int fi=0; fi < FINGER_COUNT; fi++)
  {
    // Create KDL data types
    KDL::Jacobian jac(FINGER_LENGTH);

    // Slice the finger joint angles
    KDL::JntArray q_fi;
    q_fi.data = q.data.segment(fi*FINGER_LENGTH, FINGER_LENGTH);

    // Get the jacobian for desired configuration
    int error = finger_solver_vec_[fi]->JntToJac(q_fi, jac);
    if (error == -100)
      ROS_ERROR("Cartesian force: failed to derive Jacobian, finger %d.", fi);

    // Calculate the torque using jacobian
    Eigen::MatrixXd jac_pos = jac.data.block(0,0,3,FINGER_LENGTH);
    Eigen::VectorXd t_finger = jac_pos.transpose() * w_vec[fi].data.segment(0,3);
    // Need to solve pesudo inverse to get the null space torques
    Eigen::MatrixXd jac_pinv = jac_pos.completeOrthogonalDecomposition().pseudoInverse();
    //ROS_INFO("Debug jacobian pinv: %d, %d", jac_pinv.rows(), jac_pinv.cols());
    //ROS_INFO("Debug jacobian pos: %d, %d", jac_pos.rows(), jac_pos.cols());
    Eigen::VectorXd t_finger_null = (Eigen::MatrixXd::Identity(FINGER_LENGTH, FINGER_LENGTH) - jac_pos.transpose() * jac_pinv.transpose()) *(q_ref_vec_[fi] - q_fi.data) * 0.5;
    t_finger = t_finger + t_finger_null;
    // ROS_INFO("Debug jacobian: %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f",
    //   jac.data(0,0), jac.data(0,1), jac.data(0,2), jac.data(1,0), jac.data(1,1), jac.data(1,2),
    //   jac.data(2,0), jac.data(2,1), jac.data(2,2), jac.data(3,0), jac.data(3,1), jac.data(3,2));

    // append
    Eigen::VectorXd t_tmp(t_joint.data.rows()+ t_finger.rows());
    t_tmp << t_joint.data , t_finger;
    t_joint.data = t_tmp;
  }

  return t_joint;
}

KDL::JntArray CartesianForce::computeTorquesNullSpace(const KDL::JntArray& q, const vector<KDL::Wrench>& f_cart)
{
  vector<JntArray> u_cart(FINGER_COUNT);
  for (int fi=0; fi < FINGER_COUNT; fi++)
  {
    // convert to JntArray
    u_cart[fi].resize(6);
    for (int i = 0; i < 6; ++i)
      u_cart[fi].data[i] = f_cart[fi][i];
  }
  // call the core method
  return computeTorquesNullSpace(q, u_cart);
}

vector<double> CartesianForce::computeTorquesNullSpace(const vector<double>& q, const vector< vector<double> >& f_cart)
{
  // Create KDL data types
    // inputs
  KDL::JntArray q_kdl;
  vector<KDL::JntArray> f_kdl(FINGER_COUNT);
    // output
  KDL::JntArray t_kdl;

  // convert input to kdl
  kdl_control_tools::vectorStdToKdl(q, q_kdl);
  for (int fi=0; fi < FINGER_COUNT; fi++)
    kdl_control_tools::vectorStdToKdl(f_cart[fi], f_kdl[fi]);

  // call the core method
  t_kdl = computeTorquesNullSpace(q_kdl, f_kdl);

  // convert the output to desired type
  vector<double> t_std;
  kdl_control_tools::vectorKdlToStd(t_kdl, t_std);

  return t_std;
}

KDL::JntArray CartesianForce::computeTorques(const KDL::JntArray& q, const vector<KDL::JntArray>& w_vec)
{
  // make sure that vector sizes are correct
  unsigned int jnt_count = FINGER_COUNT * FINGER_LENGTH;
  if (w_vec.size() != FINGER_COUNT || w_vec[0].rows() != 6 || q.rows() != jnt_count)
  {
    ROS_ERROR("CartesianForce: input lengths are wrong.");
    return KDL::JntArray();
  }

  // empty torque vector
  KDL::JntArray t_joint;

  // Repeat for each finger
  for (int fi=0; fi < FINGER_COUNT; fi++)
  {
    // Create KDL data types
    KDL::Jacobian jac(FINGER_LENGTH);

    // Slice the finger joint angles
    KDL::JntArray q_fi;
    q_fi.data = q.data.segment(fi*FINGER_LENGTH, FINGER_LENGTH);

    // Get the jacobian for desired configuration
    int error = finger_solver_vec_[fi]->JntToJac(q_fi, jac);
    if (error == -100)
      ROS_ERROR("Cartesian force: failed to derive Jacobian, finger %d.", fi);

    // Calculate the torque using jacobian
    Eigen::VectorXd t_finger = jac.data.transpose() * w_vec[fi].data;
    // ROS_INFO("Debug jacobian: %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f",
    //   jac.data(0,0), jac.data(0,1), jac.data(0,2), jac.data(1,0), jac.data(1,1), jac.data(1,2),
    //   jac.data(2,0), jac.data(2,1), jac.data(2,2), jac.data(3,0), jac.data(3,1), jac.data(3,2));

    // append
    Eigen::VectorXd t_tmp(t_joint.data.rows()+ t_finger.rows());
    t_tmp << t_joint.data , t_finger;
    t_joint.data = t_tmp;
  }

  return t_joint;
}
// wrapper using semantically correct dtype: Wrench
KDL::JntArray CartesianForce::computeTorques(const KDL::JntArray& q, const vector<KDL::Wrench>& f_cart)
{
  vector<JntArray> u_cart(FINGER_COUNT);
  for (int fi=0; fi < FINGER_COUNT; fi++)
  {
    // convert to JntArray
    u_cart[fi].resize(6);
    for (int i = 0; i < 6; ++i)
      u_cart[fi].data[i] = f_cart[fi][i];
  }
  // call the core method
  return computeTorques(q, u_cart);
}
// wrapper for std and ros types
// f_cart shall have size 4x6 (fingers count:4, wrench: 6)
vector<double> CartesianForce::computeTorques(const vector<double>& q, const vector< vector<double> >& f_cart)
{
  // Create KDL data types
    // inputs
  KDL::JntArray q_kdl;
  vector<KDL::JntArray> f_kdl(FINGER_COUNT);
    // output
  KDL::JntArray t_kdl;

  // convert input to kdl
  kdl_control_tools::vectorStdToKdl(q, q_kdl);
  for (int fi=0; fi < FINGER_COUNT; fi++)
    kdl_control_tools::vectorStdToKdl(f_cart[fi], f_kdl[fi]);

  // call the core method
  t_kdl = computeTorques(q_kdl, f_kdl);

  // convert the output to desired type
  vector<double> t_std;
  kdl_control_tools::vectorKdlToStd(t_kdl, t_std);

  return t_std;
}

/*********************************************************************
* Calculates cartesian wrenches, resulting from the given joint torques (tau).
*********************************************************************/
vector<KDL::Wrench> CartesianForce::computeWrenches(const KDL::JntArray& q, const KDL::JntArray& tau)
{
  // make sure that vector sizes are correct
  unsigned int jnt_count = FINGER_COUNT * FINGER_LENGTH;
  if (q.rows() != jnt_count || tau.rows() != jnt_count)
  {
    ROS_ERROR("CartesianForce: input lengths are wrong.");
    return vector<KDL::Wrench>();
  }

  // empty wrench vector to return
  vector<KDL::Wrench> w_vec(FINGER_COUNT);

  // Repeat for each finger
  for (int fi=0; fi < FINGER_COUNT; fi++)
  {
    // Create KDL data types
    KDL::Jacobian jac(FINGER_LENGTH);

    // Slice the finger joint angles & torques
    KDL::JntArray q_fi, tau_fi;
    q_fi.data = q.data.segment(fi*FINGER_LENGTH, FINGER_LENGTH);
    tau_fi.data = tau.data.segment(fi*FINGER_COUNT, FINGER_LENGTH);

    // Get the jacobian for desired configuration
    int error = finger_solver_vec_[fi]->JntToJac(q_fi, jac);
    if (error == -100)
      ROS_ERROR("Cartesian force: failed to derive Jacobian, finger %d.", fi);

    // Calculate the torque using jacobian
    Eigen::VectorXd w_finger = jac.data * tau.data;

    // append each dimension
    for (int di=0; di < 3; di++)
      w_vec[fi].force.data[di] = w_finger(di);
    for (int di=3; di < 6; di++)
      w_vec[fi].torque.data[di] = w_finger(di);
  }
  return w_vec;
}


}  // end of namespace allegro_hand_kdl
