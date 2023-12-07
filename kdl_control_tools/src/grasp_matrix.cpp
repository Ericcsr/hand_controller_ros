#include "kdl_control_tools/grasp_matrix.h"

using namespace kdl_control_tools;
/*********************************************************************
* Creates a grasp matrix for a given number of manipulators (m).
* Matrix size will be (6, 6*m).
*********************************************************************/
GraspMatrix::GraspMatrix(const vector<KDL::Vector>& relative_positions)
{
  m_ = relative_positions.size();
  createGraspMatrix(relative_positions);
  createInverseGraspMatrix(relative_positions);
}
/*********************************************************************
* updates the skew symmetrix matrix blocks in the grasp matrix
*********************************************************************/
void GraspMatrix::updateRelativePositions(const vector<KDL::Vector> relative_positions)
{
  for(int i=0; i<m_; i++){
    G_.block(3, 6*i, 3, 3) = -skewSymmetricMatrix(relative_positions[i]);
    G_inv_.block(3+(6*i), 0, 3, 3) = (1.0 / m_) * skewSymmetricMatrix(relative_positions[i]);
  }
}
/*********************************************************************
* S = 0, -z, y
*     z, 0, -x
*    -y, x, 0
*********************************************************************/
Eigen::Matrix3d GraspMatrix::skewSymmetricMatrix(KDL::Vector r)
{
  Eigen::Matrix3d s;
  s << 0.0, -r(2), r(1),
       r(2), 0.0, -r(0),
       -r(1), r(0), 0.0;
  return s;
}
/*********************************************************************
* W =  I3, O3
*    -S(r), I3
*********************************************************************/
Eigen::MatrixXd GraspMatrix::wrenchTransformMatrix(KDL::Vector r)
{
  Eigen::MatrixXd w(6,6);
  w << Eigen::MatrixXd::Identity(3,3), Eigen::MatrixXd::Zero(3,3),
      -skewSymmetricMatrix(r), Eigen::MatrixXd::Identity(3,3);
  return w;
}
/*********************************************************************
*  G = W0, W1, W2, ...
*********************************************************************/
void GraspMatrix::createGraspMatrix(const vector<KDL::Vector>& r)
{
  G_.resize(6, 6*m_);
  for(int i=0; i<m_; i++)
    G_.block(0, i*6, 6, 6) = wrenchTransformMatrix(r[i]);
}
/*********************************************************************
* G = W0-,
*     W1-,
*     W2-,
*     ...
*********************************************************************/
void GraspMatrix::createInverseGraspMatrix(const vector<KDL::Vector>& r)
{
  G_inv_.resize(6*m_, 6);
  for(int i=0; i<m_; i++)
    G_inv_.block(i*6, 0, 6, 6) = wrenchTransformMatrix(-r[i]);
  G_inv_ = (1.0 / m_) * G_inv_;
}
/*********************************************************************
* Forward transform
**********************************************************************/
KDL::Wrench GraspMatrix::transformToObjectFrame(const vector<KDL::Wrench>& contact_wrenches){
  // create the merged vector of contact wrenches
  Eigen::VectorXd h(6*m_);
  for(int i=0; i<m_; i++){
    Eigen::Matrix<double, 6, 1> h_tmp(6);
    tf::wrenchKDLToEigen(contact_wrenches[i], h_tmp);
    h.segment(6*i,6) = h_tmp;
  }
  // apply transform
  Eigen::Matrix<double, 6, 1> h_obj;
  h_obj = G_*h;
  // convert back to KDL
  KDL::Wrench object_wrench;
  tf::wrenchEigenToKDL(h_obj, object_wrench);
  return object_wrench;
}
/*********************************************************************
* Inverse transform
**********************************************************************/
vector<KDL::Wrench> GraspMatrix::transformToContactFrames(const KDL::Wrench& object_wrench){
  // create the merged vector of contact wrenches
  Eigen::Matrix<double, 6, 1> h_obj;
  tf::wrenchKDLToEigen(object_wrench, h_obj);
  // apply inverse transform
  Eigen::VectorXd h(6*m_);
  h = G_inv_*h_obj;
  // convert back to KDL
  vector<KDL::Wrench> contact_wrenches(m_);
  for(int i=0; i<m_; i++){
    tf::wrenchEigenToKDL(h.segment(6*i,6), contact_wrenches[i]);
  }
  return contact_wrenches;
}
/*********************************************************************
* getters
*********************************************************************/
Eigen::MatrixXd GraspMatrix::getGraspMatrix()
{
  return G_;
}
Eigen::MatrixXd GraspMatrix::getInverseGraspMatrix()
{
  return G_inv_;
}
Eigen::Matrix<double,6,6> GraspMatrix::getWrenchTransform(const int manipulator_index)
{
  return G_.block(0, manipulator_index*6, 6, 6);
}
Eigen::Matrix<double,6,6> GraspMatrix::getInverseWrenchTransform(const int manipulator_index)
{
  return G_inv_.block(manipulator_index*6, 0, 6, 6);
}
