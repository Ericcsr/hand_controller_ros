// Copyright (C) 2020  CRISP, Queen Mary University of London
// This code is licensed under the BSD 3-Clause license (see LICENSE for details)
/**
  * This file defines the tools to create, modify, invert a grasp matrix.
 **/
#ifndef GRASP_MATRIX_H
#define GRASP_MATRIX_H

#include <eigen3/Eigen/Dense>

#include <eigen_conversions/eigen_kdl.h>
#include <eigen_conversions/eigen_msg.h>

using namespace std;

namespace kdl_control_tools
{

/**
* Creates a grasp matrix for a given number of manipulators (m).
* Matrix size will be (6, 6*m).
**/
class GraspMatrix
{
  protected:

    int m_; // manipulator count

    Eigen::MatrixXd G_; // grasp matrix
    Eigen::MatrixXd G_inv_; // inverse grasp matrix

    Eigen::Matrix3d skewSymmetricMatrix(KDL::Vector r);
    Eigen::MatrixXd wrenchTransformMatrix(KDL::Vector r);

    void createGraspMatrix(const vector<KDL::Vector>& r);
    void createInverseGraspMatrix(const vector<KDL::Vector>& r);

  public:
    GraspMatrix(const vector<KDL::Vector>& relative_positions);
    ~GraspMatrix(){};

    void updateRelativePositions(const vector<KDL::Vector> relative_positions);

    KDL::Wrench transformToObjectFrame(const vector<KDL::Wrench>& contact_wrenches);
    vector<KDL::Wrench> transformToContactFrames(const KDL::Wrench& object_wrench);

    Eigen::MatrixXd getGraspMatrix();
    Eigen::MatrixXd getInverseGraspMatrix();
    Eigen::Matrix<double,6,6> getWrenchTransform(const int manipulator_index);
    Eigen::Matrix<double,6,6> getInverseWrenchTransform(const int manipulator_index);
};

} // end namespace kdl_control_tools

#endif // GRASP_MATRIX_H
