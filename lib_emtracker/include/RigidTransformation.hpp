//==========================================================================================================
// 			This code is a part of the concentric tube robot & robot actuated catheter project
//  		   		Canadian Surgical Technologies and Advanced Robotics (CSTAR).
// 							Copyright (C) 2022 Navid Feizi <nfeizi@uwo.ca>
//					     Copyright (C) 2022 Filipe C. Pedrosa <fpedrosa@uwo.ca>
//
//       Project developed under the supervision of Dr Jayender Jagadeesan (❖) Dr Rajni Patel (◈)
//  ========================================================================================================
//   (◈) CSTAR (Canadian Surgical Technologies & Advanced Robotics) @ Western University, London, ON  Canada
//             (❖) Surgical Planning Lab @ Brigham and Women's Hospital / Harvard Medical School
//==========================================================================================================

// The code below provides some quaternion based functions for landmark registrations and 
// regid transformations

#pragma once

#include "RigidTransformation.hpp"
#include <iostream>
#include <string>
#include <cmath>
#include <boost/qvm/all.hpp> // for quaternion and quaternion operations
#include <nlopt.hpp>         // for registration transformation calculation
#include <blaze/Blaze.h>
#include <blaze/Math.h>
#include <blaze/math/DenseMatrix.h>

typedef struct QuatTransformationStruct
{
  boost::qvm::quat<double> rotation = boost::qvm::identity_quat<double>();
  boost::qvm::vec<double, 3> translation = boost::qvm::vec<double, 3>({0.0, 0.0, 0.0});
  double error = 0.0;
  QuatTransformationStruct(const std::vector<double> &rot, const std::vector<double> &trans)
      : rotation({rot[0], rot[1], rot[2], rot[3]}), translation({trans[0], trans[1], trans[2]})
  {
  }
  // Default constructor with member initializers
  QuatTransformationStruct() = default;
} quatTransformation;

typedef struct
{
  unsigned int counts;
  std::vector<std::vector<double>> measured;
  std::vector<std::vector<double>> truth;
} objFuncData;

/* Rotates a 3D vector with RotationQuaternion */
void Quaternion_Rotate_Point(const boost::qvm::quat<double> &RotationQuaternion_Ptr,
                             const boost::qvm::vec<double, 3> &OriginalPosition_Ptr,
                             boost::qvm::vec<double, 3> *RotatedPosition_Ptr);

/* Calculates inverse of a rigid transformation */
void Inverse_Quat_Transformation(const quatTransformation &pdtXfrm,
                                 quatTransformation *pdtNewXfrm);

/* Function to combine rigid transformation from frame 0 to 1 with
    rigid transformation from frame 1 to 2
    to calculate transformation from frame 0 to 1 */
void Combine_Quat_Transformation(const quatTransformation &pdtXfrm01,
                                 const quatTransformation &pdtXfrm12,
                                 quatTransformation *pdtXfrm02);

/* Function to calculate the transformation that transfers the landmarks_measured frame
  to the landmarks_truth frame */
void Calculate_Transformation(const std::vector<std::vector<double>> &landmarks_measured,
                              const std::vector<std::vector<double>> &landmarks_truth,
                              quatTransformation *transformation);

/* Cost function of the Calculate_Transformation optimization problem
  * @param x       A vector of parameters representing the transformation model.
  * @param grad    A vector of gradients (output).
  * @param f_data  Additional data needed for the cost calculation.*/
double Objective_Function(const std::vector<double> &x, std::vector<double> &grad, void *f_data);

/* Constraint function to enforce unit length of the calcualted quaternion */
double Equality_Constraint(const std::vector<double> &x, std::vector<double> &grad, void *f_data);

/* Function to convert nlopt::result to string */
std::string nloptResult2String(nlopt::result result);

/* Function to convert rotation matrix to the quaternion */
void Rotmat2Quaternion(const blaze::DynamicMatrix<double> rotmat, boost::qvm::quat<double> &rotqaut);

/* Function to convert Euler angles rotation to quaternion */
void Euler2Quaternion(const double heading, const double attitude, const double bank, blaze::StaticVector<double, 4UL> &h);

/* Function to convert Quaternion to Euler angles rotation */
void QuaternionToEuler(const boost::qvm::quat<double> &quaternion, double *azimuth, double *elevation, double *roll);

/* Function to calculate Euclidean distance between two translation vectors */
void Euclidean_Distance(const boost::qvm::vec<double, 3> &translation, const boost::qvm::vec<double, 3> &translation2, double *distance);




/* Functions below are not verifiest - must be tested before use */
blaze::StaticMatrix<double, 3UL, 3UL> RotX(const double &theta);
blaze::StaticMatrix<double, 3UL, 3UL> RotY(const double &theta);
blaze::StaticMatrix<double, 3UL, 3UL> RotZ(const double &theta);
void EulerToRotMat(const std::vector<double> &Angles, blaze::StaticMatrix<double, 3UL, 3UL> *R);
blaze::StaticMatrix<double, 3UL, 3UL> quaternion2rotmat(std::vector<double> q);
std::vector<double> QuaternionToEulerAngles(const std::vector<double> q);

