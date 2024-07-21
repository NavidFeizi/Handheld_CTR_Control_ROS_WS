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
// #include <boost/qvm/all.hpp> // for quaternion and quaternion operations
#include <nlopt.hpp> // for registration transformation calculation
#include <blaze/Blaze.h>
#include <blaze/Math.h>
#include <blaze/math/DenseMatrix.h>

using blaze::StaticMatrix;
using blaze::StaticVector;

typedef struct
{
  unsigned int counts;
  std::vector<StaticVector<double, 3>> measured;
  std::vector<StaticVector<double, 3>> truth;
} objFuncData;

StaticVector<double, 4> quaternionMultiply(const StaticVector<double, 4> &q1, const StaticVector<double, 4> &q2);

StaticVector<double, 4> quaternionConjugate(const StaticVector<double, 4> &q); // or quaternion inverse

/* Rotates a 3D vector with RotationQuaternion */
void Quaternion_Rotate_Point(const StaticVector<double, 4> &rotationQuaternion,
                             const StaticVector<double, 3> &originalVector,
                             StaticVector<double, 3> &rotatedVector);

void calculateTransformationMatrix(const blaze::StaticVector<double, 4> &rotation,
                                   const blaze::StaticVector<double, 3> &translation,
                                   blaze::StaticMatrix<double, 4, 4> &transformationMatrix);

typedef struct QuatTransformationStruct
{
  blaze::StaticVector<double, 4> rotation;
  blaze::StaticVector<double, 3> translation;
  double error = 0.0;

  QuatTransformationStruct(const blaze::StaticVector<double, 4> &rot, const blaze::StaticVector<double, 3> &trans)
      : rotation(rot), translation(trans), error(0.0) {}

  // Default constructor with member initializers
  QuatTransformationStruct() : rotation{1.0, 0.0, 0.0, 0.0}, translation{0.0, 0.0, 0.0}, error(0.0) {}

  // Member function to get the inverse of the transformation
  QuatTransformationStruct inv() const
  {
    QuatTransformationStruct inverseTransformation;
    Inverse_Quat_Transformation(*this, inverseTransformation);
    return inverseTransformation;
  }

  // Static member function to inverse the transformation
  static void Inverse_Quat_Transformation(const QuatTransformationStruct &originalTransformation, QuatTransformationStruct &inverseTransformation)
  {
    // Inverse of the rotation quaternion
    inverseTransformation.rotation = quaternionConjugate(originalTransformation.rotation);
    // Rotate the translation using the inverse rotation
    Quaternion_Rotate_Point(inverseTransformation.rotation, originalTransformation.translation, inverseTransformation.translation);
    // Inverse of the translation vector (negate the translation vector components)
    inverseTransformation.translation = -inverseTransformation.translation;
  }

  // Member function to calculate the transformation matrix
  blaze::StaticMatrix<double, 4, 4> toMatrix() const
  {
    blaze::StaticMatrix<double, 4, 4> transformationMatrix;
    calculateTransformationMatrix(rotation, translation, transformationMatrix);
    return transformationMatrix;
  }
} quatTransformation;

/* Calculates inverse of a rigid transformation */
void Inverse_Quat_Transformation(const quatTransformation &originalTransformation,
                                 quatTransformation &inverseTransformation);

/* Function to combine rigid transformation from frame 0 to 1 with
    rigid transformation from frame 1 to 2
    to calculate transformation from frame 0 to 1 */
void Combine_Quat_Transformation(const quatTransformation &transformation01,
                                 const quatTransformation &transformation12,
                                 quatTransformation &transformation02);

/* Function to calculate the transformation that transfers the landmarks_measured frame
  to the landmarks_truth frame */
void Calculate_Transformation(const std::vector<StaticVector<double, 3>> &landmarks_measured,
                              const std::vector<StaticVector<double, 3>> &landmarks_truth,
                              quatTransformation &transformation);

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
void Rotmat2Quaternion(const StaticMatrix<double, 3, 3> &rotmat, StaticVector<double, 4> &rotqaut);

/* Function to convert Euler angles rotation to quaternion */
void Euler2Quaternion(const double heading, const double attitude, const double bank, StaticVector<double, 4UL> &h);

/* Function to convert Quaternion to Euler angles rotation */
void QuaternionToEuler(const StaticVector<double, 4> &quaternion, double *azimuth, double *elevation, double *roll);

/* Function to calculate Euclidean distance between two translation vectors */
void Euclidean_Distance(const StaticVector<double, 3> &a, const StaticVector<double, 3> &b, double &dist);

/* Functions below are not verifiest - must be tested before use */
StaticMatrix<double, 3UL, 3UL> RotX(const double &theta);
StaticMatrix<double, 3UL, 3UL> RotY(const double &theta);
StaticMatrix<double, 3UL, 3UL> RotZ(const double &theta);
void EulerToRotMat(const std::vector<double> &Angles, StaticMatrix<double, 3UL, 3UL> *R);
StaticMatrix<double, 3UL, 3UL> quaternion2rotmat(std::vector<double> q);
std::vector<double> QuaternionToEulerAngles(const std::vector<double> q);
