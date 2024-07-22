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

#include "RigidTransformation.hpp"

// Function to multiply two quaternions
StaticVector<double, 4UL> quaternionMultiply(const StaticVector<double, 4> &q1, const StaticVector<double, 4> &q2)
{
  return {
      q1[0UL] * q2[0UL] - q1[1UL] * q2[1UL] - q1[2UL] * q2[2UL] - q1[3UL] * q2[3UL],
      q1[0UL] * q2[1UL] + q1[1UL] * q2[0UL] + q1[2UL] * q2[3UL] - q1[3UL] * q2[2UL],
      q1[0UL] * q2[2UL] - q1[1UL] * q2[3UL] + q1[2UL] * q2[0UL] + q1[3UL] * q2[1UL],
      q1[0UL] * q2[3UL] + q1[1UL] * q2[2UL] - q1[2UL] * q2[1UL] + q1[3UL] * q2[0UL]};
}

// Function to conjugate a quaternion
StaticVector<double, 4UL> quaternionConjugate(const StaticVector<double, 4UL> &q)
{
  return {q[0UL], -q[1UL], -q[2UL], -q[3UL]};
}

/* Rotates a 3D vector with RotationQuaternion */
void Quaternion_Rotate_Point(const StaticVector<double, 4UL> &rotationQuaternion,
                             const StaticVector<double, 3UL> &originalVector,
                             StaticVector<double, 3UL> &rotatedVector)
{
  StaticVector<double, 4UL> V{0.00, originalVector[0UL], originalVector[1UL], originalVector[2UL]};
  StaticVector<double, 4UL> q_bar = quaternionConjugate(rotationQuaternion);
  StaticVector<double, 4UL> qV = quaternionMultiply(rotationQuaternion, V);
  StaticVector<double, 4UL> qVq_bar = quaternionMultiply(qV, q_bar);

  rotatedVector = {qVq_bar[1UL], qVq_bar[2UL], qVq_bar[3UL]};
}

/* Calculates inverse of a rigid transformation */
void Inverse_Quat_Transformation(const quatTransformation &originalTransformation,
                                 quatTransformation &inverseTransformation)
{
  // Inverse of the rotation quaternion
  inverseTransformation.rotation = quaternionConjugate(originalTransformation.rotation);

  // Rotate the translation using the inverse rotation
  Quaternion_Rotate_Point(inverseTransformation.rotation, originalTransformation.translation, inverseTransformation.translation);

  // Inverse of the translation vector (negate the translation vector components)
  inverseTransformation.translation = -inverseTransformation.translation;
}

/* Function to combine rigid transformation from frame 0 to 1 with
    rigid transformation from frame 1 to 2
    to calculate transformation from frame 0 to 1 */
void Combine_Quat_Transformation(const quatTransformation &transformation01,
                                 const quatTransformation &transformation12,
                                 quatTransformation &transformation02)
{
  // combine tranlation
  StaticVector<double, 3UL> translation_12_0; // translation from frame 1 to frame 2 in frame 0 view point
  Quaternion_Rotate_Point(transformation01.rotation, transformation12.translation, translation_12_0);
  transformation02.translation = transformation01.translation + translation_12_0; // translation from frame 0 to frame 2 in frame 0 view point

  // combine rotation
  transformation02.rotation = quaternionMultiply(transformation01.rotation, transformation12.rotation);
}

/* Function to calculate the transformation that transfers the landmarks_measured frame
    to the landmarks_truth frame */
void Calculate_Transformation(const std::vector<StaticVector<double, 3UL>> &landmarks_measured,
                              const std::vector<StaticVector<double, 3UL>> &landmarks_truth,
                              quatTransformation &transformation)
{
  // Create an instance of the NLOPT optimizer
  nlopt::opt optimizer(nlopt::LN_COBYLA, 7UL); // 7 parameters (4 for rotation, 3 for translation)
  nlopt::result result;
  objFuncData landmarks;
  landmarks.measured = landmarks_measured;
  landmarks.truth = landmarks_truth;

  // Initialize initial guess
  std::vector<double> x(7UL, 0.00);

  // Set bounds for the optimization parameters (if needed)
  const std::vector<double> lb = {-500.00, -500.00, -500.00, -0.9999, -1.00, -1.00, -1.00};
  const std::vector<double> ub = {500.00, 500.00, 500.00, 0.9999, 1.00, 1.00, 1.00};
  optimizer.set_lower_bounds(lb);
  optimizer.set_upper_bounds(ub);

  // Set the objective function and equality constraint
  optimizer.set_min_objective(Objective_Function, &landmarks);
  optimizer.add_equality_constraint(Equality_Constraint, nullptr, 1.00E-8);

  // Specify stopping criteria (optional)
  optimizer.set_maxeval(200000);    // Maximum number of function evaluations
  optimizer.set_ftol_rel(1.00E-12); // Relative tolerance for convergence
  optimizer.set_xtol_rel(1.00E-8);

  // Create a random number generator
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<double> dist;
  double error, error_min = 1000.0;

  for (size_t i = 0; i < 50; i++)
  {
    // Generate random initial guesses within the bounds
    for (size_t j = 0; j < x.size(); ++j)
    {
      x[j] = lb[j] + (ub[j] - lb[j]) * dist(gen);
    }

    // Perform the optimization
    result = optimizer.optimize(x, error);

    // Track the minimum error and the corresponding x
    if (error < error_min)
    {
      transformation.translation = {x[0UL], x[1UL], x[2UL]};
      transformation.rotation = {x[3UL], x[4UL], x[5UL], x[6UL]};
      transformation.error = error;
      error_min = error;
    }

    std::cout << "Square Error [mm]: " << error << "    "
              << "Num itter: " << optimizer.get_numevals() << "    "
              << "Stop: " << nloptResult2String(result) << std::endl;
  }

  std::cout << "Optimization Finished" << std::endl;
  std::cout << "Calculated Transformation => "
            << "X: " << transformation.translation[0UL] << "[mm]  "
            << "Y: " << transformation.translation[1UL] << "[mm]  "
            << "Z: " << transformation.translation[2UL] << "[mm]  |  "
            << "q0: " << transformation.rotation[0UL] << "   "
            << "qX: " << transformation.rotation[1UL] << "   "
            << "qY: " << transformation.rotation[2UL] << "   "
            << "qZ: " << transformation.rotation[3UL] << "  |  "
            << "RMSE: " << sqrt(transformation.error) << "[mm]  " << std::endl;
}

/** Cost function of the Calculate_Transformation optimization problem
 * @param x       A vector of parameters representing the transformation model.
 * @param grad    A vector of gradients (output).
 * @param f_data  Additional data needed for the cost calculation.*/
double Objective_Function(const std::vector<double> &x, std::vector<double> &grad, void *f_data)
{
  objFuncData *landmarks = reinterpret_cast<objFuncData *>(f_data);
  quatTransformation rigidTran, rigidTran_inv;                  // rigid transformatino from EM reference frame to CTR frame
  quatTransformation tran_measured, tran_estimated, tran_truth; // translation of the landmarks in the CTR frame
  rigidTran.translation = {x[0UL], x[1UL], x[2UL]};
  rigidTran.rotation = {x[3UL], x[4UL], x[5UL], x[6UL]};
  double dist, error = 0.00;

  for (size_t i = 0; i < landmarks->measured.size(); i++)
  {
    tran_measured.translation = {landmarks->measured[i][0UL], landmarks->measured[i][1UL], landmarks->measured[i][2UL]};
    tran_truth.translation = {landmarks->truth[i][0UL], landmarks->truth[i][1UL], landmarks->truth[i][2UL]};
    Inverse_Quat_Transformation(rigidTran, rigidTran_inv);
    Combine_Quat_Transformation(rigidTran_inv, tran_measured, tran_estimated);
    Euclidean_Distance(tran_estimated.translation, tran_truth.translation, dist);
    error += dist * dist;
  }
  return error / landmarks->measured.size();
}

/* Constraint function to enforce unit length of the calculated quaternion */
double Equality_Constraint(const std::vector<double> &x, std::vector<double> &grad, void *f_data)
{
  // This function enforces x[3]^2 + x[4]^2 + x[5]^2 - 1 = 0
  return x[3UL] * x[3UL] + x[4UL] * x[4UL] + x[5UL] * x[5UL] + x[6UL] * x[6UL] - 1.00;
}

/* Function to convert nlopt::result to string */
std::string nloptResult2String(nlopt::result result)
{
  switch (result)
  {
  case nlopt::SUCCESS:
    return "SUCCESS";
  case nlopt::STOPVAL_REACHED:
    return "STOPVAL_REACHED";
  case nlopt::FTOL_REACHED:
    return "FTOL_REACHED";
  case nlopt::XTOL_REACHED:
    return "XTOL_REACHED";
  case nlopt::MAXEVAL_REACHED:
    return "MAXEVAL_REACHED";
  case nlopt::MAXTIME_REACHED:
    return "MAXTIME_REACHED";
  case nlopt::FAILURE:
    return "FAILURE";
  case nlopt::INVALID_ARGS:
    return "INVALID_ARGS";
  case nlopt::OUT_OF_MEMORY:
    return "OUT_OF_MEMORY";
  case nlopt::ROUNDOFF_LIMITED:
    return "ROUNDOFF_LIMITED";
  default:
    return "UNKNOWN_RESULT";
  }
}

// Function to calculate the transformation matrix from quaternion and translation
void calculateTransformationMatrix(const blaze::StaticVector<double, 4UL> &rotation,
                                   const blaze::StaticVector<double, 3UL> &translation,
                                   blaze::StaticMatrix<double, 4UL, 4UL> &transformationMatrix)
{
  // Extract quaternion components
  const double q0 = rotation[0UL];
  const double q1 = rotation[1UL];
  const double q2 = rotation[2UL];
  const double q3 = rotation[3UL];

  // Compute rotation matrix components
  transformationMatrix(0UL, 0UL) = 1.00 - 2.00 * (q2 * q2 + q3 * q3);
  transformationMatrix(0UL, 1UL) = 2.00 * (q1 * q2 - q0 * q3);
  transformationMatrix(0UL, 2UL) = 2.00 * (q1 * q3 + q0 * q2);
  transformationMatrix(0UL, 3UL) = translation[0UL];

  transformationMatrix(1UL, 0UL) = 2.00 * (q1 * q2 + q0 * q3);
  transformationMatrix(1UL, 1UL) = 1.00 - 2.00 * (q1 * q1 + q3 * q3);
  transformationMatrix(1UL, 2UL) = 2.00 * (q2 * q3 - q0 * q1);
  transformationMatrix(1UL, 3UL) = translation[1UL];

  transformationMatrix(2UL, 0UL) = 2.00 * (q1 * q3 - q0 * q2);
  transformationMatrix(2UL, 1UL) = 2.00 * (q2 * q3 + q0 * q1);
  transformationMatrix(2UL, 2UL) = 1.00 - 2.00 * (q1 * q1 + q2 * q2);
  transformationMatrix(2UL, 3UL) = translation[2UL];

  transformationMatrix(3UL, 0UL) = 0.00;
  transformationMatrix(3UL, 1UL) = 0.00;
  transformationMatrix(3UL, 2UL) = 0.00;
  transformationMatrix(3UL, 3UL) = 1.00;
  return;
}

/* Function to convert rotation matrix to the quaternion */
void Rotmat2Quaternion(const StaticMatrix<double, 3UL, 3UL> &R, StaticVector<double, 4UL> &quat)
{
  double n4;                         // the norm of quaternion multiplied by 4
  const double tr = blaze::trace(R); // trace of matrix

  if (tr > 0.00)
  {
    quat[0UL] = tr + 1.00;
    quat[1UL] = R(1UL, 2UL) - R(2UL, 1UL);
    quat[2UL] = R(2UL, 0UL) - R(0UL, 2UL);
    quat[3UL] = R(0UL, 1UL) - R(1UL, 0UL);
    n4 = quat[0UL];
  }
  else
  {
    size_t i = 0UL;
    if (R(1UL, 1UL) > R(0UL, 0UL))
      i = 1UL;
    if (R(2UL, 2UL) > R(i, i))
      i = 2UL;

    switch (i)
    {
    case 0UL:
      quat[0UL] = R(1UL, 2UL) - R(2UL, 1UL);
      quat[1UL] = 1.00 + R(0UL, 0UL) - R(1UL, 1UL) - R(2UL, 2UL);
      quat[2UL] = R(1UL, 0UL) + R(0UL, 1UL);
      quat[3UL] = R(2UL, 0UL) + R(0UL, 2UL);
      n4 = quat[1UL];
      break;
    case 1UL:
      quat[0UL] = R(2UL, 0UL) - R(0UL, 2UL);
      quat[1UL] = R(1UL, 0UL) + R(0UL, 1UL);
      quat[2UL] = 1.00 + R(1UL, 1UL) - R(0UL, 0UL) - R(2UL, 2UL);
      quat[3UL] = R(2UL, 1UL) + R(1UL, 2UL);
      n4 = quat[2UL];
      break;
    case 2UL:
      quat[0UL] = R(0UL, 1UL) - R(1UL, 0UL);
      quat[1UL] = R(2UL, 0UL) + R(0UL, 2UL);
      quat[2UL] = R(2UL, 1UL) + R(1UL, 2UL);
      quat[3UL] = 1.00 + R(2UL, 2UL) - R(0UL, 0UL) - R(1UL, 1UL);
      n4 = quat[3UL];
      break;
    }
  }

  quat *= 1.00 / (2.00 * std::sqrt(n4));
}

/* Function to convert Euler angles rotation to quaternion */
void Euler2Quaternion(const double heading, const double attitude, const double bank, StaticVector<double, 4UL> &h)
{
  // yaw, pitch, roll
  const double theta = 0.50 * heading;
  const double phi = 0.50 * attitude;
  const double psi = 0.50 * bank;

  const double c1 = cos(theta), s1 = sin(theta);
  const double c2 = cos(phi), s2 = sin(phi);
  const double c3 = cos(psi), s3 = sin(psi);
  const double c1c2 = c1 * c2;
  const double s1s2 = s1 * s2;

  h[0UL] = c1c2 * c3 - s1s2 * s3;
  h[1UL] = c1c2 * s3 + s1s2 * c3;
  h[2UL] = s1 * c2 * c3 + c1 * s2 * s3;
  h[3UL] = c1 * s2 * c3 - s1 * c2 * s3;
}

/* Function to convert Quaternion to Euler angles rotation */
void QuaternionToEuler(const StaticVector<double, 4UL> &quaternion, double *azimuth, double *elevation, double *roll)
{
  // Extract quaternion components
  const double q0 = quaternion[0UL];
  const double q1 = quaternion[1UL];
  const double q2 = quaternion[2UL];
  const double q3 = quaternion[3UL];

  // Calculate azimuth (yaw)
  *azimuth = atan2(2.00 * (q2 * q3 + q0 * q1), 1.00 - 2.00 * (q1 * q1 + q2 * q2));

  // Calculate elevation (pitch)
  double sinElevation = 2.00 * (q0 * q2 - q1 * q3);
  if (std::abs(sinElevation) >= 1.00)
  {
    // Handle numerical instability near poles
    *elevation = sinElevation < 0.00 ? - M_PI_2 : M_PI_2;
  }
  else
  {
    *elevation = asin(sinElevation);
  }

  // Calculate roll (bank)
  *roll = atan2(2.00 * (q0 * q3 + q1 * q2), 1.00 - 2.00 * (q2 * q2 + q3 * q3));
}

/* Function to calculate Euclidean distance between two translation vectors */
void Euclidean_Distance(const blaze::StaticVector<double, 3UL> &a, const blaze::StaticVector<double, 3UL> &b, double &dist)
{
  dist = blaze::norm(a - b);
}

/* Functions below are not used and not verified - must be tested before use */
// function that return rotation matrix of a set of quaternions
// function that returns the rotation matrix Rx of any angle theta
StaticMatrix<double, 3UL, 3UL> RotX(const double &theta)
{
  const double c(cos(theta)), s(sin(theta));
  StaticMatrix<double, 3UL, 3UL> R = {{1, 0, 0},
       {0, c, -s},
       {0, s, c}};
  return R;
}
StaticMatrix<double, 3UL, 3UL> RotY(const double &theta)
{
  const double c(cos(theta)), s(sin(theta));
  StaticMatrix<double, 3UL, 3UL> R = {{c, 0, s},
       {0, 1, 0},
       {-s, 0, c}};

  return R;
}
StaticMatrix<double, 3UL, 3UL> RotZ(const double &theta)
{
  const double c(cos(theta)), s(sin(theta));
  StaticMatrix<double, 3UL, 3UL> R = {{c, -s, 0},
       {s, c, 0},
       {0, 0, 1}};

  return R;
}
void EulerToRotMat(const std::vector<double> &Angles, StaticMatrix<double, 3UL, 3UL> &R)
{
  R = RotZ(Angles[0UL]) * RotY(Angles[1UL]) * RotX(Angles[2UL]);
}

StaticMatrix<double, 3UL, 3UL> quaternion2rotmat(std::vector<double>& q)
{
  StaticMatrix<double, 3UL, 3UL> R = {{2.00 * (q[0UL] * q[0UL] + q[1UL] * q[1UL]) - 1.00, 2.00 * (q[1UL] * q[2UL] - q[0UL] * q[3UL]), 2.00 * (q[1UL] * q[3UL] + q[0UL] * q[2.00])},
       {2.00 * (q[1UL] * q[2UL] + q[0UL] * q[3UL]), 2.00 * (q[0UL] * q[0UL] + q[2UL] * q[2UL]) - 1.00, 2.00 * (q[2UL] * q[3UL] - q[0UL] * q[1.00])},
       {2.00 * (q[1UL] * q[3UL] - q[0UL] * q[2UL]), 2.00 * (q[2UL] * q[3UL] + q[0UL] * q[1UL]), 2.00 * (q[0UL] * q[0UL] + q[3UL] * q[3UL]) - 1.00}};

  return R;
}
std::vector<double> QuaternionToEulerAngles(const std::vector<double> &q)
{
  std::vector<double> angles(3);

  // roll (x-axis rotation)
  // q is : w,x,y,z,
  // angles: roll, pitch, yaw
  const double sinr_cosp = 2.00 * (q[0UL] * q[1UL] + q[2UL] * q[3UL]);
  const double cosr_cosp = 1.00 - 2.00 * (q[1UL] * q[1UL] + q[2UL] * q[2UL]);
  angles[0] = std::atan2(sinr_cosp, cosr_cosp);

  // pitch (y-axis rotation)
  const double sinp = 2.00 * (q[0UL] * q[2UL] - q[3UL] * q[1UL]);
  if (std::abs(sinp) >= 1)
    angles[1UL] = std::copysign(M_PI_2, sinp); // use 90 degrees if out of range
  else
    angles[1UL] = std::asin(sinp);

  // yaw (z-axis rotation)
  const double siny_cosp = 2.00 * (q[0UL] * q[3UL] + q[1UL] * q[2UL]);
  const double cosy_cosp = 1.00 - 2.00 * (q[2UL] * q[2UL] + q[3UL] * q[3UL]);
  angles[2UL] = std::atan2(siny_cosp, cosy_cosp);

  return angles;
}