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

/* Rotates a 3D vector with RotationQuaternion */
void Quaternion_Rotate_Point(const boost::qvm::quat<double> &rotationQuaternion,
                             const boost::qvm::vec<double, 3> &originalVector,
                             boost::qvm::vec<double, 3> *rotatedVector)
{
  *rotatedVector = rotationQuaternion * originalVector; // look like boost does the conjugate multiplication automatically!
}

/* Calculates inverse of a rigid transformation */
void Inverse_Quat_Transformation(const quatTransformation &origianlTransformation,
                                 quatTransformation *inverseTransformation)
{
  // Inverse of the rotation quaternion
  inverseTransformation->rotation = boost::qvm::conjugate(origianlTransformation.rotation);
  // Rotate the translation using the inverse rotation
  Quaternion_Rotate_Point(inverseTransformation->rotation, origianlTransformation.translation, &inverseTransformation->translation);
  // Inverse of the translation vector
  // Negate the translation vector components
  inverseTransformation->translation = {-boost::qvm::X(inverseTransformation->translation),
                                        -boost::qvm::Y(inverseTransformation->translation),
                                        -boost::qvm::Z(inverseTransformation->translation)};
}

/* Function to combine rigid transformation from frame 0 to 1 with
    rigid transformation from frame 1 to 2
    to calculate transformation from frame 0 to 1 */
void Combine_Quat_Transformation(const quatTransformation &transformation01,
                                 const quatTransformation &transformation12,
                                 quatTransformation *transformation02)
{
  boost::qvm::vec<double, 3> translation_12_0;
  Quaternion_Rotate_Point(transformation01.rotation, transformation12.translation, &translation_12_0);
  transformation02->translation = transformation01.translation + translation_12_0;
  transformation02->rotation = boost::qvm::conjugate(transformation01.rotation) * transformation12.rotation * transformation01.rotation;
}

/* Function to calculate the transformation that transfers the landmarks_measured frame
    to the landmarks_truth frame */
void Calculate_Transformation(const std::vector<std::vector<double>> &landmarks_measured,
                              const std::vector<std::vector<double>> &landmarks_truth,
                              quatTransformation *transformation)
{                                              // Create an instance of the NLOPT optimizer
  nlopt::opt optimizer(nlopt::LN_COBYLA, 7UL); // 7 parameters (4 for rotation, 3 for translation)
  // LN_COBYLA, LN_NEWUOA
  nlopt::result result;
  objFuncData landmarks;
  landmarks.measured = landmarks_measured;
  landmarks.truth = landmarks_truth;
  // initialte initial guess
  std::vector<double> x(7UL, 0.0);
  // Set bounds for the optimization parameters (if needed)
  std::vector<double> lb = {-100.0, -100.0, -100.0, -0.9999, -1.0, -1.0, -1.0};
  std::vector<double> ub = {100.0, 100.0, 100.0, 0.9999, 1.0, 1.0, 1.0};
  optimizer.set_lower_bounds(lb);
  optimizer.set_upper_bounds(ub);
  // void nlopt::opt::set_min_objective(nlopt::vfunc f, void* f_data);
  optimizer.set_min_objective(Objective_Function, &landmarks);
  optimizer.add_equality_constraint(Equality_Constraint, NULL, 1e-8);
  // Specify stopping criteria (optional)
  optimizer.set_maxeval(200000); // Maximum number of function evaluations
  optimizer.set_ftol_rel(1e-12); // Relative tolerance for convergence
  optimizer.set_xtol_rel(1e-8);

  // Create a random number generator
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<double> dist;
  double error, error_min = 1000.0;
  for (int i = 0; i < 50; i++)
  {
    // Generate random initial guesses within the bounds
    for (size_t i = 0; i < x.size(); ++i)
    {
      x[i] = lb[i] + (ub[i] - lb[i]) * dist(gen);
    }
    // Perform the optimization
    result = optimizer.optimize(x, error);

    // Track the minimum error and the corresponding x
    if (error < error_min)
    {
      (*transformation).translation = boost::qvm::vec<double, 3>({x[0], x[1], x[2]});
      (*transformation).rotation = boost::qvm::quat<double>({x[3], x[4], x[5], x[6]});
      (*transformation).error = error;
      error_min = error;
    }

    std::cout << "Square Error [mm]: " << error << "    "
              << "Num itter: " << optimizer.get_numevals() << "    "
              << "Stop: " << nloptResult2String(result) << std::endl;
  }
  std::cout << "Optimization Finished" << std::endl;
  std::cout << "Calculated Transforation => "
            << "X: " << boost::qvm::X((*transformation).translation) << "[mm]  "
            << "Y: " << boost::qvm::Y((*transformation).translation) << "[mm]  "
            << "Z: " << boost::qvm::Z((*transformation).translation) << "[mm]  |  "
            << "q0: " << boost::qvm::S((*transformation).rotation) << "   "
            << "qX: " << boost::qvm::X((*transformation).rotation) << "   "
            << "qY: " << boost::qvm::Y((*transformation).rotation) << "   "
            << "qZ: " << boost::qvm::Z((*transformation).rotation) << "  |  "
            << "RMSE: " << sqrt((*transformation).error) << "[mm]  " << std::endl;
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
  rigidTran.translation = boost::qvm::vec<double, 3>({x[0], x[1], x[2]});
  rigidTran.rotation = boost::qvm::quat<double>({x[3], x[4], x[5], x[6]});
  double dist, error = 0;

  for (unsigned int i = 0; i < landmarks->measured.size(); i++)
  {
    tran_measured.translation = boost::qvm::vec<double, 3>({landmarks->measured[i][0],
                                                            landmarks->measured[i][1],
                                                            landmarks->measured[i][2]});
    tran_truth.translation = boost::qvm::vec<double, 3>({landmarks->truth[i][0],
                                                         landmarks->truth[i][1],
                                                         landmarks->truth[i][2]});
    Inverse_Quat_Transformation(rigidTran, &rigidTran_inv);
    Combine_Quat_Transformation(rigidTran_inv, tran_measured, &tran_estimated);
    Euclidean_Distance(tran_estimated.translation, tran_truth.translation, &dist);
    error = error + dist * dist;
  }
  return error / landmarks->measured.size();
}

/* Constraint function to enforce unit length of the calcualted quaternion */
double Equality_Constraint(const std::vector<double> &x, std::vector<double> &grad, void *f_data)
{
  // This function enforces x[3]^2 + x[4]^2 + x[5]^2 - 1 = 0
  return x[3] * x[3] + x[4] * x[4] + x[5] * x[5] + x[6] * x[6] - 1;
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

/* Function to convert rotation matrix to the quaternion */
void Rotmat2Quaternion(const blaze::DynamicMatrix<double> rotmat, boost::qvm::quat<double> &rotqaut)
{
  // Access the elements using () operator
  double w, x, y, z;
  double trace = rotmat(0, 0) + rotmat(1, 1) + rotmat(2, 2); // you can use blaze::trace
  if (trace > 0)
  { // No need to change M_EPSILON to 0
    double s = 0.5f / std::sqrt(trace + 1.0f);
    w = 0.25f / s;
    x = (rotmat(2, 1) - rotmat(1, 2)) * s;
    y = (rotmat(0, 2) - rotmat(2, 0)) * s;
    z = (rotmat(1, 0) - rotmat(0, 1)) * s;
  }
  else
  {
    if (rotmat(0, 0) > rotmat(1, 1) && rotmat(0, 0) > rotmat(2, 2))
    {
      double s = 2.0f * std::sqrt(1.0f + rotmat(0, 0) - rotmat(1, 1) - rotmat(2, 2));
      w = (rotmat(2, 1) - rotmat(1, 2)) / s;
      x = 0.25f * s;
      y = (rotmat(0, 1) + rotmat(1, 0)) / s;
      z = (rotmat(0, 2) + rotmat(2, 0)) / s;
    }
    else if (rotmat(1, 1) > rotmat(2, 2))
    {
      double s = 2.0f * std::sqrt(1.0f + rotmat(1, 1) - rotmat(0, 0) - rotmat(2, 2));
      w = (rotmat(0, 2) - rotmat(2, 0)) / s;
      x = (rotmat(0, 1) + rotmat(1, 0)) / s;
      y = 0.25f * s;
      z = (rotmat(1, 2) + rotmat(2, 1)) / s;
    }
    else
    {
      double s = 2.0f * std::sqrt(1.0f + rotmat(2, 2) - rotmat(0, 0) - rotmat(1, 1));
      w = (rotmat(1, 0) - rotmat(0, 1)) / s;
      x = (rotmat(0, 2) + rotmat(2, 0)) / s;
      y = (rotmat(1, 2) + rotmat(2, 1)) / s;
      z = 0.25f * s;
    }
  }

  rotqaut = {w, x, y, z};
}

/* Function to convert Euler angles rotation to quaternion */
void Euler2Quaternion(const double heading, const double attitude, const double bank, blaze::StaticVector<double, 4UL> &h)
{
  // yaw, pitch, roll
  // Gotta convert the angles to radians first.
  double theta(0.50 * heading), phi(0.50 * attitude), psi(0.50 * bank);

  double c1(cos(theta)), s1(sin(theta)), c2(cos(phi)), s2(sin(phi)), c3(cos(psi)), s3(sin(psi)), c1c2, s1s2;

  c1c2 = c1 * c2;
  s1s2 = s1 * s2;

  h[0UL] = c1c2 * c3 - s1s2 * s3;
  h[1UL] = c1c2 * s3 + s1s2 * c3;
  h[2UL] = s1 * c2 * c3 + c1 * s2 * s3;
  h[3UL] = c1 * s2 * c3 - s1 * c2 * s3;
}

/* Function to convert Quaternion to Euler angles rotation */
void QuaternionToEuler(const boost::qvm::quat<double> &quaternion, double *azimuth, double *elevation, double *roll)
{
  // Extract quaternion components
  double q0 = boost::qvm::S(quaternion);
  double q1 = boost::qvm::X(quaternion);
  double q2 = boost::qvm::Y(quaternion);
  double q3 = boost::qvm::Z(quaternion);

  // double q0 = quaternion[0UL];
  // double q1 = quaternion[1UL];
  // double q2 = quaternion[2UL];
  // double q3 = quaternion[3UL];

  // Calculate roll (bank)
  *azimuth = atan2(2.0 * (q2 * q3 + q0 * q1), 1.0 - 2.0 * (q1 * q1 + q2 * q2));

  // Calculate elevation (pitch)
  double sinElevation = 2.0 * (q0 * q2 - q1 * q3);
  if (std::abs(sinElevation) >= 1.0)
  {
    // Handle numerical instability near poles
    *elevation = sinElevation < 0.0 ? -M_PI / 2.0 : M_PI / 2.0;
  }
  else
  {
    *elevation = asin(sinElevation);
  }

  // Calculate azimuth (yaw)
  *roll = atan2(2.0 * (q0 * q3 + q1 * q2), 1.0 - 2.0 * (q2 * q2 + q3 * q3));

  // // Convert angles from radians to degrees
  // azimuth *= 180.0 / M_PI;
  // elevation *= 180.0 / M_PI;
  // roll *= 180.0 / M_PI;
}

/* Function to calculate Euclidean distance between two translation vectors */
void Euclidean_Distance(const boost::qvm::vec<double, 3> &translation1, const boost::qvm::vec<double, 3> &translation2, double *distance)
{
  // Calculate the Euclidean distance between the translation vectors
  double dist = std::sqrt(
      (boost::qvm::X(translation1) - boost::qvm::X(translation2)) * (boost::qvm::X(translation1) - boost::qvm::X(translation2)) +
      (boost::qvm::Y(translation1) - boost::qvm::Y(translation2)) * (boost::qvm::Y(translation1) - boost::qvm::Y(translation2)) +
      (boost::qvm::Z(translation1) - boost::qvm::Z(translation2)) * (boost::qvm::Z(translation1) - boost::qvm::Z(translation2)));
  // Store the result in the provided pointer
  *distance = dist;
}

//
//
/* Functions below are not used and not verified - must be tested before use */
// function that return rotation matrix of a set of quaternions
// function that returns the rotation matrix Rx of any angle theta
blaze::StaticMatrix<double, 3UL, 3UL> RotX(const double &theta)
{
  blaze::StaticMatrix<double, 3UL, 3UL> R;
  double c(cos(theta)), s(sin(theta));
  R = {{1, 0, 0},
       {0, c, -s},
       {0, s, c}};
  return R;
}
blaze::StaticMatrix<double, 3UL, 3UL> RotY(const double &theta)
{
  blaze::StaticMatrix<double, 3UL, 3UL> R;
  double c(cos(theta)), s(sin(theta));
  R = {{c, 0, s},
       {0, 1, 0},
       {-s, 0, c}};
  return R;
}
blaze::StaticMatrix<double, 3UL, 3UL> RotZ(const double &theta)
{
  blaze::StaticMatrix<double, 3UL, 3UL> R;
  double c(cos(theta)), s(sin(theta));
  R = {{c, -s, 0},
       {s, c, 0},
       {0, 0, 1}};
  return R;
}
void EulerToRotMat(const std::vector<double> &Angles, blaze::StaticMatrix<double, 3UL, 3UL> *R)
{
  *R = RotZ(Angles[0]) * RotY(Angles[1]) * RotX(Angles[2]);
}
blaze::StaticMatrix<double, 3UL, 3UL> quaternion2rotmat(std::vector<double> q)
{
  blaze::StaticMatrix<double, 3UL, 3UL> R;
  R = {{2 * (q[0] * q[0] + q[1] * q[1]) - 1, 2 * (q[1] * q[2] - q[0] * q[3]), 2 * (q[1] * q[3] + q[0] * q[2])},
       {2 * (q[1] * q[2] + q[0] * q[3]), 2 * (q[0] * q[0] + q[2] * q[2]) - 1, 2 * (q[2] * q[3] - q[0] * q[1])},
       {2 * (q[1] * q[3] - q[0] * q[2]), 2 * (q[2] * q[3] + q[0] * q[1]), 2 * (q[0] * q[0] + q[3] * q[3]) - 1}};
  return R;
}
std::vector<double> QuaternionToEulerAngles(const std::vector<double> q)
{
  std::vector<double> angles(3);

  // roll (x-axis rotation)
  // q is : w,x,y,z,
  // angles: roll, pitch, yaw
  double sinr_cosp = 2 * (q[0] * q[1] + q[2] * q[3]);
  double cosr_cosp = 1 - 2 * (q[1] * q[1] + q[2] * q[2]);
  angles[0] = std::atan2(sinr_cosp, cosr_cosp);

  // pitch (y-axis rotation)
  double sinp = 2 * (q[0] * q[2] - q[3] * q[1]);
  if (std::abs(sinp) >= 1)
    angles[1] = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
  else
    angles[1] = std::asin(sinp);

  // yaw (z-axis rotation)
  double siny_cosp = 2 * (q[0] * q[3] + q[1] * q[2]);
  double cosy_cosp = 1 - 2 * (q[2] * q[2] + q[3] * q[3]);
  angles[2] = std::atan2(siny_cosp, cosy_cosp);

  return angles;
}