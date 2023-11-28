#pragma once

#include <blaze/Blaze.h>

class KalmanFilter
{
public:
    // default constructor
    KalmanFilter() = delete;

    // KalmanFilter constructor
    KalmanFilter(const double delta_t, const blaze::StaticMatrix<double, 6UL, 6UL> Q_, const blaze::StaticMatrix<double, 3UL, 3UL> R_);

    // KalmanFilter desctructor
    ~KalmanFilter() = default;

    // copy constructor
    KalmanFilter(const KalmanFilter &rhs);

    // move constructor
    KalmanFilter(KalmanFilter &&rhs) noexcept;

    // Copy assignment operator
    KalmanFilter &operator=(const KalmanFilter &rhs);

    // move assignment operator
    KalmanFilter &operator=(KalmanFilter &&rhs) noexcept;

    // update core of the Kalman filter
    void Loop(std::vector<double> &measurement, std::vector<double> &estimate);

private:
    double dt;
    blaze::StaticMatrix<double, 6UL, 6UL> A;
    blaze::StaticMatrix<double, 3UL, 6UL> C;
    blaze::StaticVector<double, 6UL> x;
    blaze::StaticVector<double, 3UL> y, y_hat;
    blaze::StaticMatrix<double, 6UL, 6UL> Pxx;
    blaze::IdentityMatrix<double> I;
    blaze::StaticMatrix<double, 6UL, 3UL> W;
    blaze::StaticMatrix<double, 3UL, 3UL> S_inv;
    blaze::StaticMatrix<double, 6UL, 6UL> Q;
    blaze::StaticMatrix<double, 3UL, 3UL> R;
};