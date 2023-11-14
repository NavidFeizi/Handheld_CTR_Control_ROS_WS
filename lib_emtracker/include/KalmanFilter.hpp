#pragma once

#include <blaze/Blaze.h>
#include <cmath>

class KalmanFilter
{
public:
    // default constructor
    KalmanFilter();

    // KalmanFilter constructor
    KalmanFilter(const double q, const double r);

    // KalmanFilter desctructor
    ~KalmanFilter();

    // copy constructor
    KalmanFilter(const KalmanFilter &rhs);

    // move constructor
    KalmanFilter(KalmanFilter &&rhs) noexcept;

    // Copy assignment operator
    KalmanFilter &operator=(const KalmanFilter &rhs);

    // move assignment operator
    KalmanFilter &operator=(KalmanFilter &&rhs) noexcept;

    // update core of the Kalman filter
    void Loop(std::vector<double> &measurement, std::vector<double> *estimate);


private:
    double dt;
    blaze::StaticMatrix<double, 6UL, 6UL> A;
    blaze::StaticMatrix<double, 3UL, 6UL> C;
    blaze::StaticVector<double, 6UL> X_ep, X;
    blaze::StaticMatrix<double, 6UL, 6UL> Pxx_ep, Pxx;
    blaze::StaticMatrix<double, 6UL, 3UL> K;
    // blaze::DiagonalMatrix<blaze::StaticMatrix<double, 6UL, 6UL>> Q;
    // blaze::DiagonalMatrix<blaze::StaticMatrix<double, 3UL, 3UL>> R;

    blaze::StaticMatrix<double, 6UL, 6UL> Q;
    blaze::StaticMatrix<double, 3UL, 3UL> R;
};