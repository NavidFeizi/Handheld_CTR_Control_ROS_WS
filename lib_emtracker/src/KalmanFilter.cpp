#include "KalmanFilter.hpp"

// default constructor
KalmanFilter::KalmanFilter()
{
    // Parameters for Kalman Filterring
    dt = 0.05;
    A = {{1.0, dt, 0.0, 0.0, 0.0, 0.0},
         {0.0, 1.0, 0.0, 0.0, 0.0, 0.0},
         {0.0, 0.0, 1.0, dt, 0.0, 0.0},
         {0.0, 0.0, 0.0, 1.0, 0.0, 0.0},
         {0.0, 0.0, 0.0, 0.0, 1.0, dt},
         {0.0, 0.0, 0.0, 0.0, 0.0, 1.0}};
    C = {{1.0, 0.0, 0.0, 0.0, 0.0, 0.0},
         {0.0, 0.0, 1.0, 0.0, 0.0, 0.0},
         {0.0, 0.0, 0.0, 0.0, 1.0, 0.0}};
    X_ep = 1.0; // state
    X = A * X_ep;
    Pxx_ep = A * X * blaze::trans(X) * blaze::trans(A);
    blaze::diagonal(Q) = 1.0; // covariance of the process noise (gets updates with the innovation process)
    blaze::diagonal(R) = 1.0; // covariance of the observation noise
}

// overloaded constructor
KalmanFilter::KalmanFilter(const double q, const double r)
{
    // Parameters for Kalman Filterring
    dt = 0.05;
    A = {{1.0, dt, 0.0, 0.0, 0.0, 0.0},
         {0.0, 1.0, 0.0, 0.0, 0.0, 0.0},
         {0.0, 0.0, 1.0, dt, 0.0, 0.0},
         {0.0, 0.0, 0.0, 1.0, 0.0, 0.0},
         {0.0, 0.0, 0.0, 0.0, 1.0, dt},
         {0.0, 0.0, 0.0, 0.0, 0.0, 1.0}};
    C = {{1.0, 0.0, 0.0, 0.0, 0.0, 0.0},
         {0.0, 0.0, 1.0, 0.0, 0.0, 0.0},
         {0.0, 0.0, 0.0, 0.0, 1.0, 0.0}};
    X_ep = 1.0; // state
    X = A * X_ep;
    Pxx_ep = A * X * blaze::trans(X) * blaze::trans(A);
    blaze::diagonal(Q) = q; // covariance of the process noise
    blaze::diagonal(R) = r; // covariance of the observation noise
}

// KalmanFilter desctructor
KalmanFilter::~KalmanFilter()
{
    // nothing to be done
}

// copy constructor
KalmanFilter::KalmanFilter(const KalmanFilter &rhs) : dt(rhs.dt), A(rhs.A), C(rhs.C), X_ep(rhs.X_ep), X(rhs.X), Pxx_ep(rhs.Pxx_ep), K(rhs.K), Q(rhs.Q), R(rhs.R){};

// move constructor
KalmanFilter::KalmanFilter(KalmanFilter &&rhs) noexcept
{
    // handling self assignment
    if (this != &rhs)
    {
        this->dt = rhs.dt;
        this->A = std::move(rhs.A);
        this->C = std::move(rhs.C);
        this->X_ep = std::move(rhs.X_ep);
        this->X = std::move(rhs.X);
        this->Pxx_ep = std::move(rhs.Pxx_ep);
        this->K = std::move(rhs.K);
        this->Q = std::move(rhs.Q);
        this->R = std::move(rhs.R);
    }
}

// Copy assignment operator
KalmanFilter &KalmanFilter::operator=(const KalmanFilter &rhs)
{
    // handling self assignment
    if (this != &rhs)
    {
        this->dt = rhs.dt;
        this->A = rhs.A;
        this->C = rhs.C;
        this->X_ep = rhs.X_ep;
        this->X = rhs.X;
        this->Pxx_ep = rhs.Pxx_ep;
        this->K = rhs.K;
        this->Q = rhs.Q;
        this->R = rhs.R;
    }

    return *this;
}

// move assignment operator
KalmanFilter &KalmanFilter::operator=(KalmanFilter &&rhs) noexcept
{
    // handling self assignment
    if (this != &rhs)
    {
        this->dt = rhs.dt;
        this->A = std::move(rhs.A);
        this->C = std::move(rhs.C);
        this->X_ep = std::move(rhs.X_ep);
        this->X = std::move(rhs.X);
        this->Pxx_ep = std::move(rhs.Pxx_ep);
        this->K = std::move(rhs.K);
        this->Q = std::move(rhs.Q);
        this->R = std::move(rhs.R);
    }

    return *this;
}

// update core of the Kalman filter
void KalmanFilter::Loop(std::vector<double> &measurement, std::vector<double> *estimate)
{
    blaze::StaticVector<double, 3UL> Z = {measurement[0UL], measurement[1UL], measurement[2UL]};
    X = A * X_ep;
    Pxx = A * Pxx_ep * blaze::trans(A) + Q;

    blaze::StaticMatrix<double, 3UL, 3UL> Aux = blaze::inv(C * Pxx * blaze::trans(C) + R);
    K = Pxx * blaze::trans(C) * Aux;
    X_ep = X + K * (Z - C * X);

    Pxx_ep = Pxx - K * C * Pxx;
    blaze::StaticVector<double, 3UL> y_h = C * X_ep;
    *estimate = {y_h[0UL], y_h[1UL], y_h[2UL]};
}