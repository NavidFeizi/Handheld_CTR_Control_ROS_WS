#include <blaze/Math.h>
#include <cmath>

template <std::size_t N>
class TimeManager
{
public:
    // default constructor
    TimeManager() = delete;

    // overloaded constructor
    TimeManager(blaze::StaticMatrix<double, 18UL, N> &z, blaze::StaticMatrix<double, 18UL, N> &zh, double alpha, double dt);

    // copy constructor
    TimeManager(const TimeManager &rhs);

    // move constructor
    TimeManager(TimeManager &&rhs) noexcept;

    // CTR destructor
    ~TimeManager() = default;

    // copy assignment operator
    TimeManager &operator=(const TimeManager &rhs);

    // move assignment operator
    TimeManager &operator=(TimeManager &&rhs) noexcept;

    // set initial conditions for the time discretization method
    void setInitialConditions();

    // getter method for returning the implicit time differentiation C0
    double getC0();

    // method for stepping time (updates the backward time differentiation)
    void stepTime(const double dt);

    // method for resetting the time variable for running a sequence of dynamic simulations
    void resetTime();

    // getter method for retrieving the dynamic actuation input profile for the tendons
    double getDynamicActuationProfile();

    void printVariables();

private:
    // simulation run time
    double m_t;
    // parameters for implicit time differentiation (BDF-alpha)
    double m_c0;
    double m_c1;
    double m_c2;
    double m_d1;

    blaze::StaticMatrix<double, 18UL, N> &m_zLag1; // state vectors one time step behind y(t-dt)   -- lag 1
    blaze::StaticMatrix<double, 18UL, N> m_zLag2;  // state vectors two time steps behind y(t-2dt) -- lag 2
    blaze::StaticMatrix<double, 18UL, N> &m_zh;    // time derivative of the state vectors
};

// overloaded constructor
template <std::size_t N>
TimeManager<N>::TimeManager(blaze::StaticMatrix<double, 18UL, N> &z, blaze::StaticMatrix<double, 18UL, N> &zh, double alpha, double dt) : m_zLag1(z), m_zLag2(z), m_zh(zh)
{
    this->m_c0 = (1.50 + alpha) / (dt * (1.00 + alpha));
    this->m_c1 = -2.00 / dt;
    this->m_c2 = (0.50 + alpha) / (dt * (1.00 + alpha));
    this->m_d1 = alpha / (1.00 + alpha);
    this->m_t = 0.00;
}

// copy constructor
template <std::size_t N>
TimeManager<N>::TimeManager(const TimeManager<N> &rhs) : m_c0(rhs.m_c0), m_c1(rhs.m_c1), m_c2(rhs.m_c2), m_d1(rhs.m_d1), m_t(rhs.m_t),
                                                         m_zLag1(rhs.m_yLag1), m_zLag2(rhs.m_zLag2), m_zh(rhs.m_zh) {}

// move constructor
template <std::size_t N>
TimeManager<N>::TimeManager(TimeManager<N> &&rhs) noexcept
{
    // handling self-assignment
    if (this != &rhs)
    {
        this->m_c0 = rhs.m_c0;
        this->m_c1 = rhs.m_c1;
        this->m_c2 = rhs.m_c2;
        this->m_d1 = rhs.m_d1;
        this->m_t = rhs.m_t;
        this->m_zLag1 = std::move(rhs.m_zLag1);
        this->m_zLag2 = std::move(rhs.m_zLag2);
        this->m_zh = std::move(rhs.m_zh);
    }
}

// copy assignment operator
template <std::size_t N>
TimeManager<N> &TimeManager<N>::operator=(const TimeManager<N> &rhs)
{
    // handling self-assignment
    if (this != &rhs)
    {
        this->m_c0 = rhs.m_c0;
        this->m_c1 = rhs.m_c1;
        this->m_c2 = rhs.m_c2;
        this->m_d1 = rhs.m_d1;
        this->m_t = rhs.m_t;
        this->m_zLag1 = std::move(rhs.m_zLag1);
        this->m_zLag2 = std::move(rhs.m_zLag2);
        this->m_zh = std::move(rhs.m_zh);
    }

    return *this;
}

// move assignment operator
template <std::size_t N>
TimeManager<N> &TimeManager<N>::operator=(TimeManager<N> &&rhs) noexcept
{
    // handling self-assignment
    if (this != &rhs)
    {
        this->m_c0 = rhs.m_c0;
        this->m_c1 = rhs.m_c1;
        this->m_c2 = rhs.m_c2;
        this->m_d1 = rhs.m_d1;
        this->m_t = rhs.m_t;
        this->m_zLag1 = std::move(rhs.m_zLag1);
        this->m_zLag2 = std::move(rhs.m_zLag2);
        this->m_zh = std::move(rhs.m_zh);
    }

    return *this;
}

// set initial conditions for the time discretization method
template <std::size_t N>
void TimeManager<N>::setInitialConditions()
{
    this->m_zh = (this->m_c1 + this->m_c2) * this->m_zLag1;
    // In the static configuration, y(t-1) = y(t-2)
    this->m_zLag2 = this->m_zLag1;
}

// getter method for returning the implicit time differentiation C0
template <std::size_t N>
double TimeManager<N>::getC0()
{
    return this->m_c0;
}

// method for stepping time (updates the backward time differentiation)
template <std::size_t N>
void TimeManager<N>::stepTime(const double dt)
{
    // take a time step -> increment time variable
    this->m_t += dt;
    // writes the current time stamp to the console
    std::cout << "TimeManager t = " << this->m_t << "\r";
    std::cout.flush();

    // updates the time derivative of the state vector
    this->m_zh = (this->m_d1 * this->m_c0 + this->m_c1) * this->m_zLag1 + (this->m_c2 * this->m_zLag2) + (this->m_d1 * this->m_zh);
    // updates the lagged state vectors
    this->m_zLag2 = this->m_zLag1;
}

// method for resetting the time variable for running a sequence of dynamic simulations
template <std::size_t N>
void TimeManager<N>::resetTime()
{
    this->m_t = 0.00;
    this->m_zh.reset();
    this->m_zLag1.reset();
    this->m_zLag2.reset();
}

// getter method for retrieving the dynamic actuation input profile for the tendons
template <std::size_t N>
double TimeManager<N>::getDynamicActuationProfile()
{
    /*
                    *** THE SIMULATION STARTS WITH THE LOWEST TENDON DISPLACEMENT (qL) ***
        -- After t1 seconds: the displacement increases linearly toward its highest (qH) within (t2 - t1) seconds
        -- The tendon displacement remain at its highest value for (t3 - t2) seconds
        -- At t3 seconds: the displacement decreases linearly toward its lowest (qL) within (t4 - t3) seconds
        -- After t4 seconds: the displacement is kept unaltered at its lowest (qL) value
    */

    const double qL = 0.00;        // lowest tendon displacement value [meters]
    const double qH = -0.020;    // lowest tendon displacement value [meters]
    const double t1 = 3.652;
    const double t2 = 3.967;
    const double t3 = 17.63;
    const double t4 = 17.94;
    
    double q_t; // tendon displacement at time t [meters]

    // computing the tendon displacment at each time
    // if ( (this->m_t <= t1) || (this->m_t > t4) )
    //     q_t = qL;                                                   // region of lowest displacement
    // else
    // {
    //     if ( (this->m_t > t1) && (this->m_t <= t2) )
    //         q_t = qL + (qH - qL)*(this->m_t - t1)/(t2 - t1);        // ramp linearly toward the highest displacement
    //     else{
    //         if ( (this->m_t > t3) && (this->m_t <= t4) )
    //             q_t = qH + (qL - qH)*(this->m_t - t3)/(t4 - t3);    // ramp linearly toward lowest displacement
    //         else
    //             q_t = qH;                                           // region of highest displacement
    //     }
    // }

    q_t = -0.01 * std::fabs(sin(20.00 * this->m_t));

    return q_t;
}

// getter method for retrieving the dynamic actuation input profile for the tendons
template <std::size_t N>
void TimeManager<N>::printVariables()
{
    std::cout << "\n\n --- Printing the first 10 columns --- " << std::endl;

    std::cout << "zLag1 =\n"
              << blaze::submatrix<0UL, 0UL, 18UL, 10UL>(this->m_zLag1) << std::endl;

    std::cout << "zLag2 =\n"
              << blaze::submatrix<0UL, 0UL, 18UL, 10UL>(this->m_zLag2) << std::endl;

    std::cout << "zh =\n"
              << blaze::submatrix<0UL, 0UL, 18UL, 10UL>(this->m_zh) << std::endl
              << std::endl;
}