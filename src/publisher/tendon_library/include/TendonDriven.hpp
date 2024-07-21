// *************************************************************************************************************** //
// *        This file is part of a C++ kinematics & Dynamics library for Tendon-Driven Continuum Robots          * //
// *																					                         * //
// *    ------------ # Copyright (C) 2021 Dr Mohammad Salehizadeh <msalehizadeh@bwh.harvard.edu> # -----------   * //
// *    ------------ #           Copyright (C) 2021 Filipe C. Pedrosa <fpedrosa@uwo.ca>          # -----------   * //
// *																					                         * //
// *       Project developed under the supervision of Dr Jayender Jagadeesan (❖) Dr Rajni Patel (◈)	            * //
// *  =======================================================================================================    * //
// *  (◈) CSTAR (Canadian Surgical Technologies & Advanced Robotics) @ Western University, London, ON  Canada   * //
// *            (❖) Surgical Planning Lab @ Brigham and Women's Hospital / Harvard Medical School				* //
// *************************************************************************************************************** //

#pragma once

#include <blaze/Math.h>
#include <array>
#include <cmath>
#include <memory>
#include <execution>
#include <iostream>

#include "TimeManager.hpp"
#include "ODESystem.hpp"
#include "PDESystem.hpp"

// class that implements the dynamics of tendon-driven robots (any arbitrary number of tendons)
template <std::size_t N, std::size_t numTendons>
class TendonDriven
{
public:
	// removes the default constructor
	TendonDriven() = delete;

	// overloaded constructor
	TendonDriven(double E, double G, double rad, double mass, double L, double alpha, double dt, double tendonOffset, double dampBendTwist, double dampShearExt, double dragCoeff);

	// copy constructor
	TendonDriven(const TendonDriven &rhs);

	// move constructor
	TendonDriven(TendonDriven &&rhs) noexcept;

	// CTR destructor
	~TendonDriven() = default;

	// copy assignment operator
	TendonDriven &operator=(const TendonDriven &rhs);

	// move assignment operator
	TendonDriven &operator=(TendonDriven &&rhs) noexcept;

	// returns the residue (violation of boundary condition) for a particular initial
	template <mathOp::cosseratEquations solType, mathOp::integrationMethod intMethod>
	blaze::StaticVector<double, 6UL> residueFunction(const blaze::StaticVector<double, 6UL> &initGuess);

	// function that computes the finite-differences Jacobian for solving the BVP
	template <mathOp::cosseratEquations solType, mathOp::integrationMethod intMethod>
	blaze::StaticMatrix<double, 6UL, 6UL> jac_BVP(const blaze::StaticVector<double, 6UL> &initGuess, const blaze::StaticVector<double, 6UL> &residue);

	// manages which method to be used for solving the Boundary-value problem
	template <mathOp::cosseratEquations solType, mathOp::rootFindingMethod method, mathOp::integrationMethod intMethod>
	bool solveBVP(blaze::StaticVector<double, 6UL> &initGuess);

	// function that increments time in the dynamics evolution problem
	void stepTime();

	// function to set the external force at the distal end of the catheter
	void setExternalDistalForce(const blaze::StaticVector<double, 3UL> &force);

	// function that sets the actuation value of joints ==> tendon displacements [m]
	void setTendonActuation(const blaze::StaticVector<double, numTendons> &q);

	// function that implements Powell's Dog Leg Method (Nonlinear root-finding method for solving the BVP)
	template <mathOp::cosseratEquations solType, mathOp::integrationMethod intMethod>
	bool PowellDogLeg(blaze::StaticVector<double, 6UL> &initGuess);

	// function that implements the Levenberg-Marquardt Method (Nonlinear root-finding method for solving the BVP)
	template <mathOp::cosseratEquations solType, mathOp::integrationMethod intMethod>
	bool Levenberg_Marquardt(blaze::StaticVector<double, 6UL> &initGuess);

	// function that implements an alternative Levenberg-Marquardt Method (Nonlinear root-finding method for solving the BVP)
	template <mathOp::cosseratEquations solType, mathOp::integrationMethod intMethod>
	bool Levenberg_Marquardt_II(blaze::StaticVector<double, 6UL> &initGuess);

	// function that implements Broyden's Nonlinear root-finding method for solving the BVP (Jacobian inverse is estimated)
	template <mathOp::cosseratEquations solType, mathOp::integrationMethod intMethod>
	bool Broyden(blaze::StaticVector<double, 6UL> &initGuess);

	// function that implements the Newton-Raphson method (Nonlinear root-finding method for solving the BVP)
	template <mathOp::cosseratEquations solType, mathOp::integrationMethod intMethod>
	bool Newton_Raphson(blaze::StaticVector<double, 6UL> &initGuess);

	// function that implements the Modified, globally convergent Newton-Raphson method (Nonlinear root-finding method for solving the BVP)
	template <mathOp::cosseratEquations solType, mathOp::integrationMethod intMethod>
	bool Modified_Newton_Raphson(blaze::StaticVector<double, 6UL> &initGuess);

	// getter method for returning the shear / extension viscous damping
	blaze::DiagonalMatrix<blaze::StaticMatrix<double, 3UL, 3UL>> get_Bse();

	// getter method for returning the bending / torsion viscous damping
	blaze::DiagonalMatrix<blaze::StaticMatrix<double, 3UL, 3UL>> get_Bbt();

	// getter method for returning the viscuous "air-drag" coefficient stiffness
	blaze::DiagonalMatrix<blaze::StaticMatrix<double, 3UL, 3UL>> get_C();

	// getter method for returning the shear / extension stiffness stiffness
	blaze::DiagonalMatrix<blaze::StaticMatrix<double, 3UL, 3UL>> get_Kse();

	// getter method for returning the bending / torsion stiffness stiffness
	blaze::DiagonalMatrix<blaze::StaticMatrix<double, 3UL, 3UL>> get_Kbt();

	// getter method for returning the actuation inputs
	blaze::StaticVector<double, numTendons> get_tau();

	// getter method for returning the i-th actuation input
	double get_tau(std::size_t i);

	// getter method for returning the radial positions of the tendons
	std::array<blaze::StaticVector<double, 3UL>, numTendons> get_r();

	// getter method for returning the radial positions of the i-th tendon
	blaze::StaticVector<double, 3UL> get_r(std::size_t i);

	// getter method for returning the linear density of the robot
	double get_rho();

	// getter method for returning the sheath's cross-sectional area
	double get_A();

	// getter method for returning the acceleration of gravity
	blaze::StaticVector<double, 3UL> get_g();

	// method for setting and initializing the time discretization variables after the initial static condition
	void setTimeDiscretization();

	// method for returning the shape of the tendon-driven robot
	blaze::StaticMatrix<double, 3UL, N> getBackboneShape();

	// method for returning the tip position of the tendon-driven robot
	blaze::StaticVector<double, 3UL> getTipPosition();

	// method for returning the distal tip position & orientation of the tendon-driven robot
	std::tuple<blaze::StaticVector<double, 3UL>, blaze::StaticVector<double, 3UL>> getTipPose();

	// method that sets the optimization parameters for the static model
	void setOptimizingStaticParameters(const double EI,
									   const double nu,
									   const double radius,
									   const double tendonCompliance,
									   const double tendonAngle);

	// method that returns the optimizig static parameters
	std::tuple<double, double, double, double, double> getOptimizingStaticParameters();

	// method that sets the optimization parameters for the dynamic model
	void setOptimizingDynamicParameters(const double E,
										const double radius,
										const double mass);

	// method that returns the optimizig dynamic parameters
	std::tuple<double, double, double> getOptimizingDynamicParameters();

	// method that reset all containers prior to running sequences of static simulations
	void resetStaticSimulation();

	// method that reset all containers prior to running sequences of dynamic simulations
	void resetDynamicSimulation();

	std::tuple<blaze::StaticVector<double, 19UL + numTendons>, blaze::StaticVector<double, numTendons>> getTrainingData();

private:
	double m_E;															// Young's modulus [GPa]
	double m_G;															// Shear modulus
	double m_rad;														// backbone radius [m]
	double m_A;															// cross-sectional area [m^2]
	double m_mass;														// total mass (backbone + disks) [kg]
	double m_rho;														// material density [kgm^-3]
	blaze::StaticVector<double, 3UL> m_g;								// acceleration of gravity [m/s^2]
	blaze::StaticVector<double, 3UL> m_f;								// external point force at the dista end of the catheter [N]
	double m_L;															// backbone total length [m]
	double m_dt;														// time step for dynamics simulation [s]
	double m_alpha;														// backward differentiation parameter (BDF_alpha)
	double m_tendonOffset;												// offset distance between the tendons and backbone [m]
	double m_tendonAngleOffset;											// angle at which the orthogonal cross-sectional arrangements of the tendons are axially rotated
	blaze::StaticVector<double, numTendons> m_tau;						// tensile actuation force on tendons
	blaze::DiagonalMatrix<blaze::StaticMatrix<double, 3UL, 3UL>> m_Bse; // shear & extension viscous-like damping [Ns]
	blaze::DiagonalMatrix<blaze::StaticMatrix<double, 3UL, 3UL>> m_Bbt; // bending & torsion viscous-like damping [Nm^2s]
	blaze::DiagonalMatrix<blaze::StaticMatrix<double, 3UL, 3UL>> m_C;	// air drag viscous-like damping [kg/m^2]
	blaze::DiagonalMatrix<blaze::StaticMatrix<double, 3UL, 3UL>> m_Kse; // stiffness matrix for sear & extension [N]
	blaze::DiagonalMatrix<blaze::StaticMatrix<double, 3UL, 3UL>> m_Kbt; // stiffness matrix for bending & torsion [Nm^2]
	blaze::DiagonalMatrix<blaze::StaticMatrix<double, 3UL, 3UL>> m_J;	// second mass moment of inertia tensor [m^4]
	std::array<blaze::StaticVector<double, 3UL>, numTendons> m_r;		// array of vectors describing the radial position of each tendon
	std::unique_ptr<TimeManager<N>> m_time;								// implements the time manager for dynamics simulation
	std::unique_ptr<ODESystem<N, numTendons>> m_ODE;					// implements the ODE state equations for a static tendon-driven robot (steady-state)
	std::unique_ptr<PDESystem<N, numTendons>> m_PDE;					// implements the PDE state equations for a dynamic tendon-driven robot
	blaze::StaticMatrix<double, 19UL + numTendons, N> m_y;				// set of N state vectors
	blaze::StaticMatrix<double, 18UL, N> m_z, m_zh;						// set of N
	mathOp::rootFindingMethod m_method;									// enum defining which nonlinear root-finder to be used
	double m_BVPAccuracy;												// accuracy of BVP solution
};

// overloaded constructor
template <std::size_t N, std::size_t numTendons>
TendonDriven<N, numTendons>::TendonDriven(double E, double G, double rad, double mass, double L, double alpha, double dt, double tendonOffset, double dampBendTwist, double dampShearExt, double dragCoeff) : m_E(E), m_G(G), m_rad(rad), m_mass(mass), m_L(L), m_alpha(alpha), m_dt(dt), m_tendonOffset(tendonOffset)
{
	this->m_g = {-9.81, 0.00, 0.00};
	this->m_f = 0.00;
	this->m_A = M_PI * pow(this->m_rad, 2); // cross-sectional area
	double I = M_PI_4 * pow(this->m_rad, 4);
	this->m_rho = this->m_mass / (this->m_L * this->m_A);
	// Shear and extension bending stiffness matrix
	this->m_Kse(0UL, 0UL) = this->m_Kse(1UL, 1UL) = this->m_G * this->m_A;
	this->m_Kse(2UL, 2UL) = this->m_E * this->m_A;
	// Bending and twisting bending stiffness matrix
	this->m_Kbt(0UL, 0UL) = this->m_Kbt(1UL, 1UL) = this->m_E * I;
	this->m_Kbt(2UL, 2UL) = this->m_G * (I + I);
	// Moment of Inertia
	this->m_J(0UL, 0UL) = this->m_J(1UL, 1UL) = I;
	this->m_J(2UL, 2UL) = I + I;
	this->m_BVPAccuracy = 1.00E-8;
	this->m_tau = 0.00;
	blaze::diagonal(this->m_Bse) = blaze::uniform(3UL, dampShearExt);
	blaze::diagonal(this->m_Bbt) = blaze::uniform(3UL, dampBendTwist);
	blaze::diagonal(this->m_C) = blaze::uniform(3UL, dragCoeff);

	// temporarly switch with static vecotor
	// blaze::DynamicVector<double, blaze::rowVector> angles;
	// angles = blaze::linspace<blaze::rowVector>(numTendons + 1, 0.00, 2.00 * M_PI);

	blaze::StaticVector<double, numTendons, blaze::rowVector> angles;
	// Manually filling the static vector with linearly spaced values
	for(size_t i = 0; i < numTendons; ++i) {
		angles[i] = i * (2.00 * M_PI / numTendons);
	}

	this->m_tendonAngleOffset = 0.00; //-0.023317;

	for (size_t idx = 0UL; idx < numTendons; ++idx)
	{
		this->m_r[idx][0UL] = m_tendonOffset * cos(angles[idx] + this->m_tendonAngleOffset);
		this->m_r[idx][1UL] = m_tendonOffset * sin(angles[idx] + this->m_tendonAngleOffset);
		this->m_r[idx][2UL] = 0.00;

		// eliminates small numbers (trash) from "zeros" entries of the radial offset vector
		this->m_r[idx] = blaze::map(this->m_r[idx], [](double d)
									{ return (std::abs(d) < 1.00E-5) ? 0.00 : d; });
	}

	this->m_time = std::make_unique<TimeManager<N>>(this->m_z, this->m_zh, this->m_alpha, this->m_dt);
	this->m_ODE = std::make_unique<ODESystem<N, numTendons>>();
	this->m_PDE = std::make_unique<PDESystem<N, numTendons>>();
}

// copy constructor
template <std::size_t N, std::size_t numTendons>
TendonDriven<N, numTendons>::TendonDriven(const TendonDriven &rhs) : m_E(rhs.m_E), m_G(rhs.m_G), m_rad(rhs.m_rad), m_mass(rhs.m_mass), m_L(rhs.m_L), m_alpha(rhs.m_alpha), m_dt(rhs.m_dt), m_tendonOffset(rhs.m_tendonOffset), m_tendonAngleOffset(rhs.m_tendonAngleOffset), m_A(rhs.m_A), m_rho(rhs.m_rho), m_Kse(rhs.m_Kse), m_Kbt(rhs.m_Kbt), m_J(rhs.m_J), m_BVPAccuracy(rhs.m_BVPAccuracy), m_tau(rhs.m_tau), m_Bse(rhs.m_Bse), m_Bbt(rhs.m_Bbt), m_C(rhs.m_C), m_r(rhs.m_r), m_g(rhs.m_g), m_f(rhs.m_g)
{
	this->m_time = std::make_unique<TimeManager<N>>(rhs.m_z, rhs.m_zh, rhs.m_alpha, rhs.m_dt);
	this->m_ODE = std::make_unique<ODESystem<N, numTendons>>();
	this->m_PDE = std::make_unique<PDESystem<N, numTendons>>();
}

// move constructor
template <std::size_t N, std::size_t numTendons>
TendonDriven<N, numTendons>::TendonDriven(TendonDriven &&rhs) noexcept
{
	// handling self-assignment
	if (this != &rhs)
	{
		this->m_E = rhs.m_E;
		this->m_G = rhs.m_G;
		this->m_rad = rhs.m_rad;
		this->m_mass = rhs.m_mass;
		this->m_L = rhs.m_L;
		this->m_alpha = rhs.m_alpha;
		this->m_dt = rhs.m_dt;
		this->m_tendonOffset = rhs.m_tendonOffset;
		this->m_tendonAngleOffset = rhs.m_tendonAngleOffset;
		this->m_A = rhs.m_A;
		this->m_rho = rhs.m_rho;
		this->m_Kse = std::move(rhs.m_Kse);
		this->m_Kbt = std::move(rhs.m_Kbt);
		this->m_J = std::move(rhs.m_J);
		this->m_BVPAccuracy = rhs.m_BVPAccuracy;
		this->m_tau = std::move(rhs.m_tau);
		this->m_Bse = std::move(rhs.m_Bse);
		this->m_Bbt = std::move(rhs.m_Bbt);
		this->m_C = std::move(rhs.m_C);
		this->m_r = std::move(rhs.m_r);
		this->m_g = std::move(rhs.m_g);
		this->m_f = std::move(rhs.m_f);
		this->m_y = std::move(rhs.m_y);
		this->m_z = std::move(rhs.m_z);
		this->m_zh = std::move(rhs.m_zh);
		this->m_time = std::move(rhs.m_time);
		this->m_ODE = std::move(rhs.m_ODE);
		this->m_PDE = std::move(rhs.m_PDE);
	}
}

// copy assignment operator
template <std::size_t N, std::size_t numTendons>
TendonDriven<N, numTendons> &TendonDriven<N, numTendons>::operator=(const TendonDriven<N, numTendons> &rhs)
{
	// handling self assignment
	if (this != &rhs)
	{
		this->m_E = rhs.m_E;
		this->m_G = rhs.m_G;
		this->m_rad = rhs.m_rad;
		this->m_mass = rhs.m_mass;
		this->m_L = rhs.m_L;
		this->m_alpha = rhs.m_alpha;
		this->m_dt = rhs.m_dt;
		this->m_tendonOffset = rhs.m_tendonOffset;
		this->m_tendonAngleOffset = rhs.m_tendonAngleOffset;
		this->m_A = rhs.m_A;
		this->m_rho = rhs.m_rho;
		this->m_Kse = rhs.m_Kse;
		this->m_Kbt = rhs.m_Kbt;
		this->m_J = rhs.m_J;
		this->m_BVPAccuracy = rhs.m_BVPAccuracy;
		this->m_tau = rhs.m_tau;
		this->m_Bse = rhs.m_Bse;
		this->m_Bbt = rhs.m_Bbt;
		this->m_C = rhs.m_C;
		this->m_r = rhs.m_r;
		this->m_g = rhs.m_g;
		this->m_f = rhs.m_f;
		this->m_y = rhs.m_y;
		this->m_z = rhs.m_z;
		this->m_zh = rhs.m_zh;
		this->m_time = std::make_unique<TimeManager<N>>(rhs.m_z, rhs.m_zh, rhs.m_alpha, rhs.m_dt);
		this->m_ODE = std::make_unique<ODESystem<N, numTendons>>();
		this->m_PDE = std::make_unique<PDESystem<N, numTendons>>();
	}

	return *this;
}

// move assignment operator
template <std::size_t N, std::size_t numTendons>
TendonDriven<N, numTendons> &TendonDriven<N, numTendons>::operator=(TendonDriven<N, numTendons> &&rhs) noexcept
{
	// handling self assignment
	if (this != &rhs)
	{
		this->m_E = rhs.m_E;
		this->m_G = rhs.m_G;
		this->m_rad = rhs.m_rad;
		this->m_mass = rhs.m_mass;
		this->m_L = rhs.m_L;
		this->m_alpha = rhs.m_alpha;
		this->m_dt = rhs.m_dt;
		this->m_tendonOffset = rhs.m_tendonOffset;
		this->m_tendonAngleOffset = rhs.m_tendonAngleOffset;
		this->m_A = rhs.m_A;
		this->m_rho = rhs.m_rho;
		this->m_Kse = std::move(rhs.m_Kse);
		this->m_Kbt = std::move(rhs.m_Kbt);
		this->m_J = std::move(rhs.m_J);
		this->m_BVPAccuracy = rhs.m_BVPAccuracy;
		this->m_tau = std::move(rhs.m_tau);
		this->m_Bse = std::move(rhs.m_Bse);
		this->m_Bbt = std::move(rhs.m_Bbt);
		this->m_C = std::move(rhs.m_C);
		this->m_r = std::move(rhs.m_r);
		this->m_g = std::move(rhs.m_g);
		this->m_f = std::move(rhs.m_f);
		this->m_y = std::move(rhs.m_y);
		this->m_z = std::move(rhs.m_z);
		this->m_zh = std::move(rhs.m_zh);
		this->m_time = std::move(rhs.m_time);
		this->m_ODE = std::move(rhs.m_ODE);
		this->m_PDE = std::move(rhs.m_PDE);
	}

	return *this;
}

// returns the residue (violation of boundary condition) for a particular initial guess
template <std::size_t N, std::size_t numTendons>
template <mathOp::cosseratEquations solType, mathOp::integrationMethod intMethod>
blaze::StaticVector<double, 6UL> TendonDriven<N, numTendons>::residueFunction(const blaze::StaticVector<double, 6UL> &initGuess)
{
	// initGuess := [internal force n(0), internal moment m(0), tendon_lengths]
	blaze::StaticVector<double, 3UL> v0, u0, e3 = {0.00, 0.00, 1.00};

	v0 = e3 + blaze::inv(this->m_Kse) * blaze::subvector<0UL, 3UL>(initGuess); // v = v* + K^(-1)*R^T*n
	u0 = blaze::inv(this->m_Kbt) * blaze::subvector<3UL, 3UL>(initGuess);	   // u = u* + K^(-1)*R^T*m

	// initial conditions for numerical integration
	blaze::StaticVector<double, 19UL + numTendons> dyds;
	blaze::subvector<0UL, 19UL>(dyds) = {
		0.00, 0.00, 0.00,		   // initial position
		1.00, 0.00, 0.00, 0.00,	   // initial orientation (the 1-0-0-0 quaternion => identity matrix)
		v0[0UL], v0[1UL], v0[2UL], // initial strain
		u0[0UL], u0[1UL], u0[2UL], // initial curvature
		0.00, 0.00, 0.00,		   // initial linear velocity q_s
		0.00, 0.00, 0.00		   // initial angular velocity w_s
	};

	blaze::subvector<19UL, numTendons>(dyds) = blaze::uniform(numTendons, 0.00); // initial tendon length

	// pass the initial condition vector to the integration methods
	blaze::column<0UL>(this->m_y) = dyds;

	// numerical integration of the model equations (handles static or dynamic solutions)
	switch (solType)
	{
	case mathOp::cosseratEquations::STATIC_SOLUTION:
		switch (intMethod)
		{
		case mathOp::integrationMethod::RK4:
			this->m_ODE->rungeKutta_ODE(this->m_y, dyds, this->m_z, this->m_L, this->m_Kse, this->m_Kbt, this->m_r, this->m_tau, this->m_A, this->m_rho, this->m_g);
			break;
		case mathOp::integrationMethod::EULER:
			this->m_ODE->euler_ODE(this->m_y, dyds, this->m_z, this->m_L, this->m_Kse, this->m_Kbt, this->m_r, this->m_tau, this->m_A, this->m_rho, this->m_g);
			break;
		default:
			std::cerr << "Integration method not selected or not supported!" << std::endl;
			exit(0);
			break;
		}
		break;

	case mathOp::cosseratEquations::DYNAMIC_SOLUTION:
		switch (intMethod)
		{
		case mathOp::integrationMethod::RK4:
			this->m_PDE->rungeKutta_PDE(this->m_y, dyds, this->m_zh, this->m_z, this->m_L, this->m_time->getC0(), this->m_Kse, this->m_Kbt, this->m_Bse, this->m_Bbt, this->m_C, this->m_J, this->m_r, this->m_tau, this->m_A, this->m_rho, this->m_g);
			break;
		case mathOp::integrationMethod::EULER:
			this->m_PDE->euler_PDE(this->m_y, dyds, this->m_zh, this->m_z, this->m_L, this->m_time->getC0(), this->m_Kse, this->m_Kbt, this->m_Bse, this->m_Bbt, this->m_C, this->m_J, this->m_r, this->m_tau, this->m_A, this->m_rho, this->m_g);
			break;
		}
		break;

	default:
		std::cerr << "Integration method not selected or not supported!" << std::endl;
		exit(0);
		break;
	}

	// variables for computing the internal loading at the distal end
	blaze::StaticVector<double, 3UL> vL, uL, vL_t, uL_t;
	vL = blaze::subvector<7UL, 3UL>(blaze::column<N - 1>(this->m_y));
	uL = blaze::subvector<10UL, 3UL>(blaze::column<N - 1>(this->m_y));

	switch (solType)
	{
	case mathOp::cosseratEquations::STATIC_SOLUTION:
		uL_t = 0.00;
		vL_t = 0.00;
		break;
	case mathOp::cosseratEquations::DYNAMIC_SOLUTION:
		double c0 = this->m_time->getC0();
		vL_t = c0 * vL + blaze::subvector<0UL, 3UL>(blaze::column<N - 1>(this->m_zh));
		uL_t = c0 * uL + blaze::subvector<3UL, 3UL>(blaze::column<N - 1>(this->m_zh));
		break;
	}

	blaze::StaticVector<double, 3UL> nb, mb;
	nb = this->m_Kse * (vL - e3) + this->m_Bse * vL_t; // internal force at distal end
	mb = this->m_Kbt * uL + this->m_Bbt * uL_t;		   // internal moment at distal end

	// computing the equilibrium error
	blaze::StaticVector<double, 3UL> forceError(-nb), momentError(-mb), pbs_i, Fb_i;
	for (size_t idx = 0; idx < numTendons; ++idx)
	{
		pbs_i = blaze::cross(uL, this->m_r[idx]) + vL;
		Fb_i = -this->m_tau[idx] * blaze::normalize(pbs_i);
		forceError += Fb_i;
		momentError += blaze::cross(this->m_r[idx], Fb_i);
	}

	// account for the external force
	forceError += this->m_f;

	// computing the residue at the distal end
	blaze::StaticVector<double, 6UL> residue;
	blaze::subvector<0UL, 3UL>(residue) = std::move(forceError);
	blaze::subvector<3UL, 3UL>(residue) = std::move(momentError);

	return residue;
}

// function that computes the finite-differences Jacobian for solving the BVP
template <std::size_t N, std::size_t numTendons>
template <mathOp::cosseratEquations solType, mathOp::integrationMethod intMethod>
blaze::StaticMatrix<double, 6UL, 6UL> TendonDriven<N, numTendons>::jac_BVP(const blaze::StaticVector<double, 6UL> &initGuess, const blaze::StaticVector<double, 6UL> &residue)
{
	const size_t sz = 6UL;
	blaze::StaticMatrix<double, sz, sz, blaze::columnMajor> jac_bvp;

	blaze::StaticVector<double, sz> initGuessPerturbed(initGuess), residuePerturbed, scaled(initGuess);
	double incr_scale = 1.00E-7, incr_floor = 1.00E-9; // 1.00E-7, incr_floor = 1.00E-9;

	scaled *= incr_scale;
	scaled = blaze::generate(sz, [&](size_t idx)
							 { return (std::fabs(scaled[idx]) > incr_floor) ? scaled[idx] : incr_floor; });

	for (size_t iter = 0UL; iter < sz; ++iter)
	{
		initGuessPerturbed[iter] += scaled[iter];
		// perturbed residue
		residuePerturbed = this->residueFunction<solType, intMethod>(initGuessPerturbed);
		// building the finite-differences Residue jacobian
		blaze::column(jac_bvp, iter) = (residuePerturbed - residue) / scaled[iter];
		// restoring the original value of the array
		initGuessPerturbed[iter] = initGuess[iter];
	}

	return jac_bvp;
}

// manages which method to be used for solving the Boundary-value problem
template <std::size_t N, std::size_t numTendons>
template <mathOp::cosseratEquations solType, mathOp::rootFindingMethod method, mathOp::integrationMethod intMethod>
bool TendonDriven<N, numTendons>::solveBVP(blaze::StaticVector<double, 6UL> &initGuess)
{
	bool convergence; // flags the convergence (or lack thereof) of the selected nonlinear root-finding method

	switch (method)
	{
	case mathOp::rootFindingMethod::POWELL_DOG_LEG:
		convergence = this->PowellDogLeg<solType, intMethod>(initGuess);
		break;
	case mathOp::rootFindingMethod::LEVENBERG_MARQUARDT:
		convergence = this->Levenberg_Marquardt<solType, intMethod>(initGuess);
		break;
	case mathOp::rootFindingMethod::BROYDEN:
		convergence = this->Broyden<solType, intMethod>(initGuess);
		break;
	case mathOp::rootFindingMethod::NEWTON_RAPHSON:
		convergence = this->Newton_Raphson<solType, intMethod>(initGuess);
		break;
	case mathOp::rootFindingMethod::MODIFIED_NEWTON_RAPHSON:
		convergence = this->Modified_Newton_Raphson<solType, intMethod>(initGuess);
		break;
	}

	return convergence;
}

// function that increments time in the dynamics evolution problem
template <std::size_t N, std::size_t numTendons>
void TendonDriven<N, numTendons>::stepTime()
{
	// time step code goes in here
	this->m_time->stepTime(this->m_dt);
}

// function to set the external force at the distal end of the catheter
template <std::size_t N, std::size_t numTendons>
void TendonDriven<N, numTendons>::setExternalDistalForce(const blaze::StaticVector<double, 3UL> &force)
{
	this->m_f = force;
}

// function that sets the actuation value of joints ==> tendon displacements [m]
template <std::size_t N, std::size_t numTendons>
void TendonDriven<N, numTendons>::setTendonActuation(const blaze::StaticVector<double, numTendons> &q)
{
	this->m_tau = q;
}

// function that implements Powell's Dog Leg Method (Nonlinear root-finding method for solving the BVP)
template <std::size_t N, std::size_t numTendons>
template <mathOp::cosseratEquations solType, mathOp::integrationMethod intMethod>
bool TendonDriven<N, numTendons>::PowellDogLeg(blaze::StaticVector<double, 6UL> &initGuess)
{
	initGuess = blaze::map(initGuess, [](double d)
						   { return (!blaze::isfinite(d)) ? 0.00 : blaze::sign(d) * std::min(blaze::abs(d), 70.00); });

	bool found;
	const size_t sz = 6UL;
	size_t k = 0UL;
	const size_t k_max = 300UL;
	double alpha, beta, delta, eps1, eps2, rho, c;
	blaze::StaticVector<double, sz> g, f, f_new, x_new, h_sd, h_gn, h_dl;
	blaze::StaticMatrix<double, sz, sz> J;

	// initializing parameters
	delta = 1.00;
	eps1 = eps2 = 1.00E-25;

	f = this->residueFunction<solType, intMethod>(initGuess);
	J = this->jac_BVP<solType, intMethod>(initGuess, f);
	g = blaze::trans(J) * f;

	// checking if the initial guess satisfies the BVP without the need of any further refinement
	found = ((blaze::linfNorm(f) <= this->m_BVPAccuracy) || (blaze::linfNorm(g) <= eps1)) ? true : false;

	while (!found && (k < k_max))
	{
		k++;

		alpha = blaze::sqrNorm(g) / blaze::sqrNorm(J * g);
		h_sd = -alpha * g;				 // steepest descend (this is a direction, not a step!)
		h_gn = -mathOp::pInv<sz>(J) * f; // Gauss-Newton step (Least Square solution)

		// two candidates for the step to take from this point, a = alpha*h_sd & b = h_gn

		// computing the dog leg direction
		if (blaze::norm(h_gn) <= delta)
			h_dl = h_gn;
		else
		{
			if (blaze::norm(h_sd) >= delta)
				h_dl = delta * blaze::normalize(h_sd);
			else
			{
				c = blaze::trans(h_sd) * (h_gn - h_sd);

				if (c <= 0.00)
				{
					beta = (-c + sqrt(c * c + blaze::sqrNorm(h_gn - h_sd) * (delta * delta - blaze::sqrNorm(h_sd)))) / blaze::sqrNorm(h_gn - h_sd);
				}
				else
				{
					beta = (delta * delta - blaze::sqrNorm(h_sd)) / (c + sqrt(c * c + blaze::sqrNorm(h_gn - h_sd) * (delta * delta - blaze::sqrNorm(h_sd))));
				}

				h_dl = h_sd + beta * (h_gn - h_sd); // Dog Leg step
			}
		}

		if (blaze::norm(h_dl) <= eps2 * (blaze::norm(initGuess) + eps2))
			found = true;
		else
		{
			x_new = initGuess + h_dl;
			f_new = this->residueFunction<solType, intMethod>(x_new);
			rho = (blaze::sqrNorm(f) - blaze::sqrNorm(f_new)) / (0.5 * blaze::trans(h_dl) * ((delta * h_dl) - g));

			if (rho > 0.00)
			{
				initGuess = std::move(x_new);
				f = std::move(f_new);
				J = this->jac_BVP<solType, intMethod>(initGuess, f);
				g = blaze::trans(J) * f;

				if ((blaze::linfNorm(f) <= this->m_BVPAccuracy) || (blaze::linfNorm(g) <= eps1))
					found = true;
			}

			if (rho > 0.75)
				delta = std::max(delta, 3.00 * blaze::norm(h_dl));
			else
			{
				if (rho < 0.25)
					delta *= 0.50;
			}

			if (delta < eps2 * (blaze::norm(initGuess) + eps2))
				found = true;
		}
	}

	// if (!found)
	// std::cout << "Powell's Dog Leg, " << k << " iterations --> Residue = " << std::setprecision(12UL) << blaze::trans(f) << std::endl;

	return found;
}

// function that implements the Levenberg-Marquardt Method (Nonlinear root-finding method for solving the BVP)
template <std::size_t N, std::size_t numTendons>
template <mathOp::cosseratEquations solType, mathOp::integrationMethod intMethod>
bool TendonDriven<N, numTendons>::Levenberg_Marquardt(blaze::StaticVector<double, 6UL> &initGuess)
{
	initGuess = blaze::map(initGuess, [](double d)
						   { return (!blaze::isfinite(d)) ? 0.00 : blaze::sign(d) * std::min(blaze::abs(d), 70.00); });

	size_t k = 0UL;
	const size_t k_max = 300UL, sz = 6UL;
	blaze::StaticVector<double, sz> h, g, f, f_new;
	blaze::StaticMatrix<double, sz, sz> J, A;
	blaze::IdentityMatrix<double> I(sz);
	double rho, nu = 2.00, mu, tau = 1.00E-2, e1 = 1.00E-20, e2 = 1.00E-25;
	bool found;

	// computing the residue and residue Jacobian associated to initGuess
	f = this->residueFunction<solType, intMethod>(initGuess);
	J = this->jac_BVP<solType, intMethod>(initGuess, f);
	A = blaze::trans(J) * J;
	g = blaze::trans(J) * f;
	found = (blaze::linfNorm(g) <= e1) ? true : false;
	mu = tau * blaze::max(blaze::diagonal(A));

	// starting the iterative minimization loop
	while ((!found) && (k < k_max))
	{
		k++;
		blaze::solve(blaze::declsym(A + (mu * I)), h, -g);

		f_new = this->residueFunction<solType, intMethod>(initGuess + h);
		rho = (blaze::sqrNorm(f) - blaze::sqrNorm(f_new)) / (0.50 * blaze::trans(h) * ((mu * h) - g));

		if (rho > 0.00)
		{
			// accept the decrease in the function
			initGuess += h;
			// computing the residue Jacobian at the new initial guess
			J = this->jac_BVP<solType, intMethod>(initGuess, f_new);
			A = blaze::trans(J) * J;
			f = std::move(f_new);
			g = blaze::trans(J) * f;
			found = (blaze::linfNorm(g) <= e1) ? true : false;
			mu = mu * std::max(0.33333333, 1 - blaze::pow(2.00 * rho - 1.00, 3));
			nu = 2.00;
		}
		else
		{
			mu = mu * nu;
			nu = 2.00 * nu;
		}

		// checking if the tolerance has been satisfied
		if (blaze::linfNorm(f) <= this->m_BVPAccuracy)
			found = true;
	}

	// if (!found)
	// std::cout << "Levenberg-Marquardt, iterations: " << k << " --> Residue = " << std::setprecision(12UL) << blaze::trans(f);

	return found;
}

// function that implements the Levenberg-Marquardt Method (Nonlinear root-finding method for solving the BVP)
template <std::size_t N, std::size_t numTendons>
template <mathOp::cosseratEquations solType, mathOp::integrationMethod intMethod>
bool TendonDriven<N, numTendons>::Levenberg_Marquardt_II(blaze::StaticVector<double, 6UL> &initGuess)
{
	initGuess = blaze::map(initGuess, [](double d)
						   { return (!blaze::isfinite(d)) ? 0.00 : blaze::sign(d) * std::min(blaze::abs(d), 70.00); });

	size_t k = 0UL;
	const size_t k_max = 300UL, sz = 6UL;
	blaze::StaticVector<double, sz> r, rhs, pc;
	blaze::StaticMatrix<double, sz, sz> J, B;
	blaze::IdentityMatrix<double> I(sz);
	double E, Ec, damp = 1.00E-2, old_damp, adapt_coeff = 0.50;
	bool found = false;

	// computing the residue and residue Jacobian associated to initGuess
	r = this->residueFunction<solType, intMethod>(initGuess);
	E = blaze::linfNorm(r);
	if (E <= this->m_BVPAccuracy)
		return true;

	// starting the iterative minimization loop
	while ((E > this->m_BVPAccuracy) && (k < k_max))
	{
		k++;

		J = this->jac_BVP<solType, intMethod>(initGuess, r);
		B = blaze::trans(J) * J;
		rhs = -blaze::trans(J) * r;

		damp *= adapt_coeff;
		// lhs is symmetric, positive semi-definite, positive definite for damp > 0
		B += damp * I;
		blaze::solve(blaze::declsym(B), pc, rhs);

		r = this->residueFunction<solType, intMethod>(initGuess + pc);
		Ec = blaze::linfNorm(r);

		if (!blaze::isnan(r) && (Ec < E))
		{
			initGuess += pc;
			E = Ec;
		}
		else
		{
			while (blaze::isnan(r) || (Ec > E))
			{
				old_damp = damp;
				damp /= adapt_coeff;
				B += (damp - old_damp) * I;
				blaze::solve(blaze::declsym(B), pc, rhs);
				r = this->residueFunction<solType, intMethod>(initGuess + pc);
				Ec = blaze::linfNorm(r);
			}

			initGuess += pc;
			E = Ec;
		}
	}

	// std::cout << "Levenberg-Marquardt, iterations: " << k << " --> Residue = " << std::setprecision(12UL) << blaze::trans(r);

	found = (E <= this->m_BVPAccuracy) ? true : false;

	return found;
}

// function that implements Broyden's Nonlinear root-finding method for solving the BVP (Jacobian inverse is estimated)
template <std::size_t N, std::size_t numTendons>
template <mathOp::cosseratEquations solType, mathOp::integrationMethod intMethod>
bool TendonDriven<N, numTendons>::Broyden(blaze::StaticVector<double, 6UL> &initGuess)
{
	// found: returns true (false) when the root-finding method converges (does not converge) within k_max iterations
	bool found;

	const size_t sz = 6UL;

	// initial Hessian matrix --> computed via finite differences
	blaze::StaticMatrix<double, sz, sz> JacInv, JacInvNew;

	// setting up and starting my handmadeBFGS method
	blaze::StaticVector<double, sz> F, Fold, X, Xold, deltaX, deltaF; // staticVectors are automatically initialized to 0

	// Residue yielded by the initial guess for the CTR BVP
	F = this->residueFunction<solType, intMethod>(initGuess); // F(x_k)	: residue
	X = std::move(initGuess);								  // x_k		: initial guess
	JacInvNew = JacInv = mathOp::pInv<sz>(this->jac_BVP<solType, intMethod>(X, F));

	// checking if the initial guess already satisfies the BVP
	found = (blaze::linfNorm(F) <= this->m_BVPAccuracy) ? true : false;

	size_t k = 0UL;
	const size_t k_max = 300UL;
	while (!found && (k < k_max))
	{
		k++;

		deltaX = X - Xold; // dX := x_k - x_k-1
		deltaF = F - Fold; // dF := F(x_k) - F(x_k-1)

		JacInv = std::move(JacInvNew);
		if ((blaze::norm(deltaX) > 0.00) && (blaze::norm(deltaF) > 0.00))
			JacInvNew = JacInv + ((deltaX - JacInv * deltaF) / (blaze::trans(deltaX) * JacInv * deltaF)) * blaze::trans(deltaX) * JacInv;
		else
			JacInvNew = JacInv;

		Xold = std::move(X);
		Fold = std::move(F);

		// update the initial guess
		X = Xold - JacInv * F;
		F = this->residueFunction<solType, intMethod>(X);

		while (blaze::isnan(F))
		{
			if (blaze::isnan(X))
				X = 0.00;
			else
				X /= blaze::max(blaze::abs(X));

			F = this->residueFunction<solType, intMethod>(X);
			JacInv = JacInvNew = mathOp::pInv<sz>(this->jac_BVP<solType, intMethod>(X, F));
			Xold = std::move(X);
			X = Xold - JacInv * F;
		}

		if (k % 10 == 0.00)
		{
			JacInv = JacInvNew = mathOp::pInv<sz>(this->jac_BVP<solType, intMethod>(X, F));
			X = Xold - JacInv * F;
		}

		if (blaze::linfNorm(F) <= this->m_BVPAccuracy)
			found = true;
	}

	// std::cout << "Broyden finalized in " << k << " iterations. | Residue = " << std::setprecision(12UL) << blaze::trans(F);

	initGuess = std::move(X);
	return found;
}

// function that implements the Newton-Raphson method (Nonlinear root-finding method for solving the BVP)
template <std::size_t N, std::size_t numTendons>
template <mathOp::cosseratEquations solType, mathOp::integrationMethod intMethod>
bool TendonDriven<N, numTendons>::Newton_Raphson(blaze::StaticVector<double, 6UL> &initGuess)
{
	bool found;
	const size_t sz = 6UL;

	// setting up and starting my handmade Newton-Raphson method
	blaze::StaticVector<double, sz> Residue, Residue_new, d_Residue, dGuess; // staticVectors are automatically initialized to 0

	// Residue of the unperturbed initial guess for the CTR
	Residue = this->residueFunction<solType, intMethod>(initGuess);

	found = (blaze::linfNorm(Residue) <= this->m_BVPAccuracy) ? true : false;

	//  Weighing matrices for adjusting the initial guess iteratively (Implementing a PD regulator)
	blaze::DiagonalMatrix<blaze::StaticMatrix<double, sz, sz, blaze::rowMajor>> Kp, Kd;
	blaze::StaticMatrix<double, sz, sz, blaze::columnMajor> jac_bvp;
	blaze::diagonal(Kp) = 0.30;	   // 0.45 | 0.6  | 0.3
	blaze::diagonal(Kd) = 2.00E-3; // 3e-3 | 5e-3 | 2e-3

	size_t k = 0UL;
	const size_t k_max = 300UL;
	// starting iterations for adjusting the initial guess "u_guess ~ initGuess"
	while (!found && (k < k_max))
	{
		k++;
		jac_bvp = this->jac_BVP<solType, intMethod>(initGuess, Residue);
		// error equation(globally asymptotically stable)
		dGuess = mathOp::pInv<sz>(jac_bvp) * (Kp * Residue + Kd * d_Residue);
		// updating the initial guess(weighted negative gradient of the cost function)
		initGuess -= dGuess;
		// computing the new cost associated to the newly readjusted initial guess
		Residue_new = this->residueFunction<solType, intMethod>(initGuess);

		// Checking if the Jacobian has large elements beyond machine precision
		while (blaze::isnan(Residue_new))
		{
			initGuess = blaze::map(initGuess, [](double d)
								   { return (!blaze::isfinite(d)) ? 0.00 : blaze::sign(d) * std::min(blaze::abs(d), 70.00); });

			Residue_new = this->residueFunction<solType, intMethod>(initGuess);
			d_Residue = Residue_new - Residue;
			Residue = std::move(Residue_new);
			continue;
		}

		// cost variation due to initial guess refinement
		d_Residue = Residue_new - Residue;
		// updating the cost
		Residue = std::move(Residue_new);

		if (blaze::linfNorm(Residue) <= this->m_BVPAccuracy)
			found = true;
	}

	// if (!found)
	// std::cout << "Newton-Raphson, iterations: " << k << " --> Residue = " << std::setprecision(12UL) << blaze::trans(Residue);

	return found;
}

// function that implements the Modified, globally convergent Newton-Raphson method (Nonlinear root-finding method for solving the BVP)
template <std::size_t N, std::size_t numTendons>
template <mathOp::cosseratEquations solType, mathOp::integrationMethod intMethod>
bool TendonDriven<N, numTendons>::Modified_Newton_Raphson(blaze::StaticVector<double, 6UL> &initGuess)
{
	/*
	Algorithm extracted from page 309 of Introduction to Numerical Analysis 3rd edition by Josef Stoer & Roland Bulirsch
	*/

	auto readjustInitialGuesses = [](blaze::StaticVector<double, 6UL> &initial_guesses)
	{
		initial_guesses = blaze::map(initial_guesses, [](double d)
									 { return (!blaze::isfinite(d)) ? 0.00 : blaze::sign(d) * std::min(blaze::abs(d), 70.00); });
	};

	readjustInitialGuesses(initGuess);

	bool found;
	const size_t sz = 6UL;
	// computes the residue associated to the initial guess
	blaze::StaticVector<double, sz> f, d;
	blaze::StaticVector<double, sz, blaze::rowVector> Dh;
	blaze::StaticMatrix<double, sz, sz> D, D_inv;
	double h, h_0, lambda, gamma, improvementFactor, d_norm, Dh_norm;

	f = this->residueFunction<solType, intMethod>(initGuess);
	size_t j = 0UL, k = 0UL;
	const size_t k_max = 300UL;
	std::vector<double> h_k; // vector to store all h_k's
	h_k.reserve(k_max);

	found = (blaze::linfNorm(f) <= this->m_BVPAccuracy) ? true : false;

	auto setupMethod = [&]() -> void
	{
		// computing the residue Jacobian
		D = this->jac_BVP<solType, intMethod>(initGuess, f);

		// verifies NAN in the Jacobian and refines initial guess if necessary
		while (!blaze::isfinite(D))
		{
			initGuess *= 0.75;
			readjustInitialGuesses(initGuess);
			// then recomputes the residue
			f = this->residueFunction<solType, intMethod>(initGuess);
			// computing the residue Jacobian
			D = this->jac_BVP<solType, intMethod>(initGuess, f);
		}

		D_inv = mathOp::pInv<sz>(D);

		// search direction (directional derivative)
		d = D_inv * f;
		gamma = 1.00 / (blaze::norm(D_inv) * blaze::norm(D)); // gamma := 1/cond(Df)
		h_0 = blaze::sqrNorm(f);							  // h := f'f
		// Dh := D(f'f) = 2f'Df
		Dh = 2.00 * blaze::trans(f) * D;
		d_norm = blaze::norm(d);
		Dh_norm = blaze::norm(Dh);
	};

	while (!found && (k < k_max))
	{
		k++;
		setupMethod();

		while (true)
		{
			f = this->residueFunction<solType, intMethod>(initGuess - blaze::pow(0.50, j) * d);
			// std::cout << "Modified_Newton_Raphson -- j = : " << j << " | residue = " << blaze::trans(f);
			while (!blaze::isfinite(f))
			{
				j++;
				f = this->residueFunction<solType, intMethod>(initGuess - blaze::pow(0.50, j) * d);
				if (j > 20UL)
				{
					j = 0UL;
					// limits the values of |u1_z(0)| and |u2_z(0)| to avoid numerical instability and lack of convergence
					initGuess *= 0.75;
					readjustInitialGuesses(initGuess);
					setupMethod();
				}
			}
			h = blaze::sqrNorm(f);
			improvementFactor = blaze::pow(0.50, j) * 0.25 * gamma * d_norm * Dh_norm;
			// storig the value of h_k to determine step size posteriorly
			h_k.push_back(h);

			if (h <= (h_0 - improvementFactor))
				break;
			else
				j++;
		}

		// retrieving the minimum h_k ==> h_k is monotonically decreasing (just grab its last element)
		lambda = blaze::pow(0.50, h_k.size() - 1UL);
		initGuess -= lambda * d;
		h_k.clear();

		// resets the exponent variable j
		j = 0UL;

		// checking the terminating condition
		if (blaze::linfNorm(f) <= this->m_BVPAccuracy)
		{
			found = true;
		}
	}

	if (!found)
	{
		readjustInitialGuesses(initGuess);
		// std::cout << "Trying PowellDogLeg()" << std::endl;
		found = this->PowellDogLeg<solType, intMethod>(initGuess);

		if (!found)
		{
			readjustInitialGuesses(initGuess);
			// std::cout << "Trying Levenberg-Marquardt()" << std::endl;
			found = this->Levenberg_Marquardt<solType, intMethod>(initGuess);
		}
	}

	// std::cout << "Finished Modified N-R. Iteration: " << k << " Residue = " << blaze::trans(f) << std::endl;

	return found;
}

// getter method for returning the shear / extension viscous damping
template <std::size_t N, std::size_t numTendons>
blaze::DiagonalMatrix<blaze::StaticMatrix<double, 3UL, 3UL>> TendonDriven<N, numTendons>::get_Bse()
{
	return this->m_Bse;
}

// getter method for returning the bending / torsion viscous damping
template <std::size_t N, std::size_t numTendons>
blaze::DiagonalMatrix<blaze::StaticMatrix<double, 3UL, 3UL>> TendonDriven<N, numTendons>::get_Bbt()
{
	return this->m_Bbt;
}

// getter method for returning the viscuous "air-drag" coefficient stiffness
template <std::size_t N, std::size_t numTendons>
blaze::DiagonalMatrix<blaze::StaticMatrix<double, 3UL, 3UL>> TendonDriven<N, numTendons>::get_C()
{
	return this->m_C;
}

// getter method for returning the shear / extension stiffness stiffness
template <std::size_t N, std::size_t numTendons>
blaze::DiagonalMatrix<blaze::StaticMatrix<double, 3UL, 3UL>> TendonDriven<N, numTendons>::get_Kse()
{
	return this->m_Kse;
}

// getter method for returning the bending / torsion stiffness stiffness
template <std::size_t N, std::size_t numTendons>
blaze::DiagonalMatrix<blaze::StaticMatrix<double, 3UL, 3UL>> TendonDriven<N, numTendons>::get_Kbt()
{
	return this->m_Kbt;
}

// getter method for returning the actuation inputs
template <std::size_t N, std::size_t numTendons>
blaze::StaticVector<double, numTendons> TendonDriven<N, numTendons>::get_tau()
{
	return this->m_tau;
}

// getter method for returning the actuation inputs
template <std::size_t N, std::size_t numTendons>
double TendonDriven<N, numTendons>::get_tau(std::size_t i)
{
	return this->m_tau[i];
}

// getter method for returning the radial positions of the tendons
template <std::size_t N, std::size_t numTendons>
std::array<blaze::StaticVector<double, 3UL>, numTendons> TendonDriven<N, numTendons>::get_r()
{
	return this->m_r;
}

// getter method for returning the radial positions of the i-th tendon
template <std::size_t N, std::size_t numTendons>
blaze::StaticVector<double, 3UL> TendonDriven<N, numTendons>::get_r(std::size_t i)
{
	return this->m_r[i];
}

// getter method for returning the linear density of the robot
template <std::size_t N, std::size_t numTendons>
double TendonDriven<N, numTendons>::get_rho()
{
	return this->m_rho;
}

// getter method for returning the sheath's cross-sectional area
template <std::size_t N, std::size_t numTendons>
double TendonDriven<N, numTendons>::get_A()
{
	return this->m_A;
}

// getter method for returning the acceleration of gravity
template <std::size_t N, std::size_t numTendons>
blaze::StaticVector<double, 3UL> TendonDriven<N, numTendons>::get_g()
{
	return this->m_g;
}

// method for setting and initializing the time discretization variables after the initial static condition
template <std::size_t N, std::size_t numTendons>
void TendonDriven<N, numTendons>::setTimeDiscretization()
{
	this->m_time->setInitialConditions();
}

// method for returning the shape of the tendon-driven robot
template <std::size_t N, std::size_t numTendons>
blaze::StaticMatrix<double, 3UL, N> TendonDriven<N, numTendons>::getBackboneShape()
{
	return blaze::submatrix<0UL, 0UL, 3UL, N>(this->m_y);
}

// method for returning the tip position of the tendon-driven robot
template <std::size_t N, std::size_t numTendons>
blaze::StaticVector<double, 3UL> TendonDriven<N, numTendons>::getTipPosition()
{
	return blaze::subvector<0UL, 3UL>(blaze::column<N - 1>(this->m_y));
}

// method for returning the distal tip position & orientation of the tendon-driven robot
template <std::size_t N, std::size_t numTendons>
std::tuple<blaze::StaticVector<double, 3UL>, blaze::StaticVector<double, 3UL>> TendonDriven<N, numTendons>::getTipPose()
{
	blaze::StaticVector<double, 3UL> position, orientation;
	blaze::StaticVector<double, 4UL> h;
	blaze::StaticMatrix<double, 3UL, 3UL> R;
	position = blaze::subvector<0UL, 3UL>(blaze::column<N - 1>(this->m_y));
	// quaternion at the distal end
	h = blaze::subvector<3UL, 4UL>(blaze::column<N - 1>(this->m_y));
	mathOp::getSO3(h, R);
	orientation = blaze::column<2UL>(R);

	return std::make_tuple(position, orientation);
}

// method that sets the optimization parameters for the static model
template <std::size_t N, std::size_t numTendons>
void TendonDriven<N, numTendons>::setOptimizingStaticParameters(const double EI,
																const double nu,
																const double radius,
																const double tendonCompliance,
																const double tendonAngle)
{
	this->m_rad = radius;
	// cross-sectional area
	this->m_A = M_PI * pow(this->m_rad, 2);
	// moment of area
	double I = M_PI_4 * pow(this->m_rad, 4);
	// linear mass density
	this->m_rho = this->m_mass / (this->m_L * this->m_A);

	this->m_E = EI / I;
	this->m_G = this->m_E / (2.00 * (1.00 + nu));
	this->m_tendonCompliance = blaze::uniform(numTendons, tendonCompliance);
	this->m_tendonAngleOffset = tendonAngle;

	// Shear and extension bending stiffness matrix
	this->m_Kse(0UL, 0UL) = this->m_Kse(1UL, 1UL) = this->m_G * this->m_A;
	this->m_Kse(2UL, 2UL) = this->m_E * this->m_A;

	// Bending and twisting bending stiffness matrix
	this->m_Kbt(0UL, 0UL) = this->m_Kbt(1UL, 1UL) = EI;
	this->m_Kbt(2UL, 2UL) = this->m_G * (I + I);

	// temporarly switch with static vecotor
	// blaze::DynamicVector<double, blaze::rowVector> angles;
	// angles = blaze::linspace<blaze::rowVector>(numTendons + 1, 0.00, 2.00 * M_PI);

	blaze::StaticVector<double, numTendons, blaze::rowVector> angles;
	// Manually filling the static vector with linearly spaced values
	for(size_t i = 0; i < numTendons; ++i) {
		angles[i] = i * (2.00 * M_PI / numTendons);
	}

	for (size_t idx = 0UL; idx < numTendons; ++idx)
	{
		this->m_r[idx][0UL] = m_tendonOffset * cos(angles[idx] + m_tendonAngleOffset);
		this->m_r[idx][1UL] = m_tendonOffset * sin(angles[idx] + m_tendonAngleOffset);
		this->m_r[idx][2UL] = 0.00;

		// eliminates small numbers (trash) from "zeros" entries of the radial offset vector
		this->m_r[idx] = blaze::map(this->m_r[idx], [](double d)
									{ return (std::fabs(d) < 1.00E-5) ? 0.00 : d; });
	}
}

// method that returns the optimizig static parameters
template <std::size_t N, std::size_t numTendons>
std::tuple<double, double, double, double, double> TendonDriven<N, numTendons>::getOptimizingStaticParameters()
{
	double EI = this->m_E * M_PI_4 * pow(this->m_rad, 4);
	double nu = this->m_E / (2.00 * this->m_G) - 1.00;

	return std::make_tuple(EI, nu, this->m_rad, this->m_tendonCompliance[0UL], this->m_tendonAngleOffset);
}

// method that sets the optimization parameters for the dynamic model
template <std::size_t N, std::size_t numTendons>
void TendonDriven<N, numTendons>::setOptimizingDynamicParameters(const double E,
																 const double radius,
																 const double mass)
{
	// computing the Poisson's ratio
	double nu = m_E / (2.00 * m_G) - 1.00;

	this->m_E = E;
	this->m_G = m_E / (2.00 * (1.00 + nu));

	this->m_rad = radius;
	this->m_mass = mass;

	// updates the other terms which are a function of the radius
	this->m_A = M_PI * pow(this->m_rad, 2); // cross-sectional area
	const double I = M_PI_4 * pow(this->m_rad, 4); // second moment of area
	this->m_rho = this->m_mass / (this->m_L * this->m_A);	

	// Shear and extension bending stiffness matrix
	this->m_Kse(0UL, 0UL) = this->m_Kse(1UL, 1UL) = this->m_G * this->m_A;
	this->m_Kse(2UL, 2UL) = this->m_E * this->m_A;
	// Bending and twisting bending stiffness matrix
	this->m_Kbt(0UL, 0UL) = this->m_Kbt(1UL, 1UL) = E *I;
	this->m_Kbt(2UL, 2UL) = this->m_G * (I + I);
	// Moment of Inertia
	this->m_J(0UL, 0UL) = this->m_J(1UL, 1UL) = I;
	this->m_J(2UL, 2UL) = I + I;
}

// method that returns the optimizig dynamic parameters
template <std::size_t N, std::size_t numTendons>
std::tuple<double, double, double> TendonDriven<N, numTendons>::getOptimizingDynamicParameters()
{
	return std::make_tuple(this->m_E, this->m_rad, this->m_mass);
}

// method that reset all containers prior to running sequences of static simulations
template <std::size_t N, std::size_t numTendons>
void TendonDriven<N, numTendons>::resetStaticSimulation()
{
	// blaze::column<0UL>(m_y) = 0.00;
	// blaze::column<0UL>(m_z) = 0.00;
	// blaze::column<0UL>(m_zh) = 0.00;
	blaze::reset(this->m_y);
	blaze::reset(this->m_z);
	blaze::reset(this->m_zh);
}

// method that reset all containers prior to running sequences of dynamic simulations
template <std::size_t N, std::size_t numTendons>
void TendonDriven<N, numTendons>::resetDynamicSimulation()
{
	this->m_time->resetTime();
}


// function that returns the data to be saved for NN training purposes
template <std::size_t N, std::size_t numTendons>
std::tuple<blaze::StaticVector<double, 19UL + numTendons>, blaze::StaticVector<double, numTendons>> TendonDriven<N, numTendons>::getTrainingData()
{
    blaze::StaticVector<double, 19UL + numTendons> states;
	blaze::StaticVector<double, numTendons> inputs;

    // double c0 = this->m_time->getC0();

    // acquiring the values of y
    subvector(states, 0UL, 19UL + numTendons) = blaze::column<N - 1>(this->m_y);
	subvector(inputs, 0UL, numTendons) = this->m_tau;

    return std::make_tuple(states, inputs);
}

