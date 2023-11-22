// This is a personal academic project. Dear PVS-Studio, please check it.
// PVS-Studio Static Code Analyzer for C, C++, C#, and Java: http://www.viva64.com

#include "CTR.hpp"

// overloaded class constructor
CTR::CTR(const std::array<std::shared_ptr<Tube>, 3UL> &Tb, blaze::StaticVector<double, 6UL> &q, double Tol, mathOp::rootFindingMethod method) : m_accuracy(Tol), m_method(method), m_Tubes(Tb), m_beta(blaze::subvector<0UL, 3UL>(q)), m_q(q)
{
	this->m_theta_0 = {0.00, q[4UL] - q[3UL], q[5UL] - q[4UL]};
	this->m_e3 = {0.00, 0.00, 1.00};
	this->m_segment = std::make_unique<Segment>(this->m_Tubes, this->m_beta);
	this->m_h_0 = {1.00, 0.00, 0.00, 0.00};
	this->m_wf = {0.00, 0.00, 0.00};
	this->m_wm = {0.00, 0.00, 0.00};

	this->m_stateEquations = std::make_unique<ODESystem>();
	this->m_stateObserver = std::make_unique<Observer>(this->m_y, this->m_s);
	this->objFuncStr = std::make_shared<objFuncData>();
	this->constrFuncStr = std::make_shared<constrFuncData>();
	this->m_EMTrack = std::make_unique<EMTracker>();
	this->m_EMTrack->Start_Read_Thread();
}

// copy constructor
CTR::CTR(const CTR &rhs) : m_accuracy(rhs.m_accuracy), m_method(rhs.m_method), m_Tubes(rhs.m_Tubes), m_beta(rhs.m_beta),
						   m_q(rhs.m_q), m_theta_0(rhs.m_theta_0), m_e3(rhs.m_e3), m_h_0(rhs.m_h_0), m_y(rhs.m_y), m_s(rhs.m_s),
						   m_wf(rhs.m_wf), m_wm(rhs.m_wm)
{
	this->m_segment = std::make_unique<Segment>(this->m_Tubes, this->m_beta);
	this->m_stateEquations = std::make_unique<ODESystem>();
	this->m_stateObserver = std::make_unique<Observer>(this->m_y, this->m_s);
	this->objFuncStr = std::make_shared<objFuncData>();
	this->constrFuncStr = std::make_shared<constrFuncData>();
	this->m_EMTrack = std::make_unique<EMTracker>();
}

// move constructor
CTR::CTR(CTR &&rhs) noexcept
{
	// handling self assignment
	if (this != &rhs)
	{
		this->m_accuracy = rhs.m_accuracy;
		this->m_method = rhs.m_method;
		this->m_Tubes = std::move(rhs.m_Tubes);
		this->m_beta = std::move(rhs.m_beta);
		this->m_q = std::move(rhs.m_q);
		this->m_theta_0 = std::move(rhs.m_theta_0);
		this->m_wf = std::move(rhs.m_wf);
		this->m_wm = std::move(rhs.m_wm);
		this->m_e3 = std::move(rhs.m_e3);
		this->m_segment = std::move(rhs.m_segment);
		this->m_h_0 = std::move(rhs.m_h_0);
		this->m_y = std::move(rhs.m_y);
		this->m_s = std::move(rhs.m_s);
		this->m_stateEquations = std::move(rhs.m_stateEquations);
		this->m_stateObserver = std::move(rhs.m_stateObserver);
		this->objFuncStr = std::move(rhs.objFuncStr);
		this->constrFuncStr = std::move(rhs.constrFuncStr);
		this->m_EMTrack = std::move(rhs.m_EMTrack);
	}
}

// copy assignment operator
CTR &CTR::operator=(const CTR &rhs)
{
	// handling self assignment
	if (this != &rhs)
	{
		this->m_accuracy = rhs.m_accuracy;
		this->m_method = rhs.m_method;
		this->m_Tubes = rhs.m_Tubes;
		this->m_beta = rhs.m_beta;
		this->m_q = rhs.m_q;
		this->m_theta_0 = rhs.m_theta_0;
		this->m_wf = rhs.m_wf;
		this->m_wm = rhs.m_wm;
		this->m_e3 = rhs.m_e3;
		this->m_segment = std::make_unique<Segment>(this->m_Tubes, this->m_beta);
		this->m_h_0 = rhs.m_h_0;
		this->m_y = rhs.m_y;
		this->m_s = rhs.m_s;
		this->m_stateEquations = std::make_unique<ODESystem>();
		this->m_stateObserver = std::make_unique<Observer>(this->m_y, this->m_s);
		this->objFuncStr = std::make_shared<objFuncData>();
		this->constrFuncStr = std::make_shared<constrFuncData>();
		this->m_EMTrack = std::make_unique<EMTracker>();
	}

	return *this;
}

// move assignment operator
CTR &CTR::operator=(CTR &&rhs) noexcept
{
	// handling self assignment
	if (this != &rhs)
	{
		this->m_accuracy = rhs.m_accuracy;
		this->m_method = rhs.m_method;
		this->m_Tubes = std::move(rhs.m_Tubes);
		this->m_beta = std::move(rhs.m_beta);
		this->m_q = std::move(rhs.m_q);
		this->m_theta_0 = std::move(rhs.m_theta_0);
		this->m_wf = std::move(rhs.m_wf);
		this->m_wm = std::move(rhs.m_wm);
		this->m_e3 = std::move(rhs.m_e3);
		this->m_segment = std::move(rhs.m_segment);
		this->m_h_0 = std::move(rhs.m_h_0);
		this->m_y = std::move(rhs.m_y);
		this->m_s = std::move(rhs.m_s);
		this->m_stateEquations = std::move(rhs.m_stateEquations);
		this->m_stateObserver = std::move(rhs.m_stateObserver);
		this->objFuncStr = std::move(rhs.objFuncStr);
		this->constrFuncStr = std::move(rhs.constrFuncStr);
		this->m_EMTrack = std::move(rhs.m_EMTrack);
	}

	return *this;
}

// function that resets the initial parameters for the ODESolver
void CTR::reset(const blaze::StaticVector<double, 5UL> &initGuess)
{
	blaze::StaticVector<double, 3UL> uz_0 = {initGuess[2UL], initGuess[3UL], initGuess[4UL]};
	// alpha1_0 =  alpha_1 - beta_1 * uz_1(0)
	double alpha1_0 = this->m_q[3UL] - this->m_beta[0UL] * uz_0[0UL];

	// clearing the observer's containers
	if (!this->m_s.empty())
	{
		this->m_y.clear();
		this->m_y.reserve(1000UL);
		this->m_s.clear();
		this->m_s.reserve(1000UL);
	}

	// theta_i(0) = alpha_1 - alpha_i - (beta_i * uz_i(0) - beta_1 * uz_1(0))
	this->m_theta_0 = {0.00,
					   this->m_q[4UL] - this->m_beta[1UL] * uz_0[1UL] - alpha1_0,
					   this->m_q[5UL] - this->m_beta[2UL] * uz_0[2UL] - alpha1_0};

	// transforming proximal orientation to quaternion representation
	mathOp::euler2Quaternion(0.00, alpha1_0, 0.00, this->m_h_0);
}

// function that solves (integrates) the CTR ode (state) equations
blaze::StaticVector<double, 5UL> CTR::ODESolver(const blaze::StaticVector<double, 5UL> &initGuess)
{
	// initGuess = [mb_x(0) mb_y(0) u1_z(0) u2_z(0) u3_z(0)]  --> vector of initial guesses for solving the BVP
	this->reset(initGuess); // resets CTR parameters and variables for a new iteration of the ode-solver

	// retrieving the bending & torsional stiffness and precurvatures in all segments of the CTR in the current
	auto [EI, GJ, U_x, U_y, S] = this->m_segment->returnParameters();

	// ##################################################### NUMERICAL METHODS FOR ODE INTEGRATION #####################################################

	// ********************************  8-th ORDER ADAPTIVE ADAMS-BASHFORTH-MOULTON STEPPER ********************************
	boost::numeric::odeint::adaptive_adams_bashforth_moulton<8UL, state_type, double, state_type, double,
															 boost::numeric::odeint::vector_space_algebra, boost::numeric::odeint::default_operations, boost::numeric::odeint::initially_resizer>
		abm8_stepper;

	// ********************************  4-th ORDER CLASSIC RUNGE-KUTTA STEPPER ********************************
	// typedef boost::numeric::odeint::runge_kutta4_classic<state_type, double, state_type, double, boost::numeric::odeint::vector_space_algebra,
	// 	boost::numeric::odeint::default_operations, boost::numeric::odeint::initially_resizer> rk4_stepper;

	// ********************************  5-th ORDER CASH-KARP STEPPER ********************************
	// typedef boost::numeric::odeint::runge_kutta_cash_karp54<state_type, double, state_type, double, boost::numeric::odeint::vector_space_algebra,
	// 	boost::numeric::odeint::default_operations, boost::numeric::odeint::initially_resizer> rkk54_stepper;

	// ********************************  5-th ORDER DORMAND-PRINCE RUNGE-KUTTA ********************************
	// typedef boost::numeric::odeint::runge_kutta_dopri5<state_type, double, state_type, double, boost::numeric::odeint::vector_space_algebra> rkd5_stepper;

	// ********************************  BULIRSCH-STOER STEPPER ********************************
	// typedef boost::numeric::odeint::bulirsch_stoer<state_type, double, state_type, double, boost::numeric::odeint::vector_space_algebra,
	// 	boost::numeric::odeint::default_operations, boost::numeric::odeint::initially_resizer> blstr_stepper;

	// ******************************** RUUNGE-KUTTA-FEHLBERG (RKF78) STEPPER ********************************
	// typedef boost::numeric::odeint::runge_kutta_fehlberg78<state_type, double, state_type, double, boost::numeric::odeint::vector_space_algebra,
	// 	boost::numeric::odeint::default_operations, boost::numeric::odeint::initially_resizer> rk78_stepper;

	// #################################################################################################################################################

	// start and end points, in terms of arc-length s, of each CTR segment and initial step-size for integration (ds)
	double s_start, s_end, ds;

	// instantiating the vector of initial conditions for solving the state equations (15 x 1)
	state_type y_0;

	/*
	 *****************************************************************************
	 * ========== initializing the initial conditions vector (15 x 1) ========== *
	 *****************************************************************************
	 */

	// 1st and 2nd element of y are the bending moments in the sheath along the x, y directions (in the local/body frame)
	y_0[0UL] = initGuess[0UL];
	y_0[1UL] = initGuess[1UL];
	// 3rd-5th elements of y are the torsional curvatures for the three tubes, e.g., y = [u1_z  u2_z  u3_z]
	y_0[2UL] = initGuess[2UL];
	y_0[3UL] = initGuess[3UL];
	y_0[4UL] = initGuess[4UL];
	// 6th-8th elements of y are twist angles, theta_i = [theta_1  theta_2  theta_3]
	y_0[5UL] = this->m_theta_0[0UL];
	y_0[6UL] = this->m_theta_0[1UL];
	y_0[7UL] = this->m_theta_0[2UL];
	// 9th-11th elements of y define the origin of the local frame propagates along the backbone at each arc-lengh 0 < s < L
	y_0[8UL] = 0.00;
	y_0[9UL] = 0.00;
	y_0[10UL] = 0.00;
	// 12th-15th elements of y are the (quaternion-orientations) of the local frame at each arc-length s
	y_0[11UL] = this->m_h_0[0UL];
	y_0[12UL] = this->m_h_0[1UL];
	y_0[13UL] = this->m_h_0[2UL];
	y_0[14UL] = this->m_h_0[3UL];

	// iterating through the tube segments comprising the CTR
	size_t len_seg = S.size() - 1UL;
	for (size_t seg = 0; seg < len_seg; ++seg)
	{
		// specifying the interval of integration (in terms of tube segment arc-lengths)
		s_start = S[seg];
		s_end = S[seg + 1UL];
		ds = (s_end - s_start) / 25.00; // 25 points per segment

		// passing the tube parameters in the segment to the state equation method
		this->m_stateEquations->setEquationParameters(blaze::column(U_x, seg), blaze::column(U_y, seg), blaze::column(EI, seg), blaze::column(GJ, seg), this->m_wf);

		// ##################################################### NUMERICAL INTEGRATION #####################################################
		// Employs the selected stepper (Numerical method) and integrates the system of ODEs along the segment considered
		boost::numeric::odeint::integrate_adaptive(abm8_stepper, *this->m_stateEquations, y_0, s_start, s_end, ds, *this->m_stateObserver);
	}

	//
	//	****************  #####  -------------- ___ DISTAL BOUNDARY CONDITIONS ___ --------------  #####  ****************
	//			1) internal moment at the tip of tube 1 must equal the external moment applied
	//			   Namely, mb_xy(L1) - (R1'L)_xy = 0
	//
	//			2) at the distal ends of the remaining tubes, the axial component of the internal moments must equal zero
	//			   Namely, ui_z(Li) - ui_z*(Li) = 0

	blaze::StaticMatrix<double, 3UL, 3UL, blaze::columnMajor> R1;

	// grabbing the orientation at the distal end
	mathOp::getSO3(blaze::subvector<11UL, 4UL>(y_0), R1);

	blaze::StaticVector<double, 3UL> distalMoment = blaze::trans(R1) * m_wm;

	// Residue vector due to infringment of the distal boundary conditions || Residue = [ mb_x - Wm_x, mb_y - Wm_y, u1_z, u2_z, u3_z ]
	blaze::StaticVector<double, 5UL> Residue = {y_0[0UL] - distalMoment[0UL],
												y_0[1UL] - distalMoment[1UL],
												GJ(0UL, 0UL) * y_0[2UL] - blaze::trans(m_e3) * distalMoment,
												0.00,
												0.00};

	// lambda function that finds the u_z curvatures at the distal ends of tubes 2 and 3
	auto computeResidue = [&](double distalEnd, size_t index) -> void
	{
		// must use some tolerance when comparing floating points
		auto itt = std::lower_bound(this->m_s.begin(), this->m_s.end(), distalEnd - 1.00E-7); // finds where tube ends (with a 0.0001mm tolerance)

		auto id = std::distance(this->m_s.begin(), itt);
		// ui_z at the distal end of the i-th tube
		Residue[2UL + index] = this->m_y[id][2UL + index];
	};

	// Computing the Residues associated to the twist curvatures (rate of twist) at the distal ends of tubes 2 and 3
	blaze::StaticVector<double, 3UL> distEnd(this->m_segment->getDistalEnds()); // arc-lengths at which the distal ends of the tubes are currently

	computeResidue(distEnd[1UL], 1UL);
	computeResidue(distEnd[2UL], 2UL);

	return Residue;
}

// function that computes the finite-differences Jacobian for solving the BVP
blaze::StaticMatrix<double, 5UL, 5UL> CTR::jac_BVP(const blaze::StaticVector<double, 5UL> &initGuess, const blaze::StaticVector<double, 5UL> &residue)
{
	blaze::StaticMatrix<double, 5UL, 5UL, blaze::columnMajor> jac_bvp;

	blaze::StaticVector<double, 5UL> initGuessPerturbed(initGuess), residuePerturbed, scaled(initGuess);
	double incr_scale = 1.00E-7, incr_floor = 1.00E-9; // 1.00E-7, incr_floor = 1.00E-9 ==>> SEEM TO BE OPTIMAL;

	scaled *= incr_scale;
	scaled = blaze::generate(5UL, [&](size_t idx)
							 { return (std::fabs(scaled[idx]) > incr_floor) ? scaled[idx] : incr_floor; });

	for (size_t iter = 0UL; iter < 5UL; ++iter)
	{
		initGuessPerturbed[iter] += scaled[iter];
		// perturbed residue
		residuePerturbed = this->ODESolver(initGuessPerturbed);
		// building the finite-differences Residue jacobian
		blaze::column(jac_bvp, iter) = (residuePerturbed - residue) / scaled[iter];
		// restoring the original value of the array
		initGuessPerturbed[iter] = initGuess[iter];
	}

	return jac_bvp;
}

// function that computes the finite-differences Jacobian wrt actuation inputs
blaze::StaticMatrix<double, 3UL, 6UL> CTR::jacobian(const blaze::StaticVector<double, 5UL> &initGuess, const blaze::StaticVector<double, 3UL> &tipPos)
{
	blaze::StaticMatrix<double, 3UL, 6UL, blaze::columnMajor> jac;
	blaze::StaticVector<double, 6UL> q_Original(this->m_q), q_Perturbed(this->m_q), q_Scaled(this->m_q);
	double incr_scale = 1.00E-2, incr_floor = 1.00E-5; // 1.00E-7, incr_floor ==>> 1.00E-9 SEEM TO BE OPTIMAL;

	q_Scaled *= incr_scale;
	q_Scaled = blaze::generate(6UL, [&](size_t idx)
							   { return (std::fabs(q_Scaled[idx]) > incr_floor) ? q_Scaled[idx] : incr_floor; });

	for (size_t iter = 0UL; iter <= 5UL; ++iter)
	{
		// eliminates the computations for Tube 3 (remains unactuated)
		if (iter == 2UL || iter == 5UL)
			continue;

		q_Perturbed[iter] += q_Scaled[iter];
		this->setConfiguration(q_Perturbed);

		// recalculates the CTR transition points and segments for beta actuation
		if (iter <= 3UL)
			m_segment->recalculateSegments(this->m_Tubes, this->m_beta);

		this->ODESolver(initGuess);

		// computing the tip position of the perturbed CTR
		blaze::column(jac, iter) = (this->getTipPos() - tipPos) / q_Scaled[iter];
		// restoring the original CTR joint values
		q_Perturbed[iter] = q_Original[iter];
	}

	// sets the joint values to their original, unperturbed configuration values
	this->setConfiguration(q_Original);
	// this->ODESolver(initGuess);

	return jac;
}

// function that implements Powell's Dog Leg Method (Nonlinear root-finding method for solving the BVP)
bool CTR::PowellDogLeg(blaze::StaticVector<double, 5UL> &initGuess)
{
	bool found;
	size_t k = 0UL;
	const size_t k_max = 300UL;
	double alpha, beta, delta, eps1, eps2, rho, c;
	blaze::StaticVector<double, 5UL> g, f, f_new, x_new, h_sd, h_gn, h_dl;
	blaze::StaticMatrix<double, 5UL, 5UL> J;

	// zeroes |mb_x(0)|, |mb_y(0)| and |u3_z(0)| and limits the values of |u1_z(0)| and |u2_z(0)| to avoid numerical instability and lack of convergence
	auto readjustInitialGuesses = [](blaze::StaticVector<double, 5UL> &initial_guesses)
	{
		blaze::subvector<2UL, 2UL>(initial_guesses) = blaze::map(blaze::subvector<2UL, 2UL>(initial_guesses), [](double d)
																 { return (!blaze::isfinite(d)) ? 0.00 : blaze::sign(d) * std::min(blaze::abs(d), 50.00); });
		// mb_x(0) = mb_y(0) = u3_z(0) = 0.00;
		initial_guesses[0UL] = initial_guesses[1UL] = initial_guesses[4UL] = 0.00;
	};

	readjustInitialGuesses(initGuess);

	// initializing parameters
	delta = 1.00;
	eps1 = eps2 = 1.00e-22;

	f = this->ODESolver(initGuess);
	J = this->jac_BVP(initGuess, f);
	g = blaze::trans(J) * f;

	// checking if the initial guess satisfies the BVP without the need of any further refinement
	found = ((blaze::linfNorm(f) <= this->m_accuracy) || (blaze::linfNorm(g) <= eps1)) ? true : false;

	while (!found && (k < k_max))
	{
		k++;

		alpha = blaze::sqrNorm(g) / blaze::sqrNorm(J * g);
		h_sd = -alpha * g;			 // steepest descend (this is a direction, not a step!)
		h_gn = -mathOp::pInv(J) * f; // Gauss-Newton step (Least Square solution)

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

			f_new = this->ODESolver(x_new);
			rho = (blaze::sqrNorm(f) - blaze::sqrNorm(f_new)) / (0.50 * blaze::trans(h_dl) * ((delta * h_dl) - g));

			if (rho > 0.00)
			{
				initGuess = std::move(x_new);
				f = std::move(f_new);
				J = this->jac_BVP(initGuess, f);
				g = blaze::trans(J) * f;

				if ((blaze::linfNorm(f) <= this->m_accuracy) || (blaze::linfNorm(g) <= eps1))
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

	return (blaze::linfNorm(f) <= this->m_accuracy) ? true : false;
}

// function that implements the Levenberg-Marquardt Method (Nonlinear root-finding method for solving the BVP)
bool CTR::Levenberg_Marquardt(blaze::StaticVector<double, 5UL> &initGuess)
{
	size_t k = 0UL;
	const size_t k_max = 300UL;
	blaze::StaticVector<double, 5UL> h, g, f, f_new;
	blaze::StaticMatrix<double, 5UL, 5UL, blaze::columnMajor> J, A;
	blaze::IdentityMatrix<double> I(5UL);
	double rho, nu = 2.00, mu, tau = 1.00e-3, e1 = 1.00e-18, e2 = 1.00e-25;
	bool found;

	// zeroes |mb_x(0)|, |mb_y(0)| and |u3_z(0)| and limits the values of |u1_z(0)| and |u2_z(0)| to avoid numerical instability and lack of convergence
	auto readjustInitialGuesses = [](blaze::StaticVector<double, 5UL> &initial_guesses)
	{
		blaze::subvector<2UL, 2UL>(initial_guesses) = blaze::map(blaze::subvector<2UL, 2UL>(initial_guesses), [](double d)
																 { return (!blaze::isfinite(d)) ? 0.00 : blaze::sign(d) * std::min(blaze::abs(d), 50.00); });
		// mb_x(0) = mb_y(0) = u3_z(0) = 0.00;
		initial_guesses[0UL] = initial_guesses[1UL] = initial_guesses[4UL] = 0.00;
	};

	readjustInitialGuesses(initGuess);

	// computing the residue and residue Jacobian associated to initGuess
	f = this->ODESolver(initGuess);
	J = this->jac_BVP(initGuess, f);
	A = blaze::trans(J) * J;
	g = blaze::trans(J) * f;
	found = (blaze::linfNorm(g) <= e1) ? true : false;
	mu = tau * blaze::max(blaze::diagonal(A));

	// starting the iterative minimization loop
	while ((!found) && (k < k_max))
	{
		k++;
		blaze::solve(blaze::declsym(A + (mu * I)), h, -g);

		f_new = this->ODESolver(initGuess + h);
		rho = (blaze::sqrNorm(f) - blaze::sqrNorm(f_new)) / (0.50 * blaze::trans(h) * ((mu * h) - g));

		if (rho > 0.00)
		{
			// accept the decrease in the function
			initGuess += h;
			// computing the residue Jacobian at the new initial guess
			J = this->jac_BVP(initGuess, f_new);
			A = blaze::trans(J) * J;
			f = std::move(f_new);
			g = blaze::trans(J) * f;
			found = (blaze::linfNorm(g) <= e1) ? true : false;
			mu = mu * std::max(0.33333333, 1.00 - blaze::pow(2.00 * rho - 1.00, 3.00));
			nu = 2.00;
		}
		else
		{
			mu = mu * nu;
			nu = 2.00 * nu;
		}

		// checking if the tolerance has been satisfied
		if (blaze::linfNorm(f) <= this->m_accuracy)
			found = true;
	}

	return (blaze::linfNorm(f) <= this->m_accuracy) ? true : false;
}

// function that implements the Broyden (Nonlinear root-finding method for solving the BVP)
bool CTR::Broyden(blaze::StaticVector<double, 5UL> &initGuess)
{
	// found: returns true (false) when the root-finding method converges (does not converge) within k_max iterations
	bool found;

	// zeroes |mb_x(0)|, |mb_y(0)| and |u3_z(0)| and limits the values of |u1_z(0)| and |u2_z(0)| to avoid numerical instability and lack of convergence
	auto readjustInitialGuesses = [](blaze::StaticVector<double, 5UL> &initial_guesses)
	{
		blaze::subvector<2UL, 2UL>(initial_guesses) = blaze::map(blaze::subvector<2UL, 2UL>(initial_guesses), [](double d)
																 { return (!blaze::isfinite(d)) ? 0.00 : blaze::sign(d) * std::min(blaze::abs(d), 50.00); });
		// mb_x(0) = mb_y(0) = u3_z(0) = 0.00;
		initial_guesses[0UL] = initial_guesses[1UL] = initial_guesses[4UL] = 0.00;
	};

	readjustInitialGuesses(initGuess);

	// initial Hessian matrix --> computed via finite differences
	blaze::StaticMatrix<double, 5UL, 5UL, blaze::columnMajor> JacInv, JacInvNew;

	// setting up and starting my handmadeBFGS method
	blaze::StaticVector<double, 5UL> F, Fold, X, Xold, deltaX, deltaF; // staticVectors are automatically initialized to 0

	// Residue yielded by the initial guess for the CTR BVP
	F = this->ODESolver(initGuess); // F(x_k)	: residue
	X = std::move(initGuess);		// x_k		: initial guess
	JacInvNew = JacInv = mathOp::pInv(this->jac_BVP(X, F));

	// checking if the initial guess already satisfies the BVP
	found = (blaze::linfNorm(F) <= this->m_accuracy) ? true : false;

	size_t k = 0UL;
	const size_t k_max = 300UL;
	while (!found && (k < k_max))
	{
		k++;

		deltaX = X - Xold; // dX := x_k - x_k-1
		deltaF = F - Fold; // dF := F(x_k) - F(x_k-1)

		JacInv = std::move(JacInvNew);
		if ((blaze::norm(deltaX) > 0.0) && (blaze::norm(deltaF) > 0.00))
			JacInvNew = JacInv + ((deltaX - JacInv * deltaF) / (blaze::trans(deltaX) * JacInv * deltaF)) * blaze::trans(deltaX) * JacInv;
		else
			JacInvNew = JacInv;

		Xold = std::move(X);
		Fold = std::move(F);

		// update the initial guess
		X = Xold - JacInv * F;
		F = this->ODESolver(X);

		while (blaze::isnan(F))
		{
			X *= 0.75;
			readjustInitialGuesses(X);

			F = this->ODESolver(X);
			JacInv = JacInvNew = mathOp::pInv(this->jac_BVP(X, F));
			Xold = std::move(X);
			X = Xold - JacInv * F;
		}

		if (k % 10 == 0.00)
		{
			JacInv = JacInvNew = mathOp::pInv(this->jac_BVP(X, F));
			X = Xold - JacInv * F;
		}

		if (blaze::linfNorm(F) <= this->m_accuracy)
			found = true;
	}

	initGuess = std::move(X);
	return (blaze::linfNorm(F) <= this->m_accuracy) ? true : false;
}

// function that implements Broyden's Nonlinear root-finding method for solving the BVP
bool CTR::Broyden_II(blaze::StaticVector<double, 5UL> &initGuess)
{
	bool found;

	// zeroes |mb_x(0)|, |mb_y(0)| and |u3_z(0)| and limits the values of |u1_z(0)| and |u2_z(0)| to avoid numerical instability and lack of convergence
	auto readjustInitialGuesses = [](blaze::StaticVector<double, 5UL> &initial_guesses)
	{
		blaze::subvector<2UL, 2UL>(initial_guesses) = blaze::map(blaze::subvector<2UL, 2UL>(initial_guesses), [](double d)
																 { return (!blaze::isfinite(d)) ? 0.00 : blaze::sign(d) * std::min(blaze::abs(d), 50.00); });
		// mb_x(0) = mb_y(0) = u3_z(0) = 0.00;
		initial_guesses[0UL] = initial_guesses[1UL] = initial_guesses[4UL] = 0.00;
	};

	readjustInitialGuesses(initGuess);

	// initial Hessian matrix --> computed via finite differences
	blaze::StaticMatrix<double, 5UL, 5UL, blaze::columnMajor> Jac, JacNew;

	// setting up and starting my handmadeBFGS method
	blaze::StaticVector<double, 5UL> F, Fold, X, Xold, deltaX, deltaF; // staticVectors are automatically initialized to 0

	// Residue yielded by the initial guess for the CTR BVP
	F = this->ODESolver(initGuess); // F(x_k)	: residue
	X = std::move(initGuess);		// x_k		: initial guess
	JacNew = this->jac_BVP(initGuess, F);

	// checking if the initial guess already satisfies the BVP
	found = (blaze::linfNorm(F) <= this->m_accuracy) ? true : false;

	size_t k = 0UL;
	const size_t k_max = 300UL;
	while (!found && (k < k_max))
	{
		k++;

		deltaX = X - Xold; // dX := x_k - x_k-1
		deltaF = F - Fold; // dF := F(x_k) - F(x_k-1)

		Jac = std::move(JacNew);
		if (blaze::sqrNorm(deltaX) > 0.00)
			JacNew = Jac + blaze::sqrNorm(deltaX) * (deltaF - (Jac * deltaX)) * blaze::trans(deltaX);
		else
			JacNew = Jac;

		Xold = std::move(X);
		Fold = std::move(F);

		// update the initial guess
		X = Xold - mathOp::pInv(Jac) * F;
		F = this->ODESolver(X);

		while (blaze::isnan(F))
		{
			X *= 0.75;
			readjustInitialGuesses(X);

			F = this->ODESolver(X);
			JacNew = this->jac_BVP(X, F);
			Xold = std::move(X);
			X = Xold - mathOp::pInv(Jac) * F;
		}

		if (k % 10 == 0.00)
		{
			JacNew = this->jac_BVP(X, F);
			X = Xold - mathOp::pInv(Jac) * F;
		}

		if (blaze::linfNorm(F) <= this->m_accuracy)
			found = true;
	}

	initGuess = std::move(X);
	return (blaze::linfNorm(F) <= this->m_accuracy) ? true : false;
}

// function that implements the Newton-Raphson method (Nonlinear root-finding method for solving the BVP)
bool CTR::Newton_Raphson(blaze::StaticVector<double, 5UL> &initGuess)
{
	bool found;
	// setting up and starting my handmade Newton-Raphson method
	blaze::StaticVector<double, 5UL> Residue, Residue_new, d_Residue, int_Residue, dGuess; // staticVectors are automatically initialized to 0

	// zeroes |mb_x(0)|, |mb_y(0)| and |u3_z(0)| and limits the values of |u1_z(0)| and |u2_z(0)| to avoid numerical instability and lack of convergence
	auto readjustInitialGuesses = [](blaze::StaticVector<double, 5UL> &initial_guesses)
	{
		blaze::subvector<2UL, 2UL>(initial_guesses) = blaze::map(blaze::subvector<2UL, 2UL>(initial_guesses), [](double d)
																 { return (!blaze::isfinite(d)) ? 0.00 : blaze::sign(d) * std::min(blaze::abs(d), 50.00); });
		// mb_x(0) = mb_y(0) = u3_z(0) = 0.00;
		initial_guesses[0UL] = initial_guesses[1UL] = initial_guesses[4UL] = 0.00;
	};

	readjustInitialGuesses(initGuess);

	// Residue of the unperturbed initial guess for the CTR
	Residue = this->ODESolver(initGuess);

	found = (blaze::linfNorm(Residue) <= this->m_accuracy) ? true : false;

	//  Weighing matrices for adjusting the initial guess iteratively (Implementing a PD regulator)
	blaze::DiagonalMatrix<blaze::StaticMatrix<double, 5UL, 5UL, blaze::rowMajor>> Kp, Ki, Kd;
	blaze::StaticMatrix<double, 5UL, 5UL, blaze::columnMajor> jac_bvp;
	blaze::diagonal(Kp) = 0.450; // 0.45 | 0.6  | 0.3
	blaze::diagonal(Ki) = 0.005;
	blaze::diagonal(Kd) = 0.002; // 3e-3 | 5e-3 | 2e-3

	size_t k = 0UL;
	const size_t k_max = 300UL;

	// starting iterations for adjusting the initial guess "u_guess ~ initGuess"
	while (!found && (k < k_max))
	{
		k++;
		jac_bvp = this->jac_BVP(initGuess, Residue);
		// error equation(globally asymptotically stable)
		dGuess = mathOp::pInv(jac_bvp) * (Kp * Residue + Ki * int_Residue + Kd * d_Residue);
		// updating the initial guess(weighted negative gradient of the cost function)
		initGuess -= dGuess;

		readjustInitialGuesses(initGuess);

		// computing the new cost associated to the newly readjusted initial guess
		Residue_new = this->ODESolver(initGuess);

		// cost variation due to initial guess refinement
		d_Residue = Residue_new - Residue;
		// integral of the residue
		int_Residue += Residue_new;
		// updating the cost
		Residue = std::move(Residue_new);

		if (blaze::linfNorm(Residue) <= this->m_accuracy)
			found = true;
	}

	return (blaze::linfNorm(Residue) <= this->m_accuracy) ? true : false;
}

// function that implements the Modified, globally convergent Newton-Raphson method (Nonlinear root-finding method for solving the BVP)
bool CTR::Modified_Newton_Raphson(blaze::StaticVector<double, 5UL> &initGuess)
{
	/*
		Algorithm extracted from page 309 of Introduction to Numerical Analysis 3rd edition by Josef Stoer & Roland Bulirsch
	*/

	// zeroes |mb_x(0)|, |mb_y(0)| and |u3_z(0)| and limits the values of |u1_z(0)| and |u2_z(0)| to avoid numerical instability and lack of convergence
	auto readjustInitialGuesses = [](blaze::StaticVector<double, 5UL> &initial_guesses)
	{
		blaze::subvector<2UL, 2UL>(initial_guesses) = blaze::map(blaze::subvector<2UL, 2UL>(initial_guesses), [](double d)
																 { return (!blaze::isfinite(d)) ? 0.00 : blaze::sign(d) * std::min(blaze::abs(d), 50.00); });
		// mb_x(0) = mb_y(0) = u3_z(0) = 0.00;
		initial_guesses[0UL] = initial_guesses[1UL] = initial_guesses[4UL] = 0.00;
	};

	readjustInitialGuesses(initGuess);

	bool found;
	// computes the residue associated to the initial guess
	blaze::StaticVector<double, 5UL> f(this->ODESolver(initGuess)), d;
	blaze::StaticVector<double, 5UL, blaze::rowVector> Dh;
	blaze::StaticMatrix<double, 5UL, 5UL> D, D_inv;
	double h, h_0, lambda, gamma, improvementFactor, d_norm, Dh_norm;
	size_t j = 0UL, k = 0UL;
	const size_t k_max = 300UL;
	std::vector<double> h_k; // vector to store all h_k's
	h_k.reserve(k_max);

	found = (blaze::linfNorm(f) <= this->m_accuracy) ? true : false;

	auto setupMethod = [&]() -> void
	{
		// then recomputes the residue
		f = this->ODESolver(initGuess);

		// computing the residue Jacobian
		D = this->jac_BVP(initGuess, f);

		// verifies NAN in the Jacobian and refines initial guess if necessary
		while (!blaze::isfinite(D))
		{
			initGuess *= 0.75;
			readjustInitialGuesses(initGuess);
			// then recomputes the residue
			f = this->ODESolver(initGuess);
			// computing the residue Jacobian
			D = this->jac_BVP(initGuess, f);
		}

		// pseudo-inverse of the residue Jacobian
		D_inv = mathOp::pInv(D);

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
			f = this->ODESolver(initGuess - blaze::pow(0.50, j) * d);
			// std::cout << "Modified_Newton_Raphson -- j = : " << j << " | residue = " << blaze::trans(f);
			while (!blaze::isfinite(f))
			{
				j++;
				f = this->ODESolver(initGuess - blaze::pow(0.50, j) * d);
				if (j > 20UL)
				{
					initGuess *= 0.75;
					readjustInitialGuesses(initGuess);
					setupMethod();
					j = 0UL;
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
		lambda = blaze::pow(0.50, h_k.size() - 1);
		initGuess -= lambda * d;
		h_k.clear();

		// resets the exponent variable j
		j = 0UL;

		// compute the residue associated to the newly refined initGuess
		f = this->ODESolver(initGuess);

		// checking the terminating condition
		if (blaze::linfNorm(f) <= this->m_accuracy)
		{
			return true;
		}
	}

	if (!found)
	{
		readjustInitialGuesses(initGuess);
		initGuess *= 0.75;
		found = this->PowellDogLeg(initGuess);

		if (!found)
		{
			readjustInitialGuesses(initGuess);
			initGuess *= 0.75;
			found = this->Levenberg_Marquardt(initGuess);
		}
	}

	return found;
}

// function that implements the CTR actuation for any inputs joint values q
bool CTR::actuate_CTR(blaze::StaticVector<double, 5UL> &initGuess, const blaze::StaticVector<double, 6UL> &q_input)
{
	// boolean flag for indicating convergence (1: zero found | 0: zero not found)
	bool found = false;

	// updating the CTR joints for desired input values
	this->setConfiguration(q_input);

	// recalculates the CTR transition points and segments
	m_segment->recalculateSegments(this->m_Tubes, this->m_beta);

	// initial guess for proximal boundary condition--[u_x(0) u_y(0) u1_z(0) u2_z(0) u3_z(0)]
	switch (this->m_method)
	{
	case mathOp::rootFindingMethod::NEWTON_RAPHSON:
		found = this->Newton_Raphson(initGuess);
		break;
	case mathOp::rootFindingMethod::LEVENBERG_MARQUARDT:
		found = this->Levenberg_Marquardt(initGuess);
		break;
	case mathOp::rootFindingMethod::POWELL_DOG_LEG:
		found = this->PowellDogLeg(initGuess);
		break;
	case mathOp::rootFindingMethod::MODIFIED_NEWTON_RAPHSON:
		found = this->Modified_Newton_Raphson(initGuess);
		break;
	case mathOp::rootFindingMethod::BROYDEN:
		found = this->Broyden(initGuess);
		break;
	case mathOp::rootFindingMethod::BROYDEN_II:
		found = this->Broyden_II(initGuess);
		break;
	}

	return found;
}

// function that implements the position control ==> returns timeout [bool]
bool CTR::posCTRL(blaze::StaticVector<double, 5UL> &initGuess, const blaze::StaticVector<double, 3UL> &target, const double posTol)
{
	double minError = 1.00E3;										 // minimum distance to target
	bool status;													 // status = TRUE (FALSE) indicates convergence (lack thereof)
	blaze::StaticMatrix<double, 3UL, 6UL, blaze::columnMajor> J;	 // Jacobian matrix
	blaze::StaticMatrix<double, 6UL, 3UL, blaze::columnMajor> J_inv; // Jacobian pseudoinverse
	blaze::IdentityMatrix<double, blaze::rowMajor> I(6UL);			 // 6 x 6 Identity matrix

	// zeroes |mb_x(0)|, |mb_y(0)| and |u3_z(0)| and limits the values of |u1_z(0)| and |u2_z(0)| to avoid numerical instability and lack of convergence
	auto readjustInitialGuesses = [](blaze::StaticVector<double, 5UL> &initial_guesses)
	{
		blaze::subvector<2UL, 2UL>(initial_guesses) = blaze::map(blaze::subvector<2UL, 2UL>(initial_guesses), [](double d)
																 { return (!blaze::isfinite(d)) ? 0.00 : blaze::sign(d) * std::min(blaze::abs(d), 50.00); });
		// mb_x(0) = mb_y(0) = u3_z(0) = 0.00;
		initial_guesses[0UL] = initial_guesses[1UL] = initial_guesses[4UL] = 0.00;
	};

	readjustInitialGuesses(initGuess);

	// container variables for EM reading
	blaze::StaticVector<double, 3UL> position_CTR, position_CTR_KF;

	// proportional, derivative, and integral gains for position control
	blaze::DiagonalMatrix<blaze::StaticMatrix<double, 3UL, 3UL, blaze::columnMajor>> Kp, Kd, Ki;
	blaze::diagonal(Kp) = 1.000; // 1.000
	blaze::diagonal(Ki) = 0.050; // 0.050
	blaze::diagonal(Kd) = 0.001; // 0.001

	// Capturing the CTR's current joint configuration
	blaze::StaticVector<double, 6UL> dqdt, q_min(this->m_q), q(this->m_q);
	// Capturing the proximal BC for the minimum distance
	blaze::StaticVector<double, 5UL> initGuessMin(initGuess);
	// Calculate the CTR Jacobian in the present configuration and retrieves convergence status
	status = this->actuate_CTR(initGuess, q);

	// failure of convergence of the nonlinear root-finders
	if (!status)
	{
		std::cout << "=============>> EXITING " << __PRETTY_FUNCTION__ << " ON LACK OF CONVERGENCE! <<=============" << std::endl;
		return status;
	}

	// Capturing the position error, and its derivative and integral
	blaze::StaticVector<double, 3UL> x_CTR, tipError, d_tipError, last_tipError, int_tipError;

	// acquiring the tip position as predicted by the model
	x_CTR = this->getTipPos();
	blaze::StaticVector<double, 3UL> x_CTR_init(x_CTR);

	// acquiring the current tip position as read by the EM sensor
	std::tie(position_CTR, position_CTR_KF) = this->acquireEMData();

	// computing the disagreement between model and EM readings as a DC error
	blaze::StaticVector<double, 3UL> DC_error(position_CTR_KF - x_CTR_init);

	// Current position error
	// DC_error = 0.00;
	// tipError = target - this->getTipPos();
	tipError = target - position_CTR_KF;

	// std::cout << "|DC_error| = " << blaze::norm(DC_error) << " make \t DC_error = " << blaze::trans(DC_error)
	// 		  << "|tipError| = " << blaze::norm(tipError) << " make \t Current error = " << blaze::trans(tipError) << std::endl;

	// Euclidean distance to target
	double dist2Tgt = blaze::norm(tipError);

	if (dist2Tgt < minError)
	{
		minError = dist2Tgt;
		q_min = q;

		if (dist2Tgt <= posTol)
		{
			// std::cout << "CTR end-effector already at the target!" << std::endl;
			return status;
		}
	}

	// function to implement actuators sigularity avoidance
	blaze::StaticVector<double, 6UL> f;
	// clearance between linear actuators
	double Clr = 30.00E-3, deltaBar = 0.00;
	// lengths of straight sections of the CTR tubes
	blaze::StaticVector<double, 3UL> ls, L;
	L[0UL] = this->m_Tubes[0UL]->getTubeLength();
	L[1UL] = this->m_Tubes[1UL]->getTubeLength();
	L[2UL] = this->m_Tubes[2UL]->getTubeLength();
	ls[0UL] = this->m_Tubes[0UL]->getStraightLen();
	ls[1UL] = this->m_Tubes[1UL]->getStraightLen();
	ls[2UL] = this->m_Tubes[2UL]->getStraightLen();
	// lower and upper bounds on prismatic joint limits
	blaze::StaticVector<double, 3UL> betaMax, betaMin;

	size_t N_itr = 0UL;			  // iterations counter
	const size_t maxIter = 100UL; // maximum admissible number of iterations in the position control loop

	// parameters for local optimization (joint limits avoidance)
	double ke = 2.00;

	// penalty function for implementing actuator collision avoidance
	auto f1 = blaze::subvector<0UL, 3UL>(f);

	// position control loop
	while ((dist2Tgt > posTol) && (N_itr < maxIter))
	{
		// incrementing the number of iterations
		N_itr++;

		// compute the Jacobian in the present configuration
		J = this->jacobian(initGuess, x_CTR);
		// Pseudo-inverse of Jacobian for resolving CTR joint motion rates
		J_inv = mathOp::pInv(J);

		// Nullspace control (collision and actuation limits)
		betaMin[0UL] = std::max({-ls[0UL] + deltaBar, L[1UL] + this->m_beta[1UL] - L[0UL], L[2UL] + this->m_beta[2UL] - L[0UL]});
		betaMin[1UL] = std::max({-ls[1UL] + deltaBar, this->m_beta[0UL] + Clr, L[2UL] + this->m_beta[2UL] - L[1UL]});
		betaMin[2UL] = std::max(-ls[2UL] + deltaBar, this->m_beta[1UL] + Clr);

		betaMax[0UL] = m_beta[1UL] - Clr;
		betaMax[1UL] = std::min(this->m_beta[2UL] - Clr, L[0UL] + this->m_beta[0UL] - L[1UL]);
		betaMax[2UL] = std::min({-deltaBar, L[1UL] + this->m_beta[1UL] - L[2UL], L[0UL] + this->m_beta[0UL] - L[2UL]});

		// Inverse kinematics ==> penalty function for local optimization (actuator collision avoidance)
		f1 = blaze::pow(blaze::abs((betaMax + betaMin - 2.00 * this->m_beta) / (betaMax - betaMin + 1.00E-10)), ke) * blaze::sign(this->m_beta - (betaMax + betaMin) * 0.50);

		// resolved rates -- Nullspacec local optimization (joint limit avoidance)
		dqdt = J_inv * (Kp * tipError + Kd * d_tipError + Ki * int_tipError) + (I - blaze::trans(J_inv * J)) * (-f);

		auto rescale_dqdt = [&]() -> void // rescaling linear joint variables for limit avoidance
		{
			for (size_t i = 0UL; i < 3UL; ++i)
			{
				if (this->m_beta[i] + dqdt[i] > betaMax[i])
					dqdt[i] = (betaMax[i] - this->m_beta[i]) * 0.50;

				if (this->m_beta[i] + dqdt[i] < betaMin[i])
					dqdt[i] = (betaMin[i] - this->m_beta[i]) * 0.50;
			}
		};

		rescale_dqdt();

		// updating the CTR joints->q: [beta, theta]
		q += dqdt;
		// Tube 3 remains unactuated
		q[2UL] = q[5UL] = 0.00;

		// wrapping the actuation angles to the [0.00,2Pi) interval
		blaze::subvector<3UL, 3UL>(q) = blaze::map(blaze::subvector<3UL, 3UL>(q), [](double theta)
												   {
													   static constexpr double TWO_PI = 2.00 * M_PI;

													   return std::remainder(theta, TWO_PI); });

		// actuate the CTR to new configuration and retrieve execution timeout status
		status = this->actuate_CTR(initGuess, q);

		// interrupts the loop execution if actuation fails
		if (!status)
		{
			initGuess *= 0.50;
			readjustInitialGuesses(initGuess);
			status = this->actuate_CTR(initGuess, q);

			if (!status)
			{
				// std::cout << "Nonlinear root-finders failed to converge: " << __PRETTY_FUNCTION__ << std::endl;
				initGuess = initGuessMin;
				q = q_min;
				this->actuate_CTR(initGuess, q);
				// return status;
			}
		}

		// tip position as predicted by the model
		x_CTR = this->getTipPos();

		// current position error
		tipError = target - (x_CTR + DC_error);
		// integrating the position error
		int_tipError += tipError;
		// derivative of the positio
		d_tipError = tipError - last_tipError;
		// updating the last tip error variable
		last_tipError = tipError;

		dist2Tgt = blaze::norm(tipError);

		if (dist2Tgt < minError)
		{
			minError = dist2Tgt;
			q_min = q;
			initGuessMin = initGuess;
		}

		// stops the control loop when the position update becomes significantly small
		if (blaze::linfNorm(dqdt) <= 1.00E-6)
		{
			initGuess = initGuessMin;
			status = this->actuate_CTR(initGuess, q_min);
			std::cout << "Exited out of position control loop due small incremental threshold!" << std::endl;
			return status;
		}
	}

	// std::cout << "Exiting posCTR() with error: " << minError << " at iteration i = " << N_itr << std::endl;

	// Actuating the CTR to the configuration which yields the minimum position error
	initGuess = initGuessMin;
	status = this->actuate_CTR(initGuess, q_min);

	return status;
}

// function that implements the constrained position control || inputs: [initGuess, calyx target, distal target, tolerance] ==> returns [minDist2Tgt_s, minDist2Tgt_e, status]
std::tuple<double, double, bool> CTR::constrainedPosCTRL(blaze::StaticVector<double, 5UL> &initGuess, const blaze::StaticVector<double, 3UL> &tgt_clx, const blaze::StaticVector<double, 3UL> &tgt_ee, const double posTol)
{
	// status = TRUE indicates convergence of the nonlinear root-finder method
	bool status = false;
	// resolved rate for joint actuation, joint values of position tracking solution
	blaze::StaticVector<double, 6UL> q_min(this->m_q), q_dot, q(this->m_q);
	// Capturing the proximal BC for the minimum distance
	blaze::StaticVector<double, 5UL> initGuessMin(initGuess);
	// lengths of straight segments and overall length of the tubes
	blaze::StaticVector<double, 3UL> ls, L;
	// lower and upper bounds on joint limits
	blaze::StaticVector<double, 6UL> qMax, qMin;
	// arc-length closest to the calyx
	double s_min, s;
	// position of end effector & point on backbone closest to the calyx
	blaze::StaticVector<double, 3UL> x_ee, x_s;
	// scalar for resolved joint rate & minimum position error
	double a = 1.00, minError;
	// clearance between linear actuators (prismatic joints)
	double Clr = 28.00e-3, deltaBar = 0.00;
	// slack variables (threshold for velocities)
	blaze::StaticVector<double, 6UL> d_p, d_m;
	// d_p = {5e-5, 5e-5, 5e-5, 1e-4, 1e-4, 1e-4};
	// d_m = {-5e-5, -5e-5, -5e-5, -1e-4, -1e-4, -1e-4};
	d_p = 5.00E-4;
	d_m = -5.00E-4;

	// zeroes |mb_x(0)|, |mb_y(0)| and |u3_z(0)| and limits the values of |u1_z(0)| and |u2_z(0)| to avoid numerical instability and lack of convergence
	auto readjustInitialGuesses = [](blaze::StaticVector<double, 5UL> &initial_guesses)
	{
		blaze::subvector<2UL, 2UL>(initial_guesses) = blaze::map(blaze::subvector<2UL, 2UL>(initial_guesses), [](double d)
																 { return (!blaze::isfinite(d)) ? 0.00 : blaze::sign(d) * std::min(blaze::abs(d), 50.00); });
		// mb_x(0) = mb_y(0) = u3_z(0) = 0.00;
		initial_guesses[0UL] = initial_guesses[1UL] = initial_guesses[4UL] = 0.00;
	};

	readjustInitialGuesses(initGuess);

	// Qq and Qd penalty costs
	const double Qq = 10000, Qd = 500; // 10,000 | 500
	// damping coefficients for the Jacobian matrices (@ end-effector and calyx)
	double dampTip = 0.00, dampClx = 0.00;
	// maximun radial distance from which the Jacobian damping starts to be considered
	double dampDist = 5.00e-3; // initially 5mm
	// minimum distances to calyx and end target
	double minDist2Tgt_e, minDist2Tgt_s;
	// 6x6 identity matrix | 3x3 identity matrix
	blaze::IdentityMatrix<double> I(6UL), I_damp(3UL);

	// computing the point on the CTR backbone closest to the calyx
	s = this->returnArcLength(tgt_clx);
	// retrieving the Jacobian matrices @ end-effector & calyx
	std::tie(this->constrFuncStr->JeeView, this->constrFuncStr->JpView, x_s, x_ee) = this->computeAnatomicalJacobians(initGuess, s);

	this->constrFuncStr->AIView = (-1.00) * I;
	this->objFuncStr->r_clx = x_s;	 // point on CTR backbone closest to the calyx
	this->objFuncStr->clx = tgt_clx; // coordinate of the renal calyx

	// augmented vector of desired spatial velocities x_dot := [(xd - x_ee) 0]^T
	this->constrFuncStr->xdotView_e = tgt_ee - x_ee;
	this->constrFuncStr->xdotView_s = tgt_clx - x_s;

	// Current position error -- Euclidean distance wrt end-effector and calyx targets
	double dist2Tgt_e = blaze::norm(tgt_ee - x_ee);
	double dist2Tgt_s = blaze::norm(tgt_clx - x_s);
	double posError = dist2Tgt_e + dist2Tgt_s;

	// printf("\n ################################### - INITIAL ERRORS - ################################### \n");
	// printf("CTR tip position: [ %lf, %lf, %lf ] \n", x_ee[0], x_ee[1], x_ee[2]);
	// printf("Distal end: %lf | Renal pyramid: %lf | Cumulative error: %lf", dist2Tgt_e, dist2Tgt_s, posError);
	// printf("\n ######################################################################################## \n\n\n");

	// the first observed posError is always the smallest prior to the first iteration
	minError = posError;
	minDist2Tgt_e = dist2Tgt_e;
	minDist2Tgt_s = dist2Tgt_s;
	s_min = s;

	this->objFuncStr->QqView = Qq * I; // penalize joint velocities: minimimum norm solution
	this->objFuncStr->QdView = I;	   // penalizes slack velocities: restrict motion on renal pyramids

	// overall length of each tube
	L[0UL] = this->m_Tubes[0UL]->getTubeLength();
	L[1UL] = this->m_Tubes[1UL]->getTubeLength();
	L[2UL] = this->m_Tubes[2UL]->getTubeLength();
	// lengths of the straight sections of each tube
	ls[0UL] = this->m_Tubes[0UL]->getStraightLen();
	ls[1UL] = this->m_Tubes[1UL]->getStraightLen();
	ls[2UL] = this->m_Tubes[2UL]->getStraightLen();

	// lower & upper bounds lb := [q_dot^- delta^-], ub := [q_dot^+ delta^+]
	blaze::StaticVector<double, 12UL> lb, ub;
	auto lbView = blaze::subvector<0UL, 6UL>(lb);
	auto ubView = blaze::subvector<0UL, 6UL>(ub);
	std::vector<double> lbVec(12UL), ubVec(12UL);
	// lower and upper bounds --> constant values related to delta +,-
	blaze::subvector<6UL, 6UL>(lb) = std::move(d_m);
	blaze::subvector<6UL, 6UL>(ub) = std::move(d_p);

	// ------------------------------ # # # . . . Defining and instantiating the optimization problem . . . # # # ----------------------------------------
	// nlopt::opt qpOPT(nlopt::LN_AUGLAG_EQ, 12UL); // LN_AUGLAG_EQ, LD_SLSQP, LN_SBPLX, LN_BOBYQA, LN_COBYLA, GN_DIRECT_L
	// nlopt::opt localOPT(nlopt::LN_COBYLA, 12UL); // COBYLA, LBFGS, MMA, or SLSQP
	nlopt::opt qpOPT(nlopt::LD_AUGLAG_EQ, 12UL); // LN_AUGLAG_EQ, LD_SLSQP, LN_SBPLX, LN_BOBYQA, LN_COBYLA, GN_DIRECT_L
	nlopt::opt localOPT(nlopt::LD_SLSQP, 12UL);	 // LD_SLSQP, LD_LBFGS (not good), LD_CCSAQ (too slow)
	localOPT.set_ftol_rel(1.00E-5);
	localOPT.set_ftol_abs(1.00E-5);
	localOPT.set_xtol_rel(1.00E-5);
	localOPT.set_maxeval(2000);
	localOPT.set_maxtime(20);

	// setting the local optimizer (Constrained Optimization By Linear Approximations -- COBYLA)
	qpOPT.set_local_optimizer(localOPT);

	// lambda implementing the objective function
	auto func = [](const std::vector<double> &x, std::vector<double> &grad, void *f_data) -> double
	{
		blaze::StaticVector<double, 12UL, blaze::columnVector> xOpt = {x[0UL], x[1UL], x[2UL], x[3UL], x[4UL], x[5UL], x[6UL], x[7UL], x[8UL], x[9UL], x[10UL], x[11UL]};
		objFuncData *data = reinterpret_cast<objFuncData *>(f_data);

		// computing the gradient of the objective function wrt the optimizing variables
		if (!grad.empty())
		{
			blaze::StaticVector<double, 12UL> gradFunc = data->Q * xOpt;
			for (size_t idx = 0UL; idx < 12UL; ++idx)
				grad[idx] = gradFunc[idx];
		}

		return 0.50 * blaze::trans(xOpt) * data->Q * xOpt; // -log(std::min(blaze::norm(data->r_clx - data->clx), data->rad) - data->rad);
	};

	// lambda implementing the equality constraint
	auto constraint = [](unsigned m, double *result, unsigned n, const double *x, double *grad, void *f_data) -> void
	{
		blaze::StaticVector<double, 12UL, blaze::columnVector> xOpt = {x[0UL], x[1UL], x[2UL], x[3UL], x[4UL], x[5UL], x[6UL], x[7UL], x[8UL], x[9UL], x[10UL], x[11UL]};
		constrFuncData *data = reinterpret_cast<constrFuncData *>(f_data);

		// computing the gradient of the constraint equations wrt the optimizing variables
		if (grad != nullptr)
		{
			blaze::StaticMatrix<double, 12UL, 6UL> gradConstr = blaze::trans(data->A);
			for (size_t i = 0UL; i < gradConstr.rows(); ++i)
			{
				for (size_t j = 0UL; j < gradConstr.columns(); ++j)
				{
					*grad = gradConstr(i, j);
					grad++;
				}
			}
		}

		blaze::StaticVector<double, 6UL> h_eq;
		h_eq = data->A * xOpt - data->xdot; // equality constraint
		result[0UL] = h_eq[0UL];
		result[1UL] = h_eq[1UL];
		result[2UL] = h_eq[2UL];
		result[3UL] = h_eq[3UL];
		result[4UL] = h_eq[4UL];
		result[5UL] = h_eq[5UL];
	};

	// void nlopt::opt::set_min_objective(nlopt::vfunc f, void* f_data);
	// void nlopt::opt::set_max_objective(nlopt::vfunc f, void* f_data);
	qpOPT.set_min_objective(func, objFuncStr.get());

	// void nlopt::opt::add_inequality_mconstraint(nlopt::mfunc c, void *c_data, const vector`<double>` &tol);
	// void nlopt::opt::add_equality_mconstraint(nlopt::mfunc c, void *c_data, const vector`<double>` &tol);
	std::vector<double> constrTol(6UL, 1.00E-5); // tolerances in each constraint dimension
	qpOPT.add_equality_mconstraint(constraint, constrFuncStr.get(), constrTol);

	// initial point for optimization
	std::vector<double> x(12UL);
	// initial point for optimization -- portion related to the slack delta +,-
	std::fill_n(x.begin() + 6UL, 6UL, 0.0000);

	// value of the objective function
	double minFunc;
	// nlopt container to store optimization status
	nlopt::result result;

	// number of iterations in the control loop
	size_t N_iter = 0UL;
	const size_t maxIter = 250UL;

	// position control loop
	while ((posError > posTol) && (N_iter < maxIter))
	{
		// increment the iterations counter
		N_iter++;

		// joint limits -- (Legal Configurations & Collision Avoidance of Linear Actuators)
		qMin[0UL] = std::max({-ls[0UL] + deltaBar, L[1UL] + this->m_beta[1UL] - L[0UL], L[2UL] + this->m_beta[2UL] - L[0UL]});
		qMin[1UL] = std::max({-ls[1UL] + deltaBar, this->m_beta[0UL] + Clr, L[2UL] + this->m_beta[2UL] - L[1UL]});
		qMin[2UL] = std::max(-ls[2UL] + deltaBar, this->m_beta[1UL] + Clr);
		qMin[3UL] = qMin[4UL] = qMin[5UL] = -M_PI;

		qMax[0UL] = m_beta[1UL] - Clr;
		qMax[1UL] = std::min(this->m_beta[2UL] - Clr, L[0UL] + this->m_beta[0UL] - L[1UL]);
		qMax[2UL] = std::min({-deltaBar, L[1UL] + this->m_beta[1UL] - L[2UL], L[0UL] + this->m_beta[0UL] - L[2UL]});
		qMax[3UL] = qMax[4UL] = qMax[5UL] = M_PI;

		// UPDATING --> lower bounds lb := [q_dot^- delta^-]
		lbView = (qMin - q) / a;
		lbVec = {lb[0UL], lb[1UL], lb[2UL], lb[3UL], lb[4UL], lb[5UL], lb[6UL], lb[7UL], lb[8UL], lb[9UL], lb[10UL], lb[11UL]};

		// UPDATING --> upper bounds ub := [q_dot^+ delta^+]
		ubView = (qMax - q) / a;
		ubVec = {ub[0UL], ub[1UL], ub[2UL], ub[3UL], ub[4UL], ub[5UL], ub[6UL], ub[7UL], ub[8UL], ub[9UL], ub[10UL], ub[11UL]};

		// must reinvoke these functions to update the lower & upper bounds
		localOPT.set_lower_bounds(lbVec);
		localOPT.set_upper_bounds(ubVec);
		qpOPT.set_lower_bounds(lbVec);
		qpOPT.set_upper_bounds(ubVec);

		// initial point for optimization -- portion related to q_dot variables
		for (size_t idx = 0UL; idx < 6UL; ++idx)
			x[idx] = (ubVec[idx] + lbVec[idx]) * 0.50;

		std::fill_n(x.begin() + 6UL, 6UL, 0.0000);

		try
		{
			// recompute optimization
			// nlopt::result nlopt::opt::optimize(std::vector<double>&x, double &opt_f);
			result = qpOPT.optimize(x, minFunc);
		}
		catch (std::exception &e)
		{
			// std::cerr << "nlopt has failed: " << e.what() << "  | nlopt::result: " << result << std::endl;
			// std::cerr << "Failed at iteration: " << N_iter << std::endl;

			try
			{
				// recompute optimization
				// nlopt::result nlopt::opt::optimize(std::vector<double>&x, double &opt_f);
				result = qpOPT.optimize(x, minFunc);
			}
			catch (std::exception &e)
			{
				// std::cerr << "nlopt has failed a second time: " << e.what() << "  | nlopt::result: " << result << std::endl;
				// break;
			}
		}

		// retrieving the optimized parameters
		q_dot = {x[0UL], x[1UL], x[2UL], x[3UL], x[4UL], x[5UL]};

		// updating the CTR joints: q = [beta, theta]
		q += a * q_dot;
		// Tube 3 remains unactuated
		q[2UL] = q[5UL] = 0.00;

		// actuates the CTR to new configuration and retries the execution timeout status
		initGuess[0UL] = initGuess[1UL] = initGuess[4UL] = 0.00;
		status = this->actuate_CTR(initGuess, q);

		if (!status)
		{
			initGuess *= 0.75;
			readjustInitialGuesses(initGuess);
			status = this->actuate_CTR(initGuess, q);

			if (!status)
			{
				std::cerr << "Nonlinear root-finders failed to converge when solving the BVP problem - " << __func__ << std::endl;
				this->actuate_CTR(initGuessMin, q_min);
				return std::make_tuple(minDist2Tgt_s, minDist2Tgt_e, status);
			}
		}

		// retrieving the backbone arc-length closest to the calyx
		s = this->returnArcLength(tgt_clx);

		// retrieving the new Jacobian matrices at the end-effector and parenchyma
		std::tie(this->constrFuncStr->JeeView, this->constrFuncStr->JpView, x_s, x_ee) = this->computeAnatomicalJacobians(initGuess, s);

		this->objFuncStr->r_clx = x_s;

		// augmented vector of desired spatial velocities x_dot := [(xd - x_ee) 0]^T
		this->constrFuncStr->xdotView_e = tgt_ee - x_ee;
		this->constrFuncStr->xdotView_s = tgt_clx - x_s;

		// Current position error -- Euclidean distance wrt end-effector and calyx targets
		dist2Tgt_e = blaze::norm(tgt_ee - x_ee);
		dist2Tgt_s = blaze::norm(tgt_clx - x_s);
		posError = dist2Tgt_e + dist2Tgt_s;

		if (std::min(dist2Tgt_e, dist2Tgt_s) < dampDist)
		{
			dampDist = 1.05 * std::min(dist2Tgt_e, dist2Tgt_s);

			// ### -- ### APPLYING DAMPING COEFFICIENT TO JACOBIAN MATRICES WHEN APPLICABLE ### --- ###
			if ((dist2Tgt_e < dampDist) && (dist2Tgt_s > dampDist))
			{
				dampTip = 0.90; // damp the rows of Jee by 90%
				dampClx = 0.00; // zero damping on the calyx Jacobian matrix
			}
			else
			{
				if ((dist2Tgt_e > dampDist) && (dist2Tgt_s < dampDist))
				{
					dampTip = 0.00; // zero damping on the end-effector Jacobian matrix
					dampClx = 0.90; // damp the rows of Js by 90%
				}
			}
		}
		else
		{
			dampTip = 0.00; // zero damping on the end-effector Jacobian matrix
			dampClx = 0.00; // zero damping on the renal calyx Jacobian matrix
		}

		// Finally, apply the damping coefficients on the Jacobian matrices for subsequent iterations
		this->constrFuncStr->JeeView = (1.00 - dampTip) * I_damp * this->constrFuncStr->JeeView;
		this->constrFuncStr->JpView = (1.00 - dampClx) * I_damp * this->constrFuncStr->JpView;

		// std::cout << "posError = " << posError << "  | minFunc = " << minFunc << std::endl;
		if (posError < minError)
		{
			// printf(">>> -- MIN ERROR DETECTED! -- <<< (calyx: %.6lf, distal-end: %.6lf)\n", dist2Tgt_s, dist2Tgt_e);
			minError = posError;
			minDist2Tgt_e = dist2Tgt_e;
			minDist2Tgt_s = dist2Tgt_s;
			// printf("End-effector: %lf [mm] -- calyx error: %lf [mm] -- accumulated errors: %lf [mm]\n", dist2Tgt_e * 1e3, dist2Tgt_s * 1e3, (dist2Tgt_e + dist2Tgt_s) * 1e3);
			this->objFuncStr->QdView = (Qd / minError) * I; // penalizes slack velocities: restrict motion on renal pyramids
			q_min = q;
			s_min = s;
			initGuessMin = initGuess;
		}
	}

	initGuess = std::move(initGuessMin);
	this->actuate_CTR(initGuess, q_min);

	// std::cout << "Constrained control ended in " << N_iter << " iterations. Error Calyx: " << minDist2Tgt_s << " | Error distal: " << minDist2Tgt_e << std::endl;

	return std::make_tuple(minDist2Tgt_s, minDist2Tgt_e, status);
}

//  computes the spatial velocity Jacobians at the end-effector and at the arc-length "s" (at the CTR/calyx interaction)
std::tuple<blaze::StaticMatrix<double, 3UL, 6UL>, blaze::StaticMatrix<double, 3UL, 6UL>, blaze::StaticVector<double, 3UL>, blaze::StaticVector<double, 3UL>> CTR::computeAnatomicalJacobians(const blaze::StaticVector<double, 5UL> &initGuess, const double s)
{
	blaze::StaticMatrix<double, 3UL, 6UL, blaze::columnMajor> J_ee, J_par;

	// unperturbed CTR tip position
	blaze::StaticVector<double, 3UL> pp_disturb, p_ee(this->getTipPos());
	// perturbed joint values for the CTR

	blaze::StaticVector<double, 6UL> q_Original(this->m_q), q_Perturbed(this->m_q), q_Scaled(this->m_q);
	double incr_scale = 1.00E-2, incr_floor = 1.00E-5; // 1.00E-7, incr_floor ==>> 1.00E-9 SEEM TO BE OPTIMAL;

	q_Scaled *= incr_scale;
	q_Scaled = blaze::generate(6UL, [&](size_t idx)
							   { return (std::fabs(q_Scaled[idx]) > incr_floor) ? q_Scaled[idx] : incr_floor; });

	// finding the index in the tube length array -- point on backbone closest to the renal calix
	auto it = std::lower_bound(this->m_s.begin(), this->m_s.end(), s);
	size_t idx = std::distance(this->m_s.begin(), it);

	// checking if the CTR is retracted past (before) the point of interest (calyx)
	if (idx == this->m_s.size())
		idx--;

	// gets the undisturbed position of the backbone at the arc-length "s" (at the parenchyma)
	blaze::StaticVector<double, 3UL> p_par = {this->m_y[idx][8UL], this->m_y[idx][9UL], this->m_y[idx][10UL]};

	for (size_t iter = 0UL; iter <= 5UL; ++iter)
	{
		q_Perturbed[iter] += q_Scaled[iter];
		this->setConfiguration(q_Perturbed);

		// recalculates the CTR transition points and segments for beta actuation
		if (iter <= 3UL)
			this->m_segment->recalculateSegments(this->m_Tubes, this->m_beta);

		this->ODESolver(initGuess);

		// computing the spatial velocities at the distal end
		blaze::column(J_ee, iter) = (this->getTipPos() - p_ee) / q_Scaled[iter];
		// computing the spatial velocities at the calyx
		pp_disturb = blaze::subvector<8UL, 3UL>(this->m_y[idx]);
		blaze::column(J_par, iter) = (pp_disturb - p_par) / q_Scaled[iter];
		// restoring the original CTR joint values
		q_Perturbed[iter] = q_Original[iter];
	}
	// sets the joint values to their original, unperturbed configuration values
	this->setConfiguration(q_Original);
	// this->ODESolver(initGuess);

	return std::make_tuple(J_ee, J_par, p_par, p_ee);
}

// returns the arc-length of the backbone which is the closest point to the calyx / renal pyramid
double CTR::returnArcLength(const blaze::StaticVector<double, 3UL> &calyx)
{
	// lambda comparator: compute the point on the backbone closest to the renal calyx
	auto dist = [&](state_type x, state_type y) -> bool
	{
		return blaze::norm(blaze::subvector<8UL, 3UL>(x) - calyx) < blaze::norm(blaze::subvector<8UL, 3UL>(y) - calyx);
	};

	auto it = std::min_element(this->m_y.begin(), this->m_y.end(), dist);
	// index of the nearest neighbor on the array
	size_t idx = std::distance(this->m_y.begin(), it);

	// returning the arc-length value
	return this->m_s[idx];
}

// function that returns the Vector of tubes comprising the CTR
std::array<std::shared_ptr<Tube>, 3UL> CTR::getTubes()
{
	return this->m_Tubes;
}

// function that returns the current linear joint values of the CTR
blaze::StaticVector<double, 3UL> CTR::getBeta()
{
	return this->m_beta;
}

// function that returns the current joint values of the CTR
blaze::StaticVector<double, 6UL> CTR::getConfiguration()
{
	return this->m_q;
}

// function that returns the position of the CTR tip
blaze::StaticVector<double, 3UL> CTR::getTipPos()
{
	blaze::StaticVector<double, 3UL> pos;
	if (!this->m_y.empty())
		pos = blaze::subvector<8UL, 3UL>(this->m_y.back());

	return pos;
}

// function that returns the arc-lenghts at each tube's distal end
blaze::StaticVector<double, 3UL> CTR::getDistalEnds()
{
	return this->m_segment->getDistalEnds();
}

// function that returns the individual tube shapes
std::tuple<blaze::HybridMatrix<double, 3UL, 1000UL, blaze::columnMajor>, blaze::HybridMatrix<double, 3UL, 1000UL, blaze::columnMajor>, blaze::HybridMatrix<double, 3UL, 1000UL, blaze::columnMajor>> CTR::getTubeShapes()
{
	blaze::HybridMatrix<double, 3UL, 1000UL, blaze::columnMajor> Tb_1(3UL, this->m_y.size());

	// arc-lengths at the distal ends of each tube
	blaze::StaticVector<double, 3UL> distal_idx, distalEnds(this->m_segment->getDistalEnds());

	for (size_t col = 0UL; col < Tb_1.columns(); ++col)
	{
		blaze::column(Tb_1, col) = blaze::subvector<8UL, 3UL>(this->m_y[col]);
	}

	// lambda returns the array index at which the ith tube ends
	auto tubeEndIndex = [&](size_t tube_index) -> size_t
	{
		// find the index in the arc-length vector at which each tube ends
		std::vector<double>::iterator it = std::lower_bound(this->m_s.begin(), this->m_s.end(), distalEnds[tube_index] - 1.00E-7); // finds where tube ends (0.0001mm tolerance)

		return std::distance(this->m_s.begin(), it);
	};

	size_t distalIndex_Tb2, distalIndex_Tb3;

	// index at which tube 2 ends
	distalIndex_Tb2 = tubeEndIndex(1UL);
	// index at which tube 3 ends
	distalIndex_Tb3 = tubeEndIndex(2UL);

	// number of columns in the shape matrices: index + 1
	auto Tb_2 = blaze::submatrix(Tb_1, 0UL, 0UL, 3UL, distalIndex_Tb2 + 1);
	auto Tb_3 = blaze::submatrix(Tb_1, 0UL, 0UL, 3UL, distalIndex_Tb3 + 1);

	// returns the tuple containing the shape of the tubes
	return std::make_tuple(Tb_1, Tb_2, Tb_3);
}

// function that returns a vector with the CTR shape
std::tuple<std::vector<double>, std::vector<double>, std::vector<double>> CTR::getShape()
{
	std::vector<double> r_x, r_y, r_z;
	r_x.reserve(1000UL);
	r_y.reserve(1000UL);
	r_z.reserve(1000UL);

	if (this->m_y.size() > 0UL)
	{
		for (auto &y : this->m_y)
		{
			r_x.push_back(y[8UL]);
			r_y.push_back(y[9UL]);
			r_z.push_back(y[10UL]);
		}
	}

	return std::make_tuple(r_x, r_y, r_z);
}

// setter method for setting the actuation joint values (without actuating the CTR) <--> used for computing the Jacobian
void CTR::setConfiguration(const blaze::StaticVector<double, 6UL> &q)
{
	this->m_q = q;
	this->m_beta = blaze::subvector<0UL, 3UL>(this->m_q);
}

// function that sets which method to use for solving the BVP
void CTR::setBVPMethod(const mathOp::rootFindingMethod &mthd)
{
	this->m_method = mthd;
}

// function that performs readings to acquire EM Data from the NDI EM tracker
std::tuple<blaze::StaticVector<double, 3UL>, blaze::StaticVector<double, 3UL>> CTR::acquireEMData()
{
	blaze::StaticVector<double, 3UL> position_CTR, position_CTR_KF;
	// Acquires EM readins of the sensors
	this->m_EMTrack->Get_TipPosition(&position_CTR, &position_CTR_KF);

	return std::make_tuple(position_CTR * 1.00E-3, position_CTR_KF * 1.00E-3);
}

// function that actuates the CTR motors to a particular configuration
void CTR::actuateMotors(const blaze::StaticVector<double, 6UL> &q_Current)
{
	// write your own routine here to command the configuration q_Current to the motors
	// std::cout << "Update the function " << __PRETTY_FUNCTION__ << " with your own code!" << std::endl;
	// auto q = q2qRobot(q_Current);
	// this->m_rbt->Set_Target_AbsPosition(q);
	// std::cout << "Target sent to the robot:   R1: " << q[0] << "    T1: " << q[1]  << "    R2: " << q[2]   << "    T2: " << q[3]<< " with your own code!" << std::endl;
	std::cout << "Update the function " << __PRETTY_FUNCTION__ << " with your own code!" << std::endl;
}