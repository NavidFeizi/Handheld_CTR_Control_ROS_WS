#pragma once
#include <vector>
#include <array>
#include "Tube.hpp"

class Segment
{
private:
	std::vector<double> m_S;										  // arc-length of each tube transition point
	blaze::HybridMatrix<double, 3UL, 18UL, blaze::columnMajor> m_EI;  // tubes bending stiffness in x,y directions
	blaze::HybridMatrix<double, 3UL, 18UL, blaze::columnMajor> m_GJ;  // tubes torsional stiffness
	blaze::HybridMatrix<double, 3UL, 18UL, blaze::columnMajor> m_U_x; // tubes' precurvature in the x  direction
	blaze::HybridMatrix<double, 3UL, 18UL, blaze::columnMajor> m_U_y; // tubes' precurvature in the y direction
	blaze::StaticVector<double, 3UL> m_len_curv;					  // arc-length at which precurvature starts in the tubes
	blaze::StaticVector<double, 3UL> m_dist_end;					  // arc-length of tubes' distal ends

public:
	// default class constructor
	Segment();

	// overloaded class constructor
	Segment(const std::array<std::shared_ptr<Tube>, 3UL> &Tb, const blaze::StaticVector<double, 3UL> &beta);

	// copy constructor
	Segment(const Segment &rhs);

	// move constructor
	Segment(Segment &&rhs) noexcept;

	// Segment desctructor
	~Segment() = default;

	double getDistStraightEndTb1()
	{
		return m_len_curv[0];
	}

	// copy assignment operator
	Segment &operator=(const Segment &rhs);

	// move assignment operator
	Segment &operator=(Segment &&rhs) noexcept;

	// implements a functor to overload the constructors signature and allow recalculation of the CTR segmentation
	void recalculateSegments(const std::array<std::shared_ptr<Tube>, 3> &Tb, const blaze::StaticVector<double, 3UL> &beta);

	// getter method for retrieving the transition points defining the boundaries of all CTR segments
	const std::vector<double>& get_S() const;

	// getter method for returning the distal ends of all CTR tubes
	const blaze::StaticVector<double, 3UL> &getDistalEnds() const;

	// getter method for retrieving the vectors of tube bending stiffness in all CTR segments
	const blaze::HybridMatrix<double, 3UL, 18UL, blaze::columnMajor> &get_EI() const;

	// getter method for retrieving the vectors of tube torional stiffness in all CTR segments
	const blaze::HybridMatrix<double, 3UL, 18UL, blaze::columnMajor> &get_GJ() const;

	// getter method for retrieving the vectors of tube precurvatures along X in all CTR segments
	const blaze::HybridMatrix<double, 3UL, 18UL, blaze::columnMajor> &get_U_x() const;

	// getter method for retrieving the vectors of tube precurvatures along Y in all CTR segments
	const blaze::HybridMatrix<double, 3UL, 18UL, blaze::columnMajor> &get_U_y() const;

	// method for returning all parameters along all CTR segments
	std::tuple<const blaze::HybridMatrix<double, 3UL, 18UL, blaze::columnMajor> &,
			   const blaze::HybridMatrix<double, 3UL, 18UL, blaze::columnMajor> &,
			   const blaze::HybridMatrix<double, 3UL, 18UL, blaze::columnMajor> &,
			   const blaze::HybridMatrix<double, 3UL, 18UL, blaze::columnMajor> &,
			   const std::vector<double> &>
	returnParameters() const;
};