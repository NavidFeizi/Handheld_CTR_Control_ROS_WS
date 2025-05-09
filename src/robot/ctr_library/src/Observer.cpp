// This is a personal academic project. Dear PVS-Studio, please check it.
// PVS-Studio Static Code Analyzer for C, C++, C#, and Java: http://www.viva64.com
#include "Observer.hpp"

// Overloaded constructor to initialize the Observer with state and arc length vectors.
Observer::Observer(std::vector<state_type>& states, std::vector<double>& s)
    : m_states(states), m_arcLength(s) {}

// Observer functor for capturing data from Boost::odeInt integrator
void Observer::operator()(const state_type& states, double s) noexcept
{
    m_states.emplace_back(states);
    m_arcLength.emplace_back(s);
}