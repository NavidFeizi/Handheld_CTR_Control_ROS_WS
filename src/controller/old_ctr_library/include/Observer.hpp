#pragma once

#include <blaze/Math.h>
#include <vector>

typedef blaze::StaticVector<double, 15UL> state_type;

/**
 * @brief Observer class to capture states and arc lengths from the Boost::odeInt integrator.
 */
class Observer
{
private:
    std::vector<state_type>& m_states; // Vector with all CTR states along the entire backbone
    std::vector<double>& m_arcLength;  // Vector with all discretized arc-length points along the backbone (CTR shape)

public:
    // Deleted default constructor
    Observer() = delete;

    /**
     * @brief Overloaded constructor to initialize the Observer with state and arc length vectors.
     * @param states Reference to the vector of state vectors.
     * @param s Reference to the vector of arc lengths.
     */
    Observer(std::vector<state_type>& states, std::vector<double>& s);

    // Class destructor
    ~Observer() = default;

    /**
     * @brief Functor to capture data from Boost::odeInt integrator.
     * @param states Current state vector.
     * @param s Current arc length.
     */
    void operator()(const state_type& states, double s) noexcept;
};