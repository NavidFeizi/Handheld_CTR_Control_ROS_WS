#include <vector>
#include <cmath>
#include <random>
#include <iostream>
#include <memory>
#include <algorithm>

enum class TrajectoryType
{
    MultiSine,
    NearStep,
    CyclycNearPulse
};

class TrajectoryGenerator
{
public:
    virtual ~TrajectoryGenerator() = default;
    virtual void flip_trajectory_sign() = 0;
    virtual void gen_trajectory() = 0;
    virtual std::vector<double> get_trajectory() = 0;
    virtual double get_next_signal() = 0;

protected:
    std::vector<double> m_trajectory; // Stored trajectory
    int m_currentIndex = 0;           // Current index in the trajectory for next signal
};

// Struct for MultiSineGenerator parameters
struct MultiSineParams
{
    double sample_time;
    int num_waves;
    double min_frequency;
    double max_frequency;
    double min_amplitude;
    double max_amplitude;
    double total_time;
};

// MultiSineGenerator class inheriting from TrajectoryGenerator
class MultiSineGenerator : public TrajectoryGenerator
{
public:
    MultiSineGenerator(const MultiSineParams &params)
        : m_params(params)
    {
        m_total_samples = static_cast<int>(m_params.total_time / m_params.sample_time);
    }

    void gen_trajectory() override
    {
        double t = 0.0; // Start time

        std::vector<double> temp_trajectory(m_total_samples, 0.0);
        m_trajectory.clear(); // Clear previous trajectory
        m_currentIndex = 0;   // Reset index

        // Random device and engine for generation
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> freqDist(m_params.min_frequency, m_params.max_frequency);
        std::uniform_real_distribution<> ampDist(m_params.min_amplitude, m_params.max_amplitude);
        std::uniform_real_distribution<> phaseDist(0, 2 * M_PI);

        // Generate random parameters for each wave
        std::vector<double> omegas(m_params.num_waves), amplitudes(m_params.num_waves), phases(m_params.num_waves);
        for (int i = 0; i < m_params.num_waves; ++i)
        {
            omegas[i] = freqDist(gen);
            amplitudes[i] = ampDist(gen);
            phases[i] = phaseDist(gen);
        }

        // Generate the trajectory
        for (int i = 0; i < m_total_samples; ++i)
        {
            double signal = 0.0;
            for (int j = 0; j < m_params.num_waves; ++j)
            {
                signal += amplitudes[j] * std::sin(omegas[j] * t * 2 * M_PI + phases[j]);
            }
            temp_trajectory[i] = signal;
            t += m_params.sample_time;
        }

        // Normalize and shift trajectory to desired range
        double minVal = *std::min_element(temp_trajectory.begin(), temp_trajectory.end());
        double maxVal = *std::max_element(temp_trajectory.begin(), temp_trajectory.end());
        double maxAbsVal = std::max(std::abs(maxVal), std::abs(minVal)); // Get the maximum of the absolute values
        for (auto &val : temp_trajectory)
        {
            val = val * m_params.max_amplitude / maxAbsVal; // Normalize values based on the maximum absolute value
        }
        m_trajectory = temp_trajectory; // Store the adjusted trajectory
    }

    void flip_trajectory_sign() override
    {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::bernoulli_distribution dist(0.5); // 50% chance to flip the sign of all elements

        if (dist(gen)) // Check if we should flip the signs
        {
            for (double &value : m_trajectory)
            {
                value *= -1; // Flip the sign
            }
        }
    }

    std::vector<double> get_trajectory() override
    {
        return m_trajectory;
    }

    double get_next_signal() override
    {
        if (m_currentIndex < m_total_samples)
        {
            return m_trajectory[m_currentIndex++];
        }
        else
        {
            // Handle end of trajectory; could loop, reset, or handle differently as needed
            return m_trajectory.back(); // Just return the last value as a simple approach
        }
    }

private:
    MultiSineParams m_params;
    int m_total_samples;
};

// Struct for LinearDecreaseGenerator parameters
struct LinearDecreaseParams
{
    double max_amplitude;
    int rise_samples;
    int hold_samples;
    int fall_samples;
    int total_samples;
};

// LinearDecreaseGenerator class inheriting from TrajectoryGenerator
class LinearDecreaseGenerator : public TrajectoryGenerator
{
public:
    LinearDecreaseGenerator(const LinearDecreaseParams &params)
        : m_params(params)
    {
        gen_trajectory();
    }

    void gen_trajectory() override
    {
        m_trajectory.clear(); // Clear previous trajectory
        m_currentIndex = 0;   // Reset index

        // Generate a random maximum value between 0 and initialMaxValue_
        std::random_device rd; // Seed with a real random value, if available
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis(0, m_params.max_amplitude);
        m_maxValue = dis(gen); // Assign a new random maxValue_

        std::vector<double> temp_trajectory(m_params.total_samples, 0.0);

        // Phase 1: Rise with a sinusoidal function
        for (int i = 0; i < m_params.rise_samples; ++i)
        {
            double fraction = static_cast<double>(i) / m_params.rise_samples;
            temp_trajectory[i] = m_maxValue * sin(fraction * M_PI / 2.0);
        }

        // Phase 2: Hold at maximum value
        for (int i = m_params.rise_samples; i < m_params.rise_samples + m_params.hold_samples; ++i)
        {
            temp_trajectory[i] = m_maxValue;
        }

        // Phase 3: Fall with a sinusoidal function
        for (int i = 0; i < m_params.fall_samples; ++i)
        {
            if (i + m_params.rise_samples + m_params.hold_samples < m_params.total_samples)
            { // Ensure not exceeding total_samples_
                double fraction = static_cast<double>(i) / m_params.fall_samples;
                temp_trajectory[i + m_params.rise_samples + m_params.hold_samples] = m_maxValue * sin((1.0 - fraction) * M_PI / 2.0);
            }
        }

        m_trajectory = temp_trajectory; // Store the adjusted trajectory
    }

    void flip_trajectory_sign() override
    {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::bernoulli_distribution dist(0.5); // 50% chance to flip the sign of all elements

        if (dist(gen)) // Check if we should flip the signs
        {
            for (double &value : m_trajectory)
            {
                value *= -1; // Flip the sign
            }
        }
    }

    std::vector<double> get_trajectory() override
    {
        return m_trajectory;
    }

    double get_next_signal() override
    {
        if (m_currentIndex < m_params.total_samples)
        {
            return m_trajectory[m_currentIndex++];
        }
        else
        {
            // Handle end of trajectory; could loop, reset, or handle differently as needed
            return m_trajectory.back(); // Just return the last value as a simple approach
        }
    }

private:
    LinearDecreaseParams m_params;
    double m_maxValue; // Current maximum value for the generated trajectory
};

//
struct MultiCycleParams
{
    double sample_time;
    double max_amplitude;
    double rise_duration;
    double unforced_time;
    double fall_vel;
    double cycle_period;
    double total_time;
};

//
class MultiCycleGenerator : public TrajectoryGenerator
{
public:
    MultiCycleGenerator(const MultiCycleParams &params)
        : m_params(params)
    {
        gen_trajectory();
    }

    void gen_trajectory() override
    {
        m_trajectory.clear(); // Clear previous trajectory
        m_currentIndex = 0;   // Reset index

        // Random device and engine for amplitude generation
        std::random_device rd; // Seed with a real random value, if available
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> amplituded_dist(0, m_params.max_amplitude);

        int total_samples = static_cast<int>(m_params.total_time / m_params.sample_time);
        std::vector<double> temp_trajectory(total_samples, 0.0);

        int num_cycle_smaples = static_cast<int>(m_params.cycle_period / m_params.sample_time);
        int cycle_count = static_cast<int>(m_params.total_time / m_params.cycle_period);

        for (int cycle = 0; cycle < cycle_count; ++cycle)
        {
            int cycle_start = cycle * num_cycle_smaples;
            double cycle_amplitude = amplituded_dist(gen); // Random amplitude for each cycle
            int rise_duration_samples = static_cast<int>(m_params.rise_duration / m_params.sample_time);
            int fall_duration_samples = static_cast<int>((cycle_amplitude / m_params.fall_vel) / m_params.sample_time) + 1;
            int unforced_sample = static_cast<int>(m_params.unforced_time / m_params.sample_time);

            // Phase 1: Rise with a sinusoidal function
            for (int i = 0; i < rise_duration_samples; ++i)
            {
                double fraction = static_cast<double>(i) / rise_duration_samples;
                temp_trajectory[cycle_start + i] = cycle_amplitude * sin(fraction * M_PI / 2.0);
            }

            // Phase 2: Hold at maximum value
            for (int i = rise_duration_samples; i < unforced_sample - fall_duration_samples; ++i)
            {
                temp_trajectory[cycle_start + i] = cycle_amplitude;
            }

            // Phase 3: Fall with a linear function adjusted by maxSlope
            for (int i = 0; i < fall_duration_samples; ++i)
            {
                if (i + unforced_sample - fall_duration_samples < num_cycle_smaples)
                { // Ensure not exceeding cycle length
                    double fraction = static_cast<double>(i) / fall_duration_samples;
                    temp_trajectory[cycle_start + i + unforced_sample - fall_duration_samples] = (1.0 - fraction) * cycle_amplitude;
                }
            }
        }

        m_trajectory = temp_trajectory; // Store the adjusted trajectory
    }

    void flip_trajectory_sign() override
    {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::bernoulli_distribution dist(0.5); // 50% chance to flip the sign of all elements

        if (dist(gen)) // Check if we should flip the signs
        {
            for (double &value : m_trajectory)
            {
                value *= -1; // Flip the sign
            }
        }
    }

    std::vector<double> get_trajectory() override
    {
        return m_trajectory;
    }

    double get_next_signal() override
    {
        if (m_currentIndex < static_cast<int>(m_params.total_time / m_params.sample_time))
        {
            return m_trajectory[m_currentIndex++];
        }
        else
        {
            // Handle end of trajectory; could loop, reset, or handle differently as needed
            return m_trajectory.back(); // Just return the last value as a simple approach
        }
    }

private:
    MultiCycleParams m_params;
};

// Factory function to create trajectory generator based on the type and parameters
std::unique_ptr<TrajectoryGenerator> createTrajectoryGenerator(TrajectoryType type,
                                                               const MultiSineParams &multiSineParams,
                                                               const LinearDecreaseParams &linearDecreaseParams,
                                                               const MultiCycleParams &multiCycleParams)
{
    if (type == TrajectoryType::MultiSine)
    {
        return std::make_unique<MultiSineGenerator>(multiSineParams);
    }
    else if (type == TrajectoryType::NearStep)
    {
        return std::make_unique<LinearDecreaseGenerator>(linearDecreaseParams);
    }
    else if (type == TrajectoryType::CyclycNearPulse)
    {
        return std::make_unique<MultiCycleGenerator>(multiCycleParams);
    }
    return nullptr;
}

// int main()
// {
//     TrajectoryType chosenType = TrajectoryType::MultiSine; // or TrajectoryType::LinearDecrease

//     MultiSineParams multiSineParams{0.01, 5, 0.5, 5.0, 0.1, 1.0, 500};
//     LinearDecreaseParams linearDecreaseParams{10.0, 50, 100, 50, 500};

//     auto generator = createTrajectoryGenerator(chosenType, multiSineParams, linearDecreaseParams);
//     if (generator)
//     {
//         generator->genTrajectory();
//         std::vector<double> trajectory = generator->getTrajectory();

//         for (const auto &value : trajectory)
//         {
//             std::cout << value << std::endl;
//         }
//     }

//     return 0;
// }
