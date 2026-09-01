#ifndef CTR_KINEMATICS_PINN__CTR_PINN_INFERENCE_HPP_
#define CTR_KINEMATICS_PINN__CTR_PINN_INFERENCE_HPP_

#include <torch/torch.h>
#include <torch/script.h>

#include <blaze/Math.h>
#include <blaze/Forward.h>

#include <iostream>
#include <memory>
#include <cassert>
#include <cmath>
#include <fstream>
#include <sstream>
#include <array>
#include <vector>
#include <type_traits>
#include <iomanip>
#include <algorithm>
#include <cstdint>
#include <random>
#include <tuple>

#include <nlohmann/json.hpp>

#include "ctr_kinematics_pinn/dataset_bounds.hpp"

// Custom exception for parameter loading errors
struct ParameterLoadError : public std::runtime_error
{
    using std::runtime_error::runtime_error;
};

struct PhysicsParameters
{
    std::vector<double> E, G, rc, Ls, Lc, Do, Di, L;
};

struct ModelParameters
{
    std::vector<int> tau_index;
    std::vector<int> layers;
};

struct DatasetParameters
{
    std::array<double, 2UL> beta1_range;
    std::array<double, 2UL> beta2_range;
    std::array<double, 2UL> beta3_range;
    std::array<double, 2UL> alpha1_range;
    std::array<double, 2UL> alpha2_range;
    std::array<double, 2UL> alpha3_range;
};

/// Optional, purely observational report from one posCTRL solve. posCTRL is
/// otherwise a black box -- it returns void and its iteration count, restart
/// count and conditioning are invisible to callers, which makes convergence
/// impossible to measure. Pass one of these to get the numbers out.
struct IkDiagnostics
{
    size_t iterations = 0UL;         ///< descent steps consumed across all attempts
    size_t restarts = 0UL;           ///< re-seeds used (0 = solved from the caller's guess)
    size_t clampedSteps = 0UL;       ///< steps where a prismatic joint hit its window edge
    size_t alphaCappedSteps = 0UL;   ///< steps where a revolute rate hit the trust-region cap
    size_t nonMonotonicSteps = 0UL;  ///< steps where the tip error GREW
    double maxJinvNorm = 0.00;       ///< largest ||J^+||_F seen; a singularity proxy
    double initialError = 0.00;      ///< ||target - tip|| at the caller's initial guess
    double finalError = 0.00;        ///< ||target - tip|| of the returned configuration
    bool converged = false;          ///< finalError <= posTol

    // --- domain telemetry (4-DoF only; zeros otherwise) ---
    double maxAbsAlpha2Queried = 0.00;   ///< largest |α₂| the network was asked to evaluate.
                                         ///< Tripwire: > alpha2_range means extrapolation.
    double maxAbsAlphaRelQueried = 0.00; ///< largest |α₁ − α₂| queried; > π means extrapolation.
    std::array<double, 4UL> preProjection{};      ///< best configuration BEFORE the final projection
    std::array<double, 4UL> projectionDelta{};    ///< finish() adjustment per joint (0 = untouched)
};

namespace detail
{
    inline std::vector<int> vec_int_or_throw(const nlohmann::json &obj, const char *key)
    {
        if (!obj.contains(key))
            throw std::runtime_error(std::string("Missing key '") + key + "' in model_params");
        const auto &v = obj.at(key);
        if (!v.is_array())
            throw std::runtime_error(std::string("Key '") + key + "' must be an array");

        std::vector<int> out;
        out.reserve(v.size());
        for (size_t i = 0; i < v.size(); ++i)
        {
            if (!v[i].is_number_integer())
                throw std::runtime_error(std::string("Key '") + key + "' element " + std::to_string(i) + " is not an integer");
            out.push_back(v[i].get<int>());
        }
        return out;
    }

    inline std::vector<double> vec_double_or_throw(const nlohmann::json &obj, const char *key)
    {
        if (!obj.contains(key))
            throw std::runtime_error(std::string("Missing key '") + key + "' in model_params");
        const auto &v = obj.at(key);
        if (!v.is_array())
            throw std::runtime_error(std::string("Key '") + key + "' must be an array");

        std::vector<double> out;
        out.reserve(v.size());
        for (size_t i = 0; i < v.size(); ++i)
        {
            if (!v[i].is_number())
                throw std::runtime_error(std::string("Key '") + key + "' element " + std::to_string(i) + " is not a number");
            out.push_back(v[i].get<double>());
        }
        return out;
    }
}

// function that converts blaze static vector of input size to torch tensor
template <size_t N>
void blazeVec2Tensor(const blaze::StaticVector<double, N> &vec, torch::Tensor &tensor);

template <size_t controlInputs>
class PINNs
{
private:
    // formatting constants
    static constexpr size_t kStateDim = 15UL;
    static constexpr size_t kPositionStartCol = 8UL;
    static constexpr size_t kForceDim = 3UL;

public:
    // Constructor takes model name
    explicit PINNs(std::string models_dir, std::string model_name, size_t batch_size, size_t num_nodes);

    // Disable copy to avoid accidental heavy copies (shared_ptr for module is cheap)
    PINNs(const PINNs &) = delete;
    PINNs &operator=(const PINNs &) = delete;

    // Move allowed
    PINNs(PINNs &&) = default;
    PINNs &operator=(PINNs &&) = default;

    //-------------------- Public inference API --------------------//

    /// @brief function that computes the distal position given control inputs
    /// @param tau [controlInputs] input control inputs
    /// @param wf [kForceDim] input force vector
    /// @param pos [3] output distal position
    void getPosDistal(const blaze::StaticVector<double, controlInputs> &tau,
                      const blaze::StaticVector<double, kForceDim> &wf,
                      blaze::StaticVector<double, 3UL> &pos) const;

    /// @brief function that computes the distal position given control inputs
    /// @param tau [controlInputs] input control inputs
    /// @param wf [kForceDim] input force vector
    /// @param pos [7] output distal position
    void getPosDistal(const blaze::StaticVector<double, controlInputs> &tau,
                      const blaze::StaticVector<double, kForceDim> &wf,
                      blaze::StaticVector<double, 7UL> &pos) const;

    /// @brief function that computes the positions of the three tubes' distal ends given control inputs
    /// @param tau [controlInputs] input control inputs
    /// @param wf [kForceDim] input force vector
    /// @param pos_t3 [3] output distal position of tube 3
    /// @param pos_t2 [3] output distal position of tube 2
    /// @param pos_t1 [3] output distal position of tube 1
    void getPosTubes(const blaze::StaticVector<double, controlInputs> &tau,
                     const blaze::StaticVector<double, kForceDim> &wf,
                     blaze::StaticVector<double, 3UL> &pos_t3,
                     blaze::StaticVector<double, 3UL> &pos_t2,
                     blaze::StaticVector<double, 3UL> &pos_t1) const;

    /// @brief function that computes the shape of the CTR given control inputs
    /// @param tau [controlInputs] input control inputs batch
    /// @param wf [kForceDim] input force vector
    /// @param pos [num_nodes, 3] output shape batch
    void getShape(const blaze::StaticVector<double, controlInputs> &tau,
                  const blaze::StaticVector<double, kForceDim> &wf,
                  blaze::DynamicMatrix<double, blaze::rowMajor> &shape) const;

    /// @brief function that computes the shapes of all three tubes given control inputs
    /// @param tau [controlInputs] input control inputs
    /// @param wf [kForceDim] input force vector
    /// @return tuple of three matrices representing the shapes of tube 3, tube 2, and tube 1
    std::tuple<blaze::DynamicMatrix<double, blaze::rowMajor>, blaze::DynamicMatrix<double, blaze::rowMajor>, blaze::DynamicMatrix<double, blaze::rowMajor>> getAllTubesShape(const blaze::StaticVector<double, controlInputs> &tau, const blaze::StaticVector<double, kForceDim> &wf) const;

    /// @brief function that computes the entire state of the CTR given control inputs
    /// @param tau [controlInputs] input control inputs
    /// @param wf [kForceDim] input force vector
    /// @param states [num_nodes, 15] output entire state batch
    void getEntireState(const blaze::StaticVector<double, controlInputs> &tau,
                        const blaze::StaticVector<double, kForceDim> &wf,
                        blaze::DynamicMatrix<double, blaze::rowMajor> &states) const;

    /// @brief function that computes the Jacobian of the distal position w.r.t. control inputs using autograd
    /// @param tau [controlInputs] input control inputs
    /// @param J [3, controlInputs] output Jacobian matrix
    void jacobian(const blaze::StaticVector<double, controlInputs> &tau,
                  blaze::StaticMatrix<double, 3UL, controlInputs, blaze::columnMajor> &J) const;

    /// @brief function that computes the Jacobian of the distal position w.r.t. control inputs using autograd
    /// @param tau [controlInputs] input control inputs
    /// @param wf [kForceDim] input force vector
    /// @param J [3, controlInputs] output Jacobian matrix
    void jacobian(const blaze::StaticVector<double, controlInputs> &tau,
                  const blaze::StaticVector<double, kForceDim> &wf,
                  blaze::StaticMatrix<double, 3UL, controlInputs, blaze::columnMajor> &J) const;

    /// @brief function that computes the Jacobian of the distal position w.r.t. control inputs using autograd in batched mode
    /// @param tau_batch [Batch, controlInputs] input control inputs batch
    /// @param J [Batch*3, Batch*controlInputs] output Jacobian matrix with (m_batch_size * 3 * controlInputs) non-zero
    void jacobianBatched(const blaze::DynamicMatrix<double, blaze::rowMajor> &tau_batch,
                         blaze::CompressedMatrix<double, blaze::rowMajor> &J) const;

    /// @brief function that computes the Jacobian of the distal position w.r.t. control inputs using finite differences
    /// @param tau [controlInputs] input control inputs
    /// @param J [3, controlInputs] output Jacobian matrix
    void jacobianFinDif(const blaze::StaticVector<double, controlInputs> &tau,
                        blaze::StaticMatrix<double, 3UL, controlInputs, blaze::columnMajor> &J) const;

    /// @brief function that computes the Jacobian of the distal position w.r.t. control inputs using autograd
    /// @param tau [controlInputs] input control inputs
    /// @param wf [kForceDim] input force vector
    /// @param J [3, controlInputs] output Jacobian matrix
    void jacobian_wrt_force(const blaze::StaticVector<double, controlInputs> &tau,
                            const blaze::StaticVector<double, kForceDim> &wf,
                            blaze::StaticMatrix<double, 3UL, kForceDim, blaze::columnMajor> &J) const;

    /// @brief function that computes the Jacobian of the distal position w.r.t. control inputs using autograd
    /// @param tau [controlInputs] input control inputs
    /// @param wf [kForceDim] input force vector
    /// @param J [3, controlInputs] output Jacobian matrix
    void jacobian_wrt_force(const blaze::StaticVector<double, controlInputs> &tau,
                            const blaze::StaticVector<double, kForceDim> &wf,
                            blaze::StaticMatrix<double, 7UL, kForceDim, blaze::columnMajor> &J) const;

    /// @brief function that computes the distal position given control inputs
    /// @param tau [controlInputs] input control inputs
    /// @param pos [3] output distal position
    void getPosDistal(const blaze::StaticVector<double, controlInputs> &tau,
                      blaze::StaticVector<double, 3UL> &pos) const;

    /// @brief function that computes the distal position given control inputs in batched mode
    /// @param tau_batch [Batch, controlInputs] input control inputs batch
    /// @param pos_batch [Batch, 3] output distal position batch
    void getPosDistalBatched(const blaze::DynamicMatrix<double, blaze::rowMajor> &tau_batch,
                             blaze::DynamicMatrix<double, blaze::rowMajor> &pos_batch) const;

    /// @brief function that computes the positions of the three tubes' distal ends given control inputs
    /// @param tau [controlInputs] input control inputs
    /// @param pos_t3 [3] output distal position of tube 3
    /// @param pos_t2 [3] output distal position of tube 2
    /// @param pos_t1 [3] output distal position of tube 1
    void getPosTubes(const blaze::StaticVector<double, controlInputs> &tau,
                     blaze::StaticVector<double, 3UL> &pos_t3,
                     blaze::StaticVector<double, 3UL> &pos_t2,
                     blaze::StaticVector<double, 3UL> &pos_t1) const;

    /// @brief function that computes the shape of the CTR given control inputs
    /// @param tau [controlInputs] input control inputs batch
    /// @param pos [num_nodes, 3] output shape batch
    void getShape(const blaze::StaticVector<double, controlInputs> &tau,
                  blaze::DynamicMatrix<double, blaze::rowMajor> &shape) const;

    /// @brief function that computes the entire state of the CTR given control inputs
    /// @param tau [controlInputs] input control inputs
    /// @param states [num_nodes, 15] output entire state batch
    void getEntireState(const blaze::StaticVector<double, controlInputs> &tau,
                        blaze::DynamicMatrix<double, blaze::rowMajor> &states) const;

    // function that returns the arclength at the ends of the CTR
    [[nodiscard]] blaze::StaticVector<double, 3UL> getArclengthEnd(const blaze::StaticVector<double, controlInputs> &tau) const;

    // function that returns the lengths of the straight sections of the CTR component tubes
    [[nodiscard]] blaze::StaticVector<double, 3UL> getStraightLen() const;

    // function that returns the overall lengths of the CTRcomponent tubes
    [[nodiscard]] blaze::StaticVector<double, 3UL> getOverallLen() const;

    /// ABSOLUTE joint box bounds in the robot's own joint frame, ordered
    /// [β₁, β₂, α₁, α₂] (4 inputs) or [β₁, β₂, β₃, α₁, α₂, α₃] (6 inputs).
    /// For 4 inputs β₁ is converted out of the dataset's β₂-relative frame -- see
    /// dataset_bounds.hpp. Use this for anything that constrains a joint value
    /// directly (OMPL bounds, validity checks, IK joint-limit avoidance).
    [[nodiscard]] std::tuple<blaze::StaticVector<double, controlInputs>, blaze::StaticVector<double, controlInputs>> getInputPosBounds() const;

    /// The training dataset's own sampling ranges, verbatim. For 4 inputs the β₁
    /// entry is the β₂-RELATIVE coupling window, not an absolute bound. Use this
    /// only where the caller re-applies the coupling itself by shifting the
    /// window by the live β₂ (ctr_common::clampJointPositions).
    [[nodiscard]] std::tuple<blaze::StaticVector<double, controlInputs>, blaze::StaticVector<double, controlInputs>> getDatasetInputRanges() const;

    /// The feasible joint set in the shared form (see dataset_bounds.hpp). Use this
    /// anywhere a configuration must be validated, so every component agrees on
    /// what "legal" means. 4-DoF layout only.
    [[nodiscard]] const ctr_kinematics_pinn::JointLimits4 &getJointLimits4() const { return m_jointLimits4; }

    // function that returns the number of nodes (discrete points) along the CTR backbone
    [[nodiscard]] size_t getNumNodes() const { return m_num_nodes; }

    // function that returns the allowable range of travel for the prismatic joints of the CTR
    [[nodiscard]] blaze::StaticVector<double, controlInputs> getPrismaticJointRanges() const;

    // function that returns the allowable range of travel for the revolute joints of the CTR
    [[nodiscard]] blaze::StaticVector<double, controlInputs> getRevoluteJointRanges() const;

    // function that returns the stage thickness of the linear guide actuators
    [[nodiscard]] double getStageThickness() const;

    // damped pseudoinverse of a 3x6 Jacobian: M+ = M^T (M M^T + lambda*I)^-1
    blaze::StaticMatrix<double, 6UL, 3UL, blaze::columnMajor> pInv(const blaze::StaticMatrix<double, 3UL, 6UL, blaze::columnMajor> &M);

    // Generic pseudoinverse for any 3xN Jacobian -> Nx3 result
    template <size_t N>
    static blaze::StaticMatrix<double, N, 3UL, blaze::columnMajor> pInvN(const blaze::StaticMatrix<double, 3UL, N, blaze::columnMajor> &M);

    // ---------------- Inverse Kinematics ----------------//

    // resolved-rate inverse kinematics for the CTR under an external tip force
    // wf (the Jacobian and forward model are both evaluated on the loaded robot)
    void posCTRL(blaze::StaticVector<double, controlInputs> &tau, const blaze::StaticVector<double, 3UL> &target, const double posTol,
                 const blaze::StaticVector<double, kForceDim> &wf, IkDiagnostics *diag = nullptr);

    // zero-force overload kept for callers that do not model tip loads
    void posCTRL(blaze::StaticVector<double, controlInputs> &tau, const blaze::StaticVector<double, 3UL> &target, const double posTol);

private:
    size_t m_batch_size;
    size_t m_num_nodes;

    // model
    std::shared_ptr<torch::jit::Module> m_dnn;
    PhysicsParameters m_physics_params{};
    ModelParameters m_model_params{};
    // Set from parameters.json's layers[0] at load: true for the force-aware
    // input layout [s, tau, wf], false for the force-free [s, tau]. See
    // loadParameters().
    bool m_model_takes_force = true;
    DatasetParameters m_dataset_params{};
    // The feasible joint set, cached in the shared form so posCTRL and the
    // planner's validity checker cannot disagree about it. 4-DoF layout only.
    ctr_kinematics_pinn::JointLimits4 m_jointLimits4{};

    // inference buffers (mutable so const methods can reuse them)
    mutable torch::Tensor s_buffer_;              // [1, 1]
    mutable torch::Tensor q_buffer_;              // [1, 6]
    mutable torch::Tensor wf_buffer_;             // [1, 3]
    mutable torch::Tensor input_buffer_;          // [1, 7]
    mutable torch::Tensor output_buffer_;         // [1, 15]
    mutable torch::Tensor s_buffer_batched_;      // [B, 1]
    mutable torch::Tensor q_buffer_batched_;      // [B, 6]
    mutable torch::Tensor wf_buffer_batched_;     // [B, 3]
    mutable torch::Tensor input_buffer_batched_;  // [B, 7]
    mutable torch::Tensor output_buffer_batched_; // [B, 15]

    // temp
    std::array<int64_t, controlInputs> tau_index_map_; // compile-time size
    size_t tau_input_size_;

    //-------------------- helper functions --------------------//

    // function that converts blaze dynamic matrix of batched input size to torch tensor
    void blazeMat2Tensor(const blaze::DynamicMatrix<double, blaze::rowMajor> &tau_batch, torch::Tensor &tau_tensor_batched) const;

    // loads model, physics, and dataset parameters from JSON file
    void loadParameters(const std::string &params_path);

    // loads the scripted model from file
    std::shared_ptr<torch::jit::Module> loadModel(const std::string &model_path);
};

template <size_t N>
void blazeVec2Tensor(const blaze::StaticVector<double, N> &vec, torch::Tensor &tensor)
{
    assert(tensor.numel() == N && "Output tensor size mismatch");

    // copy blaze rowMajor matrix to torch::tensor buffer
    double *dest = tensor.data_ptr<double>();
    const double *src = vec.data();
    const std::size_t count = N * sizeof(double);
    std::memcpy(dest, src, count);
}

template <size_t controlInputs>
PINNs<controlInputs>::PINNs(std::string models_dir, std::string model_name, size_t batch_size, size_t num_nodes)
{
    std::cout << "Initiating '" << model_name << "' PINN..." << std::endl;

    torch::set_num_threads(2);
    const std::string model_dir = std::move(models_dir);

    // Load the models
    const std::string model_path = model_dir + '/' + model_name + "/model_scripted.pt";
    m_dnn = loadModel(model_path);

    // Load the parameters
    const std::string params_path = model_dir + '/' + model_name + "/parameters.json";
    loadParameters(params_path);

    if constexpr (controlInputs == 4)
    {
        m_jointLimits4.beta2_absolute  = m_dataset_params.beta2_range;
        m_jointLimits4.beta1_relative  = m_dataset_params.beta1_range;  // RELATIVE to β₂
        m_jointLimits4.alpha2_absolute = m_dataset_params.alpha2_range; // α₃ ≡ 0, so absolute
        m_jointLimits4.alpha1_relative = m_dataset_params.alpha1_range; // RELATIVE to α₂
    }

    m_batch_size = batch_size;
    m_num_nodes = num_nodes;

    // tau_input_size_ = m_model_params.tau_index.size();

    // // --- Validate and fill index map ---
    // tau_index_map_.fill(-1); // sentinel
    // for (size_t i = 0; i < tau_input_size_; ++i)
    // {
    //     int idx = m_model_params.tau_index[i];
    //     if (idx < 0 || static_cast<size_t>(idx) >= controlInputs)
    //         throw std::out_of_range("Invalid tau_index: " + std::to_string(idx));
    //     tau_index_map_[i] = idx;
    // }

    // Reserve buffers
    s_buffer_ = torch::empty({1UL, 1UL}, torch::TensorOptions().dtype(torch::kFloat64));
    q_buffer_ = torch::empty({1UL, static_cast<int64_t>(controlInputs)}, torch::TensorOptions().dtype(torch::kFloat64));
    wf_buffer_ = torch::empty({1UL, static_cast<int64_t>(kForceDim)}, torch::TensorOptions().dtype(torch::kFloat64));
    input_buffer_ = torch::empty({1UL, static_cast<int64_t>(1UL + kForceDim + controlInputs)}, torch::TensorOptions().dtype(torch::kFloat64));
    output_buffer_ = torch::empty({1UL, static_cast<int64_t>(kStateDim)}, torch::TensorOptions().dtype(torch::kFloat64));

    s_buffer_batched_ = torch::empty({static_cast<int64_t>(m_batch_size), 1UL}, torch::TensorOptions().dtype(torch::kFloat64));
    q_buffer_batched_ = torch::empty({static_cast<int64_t>(m_batch_size), static_cast<int64_t>(controlInputs)}, torch::TensorOptions().dtype(torch::kFloat64));
    wf_buffer_batched_ = torch::empty({static_cast<int64_t>(m_batch_size), static_cast<int64_t>(kForceDim)}, torch::TensorOptions().dtype(torch::kFloat64));
    input_buffer_batched_ = torch::empty({static_cast<int64_t>(m_batch_size), static_cast<int64_t>(1UL + kForceDim + controlInputs)}, torch::TensorOptions().dtype(torch::kFloat64));
    output_buffer_batched_ = torch::empty({static_cast<int64_t>(m_batch_size), static_cast<int64_t>(kStateDim)}, torch::TensorOptions().dtype(torch::kFloat64));
}

template <size_t controlInputs>
void PINNs<controlInputs>::blazeMat2Tensor(const blaze::DynamicMatrix<double, blaze::rowMajor> &tau_batch, torch::Tensor &tau_tensor_batched) const
{
    assert(tau_batch.rows() == m_batch_size && "Control input batch size mismatch");
    assert(tau_batch.columns() == controlInputs && "Control input size mismatch");
    assert(tau_tensor_batched.numel() == m_batch_size * controlInputs && "Output tensor size mismatch");

    const std::size_t rows = tau_batch.rows();
    const std::size_t cols = tau_batch.columns();
    const std::size_t spacing = tau_batch.spacing();

    const double *src = tau_batch.data();
    double *dest = tau_tensor_batched.data_ptr<double>();

    for (std::size_t i = 0; i < rows; ++i)
    {
        std::memcpy(dest + i * cols,        // contiguous in Tensor
                    src + i * spacing,      // skip Blaze padding
                    cols * sizeof(double)); // only real entries
    }

    // // copy blaze rowMajor matrix to torch::tensor buffer - does not work propoerly dues to blaze padding
    // double *dest = tau_tensor_batched.data_ptr<double>();
    // const double *src = tau_batch.data();
    // const std::size_t count = m_batch_size * controlInputs * sizeof(double);
    // std::memcpy(dest, src, count);

    // std::cout << "tau_batch: \n" << tau_batch << std::endl;
    // std::cout << "tau_tensor_batched: \n" << tau_tensor_batched << std::endl;
}

template <size_t controlInputs>
void PINNs<controlInputs>::getPosDistal(const blaze::StaticVector<double, controlInputs> &tau, const blaze::StaticVector<double, kForceDim> &wf, blaze::StaticVector<double, 3UL> &pos) const
{
    torch::InferenceMode guard;

    blazeVec2Tensor(tau, q_buffer_);
    blazeVec2Tensor(wf, wf_buffer_);
    s_buffer_.fill_(m_physics_params.L[0] + tau[0]);
    torch::cat_out(input_buffer_, {s_buffer_, q_buffer_, wf_buffer_}, 1);
    output_buffer_ = m_dnn->forward({input_buffer_}).toTensor(); // (1, kStateDim)
    const double *src = output_buffer_.data_ptr<double>() + kPositionStartCol;
    std::memcpy(pos.data(), src, 3UL * sizeof(double)); // (1, 3UL)
}

template <size_t controlInputs>
void PINNs<controlInputs>::getPosDistal(const blaze::StaticVector<double, controlInputs> &tau, const blaze::StaticVector<double, kForceDim> &wf, blaze::StaticVector<double, 7UL> &pos) const
{
    torch::InferenceMode guard;

    blazeVec2Tensor(tau, q_buffer_);
    blazeVec2Tensor(wf, wf_buffer_);
    s_buffer_.fill_(m_physics_params.L[0] + tau[0]);
    torch::cat_out(input_buffer_, {s_buffer_, q_buffer_, wf_buffer_}, 1);
    output_buffer_ = m_dnn->forward({input_buffer_}).toTensor(); // (1, kStateDim)
    const double *src = output_buffer_.data_ptr<double>() + kPositionStartCol;
    std::memcpy(pos.data(), src, 7UL * sizeof(double)); // (1, 7UL)
}

template <size_t controlInputs>
void PINNs<controlInputs>::jacobian(const blaze::StaticVector<double, controlInputs> &tau, const blaze::StaticVector<double, kForceDim> &wf, blaze::StaticMatrix<double, 3UL, controlInputs, blaze::columnMajor> &Jac) const
{
    using namespace torch::indexing;

    // TODO: use the same buffers as getPosDistal to avoid repeated allocations
    torch::Tensor q = torch::empty({1UL, static_cast<int64_t>(controlInputs)}, torch::TensorOptions().dtype(torch::kFloat64));
    torch::Tensor wf_buffer = torch::empty({1UL, static_cast<int64_t>(kForceDim)}, torch::TensorOptions().dtype(torch::kFloat64));
    torch::Tensor s = torch::empty({1UL, 1UL}, torch::TensorOptions().dtype(torch::kFloat64));

    blazeVec2Tensor(tau, q);
    blazeVec2Tensor(wf, wf_buffer);
    q.set_requires_grad(true);
    s.fill_(m_physics_params.L[0]);
    // s = torch::full({1, 1}, m_physics_params.L[0], torch::TensorOptions().dtype(torch::kFloat64));
    s.add_(q.index({Slice(), Slice(0, 1)}));
    torch::Tensor input = torch::cat({s, q, wf_buffer}, 1);
    torch::Tensor output = m_dnn->forward({input}).toTensor();                                        // (kStateDim,)
    torch::Tensor y = output.index({Slice(), Slice(kPositionStartCol, kPositionStartCol + 3UL)}); // (3,)

    // 3) Compute J (3 x N)
    for (int64_t k = 0; k < 3UL; ++k)
    {
        auto go = torch::zeros_like(y);   // (3,)
        go.index_put_({Slice(), k}, 1.0); // one-hot: d y_k
        auto gradk = torch::autograd::grad(
            /*outputs=*/{y},
            /*inputs=*/{q},
            /*grad_outputs=*/{go},
            /*retain_graph=*/(k < 2),
            /*create_graph=*/false)[0]; // (N,)

        for (size_t j = 0; j < controlInputs; ++j)
        {
            Jac(k, j) = gradk[0][j].item<double>();
        }
    }
}

template <size_t controlInputs>
void PINNs<controlInputs>::jacobian_wrt_force(const blaze::StaticVector<double, controlInputs> &tau, const blaze::StaticVector<double, kForceDim> &wf, blaze::StaticMatrix<double, 3UL, kForceDim, blaze::columnMajor> &Jac) const
{
    using namespace torch::indexing;

    // TODO: use the same buffers as getPosDistal to avoid repeated allocations
    torch::Tensor q = torch::empty({1UL, static_cast<int64_t>(controlInputs)}, torch::TensorOptions().dtype(torch::kFloat64));
    torch::Tensor wf_buffer = torch::empty({1UL, static_cast<int64_t>(kForceDim)}, torch::TensorOptions().dtype(torch::kFloat64));
    torch::Tensor s = torch::empty({1UL, 1UL}, torch::TensorOptions().dtype(torch::kFloat64));

    blazeVec2Tensor(tau, q);
    blazeVec2Tensor(wf, wf_buffer);
    wf_buffer.set_requires_grad(true);
    s.fill_(m_physics_params.L[0]);
    // s = torch::full({1, 1}, m_physics_params.L[0], torch::TensorOptions().dtype(torch::kFloat64));
    s.add_(q.index({Slice(), Slice(0, 1)}));
    torch::Tensor input = torch::cat({s, q, wf_buffer}, 1);
    torch::Tensor output = m_dnn->forward({input}).toTensor();                                        // (kStateDim,)
    torch::Tensor y = output.index({Slice(), Slice(kPositionStartCol, kPositionStartCol + 3UL)}); // (3,)

    // 3) Compute J (3 x N)
    for (int64_t k = 0; k < 3UL; ++k)
    {
        auto go = torch::zeros_like(y);   // (3,)
        go.index_put_({Slice(), k}, 1.0); // one-hot: d y_k
        auto gradk = torch::autograd::grad(
            /*outputs=*/{y},
            /*inputs=*/{wf_buffer},
            /*grad_outputs=*/{go},
            /*retain_graph=*/(k < 2),
            /*create_graph=*/false)[0]; // (N,)

        for (size_t j = 0; j < kForceDim; ++j)
        {
            Jac(k, j) = gradk[0][j].item<double>();
        }
    }
}

template <size_t controlInputs>
void PINNs<controlInputs>::jacobian_wrt_force(const blaze::StaticVector<double, controlInputs> &tau, const blaze::StaticVector<double, kForceDim> &wf, blaze::StaticMatrix<double, 7UL, kForceDim, blaze::columnMajor> &Jac) const
{
    using namespace torch::indexing;

    // TODO: use the same buffers as getPosDistal to avoid repeated allocations
    torch::Tensor q = torch::empty({1UL, static_cast<int64_t>(controlInputs)}, torch::TensorOptions().dtype(torch::kFloat64));
    torch::Tensor wf_buffer = torch::empty({1UL, static_cast<int64_t>(kForceDim)}, torch::TensorOptions().dtype(torch::kFloat64));
    torch::Tensor s = torch::empty({1UL, 1UL}, torch::TensorOptions().dtype(torch::kFloat64));

    blazeVec2Tensor(tau, q);
    blazeVec2Tensor(wf, wf_buffer);
    wf_buffer.set_requires_grad(true);
    s.fill_(m_physics_params.L[0]);
    // s = torch::full({1, 1}, m_physics_params.L[0], torch::TensorOptions().dtype(torch::kFloat64));
    s.add_(q.index({Slice(), Slice(0, 1)}));
    torch::Tensor input = torch::cat({s, q, wf_buffer}, 1);
    torch::Tensor output = m_dnn->forward({input}).toTensor();                                        // (kStateDim,)
    torch::Tensor y = output.index({Slice(), Slice(kPositionStartCol, kPositionStartCol + 7UL)}); // (3,)

    // 3) Compute J (3 x N)
    for (int64_t k = 0; k < 3UL; ++k)
    {
        auto go = torch::zeros_like(y);   // (3,)
        go.index_put_({Slice(), k}, 1.0); // one-hot: d y_k
        auto gradk = torch::autograd::grad(
            /*outputs=*/{y},
            /*inputs=*/{wf_buffer},
            /*grad_outputs=*/{go},
            /*retain_graph=*/(k < 6),
            /*create_graph=*/false)[0]; // (N,)

        for (size_t j = 0; j < kForceDim; ++j)
        {
            Jac(k, j) = gradk[0][j].item<double>();
        }
    }
}

template <size_t controlInputs>
void PINNs<controlInputs>::getPosTubes(const blaze::StaticVector<double, controlInputs> &tau, const blaze::StaticVector<double, kForceDim> &wf, blaze::StaticVector<double, 3UL> &pos_1, blaze::StaticVector<double, 3UL> &pos_2, blaze::StaticVector<double, 3UL> &pos_3) const
{
    torch::InferenceMode guard; // <‑‑ disables grad completely

    // Assemble the input tensor (batch, s + num_tendon)
    torch::Tensor s = torch::tensor({m_physics_params.L[2] + tau[2], m_physics_params.L[1] + tau[1], m_physics_params.L[0] + tau[0]}, torch::TensorOptions().dtype(torch::kFloat64)).unsqueeze(1);
    torch::Tensor tau_row = torch::empty({1UL, static_cast<int64_t>(controlInputs)}, torch::TensorOptions().dtype(torch::kFloat64));
    torch::Tensor wf_row = torch::empty({1UL, static_cast<int64_t>(kForceDim)}, torch::TensorOptions().dtype(torch::kFloat64));

    blazeVec2Tensor(tau, tau_row);                   // (1, K), float64 on CPU
    blazeVec2Tensor(wf, wf_row);                     // (1, K), float64 on CPU
    torch::Tensor tau_cols = tau_row.repeat({3, 1}); // (Nodes, K)
    torch::Tensor wf_cols = wf_row.repeat({3, 1});   // (Nodes, K)
    torch::Tensor input = torch::cat({s, tau_cols, wf_cols}, /*dim=*/1);
    torch::Tensor states = m_dnn->forward({input}).toTensor(); // for some unknown reasons! it automatically does the normalization!

    for (size_t j = kPositionStartCol; j < kPositionStartCol + 3UL; ++j)
    {
        pos_1[j - kPositionStartCol] = states.index({(long)0, (long)j}).item<double>();
        pos_2[j - kPositionStartCol] = states.index({(long)1, (long)j}).item<double>();
        pos_3[j - kPositionStartCol] = states.index({(long)2, (long)j}).item<double>();
    }
}

template <size_t controlInputs>
void PINNs<controlInputs>::getEntireState(const blaze::StaticVector<double, controlInputs> &tau, const blaze::StaticVector<double, kForceDim> &wf, blaze::DynamicMatrix<double, blaze::rowMajor> &states) const
{
    if (states.rows() != m_num_nodes || states.columns() != kStateDim)
    {
        std::cout << "States size mismatch: expected [" << m_num_nodes << ", " << kStateDim
                  << "], got [" << states.rows() << ", " << states.columns() << "]" << std::endl;
        assert(false && "States output size mismatch");
    }

    torch::InferenceMode guard; // <‑‑ disables grad completely

    // Assemble the input tensor (batch, s + num_tendon)
    torch::Tensor s = torch::linspace(0.0, m_physics_params.L[0] + tau[0], m_num_nodes, torch::kFloat64).unsqueeze(1);
    torch::Tensor tau_row = torch::empty({1UL, static_cast<int64_t>(controlInputs)}, torch::TensorOptions().dtype(torch::kFloat64));
    torch::Tensor wf_row = torch::empty({1UL, static_cast<int64_t>(kForceDim)}, torch::TensorOptions().dtype(torch::kFloat64));
    blazeVec2Tensor(tau, tau_row);                                                     // (1, K), float64 on CPU
    blazeVec2Tensor(wf, wf_row);                                                       // (1, F), float64 on CPU
    torch::Tensor tau_cols = tau_row.repeat({static_cast<int64_t>(m_num_nodes), 1UL}); // (Nodes, K)
    torch::Tensor wf_cols = wf_row.repeat({static_cast<int64_t>(m_num_nodes), 1UL});   // (Nodes, F)
    torch::Tensor input = torch::cat({s, tau_cols, wf_cols}, /*dim=*/1);
    torch::Tensor states_torch = m_dnn->forward({input}).toTensor(); // for some unknown reasons! it automatically does the normalization!

    std::memcpy(states.data(), states_torch.data_ptr<double>(), m_num_nodes * kStateDim * sizeof(double));
}

template <size_t controlInputs>
void PINNs<controlInputs>::getShape(const blaze::StaticVector<double, controlInputs> &tau, const blaze::StaticVector<double, kForceDim> &wf, blaze::DynamicMatrix<double, blaze::rowMajor> &shape) const
{
    if (shape.rows() != m_num_nodes || shape.columns() != 3UL)
    {
        std::cout << "Shape size mismatch: expected [" << m_num_nodes << ", " << 3UL
                  << "], got [" << shape.rows() << ", " << shape.columns() << "]" << std::endl;
        assert(false && "Shape output size mismatch");
    }

    torch::InferenceMode guard; // <‑‑ disables grad completely

    // Assemble the input tensor (batch, s + num_tendon)
    torch::Tensor s = torch::linspace(0.0, m_physics_params.L[0] + tau[0], m_num_nodes, torch::kFloat64).unsqueeze(1);
    torch::Tensor tau_row = torch::empty({1UL, static_cast<int64_t>(controlInputs)}, torch::TensorOptions().dtype(torch::kFloat64));
    torch::Tensor wf_row = torch::empty({1UL, static_cast<int64_t>(kForceDim)}, torch::TensorOptions().dtype(torch::kFloat64));
    blazeVec2Tensor(tau, tau_row);                                                     // (1, K), float64 on CPU
    blazeVec2Tensor(wf, wf_row);                                                       // (1, kForceDim), float64 on CPU
    torch::Tensor tau_cols = tau_row.repeat({static_cast<int64_t>(m_num_nodes), 1UL}); // (Nodes, K)
    torch::Tensor wf_cols = wf_row.repeat({static_cast<int64_t>(m_num_nodes), 1UL});   // (Nodes, kForceDim)
    torch::Tensor input = torch::cat({s, tau_cols, wf_cols}, /*dim=*/1);
    torch::Tensor states_torch = m_dnn->forward({input}).toTensor(); // for some unknown reasons! it automatically does the normalization!

    for (size_t i = 0; i < m_num_nodes; ++i)
    {
        for (size_t j = kPositionStartCol; j < kPositionStartCol + 3UL; ++j)
        {
            shape(i, j - kPositionStartCol) = states_torch.index({(long)i, (long)j}).item<double>();
        }
    }
}

template <size_t controlInputs>
std::tuple<blaze::DynamicMatrix<double, blaze::rowMajor>, blaze::DynamicMatrix<double, blaze::rowMajor>, blaze::DynamicMatrix<double, blaze::rowMajor>> PINNs<controlInputs>::getAllTubesShape(const blaze::StaticVector<double, controlInputs> &tau, const blaze::StaticVector<double, kForceDim> &wf) const
{
    torch::InferenceMode guard; // <‑‑ disables grad completely

    blaze::DynamicMatrix<double, blaze::rowMajor> shape_tube1(m_num_nodes, 3UL), shape_tube2(m_num_nodes, 3UL), shape_tube3(m_num_nodes, 3UL);

    torch::Tensor tau_row = torch::empty({1UL, static_cast<int64_t>(controlInputs)}, torch::TensorOptions().dtype(torch::kFloat64));
    torch::Tensor wf_row = torch::empty({1UL, static_cast<int64_t>(kForceDim)}, torch::TensorOptions().dtype(torch::kFloat64));
    blazeVec2Tensor(tau, tau_row); // (1, K), float64 on CPU
    blazeVec2Tensor(wf, wf_row);
    torch::Tensor tau_cols = tau_row.repeat({static_cast<int64_t>(m_num_nodes), 1UL}); // (Nodes, K)
    torch::Tensor wf_cols = wf_row.repeat({static_cast<int64_t>(m_num_nodes), 1UL});   // (Nodes, kForceDim)

    // Inner tube
    {
        torch::Tensor s_1 = torch::linspace(0.00, m_physics_params.L[0UL] + tau[0UL], m_num_nodes, torch::kFloat64).unsqueeze(1);
        torch::Tensor input = torch::cat({s_1, tau_cols, wf_cols}, 1);
        torch::Tensor states_torch = m_dnn->forward({input}).toTensor();
        torch::Tensor shape_torch = states_torch.index({torch::indexing::Slice(), torch::indexing::Slice(8, 11)}).contiguous();
        // Access tensor data as 2D array [Nodes x 3]
        auto accessor = shape_torch.accessor<double, 2>();
        // Copy column by column to respect Blaze’s column-major layout
        for (size_t j = 0; j < 3; ++j)
            for (size_t i = 0; i < m_num_nodes; ++i)
                shape_tube1(i, j) = accessor[i][j];
    }

    // Middle tube
    {
        torch::Tensor s_2 = torch::linspace(0.00, m_physics_params.L[1UL] + tau[1UL], m_num_nodes, torch::kFloat64).unsqueeze(1);
        torch::Tensor input = torch::cat({s_2, tau_cols, wf_cols}, 1);
        torch::Tensor states_torch = m_dnn->forward({input}).toTensor();
        torch::Tensor shape_torch = states_torch.index({torch::indexing::Slice(), torch::indexing::Slice(8, 11)}).contiguous();
        // Access tensor data as 2D array [Nodes x 3]
        auto accessor = shape_torch.accessor<double, 2>();
        // Copy column by column to respect Blaze’s column-major layout
        for (size_t j = 0; j < 3; ++j)
            for (size_t i = 0; i < m_num_nodes; ++i)
                shape_tube2(i, j) = accessor[i][j];
    }

    // Outer tube
    {
        torch::Tensor s_3 = torch::linspace(0.00, m_physics_params.L[2UL] + 0.0, m_num_nodes, torch::kFloat64).unsqueeze(1);
        torch::Tensor input = torch::cat({s_3, tau_cols, wf_cols}, 1);
        torch::Tensor states_torch = m_dnn->forward({input}).toTensor();
        torch::Tensor shape_torch = states_torch.index({torch::indexing::Slice(), torch::indexing::Slice(8, 11)}).contiguous();
        // Access tensor data as 2D array [Nodes x 3]
        auto accessor = shape_torch.accessor<double, 2>();
        // Copy column by column to respect Blaze’s column-major layout
        for (size_t j = 0; j < 3; ++j)
            for (size_t i = 0; i < m_num_nodes; ++i)
                shape_tube3(i, j) = accessor[i][j];
    }

    return std::make_tuple(shape_tube1, shape_tube2, shape_tube3);
}

// ==== Overload without force input ==== //
template <size_t controlInputs>
void PINNs<controlInputs>::getPosDistal(const blaze::StaticVector<double, controlInputs> &tau, blaze::StaticVector<double, 3UL> &pos) const
{
    torch::InferenceMode guard;

    blazeVec2Tensor(tau, q_buffer_);
    s_buffer_.fill_(m_physics_params.L[0] + tau[0]);
    torch::cat_out(input_buffer_, {s_buffer_, q_buffer_}, 1);
    output_buffer_ = m_dnn->forward({input_buffer_}).toTensor(); // (1, kStateDim)
    const double *src = output_buffer_.data_ptr<double>() + kPositionStartCol;
    std::memcpy(pos.data(), src, 3UL * sizeof(double)); // (1, 3UL)
}

template <size_t controlInputs>
void PINNs<controlInputs>::getPosDistalBatched(const blaze::DynamicMatrix<double, blaze::rowMajor> &tau_batch, blaze::DynamicMatrix<double, blaze::rowMajor> &pos_batch) const
{
    torch::InferenceMode guard; // <‑‑ disables grad completely
    using namespace torch::indexing;

    if (tau_batch.rows() != m_batch_size || tau_batch.columns() != controlInputs)
    {
        std::cout << "Input matrix size mismatch: expected [" << m_batch_size << ", " << controlInputs
                  << "], got [" << tau_batch.rows() << ", " << tau_batch.columns() << "]" << std::endl;
        assert(false && "Input matrix size mismatch");
    }
    if (pos_batch.rows() != m_batch_size || pos_batch.columns() != 3UL)
    {
        std::cout << "Output matrix size mismatch: expected [" << m_batch_size << ", " << 3UL
                  << "], got [" << pos_batch.rows() << ", " << pos_batch.columns() << "]" << std::endl;
        assert(false && "Output matrix size mismatch");
    }

    torch::Tensor q_buffer_batched = torch::empty({static_cast<int64_t>(m_batch_size), static_cast<int64_t>(controlInputs)}, torch::TensorOptions().dtype(torch::kFloat64));
    torch::Tensor s_buffer_batched = torch::empty({static_cast<int64_t>(m_batch_size), 1UL}, torch::TensorOptions().dtype(torch::kFloat64));
    torch::Tensor input_buffer_batched = torch::empty({static_cast<int64_t>(m_batch_size), controlInputs + 1UL}, torch::TensorOptions().dtype(torch::kFloat64));

    blazeMat2Tensor(tau_batch, q_buffer_batched);
    s_buffer_batched.fill_(m_physics_params.L[0]);
    s_buffer_batched.add_(q_buffer_batched.index({Slice(), Slice(0, 1)}));
    torch::cat_out(input_buffer_batched, {s_buffer_batched, q_buffer_batched}, 1);
    torch::Tensor output_buffer_batched = m_dnn->forward({input_buffer_batched}).toTensor(); // [B, kStateDim]

    const double *src = output_buffer_batched.data_ptr<double>();
    for (size_t i = 0; i < m_batch_size; ++i)
    {
        const double *row = src + i * kStateDim;
        pos_batch(i, 0) = row[kPositionStartCol];     // x
        pos_batch(i, 1) = row[kPositionStartCol + 1]; // y
        pos_batch(i, 2) = row[kPositionStartCol + 2]; // z
    }
}

template <size_t controlInputs>
void PINNs<controlInputs>::jacobian(const blaze::StaticVector<double, controlInputs> &tau, blaze::StaticMatrix<double, 3UL, controlInputs, blaze::columnMajor> &Jac) const
{
    using namespace torch::indexing;

    // TODO: use the same buffers as getPosDistal to avoid repeated allocations
    torch::Tensor q = torch::empty({1UL, static_cast<int64_t>(controlInputs)}, torch::TensorOptions().dtype(torch::kFloat64));
    torch::Tensor s = torch::empty({1UL, 1UL}, torch::TensorOptions().dtype(torch::kFloat64));

    blazeVec2Tensor(tau, q);
    q.set_requires_grad(true);
    s.fill_(m_physics_params.L[0]);
    // s = torch::full({1, 1}, m_physics_params.L[0], torch::TensorOptions().dtype(torch::kFloat64));
    s.add_(q.index({Slice(), Slice(0, 1)}));
    torch::Tensor input = torch::cat({s, q}, 1);
    torch::Tensor output = m_dnn->forward({input}).toTensor();                                        // (kStateDim,)
    torch::Tensor y = output.index({Slice(), Slice(kPositionStartCol, kPositionStartCol + 3UL)}); // (3,)

    // 3) Compute J (3 x N)
    for (int64_t k = 0; k < 3UL; ++k)
    {
        auto go = torch::zeros_like(y);   // (3,)
        go.index_put_({Slice(), k}, 1.0); // one-hot: d y_k
        auto gradk = torch::autograd::grad(
            /*outputs=*/{y},
            /*inputs=*/{q},
            /*grad_outputs=*/{go},
            /*retain_graph=*/(k < 2),
            /*create_graph=*/false)[0]; // (N,)

        for (size_t j = 0; j < controlInputs; ++j)
        {
            Jac(k, j) = gradk[0][j].item<double>();
        }
    }
}

template <size_t controlInputs>
void PINNs<controlInputs>::jacobianBatched(const blaze::DynamicMatrix<double, blaze::rowMajor> &tau_batch, blaze::CompressedMatrix<double, blaze::rowMajor> &J) const
{
    if (tau_batch.rows() != m_batch_size || tau_batch.columns() != controlInputs)
    {
        std::cout << "Input matrix size mismatch: expected [" << m_batch_size << ", " << controlInputs
                  << "], got [" << tau_batch.rows() << ", " << tau_batch.columns() << "]" << std::endl;
        assert(false && "Input matrix size mismatch");
    }
    if (J.rows() != m_batch_size * 3UL || J.columns() != m_batch_size * controlInputs)
    {
        std::cout << "Jacobian matrix size mismatch: expected [" << m_batch_size * 3UL << ", " << m_batch_size * controlInputs
                  << "], got [" << J.rows() << ", " << J.columns() << "]" << std::endl;
        assert(false && "Jacobian matrix size mismatch");
    }

    using namespace torch::indexing;

    torch::Tensor q = torch::empty({static_cast<int64_t>(m_batch_size), static_cast<int64_t>(controlInputs)}, torch::TensorOptions().dtype(torch::kFloat64));
    torch::Tensor s = torch::empty({static_cast<int64_t>(m_batch_size), 1UL}, torch::TensorOptions().dtype(torch::kFloat64));

    blazeMat2Tensor(tau_batch, q);
    q.set_requires_grad(true);
    s.fill_(m_physics_params.L[0]);
    s.add_(q.index({Slice(), Slice(0, 1)}));
    torch::Tensor input = torch::cat({s, q}, 1);                                                      // dim=0
    torch::Tensor output = m_dnn->forward({input}).toTensor();                                        // (B, kStateDim)
    torch::Tensor y = output.index({Slice(), Slice(kPositionStartCol, kPositionStartCol + 3UL)}); // (B, 3)

    // Compute per-output Jacobians: J_batch (B, 3, N)
    std::vector<torch::Tensor> J_parts;
    J_parts.reserve(3);

    // 3) Compute J (3 x N)
    for (int64_t k = 0; k < 3; ++k)
    {
        // grad_output: one-hot in column k, for all batch elements
        torch::Tensor go = torch::zeros_like(y); // (B,3)
        go.index_put_({Slice(), k}, 1.0);        // (B,3)

        auto gradk = torch::autograd::grad(
            /*outputs=*/{y},
            /*inputs=*/{q},
            /*grad_outputs=*/{go},
            /*retain_graph=*/(k < 2),
            /*create_graph=*/false)[0]; // (N,)

        J_parts.push_back(gradk);
    }

    torch::Tensor J_batch = torch::stack(J_parts, /*dim=*/1); // (B,3,N)

    for (size_t b = 0; b < m_batch_size; ++b)
    {
        for (size_t k = 0; k < 3UL; ++k)
        {
            auto row = J_batch[b][k]; // (N)
            for (size_t j = 0; j < static_cast<int>(controlInputs); ++j)
            {
                J(b * 3UL + k, b * controlInputs + j) = row[j].item<double>();
            }
        }
    }
}

template <size_t controlInputs>
void PINNs<controlInputs>::jacobianFinDif(const blaze::StaticVector<double, controlInputs> &tau, blaze::StaticMatrix<double, 3UL, controlInputs, blaze::columnMajor> &J) const
{
    torch::InferenceMode guard; // <‑‑ disables grad completely

    double eps = 1e-6;
    m_dnn->eval();
    for (size_t i = 0; i < controlInputs; ++i)
    {
        blaze::StaticVector<double, 3UL> pos_plus, pos_minus;
        blaze::StaticVector<double, controlInputs> tau_plus = tau;
        blaze::StaticVector<double, controlInputs> tau_minus = tau;
        tau_plus[i] += eps;
        tau_minus[i] -= eps;
        getPosDistal(tau_plus, pos_plus);
        getPosDistal(tau_minus, pos_minus);
        column(J, i) = (pos_plus - pos_minus) / (2.0 * eps);
    }
}

template <size_t controlInputs>
void PINNs<controlInputs>::getPosTubes(const blaze::StaticVector<double, controlInputs> &tau, blaze::StaticVector<double, 3UL> &pos_1, blaze::StaticVector<double, 3UL> &pos_2, blaze::StaticVector<double, 3UL> &pos_3) const
{
    torch::InferenceMode guard; // <‑‑ disables grad completely

    // Assemble the input tensor (batch, s + num_tendon)
    torch::Tensor s = torch::tensor({m_physics_params.L[2] + tau[2], m_physics_params.L[1] + tau[1], m_physics_params.L[0] + tau[0]}, torch::TensorOptions().dtype(torch::kFloat64)).unsqueeze(1);
    torch::Tensor tau_row = torch::empty({1UL, static_cast<int64_t>(controlInputs)}, torch::TensorOptions().dtype(torch::kFloat64));

    blazeVec2Tensor(tau, tau_row);                   // (1, K), float64 on CPU
    torch::Tensor tau_cols = tau_row.repeat({3, 1}); // (Nodes, K)
    torch::Tensor input = torch::cat({s, tau_cols}, /*dim=*/1);
    torch::Tensor states = m_dnn->forward({input}).toTensor(); // for some unknown reasons! it automatically does the normalization!

    for (size_t j = kPositionStartCol; j < kPositionStartCol + 3UL; ++j)
    {
        pos_1[j - kPositionStartCol] = states.index({(long)0, (long)j}).item<double>();
        pos_2[j - kPositionStartCol] = states.index({(long)1, (long)j}).item<double>();
        pos_3[j - kPositionStartCol] = states.index({(long)2, (long)j}).item<double>();
    }
}

template <size_t controlInputs>
void PINNs<controlInputs>::getEntireState(const blaze::StaticVector<double, controlInputs> &tau, blaze::DynamicMatrix<double, blaze::rowMajor> &states) const
{
    if (states.rows() != m_num_nodes || states.columns() != kStateDim)
    {
        std::cout << "States size mismatch: expected [" << m_num_nodes << ", " << kStateDim
                  << "], got [" << states.rows() << ", " << states.columns() << "]" << std::endl;
        assert(false && "States output size mismatch");
    }

    torch::InferenceMode guard; // <‑‑ disables grad completely

    // Assemble the input tensor (batch, s + num_tendon)
    torch::Tensor s = torch::linspace(0.0, m_physics_params.L[0] + tau[0], m_num_nodes, torch::kFloat64).unsqueeze(1);
    torch::Tensor tau_row = torch::empty({1UL, static_cast<int64_t>(controlInputs)}, torch::TensorOptions().dtype(torch::kFloat64));
    blazeVec2Tensor(tau, tau_row);                                                     // (1, K), float64 on CPU
    torch::Tensor tau_cols = tau_row.repeat({static_cast<int64_t>(m_num_nodes), 1UL}); // (Nodes, K)
    torch::Tensor input = torch::cat({s, tau_cols}, /*dim=*/1);
    torch::Tensor states_torch = m_dnn->forward({input}).toTensor(); // for some unknown reasons! it automatically does the normalization!

    std::memcpy(states.data(), states_torch.data_ptr<double>(), m_num_nodes * kStateDim * sizeof(double));
}

template <size_t controlInputs>
void PINNs<controlInputs>::getShape(const blaze::StaticVector<double, controlInputs> &tau, blaze::DynamicMatrix<double, blaze::rowMajor> &shape) const
{
    if (shape.rows() != m_num_nodes || shape.columns() != 3UL)
    {
        std::cout << "Shape size mismatch: expected [" << m_num_nodes << ", " << 3UL
                  << "], got [" << shape.rows() << ", " << shape.columns() << "]" << std::endl;
        assert(false && "Shape output size mismatch");
    }

    torch::InferenceMode guard; // <‑‑ disables grad completely

    // Assemble the input tensor (batch, s + num_tendon)
    torch::Tensor s = torch::linspace(0.0, m_physics_params.L[0] + tau[0], m_num_nodes, torch::kFloat64).unsqueeze(1);
    torch::Tensor tau_row = torch::empty({1UL, static_cast<int64_t>(controlInputs)}, torch::TensorOptions().dtype(torch::kFloat64));
    blazeVec2Tensor(tau, tau_row);                                                     // (1, K), float64 on CPU
    torch::Tensor tau_cols = tau_row.repeat({static_cast<int64_t>(m_num_nodes), 1UL}); // (Nodes, K)
    torch::Tensor input = torch::cat({s, tau_cols}, /*dim=*/1);
    torch::Tensor states_torch = m_dnn->forward({input}).toTensor(); // for some unknown reasons! it automatically does the normalization!

    for (size_t i = 0; i < m_num_nodes; ++i)
    {
        for (size_t j = kPositionStartCol; j < kPositionStartCol + 3UL; ++j)
        {
            shape(i, j - kPositionStartCol) = states_torch.index({(long)i, (long)j}).item<double>();
        }
    }
}

template <size_t controlInputs>
blaze::StaticVector<double, 3UL> PINNs<controlInputs>::getArclengthEnd(const blaze::StaticVector<double, controlInputs> &tau) const
{
    blaze::StaticVector<double, 3UL> ends;
    ends[0] = m_physics_params.L[0] + tau[0];
    ends[1] = m_physics_params.L[1] + tau[1];
    ends[2] = m_physics_params.L[2] + tau[2];
    return ends;
}

template <size_t controlInputs>
std::shared_ptr<torch::jit::Module> PINNs<controlInputs>::loadModel(const std::string &model_path)
{
    namespace fs = std::filesystem;
    TORCH_CHECK(fs::exists(model_path) && fs::is_regular_file(model_path), "FATAL: model file not found: ", model_path);
    auto module = std::make_shared<torch::jit::Module>(torch::jit::load(model_path, torch::kCPU));

    // Identify the weights, not just the directory name. Two model dirs in the
    // pool (..._v3 and ..._FP64) hold DIFFERENT networks of identical size whose
    // parameters.json files are byte-identical, so the directory name alone
    // cannot tell you which network a node is running. When the planner's IK
    // converges on one network and pinn_fk reports the tip from another, the
    // disagreement shows up as an unattributable tip error.
    const auto size_bytes = fs::file_size(model_path);
    std::size_t digest = 1469598103934665603ULL; // FNV-1a 64 offset basis
    {
        std::ifstream f(model_path, std::ios::binary);
        char buf[8192];
        while (f.read(buf, sizeof(buf)) || f.gcount() > 0)
        {
            const std::streamsize n = f.gcount();
            for (std::streamsize i = 0; i < n; ++i)
            {
                digest ^= static_cast<unsigned char>(buf[i]);
                digest *= 1099511628211ULL; // FNV-1a 64 prime
            }
        }
    }
    std::cout << "Model loaded successfully from:\n    " << model_path
              << "\n    size = " << size_bytes << " bytes, fnv1a64 = " << std::hex << digest
              << std::dec << std::endl;
    return module;
}

template <size_t controlInputs>
void PINNs<controlInputs>::loadParameters(const std::string &params_path)
{
    std::ifstream file(params_path);
    if (!file.is_open())
        throw std::runtime_error("Could not open parameters file: " + params_path);

    nlohmann::json json_data;
    file >> json_data;

    if (!json_data.contains("physics_params") || !json_data["physics_params"].is_object())
        throw std::runtime_error("Missing or invalid 'physics_params' section in JSON file");

    const auto &physics = json_data["physics_params"];
    m_physics_params.E = detail::vec_double_or_throw(physics, "young_modulus");
    m_physics_params.G = detail::vec_double_or_throw(physics, "shear_modulus");
    m_physics_params.rc = detail::vec_double_or_throw(physics, "curve_radius");
    m_physics_params.Ls = detail::vec_double_or_throw(physics, "straight_length");
    m_physics_params.Lc = detail::vec_double_or_throw(physics, "curve_length");
    m_physics_params.Do = detail::vec_double_or_throw(physics, "outer_diameter");
    m_physics_params.Di = detail::vec_double_or_throw(physics, "inner_diameter");

    // The loop below indexes Ls/Lc with a raw operator[] up to 3. A
    // parameters.json with a shorter array would be out-of-bounds UB, and a
    // garbage L[0] silently corrupts the arclength input `s` of every FK call --
    // producing a wrong or non-finite tip while the joint vector still looks
    // perfectly finite in any diagnostic.
    for (const auto &named : {std::pair<const char *, const std::vector<double> *>{"straight_length", &m_physics_params.Ls},
                              std::pair<const char *, const std::vector<double> *>{"curve_length", &m_physics_params.Lc},
                              std::pair<const char *, const std::vector<double> *>{"outer_diameter", &m_physics_params.Do},
                              std::pair<const char *, const std::vector<double> *>{"inner_diameter", &m_physics_params.Di}})
    {
        if (named.second->size() < 3UL)
            throw ParameterLoadError(std::string("physics_params.") + named.first + " must have 3 entries, got " +
                                     std::to_string(named.second->size()));
    }

    for (size_t i = 0; i < 3; ++i)
        m_physics_params.L.push_back(m_physics_params.Ls[i] + m_physics_params.Lc[i]);

    // ---- model_params (tau_index) ----
    if (!json_data.contains("model_params") || !json_data["model_params"].is_object())
        throw std::runtime_error("Missing or invalid 'model_params' section in JSON file");

    const auto &model = json_data["model_params"];
    m_model_params.tau_index = detail::vec_int_or_throw(model, "tau_idx");
    m_model_params.layers = detail::vec_int_or_throw(model, "layers");

    // `layers` was parsed, printed and then never used, so a model whose width
    // does not match this build went undetected until the first forward call
    // threw a LibTorch shape error from inside a timer callback.
    //
    // Two input layouts exist in the model pool:
    //   force-aware: [s, tau(controlInputs), wf(kForceDim)]  (ctr_8x91_*)
    //   force-free:  [s, tau(controlInputs)]                 (handheld_*, grassmann_*)
    // Both are legitimate -- there are getPosDistal() overloads for each -- so
    // accept either width and reject anything else. A force-free model is still
    // flagged, because every node in this workspace calls the force-aware
    // overload and would fail at the first inference.
    if (!m_model_params.layers.empty())
    {
        const int declared_in = m_model_params.layers.front();
        const int in_force_aware = static_cast<int>(1UL + controlInputs + kForceDim);
        const int in_force_free = static_cast<int>(1UL + controlInputs);

        if (declared_in == in_force_aware)
        {
            m_model_takes_force = true;
        }
        else if (declared_in == in_force_free)
        {
            m_model_takes_force = false;
            std::cerr << "WARNING: model declares a FORCE-FREE input layout (layers[0] = " << declared_in
                      << " = [s, tau(" << controlInputs << ")]). Every node in this workspace calls the "
                         "force-aware getPosDistal(tau, wf, ...), which needs "
                      << in_force_aware << " inputs and will fail on this model." << std::endl;
        }
        else
        {
            throw ParameterLoadError("model input width mismatch: parameters.json declares layers[0] = " +
                                     std::to_string(declared_in) + ", which is neither the force-aware width " +
                                     std::to_string(in_force_aware) + " [s, tau(" + std::to_string(controlInputs) +
                                     "), wf(" + std::to_string(kForceDim) + ")] nor the force-free width " +
                                     std::to_string(in_force_free) + " for controlInputs = " +
                                     std::to_string(controlInputs));
        }

        const int expected_out = static_cast<int>(kStateDim);
        if (m_model_params.layers.back() != expected_out)
            throw ParameterLoadError("model output width mismatch: parameters.json declares layers[last] = " +
                                     std::to_string(m_model_params.layers.back()) + " but this build reads " +
                                     std::to_string(expected_out) + " state columns");
    }

    // dataset params
    if (!json_data.contains("dataset_params") || !json_data["dataset_params"].is_object())
        throw ParameterLoadError("Missing or invalid 'dataset_params' section in JSON file");

    const auto &dataset = json_data["dataset_params"];
    m_dataset_params.beta1_range = {dataset.at("beta1_range").at(0).get<double>(), dataset.at("beta1_range").at(1).get<double>()};
    m_dataset_params.beta2_range = {dataset.at("beta2_range").at(0).get<double>(), dataset.at("beta2_range").at(1).get<double>()};
    m_dataset_params.beta3_range = {dataset.at("beta3_range").at(0).get<double>(), dataset.at("beta3_range").at(1).get<double>()};
    m_dataset_params.alpha1_range = {dataset.at("alpha1_range").at(0).get<double>(), dataset.at("alpha1_range").at(1).get<double>()};
    m_dataset_params.alpha2_range = {dataset.at("alpha2_range").at(0).get<double>(), dataset.at("alpha2_range").at(1).get<double>()};
    m_dataset_params.alpha3_range = {dataset.at("alpha3_range").at(0).get<double>(), dataset.at("alpha3_range").at(1).get<double>()};

    std::cout << "Parameters loaded successfully from:\n    " << params_path << std::endl;

    auto print_vec = [](const char *name, const auto &vec)
    {
        using T = typename std::decay_t<decltype(vec)>::value_type;
        static_assert(std::is_same_v<T, int> || std::is_same_v<T, double>, "print_vec only supports vector<int> or vector<double>");
        std::cout << name << "[";
        for (size_t i = 0; i < vec.size(); ++i)
        {
            if constexpr (std::is_same_v<T, double>)
                std::cout << std::setprecision(6) << vec[i];
            else
                std::cout << vec[i];
            if (i + 1 < vec.size())
                std::cout << ", ";
        }
        std::cout << "]\n";
    };

    std::cout << "Physics parameters:\n";
    print_vec("    E: ", m_physics_params.E);
    print_vec("    G: ", m_physics_params.G);
    print_vec("    rc: ", m_physics_params.rc);
    print_vec("    Ls: ", m_physics_params.Ls);
    print_vec("    Lc: ", m_physics_params.Lc);
    print_vec("    Do: ", m_physics_params.Do);
    print_vec("    Di: ", m_physics_params.Di);
    print_vec("    L: ", m_physics_params.L);
    // std::cout << std::endl;

    std::cout << "DNN parameters:\n";
    print_vec("    layers: ", m_model_params.layers);
    print_vec("    tau_index: ", m_model_params.tau_index);
    // std::cout << std::endl;

    std::cout << "Dataset parameters:\n";
    print_vec("    beta1_range: beta2 + ", m_dataset_params.beta1_range);
    print_vec("    beta2_range: beta3 + ", m_dataset_params.beta2_range);
    print_vec("    beta3_range: ", m_dataset_params.beta3_range);
    print_vec("    alpha1_range: alpha2 + ", m_dataset_params.alpha1_range);
    print_vec("    alpha2_range: alpha3 + ", m_dataset_params.alpha2_range);
    print_vec("    alpha3_range: ", m_dataset_params.alpha3_range);
    std::cout << std::endl;

    return;
}

template <size_t controlInputs>
blaze::StaticVector<double, 3UL> PINNs<controlInputs>::getStraightLen() const
{
    return {
        m_physics_params.Ls[0UL],
        m_physics_params.Ls[1UL],
        m_physics_params.Ls[2UL]};
}

template <size_t controlInputs>
blaze::StaticVector<double, 3UL> PINNs<controlInputs>::getOverallLen() const
{
    return {
        m_physics_params.Ls[0UL] + m_physics_params.Lc[0UL],
        m_physics_params.Ls[1UL] + m_physics_params.Lc[1UL],
        m_physics_params.Ls[2UL] + m_physics_params.Lc[2UL]};
}

template <size_t controlInputs>
std::tuple<blaze::StaticVector<double, controlInputs>, blaze::StaticVector<double, controlInputs>> PINNs<controlInputs>::getDatasetInputRanges() const
{
    blaze::StaticVector<double, controlInputs> lb;
    blaze::StaticVector<double, controlInputs> ub;

    if constexpr (controlInputs == 6)
    {
        lb[0UL] = m_dataset_params.beta1_range[0];
        lb[1UL] = m_dataset_params.beta2_range[0];
        lb[2UL] = m_dataset_params.beta3_range[0];
        lb[3UL] = m_dataset_params.alpha1_range[0];
        lb[4UL] = m_dataset_params.alpha2_range[0];
        lb[5UL] = m_dataset_params.alpha3_range[0];

        ub[0UL] = m_dataset_params.beta1_range[1];
        ub[1UL] = m_dataset_params.beta2_range[1];
        ub[2UL] = m_dataset_params.beta3_range[1];
        ub[3UL] = m_dataset_params.alpha1_range[1];
        ub[4UL] = m_dataset_params.alpha2_range[1];
        ub[5UL] = m_dataset_params.alpha3_range[1];
    }
    else if constexpr (controlInputs == 4)
    {
        lb[0UL] = m_dataset_params.beta1_range[0];
        lb[1UL] = m_dataset_params.beta2_range[0];
        lb[2UL] = m_dataset_params.alpha1_range[0];
        lb[3UL] = m_dataset_params.alpha2_range[0];

        ub[0UL] = m_dataset_params.beta1_range[1];
        ub[1UL] = m_dataset_params.beta2_range[1];
        ub[2UL] = m_dataset_params.alpha1_range[1];
        ub[3UL] = m_dataset_params.alpha2_range[1];
    }

    return std::make_tuple(lb, ub);
}

template <size_t controlInputs>
std::tuple<blaze::StaticVector<double, controlInputs>, blaze::StaticVector<double, controlInputs>> PINNs<controlInputs>::getInputPosBounds() const
{
    auto [lb, ub] = this->getDatasetInputRanges();

    if constexpr (controlInputs == 4)
    {
        // beta1_range is stored RELATIVE to beta2 in the 4-DoF datasets, so the
        // absolute box bound has to be recovered before use. Dropping this
        // conversion empties the reachable beta1 interval at the retracted pose,
        // and every setStartState() throws "Start State is invalid!".
        const auto beta1 = ctr_kinematics_pinn::absoluteBeta1Range(m_dataset_params.beta1_range,
                                                                   m_dataset_params.beta2_range);
        lb[0UL] = beta1[0UL];
        ub[0UL] = beta1[1UL];

        // alpha1_range is RELATIVE to α₂ in exactly the same way (the dataset
        // samples α₁ = α₂ + U[alpha1_range]); consuming it as an absolute box
        // used to pin α₁ to [-π, π] and spill α₂ out of the trained range.
        const auto alpha1 = ctr_kinematics_pinn::absoluteAlpha1Range(m_dataset_params.alpha1_range,
                                                                     m_dataset_params.alpha2_range);
        lb[2UL] = alpha1[0UL];
        ub[2UL] = alpha1[1UL];
    }

    return std::make_tuple(lb, ub);
}

// ---------------- grafted from the planner fork (PINNs.hpp) ----------------//

template <size_t controlInputs>
blaze::StaticVector<double, controlInputs> PINNs<controlInputs>::getPrismaticJointRanges() const
{
    // Packed as [min, max] pairs per actuated prismatic joint.
    if constexpr (controlInputs == 4)
    {
        // beta1_range stores RELATIVE offsets from beta2 (see dataset_bounds.hpp);
        // this getter reports absolute travel, so convert.
        const auto beta1 = ctr_kinematics_pinn::absoluteBeta1Range(m_dataset_params.beta1_range,
                                                                   m_dataset_params.beta2_range);
        return {
            beta1[0UL], beta1[1UL],
            m_dataset_params.beta2_range[0UL], m_dataset_params.beta2_range[1UL]};
    }
    else
    {
        return {
            m_dataset_params.beta1_range[0UL], m_dataset_params.beta1_range[1UL],  // min, max
            m_dataset_params.beta2_range[0UL], m_dataset_params.beta2_range[1UL],  // min, max
            m_dataset_params.beta3_range[0UL], m_dataset_params.beta3_range[1UL]}; // min, max
    }
}

template <size_t controlInputs>
blaze::StaticVector<double, controlInputs> PINNs<controlInputs>::getRevoluteJointRanges() const
{
    // Packed as [min, max] pairs per actuated revolute joint.
    if constexpr (controlInputs == 4)
    {
        return {
            m_dataset_params.alpha1_range[0UL], m_dataset_params.alpha1_range[1UL],
            m_dataset_params.alpha2_range[0UL], m_dataset_params.alpha2_range[1UL]};
    }
    else
    {
        return {
            m_dataset_params.alpha1_range[0UL], m_dataset_params.alpha1_range[1UL],  // min, max
            m_dataset_params.alpha2_range[0UL], m_dataset_params.alpha2_range[1UL],  // min, max
            m_dataset_params.alpha3_range[0UL], m_dataset_params.alpha3_range[1UL]}; // min, max
    }
}

template <size_t controlInputs>
double PINNs<controlInputs>::getStageThickness() const
{
    return 30.00E-3; // 30 mm
}

template <size_t controlInputs>
blaze::StaticMatrix<double, 6UL, 3UL, blaze::columnMajor>
PINNs<controlInputs>::pInv(const blaze::StaticMatrix<double, 3UL, 6UL, blaze::columnMajor> &M)
{
    // M+ = M^T(MM^T+λI)⁻¹
    static constexpr double lambda = 1.00E-12; // small damping factor (Tikhonov regularization)

    constexpr blaze::IdentityMatrix<double> I(3UL);

    // Compute: A = M * trans(M) + λI  (A is 3x3)
    const blaze::StaticMatrix<double, 3UL, 3UL, blaze::columnMajor> A = (M * blaze::trans(M)) + lambda * I;

    // Add damping to the diagonal
    // blaze::diagonal(A) += lambda;
    // A(i, i) += lambda;
    blaze::StaticMatrix<double, 3UL, 3UL, blaze::columnMajor> A_inv;

    try
    {
        // Compute the inverse of A (3x3)
        A_inv = blaze::inv(A);
    }
    catch (const std::exception &e)
    {
        std::cerr << "Matrix inversion failed: " << e.what() << std::endl;
        return blaze::StaticMatrix<double, 6UL, 3UL, blaze::columnMajor>(0.00);
    }

    // Compute pseudoinverse: M⁺ = trans(M) * A⁻¹   (6x3 result)
    return blaze::trans(M) * A_inv;
}

// Generic pseudoinverse: for any 3×N Jacobian → N×3 result
template <size_t controlInputs>
template <size_t N>
blaze::StaticMatrix<double, N, 3UL, blaze::columnMajor>
PINNs<controlInputs>::pInvN(const blaze::StaticMatrix<double, 3UL, N, blaze::columnMajor> &M)
{
    static constexpr double lambda = 1.00E-12;
    const blaze::IdentityMatrix<double, blaze::columnMajor> I3(3UL);
    const blaze::StaticMatrix<double, 3UL, 3UL, blaze::columnMajor> A = (M * blaze::trans(M)) + lambda * I3;
    blaze::StaticMatrix<double, 3UL, 3UL, blaze::columnMajor> A_inv;
    try
    {
        A_inv = blaze::inv(A);
    }
    catch (const std::exception &e)
    {
        std::cerr << "pInvN: matrix inversion failed: " << e.what() << std::endl;
        return blaze::StaticMatrix<double, N, 3UL, blaze::columnMajor>(0.0);
    }
    return blaze::trans(M) * A_inv;
}

template <size_t controlInputs>
void PINNs<controlInputs>::posCTRL(blaze::StaticVector<double, controlInputs> &tau, const blaze::StaticVector<double, 3UL> &target, const double posTol)
{
    this->posCTRL(tau, target, posTol, blaze::StaticVector<double, kForceDim>(0.00));
}

template <size_t controlInputs>
void PINNs<controlInputs>::posCTRL(blaze::StaticVector<double, controlInputs> &tau, const blaze::StaticVector<double, 3UL> &target, const double posTol,
                                   const blaze::StaticVector<double, kForceDim> &wf, IkDiagnostics *diag)
{
    // Local diagnostics, always maintained; copied out only if the caller asked.
    IkDiagnostics diag_local;
    // Writes the report and returns, so every exit path reports consistently.
    //
    // The projection is the load-bearing part: the caller feeds this straight to
    // Planner::setGoalState, which THROWS on an infeasible configuration, so a
    // solution that is 4 mm out is not a slightly-worse answer -- it is no plan at
    // all. Projecting here makes "posCTRL never returns a configuration the planner
    // rejects" true by construction rather than true on average. Order matters:
    // beta2 into its own absolute range first, then beta1 into the window that
    // beta2 implies (which carries both the clearance and protrusion constraints),
    // then the angles. The angles are WRAPPED, not clamped: α is periodic, so the
    // nearest feasible representative of an out-of-branch angle is a 2πk shift
    // away -- the same tube shape -- whereas a clamp to the branch edge is a real
    // rotation that silently turns a converged solve into a miss. α₂ anchors the
    // pair (it is the dataset's absolute axis); α₁ then lands in α₂ ± π, which
    // keeps the returned pair inside the trained box by construction.
    const auto finish = [&](blaze::StaticVector<double, controlInputs> &out,
                            const blaze::StaticVector<double, controlInputs> &best,
                            const double bestErr)
    {
        out = best;

        if constexpr (controlInputs == 4)
        {
            const auto &lim = m_jointLimits4;
            diag_local.preProjection = {best[0UL], best[1UL], best[2UL], best[3UL]};
            out[1UL] = std::clamp(out[1UL], lim.beta2_absolute[0UL], lim.beta2_absolute[1UL]);
            const auto w1 = ctr_kinematics_pinn::beta1Window(out[1UL], lim);
            out[0UL] = std::clamp(out[0UL], w1[0UL], w1[1UL]);
            out[3UL] = ctr_kinematics_pinn::wrapToPi(out[3UL]);
            out[2UL] = out[3UL] + ctr_kinematics_pinn::wrapToPi(out[2UL] - out[3UL]);
            for (size_t j = 0UL; j < 4UL; ++j)
                diag_local.projectionDelta[j] = out[j] - best[j];
        }

        // Report the error of what is actually returned. The projection can move the
        // configuration, so the pre-projection best would be an optimistic figure.
        double reportedErr = bestErr;
        if constexpr (controlInputs == 4)
        {
            blaze::StaticVector<double, 3UL> projTip;
            this->getPosDistal(out, wf, projTip);
            reportedErr = blaze::norm(target - projTip);
        }

        diag_local.finalError = reportedErr;
        diag_local.converged  = (reportedErr <= posTol);
        if (diag)
            *diag = diag_local;
    };

    blaze::StaticMatrix<double, 3UL, controlInputs, blaze::columnMajor> J;                       // Jacobian matrix (3 × controlInputs)
    blaze::StaticMatrix<double, controlInputs, 3UL, blaze::columnMajor> J_inv;                   // Jacobian pseudoinverse (controlInputs × 3)
    const blaze::IdentityMatrix<double, blaze::columnMajor> I(controlInputs);                    // Identity matrix

    // proportional, derivative, and integral gains for position control
    constexpr double kp = 1.000, ki = 0.05, kd = 0.001;

    const blaze::DiagonalMatrix<blaze::StaticMatrix<double, 3UL, 3UL, blaze::columnMajor>> Kp{
        {kp, 0.00, 0.00},
        {0.00, kp, 0.00},
        {0.00, 0.00, kp}};

    const blaze::DiagonalMatrix<blaze::StaticMatrix<double, 3UL, 3UL, blaze::columnMajor>> Ki{
        {ki, 0.00, 0.00},
        {0.00, ki, 0.00},
        {0.00, 0.00, ki}};

    const blaze::DiagonalMatrix<blaze::StaticMatrix<double, 3UL, 3UL, blaze::columnMajor>> Kd{
        {kd, 0.00, 0.00},
        {0.00, kd, 0.00},
        {0.00, 0.00, kd}};

    // ---------------- iteration budget ----------------
    // TOTAL number of descent steps this call may spend, summed over the initial
    // attempt and every re-seeded retry. This is the knob that bounds latency:
    // each step costs one Jacobian (a TorchScript forward plus three autograd
    // backward passes) and one forward pass.
    static constexpr size_t maxIter = 3000UL;
    // Per-attempt cap. Set from the measured distribution rather than by tradition:
    // attempts that succeed do so in well under 250 steps, while attempts that are
    // going to stall burn their whole allowance and contribute nothing. Restart
    // diversity is what actually rescues a hard target, so the same total budget
    // buys far more by funding many short tries instead of a few long ones.
    static constexpr size_t maxIterPerTry = 250UL;
    // Extra seeds tried after the caller's initial guess. 250 * (1 + 11) = maxIter.
    static constexpr size_t maxRestarts = 11UL;
    // Fixed RNG seed. Re-seeding has to be reproducible: the same target from the
    // same initial guess must always return the same joint vector, or two
    // identical plan requests would deploy the robot differently.
    static constexpr std::uint32_t restartSeed = 0x5EEDU;
    // parameters for local optimization (joint limits avoidance)
    static constexpr double ke = 4.00;
    // Fraction of a joint's half-window the limit-avoidance term may command in one
    // step. 1.0 deliberately: measurement showed the clamp rate is set by the TASK
    // step, not by this term (cutting the gain 10x left clamping at 83% unchanged),
    // while a weaker push made starts from a narrow window corner crawl -- near-target
    // convergence fell from 100% to 67%. What was actually wrong here was the
    // missing dimensional scaling below, not the strength.
    static constexpr double kNullspaceGain = 1.00;
    // Anti-windup ceiling on the integral term's contribution to the commanded
    // tip rate [m]. The accumulator is otherwise unbounded, and once ki * ∫e
    // outgrows kp * e the descent limit-cycles instead of converging -- so extra
    // iterations would buy oscillation rather than accuracy.
    static constexpr double integralTipCap = 2.00E-3;
    static constexpr double iLim = integralTipCap / ki;
    // Trust region on the revolute rates [rad/step]. The prismatic step has
    // always been limited (half the distance to the window edge); α had no cap
    // at all, and with λ = 1e-12 a near-singular J⁺ can command tens of radians
    // in one step, which the wrap then folds into an effectively random
    // configuration -- a teleport, not a descent step. 0.5 rad keeps the
    // linearisation honest while still crossing the whole branch within a
    // fraction of one 250-step attempt.
    static constexpr double kAlphaStepCap = 0.50;

    constexpr size_t nPrismatic = controlInputs / 2UL;

    // Best-seen configuration across ALL attempts. Hoisting this out of the retry
    // loop is what makes re-seeding safe: the value written back at the end can
    // never be worse than what a single descent would have produced.
    double minError = 1.00E3;

    // The caller's guess arrives in absolute motor angles that may sit whole
    // turns from the principal branch (the drives allow ±3π on α₁). Project the
    // α pair onto the trained branch BEFORE the first network query -- the first
    // forward/Jacobian pass used to run on the raw vector, i.e. potentially in
    // pure extrapolation. The shift is 2πk per tube (shape-preserving), and
    // finish() returns the principal branch regardless, so callers see no
    // representation change.
    if constexpr (controlInputs == 4)
    {
        tau[3UL] = ctr_kinematics_pinn::wrapToPi(tau[3UL]);
        tau[2UL] = tau[3UL] + ctr_kinematics_pinn::wrapToPi(tau[2UL] - tau[3UL]);
    }
    else // controlInputs == 6
    {
        tau[4UL] = ctr_kinematics_pinn::wrapToPi(tau[4UL]);
        tau[3UL] = tau[4UL] + ctr_kinematics_pinn::wrapToPi(tau[3UL] - tau[4UL]);
    }

    blaze::StaticVector<double, controlInputs> tau_min(tau);

    blaze::StaticVector<double, controlInputs> dtau_dt;
    blaze::StaticVector<double, 3UL> x_CTR;

    this->getPosDistal(tau, wf, x_CTR);

    blaze::StaticVector<double, 3UL> tipError = target - x_CTR;
    blaze::StaticVector<double, 3UL> last_tipError = tipError;
    blaze::StaticVector<double, 3UL> d_tipError, int_tipError;

    // Euclidean distance to target
    double dist2Tgt = blaze::norm(tipError);
    diag_local.initialError = dist2Tgt;

    if (dist2Tgt < minError)
    {
        minError = dist2Tgt;
        tau_min = tau;

        if (dist2Tgt <= posTol)
        {
            finish(tau, tau_min, minError);
            return;
        }
    }

    // Nullspace vector (joint-limit avoidance gradient), zero-initialised;
    // only the prismatic-joint entries are filled each iteration.
    blaze::StaticVector<double, controlInputs> f{0.0};

    // Only the 6-DoF branch of computeBetaBounds reads these now; the 4-DoF branch
    // gets its windows from the shared predicate.
    [[maybe_unused]] const blaze::StaticVector<double, 3UL> L  = this->getOverallLen();
    [[maybe_unused]] const blaze::StaticVector<double, 3UL> Ls = this->getStraightLen();
    [[maybe_unused]] const double stageThickness               = this->getStageThickness();

    // Absolute prismatic-joint bounds (beta1 already converted out of the
    // dataset's beta2-relative frame -- see dataset_bounds.hpp). Bound to plain
    // references rather than a structured binding: capturing a structured binding
    // in a lambda is only well-formed from C++20, and this is a C++17 workspace.
    const auto inputPosBounds = this->getInputPosBounds();
    [[maybe_unused]] const blaze::StaticVector<double, controlInputs> &lb = std::get<0UL>(inputPosBounds);
    [[maybe_unused]] const blaze::StaticVector<double, controlInputs> &ub = std::get<1UL>(inputPosBounds);

    // Prismatic joint limit vectors, recomputed every iteration because the
    // tube-ordering constraints couple each joint's window to its neighbours'.
    blaze::StaticVector<double, nPrismatic> betaMin, betaMax;

    // Single definition of the coupling algebra, shared by the descent and by the
    // re-seeding step so the two can never drift apart.
    auto computeBetaBounds = [&](const blaze::StaticVector<double, controlInputs> &tauCur,
                                 blaze::StaticVector<double, nPrismatic> &bMin,
                                 blaze::StaticVector<double, nPrismatic> &bMax)
    {
        if constexpr (controlInputs == 4)
        {
            // 2 prismatic joints: tauCur[0] = β₁ (inner), tauCur[1] = β₂ (middle).
            // Delegated to the shared predicate so this solver cannot drift out of
            // the set the planner accepts. The hand-rolled version this replaces
            // capped β₂ at -stageThickness (-0.030) and omitted β₂'s own dataset
            // ceiling (-0.034), so 21% of converged solves came back 4 mm inside a
            // band CTR_StateValidityChecker rejects -- setGoalState() then threw and
            // no plan was produced at all. Measured with benchmark/ik_bench.cpp.
            const auto w1 = ctr_kinematics_pinn::beta1Window(tauCur[1UL], m_jointLimits4);
            const auto w2 = ctr_kinematics_pinn::beta2Window(tauCur[0UL], m_jointLimits4);

            bMin[0UL] = w1[0UL];
            bMax[0UL] = w1[1UL];
            bMin[1UL] = w2[0UL];
            bMax[1UL] = w2[1UL];
        }
        else // controlInputs == 6
        {
            // 3 prismatic joints: tauCur[0]=β₁, tauCur[1]=β₂, tauCur[2]=β₃(static)
            const double b1 = tauCur[0UL], b2 = tauCur[1UL], b3 = tauCur[2UL];

            bMin[0UL] = std::max({-Ls[0UL], L[1UL] + b2 - L[0UL], L[2UL] + b3 - L[0UL]});
            bMin[1UL] = std::max({-Ls[1UL], b1 + stageThickness, L[2UL] + b3 - L[1UL]});
            bMin[2UL] = std::max(-Ls[2UL], b2 + stageThickness);
            bMax[0UL] = b2 - stageThickness;
            bMax[1UL] = std::min(b3 - stageThickness, L[0UL] + b1 - L[1UL]);
            bMax[2UL] = std::min(L[1UL] + b2 - L[2UL], L[0UL] + b1 - L[2UL]);
        }
    };

    // Helper: wrap an angle to any [low, high) interval
    auto wrapToRange = [](double theta, double low, double high) -> double
    {
        const double width     = high - low;
        const double inv_width = 1.00 / width;
        return low + (theta - low) - width * std::floor((theta - low) * inv_width);
    };

    // Enforce the revolute-joint invariants the PINN was trained under. The
    // dataset anchors the pair on α₂ (its range is absolute because α₃ ≡ 0) and
    // stores α₁ RELATIVE to it, so α₂ must be wrapped FIRST and α₁'s window then
    // anchored to the wrapped α₂. The previous ordering (α₁ absolute, α₂ anchored
    // to it) was the relative-vs-absolute mixup: it let α₂ walk out to ±2π, past
    // both the trained range (±1.5π) and the drives' physical travel, so the
    // network extrapolated exactly in the azimuthal wedge around α₁ ≈ ±π.
    // Wrapping α₂ to the principal branch [−π, π) keeps every queried pair
    // strictly inside the trained box.
    auto wrapAngles = [&](blaze::StaticVector<double, controlInputs> &tauCur)
    {
        if constexpr (controlInputs == 4)
        {
            // α₂ ∈ [−π, π)  (principal branch of the dataset's absolute axis)
            tauCur[3UL] = wrapToRange(tauCur[3UL], -M_PI, M_PI);
            // α₁ ∈ [α₂ − π, α₂ + π)  (the dataset's relative window)
            tauCur[2UL] = wrapToRange(tauCur[2UL], tauCur[3UL] - M_PI, tauCur[3UL] + M_PI);
        }
        else // controlInputs == 6
        {
            // β₃ (outermost tube) remains unactuated
            tauCur[2UL] = 0.00;
            // α₃ (outermost tube) remains unactuated
            tauCur[5UL] = 0.00;
            // α₂ ∈ [−π, π)
            tauCur[4UL] = wrapToRange(tauCur[4UL], -M_PI, M_PI);
            // α₁ ∈ [α₂ − π, α₂ + π)
            tauCur[3UL] = wrapToRange(tauCur[3UL], tauCur[4UL] - M_PI, tauCur[4UL] + M_PI);
        }
    };

    // Deterministic restart seeding. A resolved-rate descent is a local method, so
    // an attempt that stalls in a bad basin cannot be rescued by more steps -- only
    // by starting somewhere else.
    std::mt19937 rng(restartSeed);
    std::uniform_real_distribution<double> unit(0.00, 1.00);

    // Every restart draws uniformly. A deterministic mid-range seed for the first
    // retry was tried and measured WORSE (misses 3.2% -> 7.0%): spending one of the
    // attempts on a fixed point costs more in basin diversity than it gains in
    // avoiding the boundary corner the robot homes to. Restart diversity is what
    // does the work here, so keep all of them independent.
    auto reseed = [&](blaze::StaticVector<double, controlInputs> &tauCur)
    {
        if constexpr (controlInputs == 6)
            tauCur[2UL] = 0.00; // β₃ is static; never re-seed it

        // Prismatic joints in index order: each draw uses bounds evaluated on the
        // partially re-seeded vector, so the inter-tube coupling windows stay
        // consistent as we go.
        for (size_t i = 0UL; i < nPrismatic; ++i)
        {
            if constexpr (controlInputs == 6)
            {
                if (i == 2UL)
                    continue;
            }

            blaze::StaticVector<double, nPrismatic> bMin, bMax;
            computeBetaBounds(tauCur, bMin, bMax);

            tauCur[i] = (bMax[i] > bMin[i]) ? bMin[i] + unit(rng) * (bMax[i] - bMin[i])
                                            : 0.50 * (bMin[i] + bMax[i]); // degenerate window: midpoint is all there is
        }

        // Revolute joints: α₂ uniform on its principal branch, then α₁ anchored
        // to it -- the dataset's own sampling scheme, so every seed is trained-on.
        if constexpr (controlInputs == 4)
        {
            tauCur[3UL] = -M_PI + unit(rng) * 2.00 * M_PI;
            tauCur[2UL] = tauCur[3UL] - M_PI + unit(rng) * 2.00 * M_PI;
        }
        else // controlInputs == 6
        {
            tauCur[4UL] = -M_PI + unit(rng) * 2.00 * M_PI;
            tauCur[3UL] = tauCur[4UL] - M_PI + unit(rng) * 2.00 * M_PI;
        }

        wrapAngles(tauCur);
    };

    // total iterations consumed across every attempt
    size_t N_itr_total = 0UL;

    for (size_t attempt = 0UL; attempt <= maxRestarts; ++attempt)
    {
        if (attempt > 0UL)
        {
            diag_local.restarts++;
            reseed(tau);
            this->getPosDistal(tau, wf, x_CTR);

            tipError      = target - x_CTR;
            last_tipError = tipError;
            d_tipError    = blaze::StaticVector<double, 3UL>(0.00);
            // A wound-up integral carried across a restart would immediately
            // poison the new attempt, so the PID state resets with the seed.
            int_tipError  = blaze::StaticVector<double, 3UL>(0.00);
            dist2Tgt      = blaze::norm(tipError);

            if (dist2Tgt < minError)
            {
                minError = dist2Tgt;
                tau_min = tau;
            }
        }

        // iterations counter for this attempt
        size_t N_itr = 0UL;

        // position control loop
        while ((dist2Tgt > posTol) && (N_itr < maxIterPerTry) && (N_itr_total < maxIter))
        {
            // incrementing the number of iterations
            N_itr++;
            N_itr_total++;
            diag_local.iterations++;

            // Compute the Jacobian in the present configuration. x_CTR and tipError
            // are already current for this tau -- set before the loop, or at the
            // end of the previous iteration -- so no forward pass is needed here.
            this->jacobian(tau, wf, J);

            // Pseudo-inverse of Jacobian for resolving CTR joint motion rates
            J_inv = PINNs<controlInputs>::pInvN(J);

            // Conditioning proxy: with lambda = 1e-12 the damped pseudoinverse is
            // effectively undamped, so ||J^+|| blowing up is the signature of a
            // near-singular configuration producing a wild joint step.
            {
                const double jinvNorm = blaze::norm(J_inv);
                if (jinvNorm > diag_local.maxJinvNorm)
                    diag_local.maxJinvNorm = jinvNorm;
            }

            // Joint-limit windows and the nullspace gradient that pushes each
            // prismatic joint toward the middle of its own window.
            computeBetaBounds(tau, betaMin, betaMax);
            {
                const blaze::StaticVector<double, nPrismatic> beta = blaze::subvector<0UL, nPrismatic>(tau);
                auto f1 = blaze::subvector<0UL, nPrismatic>(f);
                // The bracketed factor is dimensionless in [0, 1]; multiplying by the
                // window half-width turns it into an actual displacement. Without that
                // scaling it was used directly as a velocity in METRES against windows
                // only ~40-54 mm wide, i.e. ~20 window-widths of commanded motion per
                // step. The prismatic clamp then fired on 83% of all iterations
                // (measured) and pinned β to its window edge, which is what pushed
                // solutions onto -- and past -- the bounds.
                f1 = kNullspaceGain * (betaMax - betaMin) * 0.50
                   * blaze::pow(blaze::abs((betaMax + betaMin - 2.00 * beta) / (betaMax - betaMin + 1.00E-10)), ke)
                   * blaze::sign(beta - (betaMax + betaMin) * 0.50);
            }

            // Resolved rates with null-space local optimization (joint-limit avoidance).
            dtau_dt = J_inv * (Kp * tipError + Kd * d_tipError + Ki * int_tipError) + (I - blaze::trans(J_inv * J)) * (-f);

            // Error before the step, so the step can be classified as up- or downhill.
            const double distBeforeStep = dist2Tgt;

            // rescaling linear joint variables for limit avoidance
            bool stepClamped = false;
            for (size_t i = 0; i < nPrismatic; ++i)
            {
                const double proposed = tau[i] + dtau_dt[i];
                if (proposed > betaMax[i])
                {
                    dtau_dt[i] = (betaMax[i] - tau[i]) * 0.50;
                    stepClamped = true;
                }
                else if (proposed < betaMin[i])
                {
                    dtau_dt[i] = (betaMin[i] - tau[i]) * 0.50;
                    stepClamped = true;
                }
            }
            if (stepClamped)
                diag_local.clampedSteps++;

            // trust region on the revolute rates (see kAlphaStepCap above)
            bool alphaCapped = false;
            for (size_t i = nPrismatic; i < controlInputs; ++i)
            {
                if (std::fabs(dtau_dt[i]) > kAlphaStepCap)
                {
                    dtau_dt[i] = std::copysign(kAlphaStepCap, dtau_dt[i]);
                    alphaCapped = true;
                }
            }
            if (alphaCapped)
                diag_local.alphaCappedSteps++;

            // updating the CTR joints: q = [beta, theta]
            tau += dtau_dt;

            wrapAngles(tau);

            // Domain tripwire: after the wrap these can never exceed the trained
            // ranges; a non-trivial reading here means the wrap itself regressed.
            if constexpr (controlInputs == 4)
            {
                diag_local.maxAbsAlpha2Queried   = std::max(diag_local.maxAbsAlpha2Queried, std::fabs(tau[3UL]));
                diag_local.maxAbsAlphaRelQueried = std::max(diag_local.maxAbsAlphaRelQueried, std::fabs(tau[2UL] - tau[3UL]));
            }

            // tip position as predicted by the model
            this->getPosDistal(tau, wf, x_CTR);

            // current position error
            tipError = target - x_CTR;

            // Integrate the position error, with anti-windup: integration freezes
            // while any joint rate is saturated -- a prismatic joint against its
            // window edge or a revolute rate against the trust region (the classic
            // remedy for a saturated actuator) -- and the accumulator is capped
            // either way so ki * ∫e can never swamp the proportional term.
            if (!stepClamped && !alphaCapped)
            {
                int_tipError += tipError;
                for (size_t k = 0UL; k < 3UL; ++k)
                    int_tipError[k] = std::clamp(int_tipError[k], -iLim, iLim);
            }

            // derivative of the position error
            d_tipError = tipError - last_tipError;
            // updating the last tip error variable
            last_tipError = tipError;

            dist2Tgt = blaze::norm(tipError);

            // The step is applied unconditionally, so nothing stops it going
            // uphill; counting those is how we tell overshoot from slow progress.
            if (dist2Tgt > distBeforeStep)
                diag_local.nonMonotonicSteps++;

            if (dist2Tgt < minError)
            {
                minError = dist2Tgt;
                tau_min = tau;
            }

            // The position update has become vanishingly small: this attempt has
            // stalled. Leave it so the retry loop can re-seed -- more steps in the
            // same basin would achieve nothing.
            if (blaze::linfNorm(dtau_dt) <= 1.00E-6)
                break;
        }

        if ((dist2Tgt <= posTol) || (N_itr_total >= maxIter))
            break;
    }

    finish(tau, tau_min, minError);

    return;
}

#endif  // CTR_KINEMATICS_PINN__CTR_PINN_INFERENCE_HPP_
