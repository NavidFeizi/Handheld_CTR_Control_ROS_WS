#ifndef PREDICTOR_HPP
#define PREDICTOR_HPP

#include <torch/torch.h>
#include <torch/script.h>
#include <iostream>
#include <memory>
#include <cassert>
#include <cmath>
#include <blaze/Math.h>
#include <fstream>
#include <iostream>
#include <sstream>

#include "predictor.hpp"
#include "synchedVector.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

std::shared_ptr<torch::jit::Module> load_model(const std::string &path);
template <size_t N>

blaze::StaticVector<float, N> torchToBlazeStaticVector(const torch::Tensor &tensor);

class Predictor
{
protected:
    static constexpr std::size_t dim_inputs = 1;
    static constexpr std::size_t dim_lift = 2;
    static constexpr std::size_t dim_states = 4;

public:
    Predictor(float sample_time, int num_pred_steps, float fc);
    blaze::StaticVector<float, dim_lift> encode(const blaze::StaticVector<float, dim_states> &x);
    blaze::StaticVector<float, dim_states> decode(const blaze::StaticVector<float, dim_lift> &y);
    double update_parameters();
    void koopman_step(const blaze::StaticVector<float, dim_inputs> &u);

    blaze::StaticVector<float, dim_lift> get_lifted_states() const;
    blaze::StaticVector<float, dim_states> get_state() const;
    blaze::StaticVector<float, dim_lift> get_lambda() const;
    blaze::StaticMatrix<float, dim_lift, dim_lift> get_G() const;
    blaze::StaticMatrix<float, dim_lift, dim_lift> get_A() const;
    blaze::StaticMatrix<float, dim_lift, dim_inputs> get_H() const;
    blaze::StaticMatrix<float, dim_lift, dim_inputs> get_B_phi() const;

private:
    void update_lambda();
    void update_states_transition_mat();
    void update_inputs_transition_mat();
    void update_jordan_canonical_disc(const blaze::StaticVector<float, dim_lift> &Lambda, blaze::StaticMatrix<float, dim_lift, dim_lift> &G);
    void update_jordan_canonical_cont(const blaze::StaticVector<float, dim_lift> &Lambda, blaze::StaticMatrix<float, dim_lift, dim_lift> &A);

    std::shared_ptr<torch::jit::Module> m_encoder;
    std::shared_ptr<torch::jit::Module> m_decoder;
    std::shared_ptr<torch::jit::Module> m_states_auxilary;
    std::shared_ptr<torch::jit::Module> m_inputs_auxilary;
    float m_sample_time;
    float m_alpha;
    int m_num_complex_pairs, m_num_realeigens, m_num_pred_steps;
    SyncedVector<dim_states> m_X;
    SyncedVector<dim_lift> m_Y;
    blaze::StaticMatrix<float, dim_lift, dim_lift> m_A;
    blaze::StaticMatrix<float, dim_lift, dim_lift> m_G;
    blaze::StaticMatrix<float, dim_lift, dim_lift> m_I;
    blaze::StaticMatrix<float, dim_lift, dim_inputs> m_B;
    blaze::StaticMatrix<float, dim_lift, dim_inputs> m_H;
    blaze::StaticVector<float, dim_lift> m_blazeLambdas;
    blaze::StaticVector<float, dim_inputs> m_filtered_U;
};

#endif // MY_CLASS_HPP