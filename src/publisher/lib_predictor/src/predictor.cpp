#include "predictor.hpp"

Predictor::Predictor(float sample_time, int num_pred_steps, float fc)
{
    torch::set_num_threads(2);
    // Load the models
    std::string model_dir = model_Directory;
    m_encoder = load_model(model_dir + "/scripted_model/TorchScript_Encoder.pt");
    m_decoder = load_model(model_dir + "/scripted_model/TorchScript_Decoder.pt");
    m_states_auxilary = load_model(model_dir + "/scripted_model/TorchScript_States_Auxilary.pt");
    m_inputs_auxilary = load_model(model_dir + "/scripted_model/TorchScript_Inputs_Auxilary.pt");

    m_sample_time = sample_time;
    m_num_complex_pairs = 1;
    m_num_realeigens = 0;
    m_num_pred_steps = num_pred_steps;

    // Calculate alpha assuming you have defined Ts and fc somewhere accessible
    // float fc = 10.0; // cutoff frequency in Hz
    m_alpha = 2 * M_PI * fc * m_sample_time / (2 * M_PI * fc * m_sample_time + 1);
    m_filtered_U = blaze::StaticVector<float, Predictor::dim_inputs>(0.0);
    std::cout << "m_filtered_U: " << m_filtered_U << "   "
              << "m_alpha: " << m_alpha << "   "
              << std::endl;

    m_A = {{0.2, 0.3}, {0.5, 0.6}};
    m_G = 0.1f;
    m_B = 0.1f;
    m_H = 0.1f;
    m_I = blaze::IdentityMatrix<float>(Predictor::dim_lift);
    // std::cout << "Checkpoint_2" << std::endl;
}

blaze::StaticVector<float, Predictor::dim_lift> Predictor::encode(const blaze::StaticVector<float, Predictor::dim_states> &input)
{
    m_X.setBlazeVec(input);
    torch::NoGradGuard no_grad_guard;                                          // disable gradient calculation for improve performance
    torch::Tensor output = m_encoder->forward({m_X.getTorchVec()}).toTensor(); // for some unknown reasons! it automatically does the normalization!
    m_Y.setTorchVec(output);
    return m_Y.getBlazeVec();
}

blaze::StaticVector<float, Predictor::dim_states> Predictor::decode(const blaze::StaticVector<float, Predictor::dim_lift> &input)
{
    m_Y.setBlazeVec(input);
    torch::NoGradGuard no_grad_guard;                                          // disable gradient calculation for improve performance
    torch::Tensor output = m_decoder->forward({m_Y.getTorchVec()}).toTensor(); // for some unknown reasons! it automatically does the normalization!
    m_X.setTorchVec(output);
    return m_X.getBlazeVec();
}

void Predictor::update_lambda()
{
    torch::NoGradGuard no_grad_guard;
    torch::Tensor L = m_states_auxilary->forward({m_Y.getTorchVec()}).toTensor();
    m_blazeLambdas = torchToBlazeStaticVector<dim_lift>(L);
}

void Predictor::update_states_transition_mat()
{
    Predictor::update_jordan_canonical_disc(this->m_blazeLambdas, this->m_G);
    Predictor::update_jordan_canonical_cont(this->m_blazeLambdas, this->m_A);
    return;
}

void Predictor::update_jordan_canonical_disc(const blaze::StaticVector<float, dim_lift> &Lambda, blaze::StaticMatrix<float, dim_lift, dim_lift> &G)
{
    // Update G for complex eigenvalues
    for (int i = 0; i < m_num_complex_pairs; ++i)
    {
        float mu = Lambda[2 * i];        // Real part
        float omega = Lambda[2 * i + 1]; // Imaginary part
        float exp_mu_dt = std::exp(mu * m_sample_time);
        float cos_omega_dt = std::cos(omega * m_sample_time);
        float sin_omega_dt = std::sin(omega * m_sample_time);
        // Constructing the 2x2 block for the Jordan block of complex eigenvalues
        G(2 * i, 2 * i) = exp_mu_dt * cos_omega_dt;
        G(2 * i, 2 * i + 1) = -exp_mu_dt * sin_omega_dt;
        G(2 * i + 1, 2 * i) = exp_mu_dt * sin_omega_dt;
        G(2 * i + 1, 2 * i + 1) = exp_mu_dt * cos_omega_dt;
    }

    // Update G for real eigenvalues
    for (int i = 0; i < m_num_realeigens; ++i)
    {
        float real_eig = Lambda[2 * m_num_complex_pairs + i];
        float exp_lambda_dt = std::exp(real_eig * m_sample_time);
        G(2 * m_num_complex_pairs + i, 2 * m_num_complex_pairs + i) = exp_lambda_dt;
    }
    return;
}

void Predictor::update_jordan_canonical_cont(const blaze::StaticVector<float, dim_lift> &Lambda, blaze::StaticMatrix<float, dim_lift, dim_lift> &A)
{
    // Update A for complex eigenvalues
    for (int i = 0; i < m_num_complex_pairs; ++i)
    {
        float mu = Lambda[2 * i];        // Real part of the complex pair
        float omega = Lambda[2 * i + 1]; // Imaginary part of the complex pair
        // Creating a 2x2 block for the Jordan block corresponding to complex eigenvalues
        A(2 * i, 2 * i) = mu;
        A(2 * i, 2 * i + 1) = -omega;
        A(2 * i + 1, 2 * i) = omega;
        A(2 * i + 1, 2 * i + 1) = mu;
    }

    // Update A for real eigenvalues
    for (int i = 0; i < m_num_realeigens; ++i)
    {
        float real_eig = Lambda[2 * m_num_complex_pairs + i];
        A(2 * m_num_complex_pairs + i, 2 * m_num_complex_pairs + i) = real_eig;
    }
    return;
}

void Predictor::update_inputs_transition_mat()
{
    torch::NoGradGuard no_grad_guard;
    auto B_torch = m_inputs_auxilary->forward({m_Y.getTorchVec()}).toTensor();
    float *data_ptr = B_torch.data_ptr<float>(); // Direct pointer to the data
    constexpr std::size_t columns = 1;
    for (size_t i = 0; i < dim_lift; ++i)
    {
        for (size_t j = 0; j < columns; ++j)
        {
            m_B(i, j) = data_ptr[i * columns + j];
        }
    }
    m_H = blaze::inv(m_A) * (m_G - m_I) * m_B;
    return;
}

double Predictor::update_parameters()
{
    auto t0 = std::chrono::high_resolution_clock::now();
    Predictor::update_lambda();
    Predictor::update_states_transition_mat();
    Predictor::update_inputs_transition_mat();
    auto t1 = std::chrono::high_resolution_clock::now();
    auto elapsed_1 = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0);
    return elapsed_1.count();
}

void Predictor::koopman_step(const blaze::StaticVector<float, Predictor::dim_inputs> &u)
{
    // std::cout << "Checkpoint_3" << std::endl;
    // Apply the low-pass filter
    m_filtered_U = m_alpha * u + (1 - m_alpha) * m_filtered_U;

    auto forced = m_G * m_Y.getBlazeVec();
    auto unforced = m_H * m_filtered_U;
    m_Y.setBlazeVec(forced + unforced);
    return;
}

blaze::StaticVector<float, Predictor::dim_lift> Predictor::get_lifted_states() const
{
    return m_Y.getBlazeVec();
}

blaze::StaticVector<float, Predictor::dim_states> Predictor::get_state() const
{
    return m_X.getBlazeVec();
}

blaze::StaticVector<float, Predictor::dim_lift> Predictor::get_lambda() const
{
    return m_blazeLambdas;
}

blaze::StaticMatrix<float, Predictor::dim_lift, Predictor::dim_lift> Predictor::get_G() const
{
    return m_G;
}

blaze::StaticMatrix<float, Predictor::dim_lift, Predictor::dim_lift> Predictor::get_A() const
{
    return m_A;
}

blaze::StaticMatrix<float, Predictor::dim_lift, Predictor::dim_inputs> Predictor::get_H() const
{
    return m_H;
}

blaze::StaticMatrix<float, Predictor::dim_lift, Predictor::dim_inputs> Predictor::get_B_phi() const
{
    return m_B;
}

// Helper function to load a model from a file
std::shared_ptr<torch::jit::Module> load_model(const std::string &model_path)
{
    // Load the model and return a shared pointer
    return std::make_shared<torch::jit::Module>(torch::jit::load(model_path));
}

template <size_t N>
blaze::StaticVector<float, N> torchToBlazeStaticVector(const torch::Tensor &tensor)
{
    assert(tensor.numel() == N); // Ensure the tensor has exactly N elements
    torch::Tensor tensor_cpu = tensor.to(torch::kCPU, torch::kFloat32);

    blaze::StaticVector<float, N> vector;
    const float *data_ptr = tensor_cpu.data_ptr<float>();
    for (size_t i = 0; i < N; ++i)
    {
        vector[i] = data_ptr[i];
    }

    return vector;
}
