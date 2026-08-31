// PINN-vs-Cosserat forward-kinematics cross-check over the alpha domain.
//
// Why this exists: the PINN is the only tip model the IK, planner and pinn_fk
// ever consult, so a region where it extrapolates cannot be detected from
// inside the PINN stack -- ik_bench defines its ground truth with the same
// network it is testing. This tool compares the PINN tip against the
// independent Cosserat-rod model (ctr_cosserat) over a grid of alpha
// configurations, INCLUDING the |alpha2| > 1.5pi wedge the pre-fix IK domain
// used to query: the network was never trained there (its baked normaliser
// spans 1.1 x 1.5pi) and the drives cannot even reach it, so large errors
// there are expected and document the "converged IK, wrong tip" failure.
//
// The Cosserat model is built from the SAME parameters.json the PINN ships
// with (physics_params), not from hardcoded constants -- the robot's
// cosserat_fk node still carries a stale inner-tube curve radius (0.064 vs the
// dataset's 0.073), which would masquerade as model error here.
//
// Build (opt-in; links LibTorch):
//   colcon build --packages-select ctr_cosserat \
//     --cmake-args -DCMAKE_BUILD_TYPE=Release -DCTR_COSSERAT_BUILD_XCHECK=ON
// Run:
//   fk_xcheck --models-dir <dir> [--model <name>] [--grid-alpha2 33]
//             [--grid-rel 13] [--beta1 -0.09] [--beta2 -0.05] [--csv out.csv]

#include "ctr_cosserat/CTR.hpp"
#include "ctr_kinematics_pinn/ctr_pinn_inference.hpp"

#include <array>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

namespace
{

constexpr size_t kControlInputs = 4UL;
constexpr size_t kBackbonePoints = 150UL;

std::string argValue(int argc, char **argv, const std::string &flag, const std::string &fallback)
{
    for (int i = 1; i + 1 < argc; ++i)
        if (flag == argv[i])
            return argv[i + 1];
    return fallback;
}

/// Cosserat CTR built from the model's own physics_params, so both models
/// describe the same physical robot.
std::shared_ptr<CTR> makeCosseratFromJson(const std::string &paramsPath)
{
    std::ifstream in(paramsPath);
    if (!in.is_open())
        throw std::runtime_error("fk_xcheck: cannot open " + paramsPath);
    const nlohmann::json j = nlohmann::json::parse(in);
    const auto &p = j.at("physics_params");

    const auto E = p.at("young_modulus").get<std::vector<double>>();
    const auto G = p.at("shear_modulus").get<std::vector<double>>();
    const auto rc = p.at("curve_radius").get<std::vector<double>>();
    const auto Ls = p.at("straight_length").get<std::vector<double>>();
    const auto Lc = p.at("curve_length").get<std::vector<double>>();
    const auto Di = p.at("inner_diameter").get<std::vector<double>>();
    const auto Do = p.at("outer_diameter").get<std::vector<double>>();

    std::array<std::shared_ptr<Tube>, 3UL> Tb;
    for (size_t i = 0UL; i < 3UL; ++i)
    {
        const blaze::StaticVector<double, 3UL> u = {1.00 / rc[i], 0.00, 0.00};
        Tb[i] = std::make_shared<Tube>(Do[i], Di[i], E[i], G[i], Ls[i], Lc[i], u);
    }

    blaze::StaticVector<double, 6UL> q_home = {-0.150, -0.072, 0.00, 0.00, 0.00, 0.00};
    return std::make_shared<CTR>(Tb, q_home, 1.00E-6,
                                 mathOp::rootFindingMethod::MODIFIED_NEWTON_RAPHSON, 30.00E-3);
}

}  // namespace

int main(int argc, char **argv)
{
    const char *envModels = std::getenv("CTR_MODELS_DIR");
    const std::string modelsDir = argValue(argc, argv, "--models-dir", envModels ? envModels : "");
    const std::string modelName = argValue(argc, argv, "--model", "ctr_8x91_0.18_tanh_9K_9K_50K_FP64");
    const size_t nAlpha2 = std::stoul(argValue(argc, argv, "--grid-alpha2", "33"));
    const size_t nRel = std::stoul(argValue(argc, argv, "--grid-rel", "13"));
    const double beta1 = std::stod(argValue(argc, argv, "--beta1", "-0.09"));
    const double beta2 = std::stod(argValue(argc, argv, "--beta2", "-0.05"));
    const std::string csvPath = argValue(argc, argv, "--csv", "");

    if (modelsDir.empty())
    {
        std::cerr << "fk_xcheck: need --models-dir <dir> (or CTR_MODELS_DIR).\n";
        return 2;
    }

    PINNs<kControlInputs> pinn(modelsDir, modelName, 1UL, kBackbonePoints);
    auto cosserat = makeCosseratFromJson(modelsDir + '/' + modelName + "/parameters.json");
    const auto lim = pinn.getJointLimits4();
    const blaze::StaticVector<double, 3UL> wf(0.00);

    std::cout << "fk_xcheck: model " << modelName << ", beta = [" << beta1 << ", " << beta2
              << "], grid " << nAlpha2 << " x " << nRel << " (alpha2 in [-2pi, 2pi], rel in [-pi, pi])\n"
              << "  trained alpha2 box: [" << lim.alpha2_absolute[0UL] << ", " << lim.alpha2_absolute[1UL] << "]\n"
              << std::endl;

    std::ofstream csv;
    if (!csvPath.empty())
    {
        csv.open(csvPath);
        csv << "alpha2,alpha_rel,alpha1,in_trained_box,cosserat_converged,"
               "pinn_x,pinn_y,pinn_z,cosserat_x,cosserat_y,cosserat_z,err_m\n"
            << std::setprecision(9);
    }

    struct Acc
    {
        size_t n = 0UL, nConv = 0UL;
        double sum = 0.00, worst = 0.00;
    };
    Acc inBox, outBox;

    for (size_t ia = 0UL; ia < nAlpha2; ++ia)
    {
        const double alpha2 = -2.0 * M_PI + (4.0 * M_PI) * static_cast<double>(ia) / static_cast<double>(nAlpha2 - 1UL);
        for (size_t ir = 0UL; ir < nRel; ++ir)
        {
            const double rel = -M_PI + (2.0 * M_PI) * static_cast<double>(ir) / static_cast<double>(nRel - 1UL);
            const double alpha1 = alpha2 + rel;

            const blaze::StaticVector<double, 4UL> q4 = {beta1, beta2, alpha1, alpha2};
            blaze::StaticVector<double, 3UL> pinnTip;
            pinn.getPosDistal(q4, wf, pinnTip);

            blaze::StaticVector<double, 6UL> q6 = {beta1, beta2, 0.00, alpha1, alpha2, 0.00};
            blaze::StaticVector<double, 5UL> initGuess(0.00);
            const bool conv = cosserat->actuate_CTR(initGuess, q6);
            const blaze::StaticVector<double, 3UL> cosTip = cosserat->getTipPos();

            const double err = blaze::norm(pinnTip - cosTip);
            const bool inTrained =
                alpha2 >= lim.alpha2_absolute[0UL] && alpha2 <= lim.alpha2_absolute[1UL];

            Acc &acc = inTrained ? inBox : outBox;
            ++acc.n;
            if (conv)
            {
                ++acc.nConv;
                acc.sum += err;
                acc.worst = std::max(acc.worst, err);
            }

            if (csv.is_open())
                csv << alpha2 << ',' << rel << ',' << alpha1 << ',' << (inTrained ? 1 : 0) << ','
                    << (conv ? 1 : 0) << ','
                    << pinnTip[0UL] << ',' << pinnTip[1UL] << ',' << pinnTip[2UL] << ','
                    << cosTip[0UL] << ',' << cosTip[1UL] << ',' << cosTip[2UL] << ',' << err << '\n';
        }
    }

    const auto report = [](const char *label, const Acc &a)
    {
        std::cout << "  " << label << ": n = " << a.n << " (" << a.nConv << " Cosserat-converged), "
                  << std::fixed << std::setprecision(2)
                  << "mean |PINN - Cosserat| = " << (a.nConv ? a.sum / static_cast<double>(a.nConv) : 0.0) * 1.0E3
                  << " mm, worst = " << a.worst * 1.0E3 << " mm\n";
    };

    std::cout << "=== PINN vs Cosserat tip agreement ===\n";
    report("inside trained alpha2 box (|a2| <= 1.5pi)", inBox);
    report("OLD SPILL WEDGE (|a2| > 1.5pi, untrained) ", outBox);
    std::cout << "\nA large out-of-box error with a small in-box error confirms the pre-fix\n"
                 "failure mechanism: the IK 'converged' on network extrapolation there.\n";

    if (csv.is_open())
        std::cout << "\nwrote " << csvPath << std::endl;

    return 0;
}
