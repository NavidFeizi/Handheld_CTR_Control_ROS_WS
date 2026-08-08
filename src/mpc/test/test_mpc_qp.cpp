#include <gtest/gtest.h>

#include <memory>

#include "mpc.hpp"
#include "mpc.tpp"

// QP-level tests with a trivial LINEAR plant standing in for the PINN:
// tip position = first three joints (integrator dynamics through the input).
// No Torch, no hardware.

namespace
{
constexpr size_t H = 5;  // horizon
constexpr size_t N4 = 4; // joints
constexpr size_t M3 = 3; // outputs

using Mpc = MPC<H, N4, M3>;

// MPC is neither copyable nor movable (solver state) — construct in place.
std::unique_ptr<Mpc> makeMpc(double dt = 0.025)
{
  auto fwd = [](const Mpc::VecN &q, const Mpc::VecM & /*wf*/) -> Mpc::VecM
  { return {q[0], q[1], q[2]}; };
  auto jac = [](const Mpc::VecN & /*q*/, const Mpc::VecM & /*wf*/) -> Mpc::MatMN
  {
    Mpc::MatMN J(0.0);
    J(0, 0) = 1.0;
    J(1, 1) = 1.0;
    J(2, 2) = 1.0;
    return J;
  };
  auto mpc = std::make_unique<Mpc>(dt, fwd, jac);

  const Mpc::VecN q_min = {-1.0, -1.0, -1.0, -1.0};
  const Mpc::VecN q_max = {1.0, 1.0, 1.0, 1.0};
  const Mpc::VecN u_max = {0.5, 0.5, 0.5, 0.5};
  const Mpc::VecN du_max = {5.0, 5.0, 5.0, 5.0};
  const Mpc::VecN margin(0.0);
  mpc->setJointsLimits(q_min, q_max, -u_max, u_max, -du_max, du_max, margin, margin, margin);
  mpc->updateWeights(Mpc::VecM(100.0), Mpc::VecM(100.0), Mpc::VecN(0.0), Mpc::VecN(0.1));
  return mpc;
}

Mpc::MatHM constantRef(const Mpc::VecM &target)
{
  Mpc::MatHM ref;
  for (size_t i = 0; i < H; ++i)
    for (size_t j = 0; j < M3; ++j)
      ref(i, j) = target[j];
  return ref;
}
}  // namespace

TEST(MpcQp, DrivesTowardTarget)
{
  auto mpc = makeMpc();
  const Mpc::VecN q0 = {0.0, 0.0, 0.0, 0.0};
  const auto ref = constantRef({0.1, -0.1, 0.05});

  Mpc::VecN u{};
  mpc->step(q0, {0, 0, 0}, ref, u);

  // The commanded velocity must push each controlled joint toward its target.
  EXPECT_GT(u[0], 0.0);
  EXPECT_LT(u[1], 0.0);
  EXPECT_GT(u[2], 0.0);
}

TEST(MpcQp, RespectsInputBounds)
{
  auto mpc = makeMpc();
  const Mpc::VecN q0 = {0.0, 0.0, 0.0, 0.0};
  const auto ref = constantRef({100.0, -100.0, 100.0}); // far target saturates u

  Mpc::VecN u{};
  mpc->step(q0, {0, 0, 0}, ref, u);
  for (size_t i = 0; i < N4; ++i)
  {
    EXPECT_LE(u[i], 0.5 + 1e-6);
    EXPECT_GE(u[i], -0.5 - 1e-6);
  }
}

TEST(MpcQp, AtTargetCommandsNearZero)
{
  auto mpc = makeMpc();
  const Mpc::VecN q0 = {0.1, -0.1, 0.05, 0.0};
  const auto ref = constantRef({0.1, -0.1, 0.05});

  Mpc::VecN u{};
  mpc->step(q0, {0, 0, 0}, ref, u);
  for (size_t i = 0; i < M3; ++i)
    EXPECT_NEAR(u[i], 0.0, 1e-2);
}

TEST(MpcQp, WarmStartMatchesReinit)
{
  // Two identical controllers, identical step sequences; the only difference
  // is that the second is forced through the full solver re-init before the
  // second step. Converged solutions must agree at solver tolerance.
  auto warm = makeMpc();
  auto cold = makeMpc();
  const Mpc::VecN q0 = {0.0, 0.0, 0.0, 0.0};
  const Mpc::VecN q1 = {0.01, -0.01, 0.005, 0.0};
  const auto refA = constantRef({0.1, -0.1, 0.05});
  const auto refB = constantRef({0.12, -0.08, 0.04});

  Mpc::VecN u_warm{}, u_cold{};
  warm->step(q0, {0, 0, 0}, refA, u_warm);
  cold->step(q0, {0, 0, 0}, refA, u_cold);

  warm->step(q1, {0, 0, 0}, refB, u_warm); // update path
  cold->forceSolverReinit();
  cold->step(q1, {0, 0, 0}, refB, u_cold); // re-init path

  for (size_t i = 0; i < N4; ++i)
    EXPECT_NEAR(u_warm[i], u_cold[i], 5e-3);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
