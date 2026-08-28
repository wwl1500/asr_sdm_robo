#include "asr_sdm_kinematic_dynamic_model/fluid_force_model.hpp"
#include "asr_sdm_kinematic_dynamic_model/robot_model_3d.hpp"

#include <cmath>
#include <iostream>

namespace
{
namespace model = asr_sdm_kinematic_dynamic_model;

bool check(bool condition, const char * message)
{
  if (!condition) {
    std::cerr << "FAILED: " << message << std::endl;
  }
  return condition;
}

bool near(double value, double expected, double tolerance = 1.0e-10)
{
  return std::abs(value - expected) <= tolerance;
}

}  // namespace

int main()
{
  model::HydrodynamicModelParameters params;
  for (auto & link : params.links) {
    link.mass = 1.0;
    link.inertia_about_center_of_mass = Eigen::Matrix3d::Identity();
    link.added_mass.setIdentity();
    link.linear_damping.setOnes();
    link.quadratic_damping.setConstant(0.5);
    link.displaced_volume = 0.001;
  }

  bool passed = true;
  model::FluidForceModel fluid(params);
  passed &= check(fluid.isValid(), "valid fluid parameters");

  auto invalid_params = params;
  invalid_params.links[0].added_mass(0, 1) = 1.0;
  model::FluidForceModel invalid_mass(invalid_params);
  passed &= check(!invalid_mass.isValid(), "nonsymmetric added mass rejected");

  invalid_params = params;
  invalid_params.links[0].linear_damping[0] = -1.0;
  model::FluidForceModel invalid_damping(invalid_params);
  passed &= check(!invalid_damping.isValid(), "negative damping rejected");

  if (fluid.isValid()) {
    model::RobotModel3D robot({0.25});
    auto state = robot.makeInitialState();
    model::BaseTwist3D base_twist;
    base_twist.linear_velocity_world.x() = 1.0;
    model::BaseAcceleration3D base_acceleration;
    base_acceleration.linear_acceleration_world.x() = 2.0;

    const auto evaluation = fluid.evaluate(
      state, base_twist, model::JointVelocity3D{}, Eigen::Vector3d::Zero(),
      base_acceleration, model::JointAcceleration3D{});

    for (std::size_t link = 0; link < model::kNumLinks; ++link) {
      passed &= check(
        near(evaluation.local_jacobians[link](0, 0), 1.0) &&
        near(evaluation.local_jacobians[link](1, 1), 1.0) &&
        near(evaluation.local_jacobians[link](2, 2), 1.0),
        "base translation Jacobian");
      passed &= check(near(evaluation.relative_twists[link](0), 1.0), "relative forward speed");
      passed &= check(near(evaluation.linear_damping_wrenches[link](0), -1.0),
        "linear damping sign");
      passed &= check(near(evaluation.quadratic_damping_wrenches[link](0), -0.5),
        "quadratic damping sign");
      passed &= check(near(evaluation.gravity_wrenches[link](2), -9.81), "gravity direction");
      passed &= check(near(evaluation.buoyancy_wrenches[link](2), 9.81), "buoyancy direction");
      passed &= check(near(evaluation.total_link_wrenches[link](2), 0.0),
        "neutral gravity-buoyancy force");
      passed &= check(near(evaluation.added_mass_wrenches[link](0), -2.0),
        "added-mass acceleration force");
    }

    passed &= check(near(evaluation.damping_force(0), -6.0), "generalized damping accumulation");
    passed &= check(near(evaluation.gravity_buoyancy_force.norm(), 0.0),
      "generalized neutral restoring force");

    model::JointVelocity3D joint_velocity;
    joint_velocity.theta_dot[model::yawIndex(0)] = -1.0;
    const auto moving_evaluation = fluid.evaluate(
      state, model::BaseTwist3D{}, joint_velocity);
    passed &= check(
      moving_evaluation.relative_twists[1].tail<3>().norm() > 0.9,
      "internal joint motion reaches downstream link");
  }

  std::cout << (passed ? "Fluid-force regression passed" : "Fluid-force regression failed") <<
    std::endl;
  return passed ? 0 : 1;
}
