#include "asr_sdm_kinematic_dynamic_model/csv_logger.hpp"
#include "asr_sdm_kinematic_dynamic_model/underwater_simulator.hpp"

#include <cmath>
#include <fstream>
#include <iostream>
#include <string>

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

}  // namespace

int main()
{
#ifndef ASR_SDM_GENERATED_URDF
  std::cerr << "FAILED: generated URDF path is not configured" << std::endl;
  return 1;
#else
  model::UnderwaterSimulatorParameters params;
  params.pinocchio.urdf_path = ASR_SDM_GENERATED_URDF;
  params.pinocchio.use_free_flyer = true;
  params.pinocchio.lock_rotor_joints = true;
  for (auto & link : params.hydrodynamics.links) {
    link.added_mass.setIdentity();
    link.linear_damping.setConstant(0.1);
    link.quadratic_damping.setConstant(0.01);
  }
  params.actuators.maximum_segment_thrust.setConstant(1.0);

  bool passed = true;
  model::UnderwaterSimulator simulator(params);
  passed &= check(simulator.isValid(), "simulator construction");
  passed &= check(simulator.pinocchioModel().nq() == model::kReducedNq, "reduced nq");
  passed &= check(simulator.pinocchioModel().nv() == model::kReducedNv, "reduced nv");

  if (simulator.isValid()) {
    auto state = simulator.makeInitialState();
    model::UnderwaterSimulatorInput input;
    const auto initial = simulator.evaluate(state, input);
    passed &= check(initial.acceleration.array().isFinite().all(), "initial acceleration finite");
    passed &= check(initial.effective_mass.isApprox(initial.effective_mass.transpose(), 1.0e-10),
      "effective mass symmetric");
    passed &= check(initial.balance_residual.norm() < 1.0e-8, "initial force balance");

    auto hydrodynamics = params.hydrodynamics;
    hydrodynamics.fluid_density = 1000.0;
    for (auto & link : hydrodynamics.links) {
      link.displaced_volume = 1.0e-5;
    }
    std::string replacement_error;
    passed &= check(
      simulator.replaceHydrodynamicParameters(hydrodynamics, &replacement_error),
      "runtime hydrodynamic replacement");
    const auto buoyant = simulator.evaluate(state, input);
    passed &= check(buoyant.buoyancy_force.norm() > 0.0, "runtime buoyancy update");
    hydrodynamics.fluid_density = -1.0;
    passed &= check(
      !simulator.replaceHydrodynamicParameters(hydrodynamics, &replacement_error),
      "invalid hydrodynamic replacement rejected");
    passed &= check(
      simulator.replaceHydrodynamicParameters(params.hydrodynamics, &replacement_error),
      "restore hydrodynamic parameters");

    input.segment_thrust(0) = 0.5;
    const auto thrust_output = simulator.step(state, 0.01, input);
    passed &= check(thrust_output.actuator_force.norm() > 0.0, "thrust generalized force");
    passed &= check(state.time > 0.0, "time integration");
    passed &= check(std::abs(state.configuration.segment<4>(3).norm() - 1.0) < 1.0e-10,
      "quaternion integration");
    for (int step = 0; step < 20; ++step) {
      simulator.step(state, 0.01, input);
    }
    passed &= check(std::abs(state.configuration.segment<4>(3).norm() - 1.0) < 1.0e-10,
      "quaternion remains normalized");

    const std::string csv_path = "/tmp/asr_sdm_underwater_simulator_test.csv";
    model::CsvLogger logger(csv_path);
    passed &= check(logger.isOpen(), "CSV logger opens");
    logger.write(state, input, simulator.evaluate(state, input));
    std::ifstream csv(csv_path);
    std::string header;
    std::string row;
    passed &= check(static_cast<bool>(std::getline(csv, header)), "CSV header exists");
    passed &= check(static_cast<bool>(std::getline(csv, row)), "CSV row exists");
    passed &= check(header.find("time_s") == 0 && row.find("nan") == std::string::npos,
      "CSV content");
  }

  std::cout << (passed ? "Underwater simulator regression passed" :
  "Underwater simulator regression failed") << std::endl;
  return passed ? 0 : 1;
#endif
}
