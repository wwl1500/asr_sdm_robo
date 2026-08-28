#include "asr_sdm_kinematic_dynamic_model/csv_logger.hpp"

#include <cmath>
#include <iomanip>
#include <stdexcept>

namespace asr_sdm_kinematic_dynamic_model
{

CsvLogger::CsvLogger(const std::string & path)
{
  if (path.empty()) {
    error_ = "CSV path is empty";
    return;
  }
  stream_.open(path);
  if (!stream_) {
    error_ = "Failed to open CSV path: " + path;
    return;
  }
  stream_ << std::setprecision(17);
  writeHeader();
}

CsvLogger::~CsvLogger()
{
  if (stream_) {
    stream_.flush();
  }
}

bool CsvLogger::isOpen() const
{
  return stream_.is_open() && error_.empty();
}

const std::string & CsvLogger::error() const
{
  return error_;
}

void CsvLogger::writeHeader()
{
  stream_ << "time_s"
          << ",q_base_x_m,q_base_y_m,q_base_z_m"
          << ",q_quat_x,q_quat_y,q_quat_z,q_quat_w";
  for (std::size_t i = 0; i < kNumJointDofs; ++i) {
    stream_ << ",q_joint_" << i << "_rad";
  }
  stream_ << ",v_base_linear_x_mps,v_base_linear_y_mps,v_base_linear_z_mps"
          << ",v_base_angular_x_rps,v_base_angular_y_rps,v_base_angular_z_rps";
  for (std::size_t i = 0; i < kNumJointDofs; ++i) {
    stream_ << ",v_joint_" << i << "_rps";
  }
  stream_ << ",a_base_linear_x_mps2,a_base_linear_y_mps2,a_base_linear_z_mps2"
          << ",a_base_angular_x_rps2,a_base_angular_y_rps2,a_base_angular_z_rps2";
  for (std::size_t i = 0; i < kNumJointDofs; ++i) {
    stream_ << ",a_joint_" << i << "_rps2";
  }
  for (std::size_t i = 0; i < kNumLinks; ++i) {
    stream_ << ",thrust_command_" << i << "_N";
  }
  for (std::size_t i = 0; i < kNumLinks; ++i) {
    stream_ << ",thrust_actual_" << i << "_N";
  }
  for (std::size_t i = 0; i < kNumJointDofs; ++i) {
    stream_ << ",joint_torque_" << i << "_Nm";
  }
  for (int link = 0; link < static_cast<int>(kNumLinks); ++link) {
    stream_ << ",damping_l" << link << "_fx_local_N"
            << ",damping_l" << link << "_fy_local_N"
            << ",damping_l" << link << "_fz_local_N"
            << ",damping_l" << link << "_mx_local_Nm"
            << ",damping_l" << link << "_my_local_Nm"
            << ",damping_l" << link << "_mz_local_Nm"
            << ",buoyancy_l" << link << "_fx_local_N"
            << ",buoyancy_l" << link << "_fy_local_N"
            << ",buoyancy_l" << link << "_fz_local_N"
            << ",buoyancy_l" << link << "_mx_local_Nm"
            << ",buoyancy_l" << link << "_my_local_Nm"
            << ",buoyancy_l" << link << "_mz_local_Nm"
            << ",total_fluid_l" << link << "_fx_local_N"
            << ",total_fluid_l" << link << "_fy_local_N"
            << ",total_fluid_l" << link << "_fz_local_N"
            << ",total_fluid_l" << link << "_mx_local_Nm"
            << ",total_fluid_l" << link << "_my_local_Nm"
            << ",total_fluid_l" << link << "_mz_local_Nm";
  }
  stream_ << '\n';
  header_written_ = true;
}

void CsvLogger::write(
  const UnderwaterSimulatorState & state,
  const UnderwaterSimulatorInput & input,
  const UnderwaterDynamicsOutput & output)
{
  if (!isOpen() || !header_written_) {
    throw std::runtime_error("Cannot write to an invalid CSV logger: " + error_);
  }
  if (!std::isfinite(state.time) || !state.configuration.array().isFinite().all() ||
    !state.velocity.array().isFinite().all() || !output.acceleration.array().isFinite().all())
  {
    throw std::invalid_argument("CSV logger refuses non-finite state or acceleration");
  }

  stream_ << state.time;
  for (std::size_t i = 0; i < kReducedNq; ++i) {
    stream_ << ',' << state.configuration(static_cast<Eigen::Index>(i));
  }
  for (std::size_t i = 0; i < kReducedNv; ++i) {
    stream_ << ',' << state.velocity(static_cast<Eigen::Index>(i));
  }
  for (std::size_t i = 0; i < kReducedNv; ++i) {
    stream_ << ',' << output.acceleration(static_cast<Eigen::Index>(i));
  }
  for (std::size_t i = 0; i < kNumLinks; ++i) {
    stream_ << ',' << input.segment_thrust(static_cast<Eigen::Index>(i));
  }
  for (std::size_t i = 0; i < kNumLinks; ++i) {
    stream_ << ',' << output.actuators.actual_segment_thrust(static_cast<Eigen::Index>(i));
  }
  for (std::size_t i = 0; i < kNumJointDofs; ++i) {
    stream_ << ',' << input.joint_torque(static_cast<Eigen::Index>(i));
  }
  for (std::size_t link = 0; link < kNumLinks; ++link) {
    for (int component = 0; component < kSpatialDofs; ++component) {
      stream_ << ',' << output.hydrodynamics.damping_wrenches[link](component);
    }
    for (int component = 0; component < kSpatialDofs; ++component) {
      stream_ << ',' << output.hydrodynamics.buoyancy_wrenches[link](component);
    }
    for (int component = 0; component < kSpatialDofs; ++component) {
      stream_ << ',' << output.hydrodynamics.total_wrenches[link](component);
    }
  }
  stream_ << '\n';
  stream_.flush();
  if (!stream_) {
    throw std::runtime_error("Failed while writing simulation CSV");
  }
}

}  // namespace asr_sdm_kinematic_dynamic_model
