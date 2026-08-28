#ifndef ASR_SDM_KINEMATIC_DYNAMIC_MODEL_CONSTANTS_HPP_
#define ASR_SDM_KINEMATIC_DYNAMIC_MODEL_CONSTANTS_HPP_

#include <cstddef>

namespace asr_sdm_kinematic_dynamic_model
{

constexpr std::size_t kNumJoints = 3;
constexpr std::size_t kNumLinks = 4;
constexpr std::size_t kNumBodyPoints = 5;
constexpr std::size_t kNumJointDofs = 6;
/// Two counter-rotating propulsion screws per segment, ordered
/// [left_0, right_0, left_1, right_1, ...].
constexpr std::size_t kNumRotors = 2 * kNumLinks;

constexpr std::size_t rotorIndex(std::size_t link, std::size_t rotor)
{
  return 2 * link + rotor;
}

constexpr std::size_t yawIndex(std::size_t joint)
{
  return 2 * joint;
}

constexpr std::size_t pitchIndex(std::size_t joint)
{
  return 2 * joint + 1;
}

}  // namespace asr_sdm_kinematic_dynamic_model

#endif  // ASR_SDM_KINEMATIC_DYNAMIC_MODEL_CONSTANTS_HPP_
