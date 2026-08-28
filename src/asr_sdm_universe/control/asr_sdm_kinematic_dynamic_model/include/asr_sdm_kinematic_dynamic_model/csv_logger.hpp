#ifndef ASR_SDM_KINEMATIC_DYNAMIC_MODEL_CSV_LOGGER_HPP_
#define ASR_SDM_KINEMATIC_DYNAMIC_MODEL_CSV_LOGGER_HPP_

#include "asr_sdm_kinematic_dynamic_model/underwater_simulator.hpp"

#include <fstream>
#include <string>

namespace asr_sdm_kinematic_dynamic_model
{

class CsvLogger
{
public:
  explicit CsvLogger(const std::string & path);
  ~CsvLogger();

  bool isOpen() const;
  const std::string & error() const;
  void write(
    const UnderwaterSimulatorState & state,
    const UnderwaterSimulatorInput & input,
    const UnderwaterDynamicsOutput & output);

private:
  void writeHeader();

  std::ofstream stream_;
  std::string error_;
  bool header_written_{false};
};

}  // namespace asr_sdm_kinematic_dynamic_model

#endif  // ASR_SDM_KINEMATIC_DYNAMIC_MODEL_CSV_LOGGER_HPP_
