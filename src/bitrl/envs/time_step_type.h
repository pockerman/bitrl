#ifndef TIME_STEP_TYPE_H
#define TIME_STEP_TYPE_H

#include "bitrl/bitrl_types.h"
#include <string>
#include <vector>

namespace bitrl
{

///
/// \brief The TimeStepTp enum
///
enum class TimeStepTp: uint_t {FIRST=0, MID=1, LAST=2, INVALID_TYPE=3};

///
/// \brief Utilities for TimeStepTp 
///
struct TimeStepEnumUtils{
		static TimeStepTp time_step_type_from_int(int aidx);
		static TimeStepTp time_step_type_from_int(uint_t aidx);
		static std::string to_string(TimeStepTp type);
		static std::vector<TimeStepTp> time_step_type_from_int(const std::vector<uint_t>& types);
	};

}

#endif // TIME_STEP_TYPE_H
