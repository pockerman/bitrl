// SPDX-License-Identifier: Apache-2.0

#ifndef BITRL_LOG_H
#define BITRL_LOG_H

/**
 * @file bitrl_log.h
 * @brief bitrl's logging: the logger of bitsim, bitsim/utils/logger.h, with
 * bitrl's messages going to the logger named "bitrl".
 *
 * @code
 * BITRL_LOG_INFO << "Episode " << episode << " reward " << reward;
 *
 * // only warnings and above from bitrl, everything else unchanged
 * bitrl::log::set_level("bitrl", bitrl::log::Level::warn);
 * @endcode
 *
 * The SPDLOG_LEVEL environment variable sets the levels without changing the
 * code: SPDLOG_LEVEL=bitrl=warn does the same as the call above.
 */

#include "bitsim/utils/logger.h"

namespace bitrl::log
{
using bitsim::log::add_file_sink;
using bitsim::log::flush;
using bitsim::log::get_level;
using bitsim::log::Level;
using bitsim::log::set_level;
using bitsim::log::set_pattern;
} // namespace bitrl::log

#define BITRL_LOG_TRACE BITSIM_LOG_TO("bitrl", ::bitsim::log::Level::trace)
#define BITRL_LOG_DEBUG BITSIM_LOG_TO("bitrl", ::bitsim::log::Level::debug)
#define BITRL_LOG_INFO BITSIM_LOG_TO("bitrl", ::bitsim::log::Level::info)
#define BITRL_LOG_WARN BITSIM_LOG_TO("bitrl", ::bitsim::log::Level::warn)
#define BITRL_LOG_ERROR BITSIM_LOG_TO("bitrl", ::bitsim::log::Level::error)
#define BITRL_LOG_CRITICAL BITSIM_LOG_TO("bitrl", ::bitsim::log::Level::critical)

#endif // BITRL_LOG_H
