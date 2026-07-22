/*
 * Copyright @ Huawei Technologies Co., Ltd. 2019-2029. All rights reserved.
 * Description: Macros.h
 * Author: l00434636
 * Create: 2020-10-27
 */

#ifndef __MACROS_H_
#define __MACROS_H_

#include <iostream>

#define ERROR(str) (std::cerr << "[ERROR (" << __FILE__ << ":" << __LINE__ << ")]: " << str << std::endl);
#define WARNING(str) (std::cerr << "[Warning (" << __FILE__ << ":" << __LINE__ << ")]: " << str << std::endl);

using std::ostream;

#define DEBUG(str) (std::cout << std::fixed << str << std::endl);
#define DEBUG_TIME(str)                                       \
    if (now() >= DEBUG_START_TIME && now() <= DEBUG_END_TIME) \
        std::cout << std::fixed << str << std::endl;
#define DEBUGN(str) std::cout << std::fixed << str;

#define PRINT(str)                                    \
    {                                                 \
        DDRSim_log << std::fixed << str << std::endl; \
        DDRSim_log.flush();                           \
    }
#define PRINTN(str)                                           \
    if (now() >= DEBUG_START_TIME && now() <= DEBUG_END_TIME) \
        DDRSim_log << std::fixed << str;                      \
    DDRSim_log.flush();
#define PRINT_SPD_PTC(channel, fmt, ...)                                                                 \
    do {                                                                                                 \
        auto logger = SPD_LOGGER::esl_log::get_logger_by_module("dmc_sim" + std::to_string(channel)); \
        if (logger && logger->should_log(spdlog::level::debug) && now() >= DEBUG_START_TIME &&           \
            now() <= DEBUG_END_TIME) {                                                                   \
                                                                                                         \
            if (logger->sinks().size() > 1) {                                                            \
                auto old_level = logger->sinks()[0] -> level();                                          \
                logger->sinks()[0]->set_level(spdlog::level::off);                                       \
                logger->debug(fmt, ##__VA_ARGS__);                                                       \
                logger->sinks()[0]->set_level(old_level);                                                \
            } else {                                                                                     \
                logger->debug(fmt, ##__VA_ARGS__);                                                       \
            }                                                                                            \
        }                                                                                                \
    } while (0);
#define PRINT_SPD_PFQ(channel, fmt, ...)                                                                 \
    do {                                                                                                 \
        auto logger = SPD_LOGGER::esl_log::get_logger_by_module("dmc_sim" + std::to_string(channel));; \
        if (logger && logger->should_log(spdlog::level::debug) && now() >= DEBUG_START_TIME &&           \
            now() <= DEBUG_END_TIME) {                                                                   \
                                                                                                         \
            if (logger->sinks().size() > 1) {                                                            \
                auto old_level = logger->sinks()[0] -> level();                                          \
                logger->sinks()[0]->set_level(spdlog::level::off);                                       \
                logger->debug(fmt, ##__VA_ARGS__);                                                       \
                logger->sinks()[0]->set_level(old_level);                                                \
            } else {                                                                                     \
                logger->debug(fmt, ##__VA_ARGS__);                                                       \
            }                                                                                            \
        }                                                                                                \
    } while (0);
#define PRINT_SPD_STATE(channel, fmt, ...)                                               \
    do {                                                                                 \
        auto logger = SPD_LOGGER::esl_log::get_logger_by_module("dmc" + std::to_string(channel)); \
        if (logger && logger->should_log(spdlog::level::info)) {                         \
            if (logger->sinks().size() > 1) {                                            \
                auto &sink = logger->sinks()[0];                                         \
                auto old_level = sink->level();                                          \
                sink->set_level(spdlog::level::off);                                     \
                try {                                                                    \
                    logger->info(fmt, ##__VA_ARGS__);                                    \
                } catch (...) {                                                          \
                }                                                                        \
                sink->set_level(old_level);                                              \
            } else {                                                                     \
                logger->info(fmt, ##__VA_ARGS__);                                        \
            }                                                                            \
        }                                                                                \
    } while (0);
#define STATE_PRINTN(channel, str)               \
    if ((channel) < state_log.size()) {          \
        state_log[channel] << std::fixed << str; \
        state_log[channel].flush();              \
    }
#define TRACE_PRINT(channel, str)                                                         \
    if ((channel) < trace_log.size() && channel_ohot == (channel_ohot & PRINT_CH_OHOT)) { \
        trace_log[channel] << std::fixed << str;                                          \
        trace_log[channel].flush();                                                       \
    }
#define CMDNUM_PRINT(str)                                 \
    if (channel_ohot == (channel_ohot & PRINT_CH_OHOT)) { \
        cmdnum_log << std::fixed << str;                  \
        cmdnum_log.flush();                               \
    }
#define DRAM_PRINT(str)                                   \
    if (channel_ohot == (channel_ohot & PRINT_CH_OHOT)) { \
        dram_log << std::fixed << str << endl;            \
        dram_log.flush();                                 \
    }
#define PRINTR(str)                      \
    {                                    \
        DDRSim_log << std::fixed << str; \
    }
#define PRINTN_M(channel, str)                                                                 \
    if ((channel) < DDRSim_log.size() && now() >= DEBUG_START_TIME && now() <= DEBUG_END_TIME) \
        DDRSim_log[channel] << std::fixed << str;                                              \
    DDRSim_log[channel].flush();
#define PRINTN_PFQ(id, str)                                                                   \
    if ((id) < DDRSim_pfq_log.size() && now() >= DEBUG_START_TIME && now() <= DEBUG_END_TIME) \
        DDRSim_pfq_log[id] << std::fixed << str;                                              \
    DDRSim_pfq_log[id].flush();

#define ALIGNED_SIZE (0x7)
#define ALIGNED_DATA_64B (0xFFFFFF80)
#define ALIGNED_NUMB_64B (0x40)
#define INVALID_POSITION (0xffffffff)

// ---------------------------------------------------------------------
// general useful macro function definitions
// ---------------------------------------------------------------------

// BitValue : recieve a specific bit value from a data
#ifndef BitValue
#define BitValue(data, bit) (((data) >> (bit)) & ((uint32_t)(0x1)))
#endif

// BitValueInBool : receive a specific bit value from a data, and then
// convert this value into bool format
#ifndef BitValueInBool
#define BitValueInBool(data, bit) (bool)BitValue(data, bit)
#endif

// SetBitValue : set a specific bit value to '1' in a data
#ifndef SetBitValue
#define SetBitValue(data, bit) ((data) = (data) | (((uint32_t)(0x1)) << (bit)))
#endif

// ClrBitValue : set a specific bit value to '0' in a data
#ifndef ClrBitValue
#define ClrBitValue(data, bit) ((data) = (data) & (~(((uint32_t)(0x1)) << (bit))))
#endif

#define EMB_GET_PARAM(para1, para2, type) para1 = cfg->type(para2);
#define GET_PARAM(para1, para2, type) \
    if (cfg.has(para2)) {             \
        para1 = cfg.type(para2);      \
    }
#define dmc_random(value, x, y) \
    srand((int)time(0));        \
    value = (rand() % (y - x + 1)) + x;
#define DmcLog2(x, y) pow(2, floor(log2(x *y)))
#define MidMax 256
#define BIT_GET(value, x, y) ((unsigned(value) >> unsigned(x)) & unsigned(pow(2, unsigned(y)) - 1))

// #define SYSARCH_PLATFORM

#endif /*PRINT_MACROS_H*/
