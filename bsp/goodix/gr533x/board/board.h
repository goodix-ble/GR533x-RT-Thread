/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-09-22     Bernard      add board.h to this bsp
 */

// <<< Use Configuration Wizard in Context Menu >>>
#ifndef __BOARD_H__
#define __BOARD_H__

#include <rthw.h>
#include "rtthread.h"

#if !CFG_LPCLK_INTERNAL_EN
#define RTC_CLOCK_HZ                   (32768U)
#else
/* If CFG_LPCLK_INTERNAL_EN == 1, RTC1 using RNG clock. */
#define RTC_CLOCK_HZ                   (32000U)
#endif
#define RTC_COUNT_PERIOD_TICK          ((uint32_t) ((RTC_CLOCK_HZ/RT_TICK_PER_SECOND)))
#define RTCCOUNT_TO_RTOSTICK(COUNT)    ((((uint64_t)(COUNT))*RT_TICK_PER_SECOND)/RTC_CLOCK_HZ)
#define RTOSTICK_TO_RTCCOUNT(TICK)     ((((uint64_t)(TICK))*RTC_CLOCK_HZ)/RT_TICK_PER_SECOND)

/* board configuration */
void rt_hw_board_init(void);

#endif /* __BOARD_H__ */
