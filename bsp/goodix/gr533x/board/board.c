/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-01-05     Bernard      first implementation
 * 2013-07-12     aozima       update for auto initial.
 */
#include <rthw.h>
#include <rtthread.h>

#include "board.h"
#include "system_gr533x.h"
#include "gr533x_hal_cortex.h"
#include "core_cm4.h"
#include "drivers/pm.h"
#include "gr533x_hal_pwr.h"
#include "app_timer.h"
#include "gr533x_hal_rtc.h"
#include "gr_soc.h"

extern void app_periph_init(void);    
extern void system_platform_init(void);
extern void app_log_flush(void);

/**
 * @addtogroup GR5526
 */

/*@{*/

#if defined(RT_USING_USER_MAIN) && defined(RT_USING_HEAP)

#define RT_HEAP_SIZE   2560
static uint32_t rt_heap[RT_HEAP_SIZE];	// heap default size: 10K(2560 * 4)

RT_WEAK void *rt_heap_begin_get(void)
{
    return rt_heap;
}

RT_WEAK void *rt_heap_end_get(void)
{
    return rt_heap + RT_HEAP_SIZE;
}
#endif


void rt_hw_console_output(const char *str)
{ 
    /* empty console output */
    printf("%s", str);
    app_log_flush();  
}

/*-----------------------------------------------------------
 * Possible configurations for system timer
 */
#define RT_THREAD_USE_RTC      0 /**< Use real time clock for the system */
#define RT_THREAD_USE_SYSTICK  1 /**< Use SysTick timer for system */

#ifdef RT_TICK_USING_SYSTICK
#define configTICK_SOURCE RT_THREAD_USE_SYSTICK
#else
#define configTICK_SOURCE RT_THREAD_USE_RTC
#endif

#if configTICK_SOURCE == RT_THREAD_USE_SYSTICK
/**
 * This is the timer interrupt service routine.
 *
 */
void SysTick_Handler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    rt_tick_increase();

    /* leave interrupt */
    rt_interrupt_leave();
}

void rt_thread_tick_configuration(void)
{
		/* Configure the SysTick */
    SysTick_Config( SystemCoreClock / RT_TICK_PER_SECOND );
    NVIC_EnableIRQ(SysTick_IRQn);
}
#elif configTICK_SOURCE == RT_THREAD_USE_RTC
/**
 * Replace systick with rtc1.
 *
 */
rtc_handle_t s_rtc1_handle; 
uint32_t s_rtc_tick_cnt = 0;
uint32_t s_sys_tick_cnt = 0;


SECTION_RAM_CODE void CALENDAR_IRQHandler(void)
{
    hal_rtc_irq_handler(&s_rtc1_handle);
}

SECTION_RAM_CODE void hal_rtc_tick_callback(rtc_handle_t *p_rtc)
{
    if((AUTO_RELOAD == p_rtc->tick.mode) && \
    (HAL_RTC_ERROR_NONE == p_rtc->error_code))
    {
        /* enter interrupt */
        rt_interrupt_enter();

        rt_tick_increase();

        /* leave interrupt */
        rt_interrupt_leave();
    }
}

void rt_thread_tick_configuration(void)
{
    s_rtc1_handle.p_instance = CALENDAR;
    s_rtc1_handle.init.prescaler_div = RTC_DIV_NONE;
    s_rtc1_handle.init.overflow_det_state = OPENED;
    s_rtc1_handle.init.start_value = 0x0;
    soc_register_nvic(CALENDAR_IRQn, (uint32_t)CALENDAR_IRQHandler);
    hal_rtc_deinit(&s_rtc1_handle);
#if CFG_LPCLK_INTERNAL_EN
    ll_rtc_timer_set_clk(CALENDAR, LL_RTC_TIMER_CLK_SEL_RNG);
#endif
    hal_rtc_init(&s_rtc1_handle);
    NVIC_SetPriority(CALENDAR_IRQn, 0xFF);
    hal_rtc_stop_tick(&s_rtc1_handle);
    hal_rtc_set_tick_and_start(&s_rtc1_handle, AUTO_RELOAD, RTC_COUNT_PERIOD_TICK);
    s_rtc_tick_cnt = ll_rtc_get_read_counter(s_rtc1_handle.p_instance);
    s_sys_tick_cnt = rt_tick_get();
    
}

#endif	

/**
 * This function will initial GR5526 board.
 */
void rt_hw_board_init(void)
{
    //system_platform_init();

    rt_thread_tick_configuration();

    app_periph_init();

#ifdef RT_USING_SERIAL
    //rt_hw_usart_init();
#endif

#ifdef RT_USING_COMPONENTS_INIT
    rt_components_board_init();
#endif

#if defined(RT_USING_USER_MAIN) && defined(RT_USING_HEAP)
    rt_system_heap_init(rt_heap_begin_get(), rt_heap_end_get());
#endif
    
#if defined(RT_USING_CONSOLE) && defined(RT_USING_DEVICE)
    rt_console_set_device(RT_CONSOLE_DEVICE_NAME);
#endif

}

/*@}*/
