#include "grx_hal.h"
#include "grx_sys.h"
#include "gr_soc.h"

#include "rthw.h"
#include "rtthread.h"
#include "pm.h"
#include "board.h"


#if( configOVERRIDE_DEFAULT_TICK_CONFIGURATION == 1 )
/*
 * LOCAL MACRO DEFINITIONS
 *****************************************************************************************
 */
#define MAX_SYS_SLEEP_TICKS            (100000) /* Unit: RTOS TICK */

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
extern rtc_handle_t s_rtc1_handle;
extern uint32_t s_sys_tick_cnt;
extern uint32_t s_rtc_tick_cnt;
static uint32_t s_expect_sleep_ticks = 0;
static uint32_t s_wakeup_tick_cnt = 0;

/**
  * @brief Function Implementations
  */
SECTION_RAM_CODE uint32_t vPortLocker(void)
{
	    return rt_hw_interrupt_disable();
}

SECTION_RAM_CODE void vPortUnLocker(uint32_t set_pri)
{
			rt_hw_interrupt_enable(set_pri);
}

SECTION_RAM_CODE static uint32_t vDisableIRQExeptBLE(void)
{
    uint32_t __l_irq_rest=__get_BASEPRI();
    __set_BASEPRI(NVIC_GetPriority(BLE_IRQn) +(1 << (NVIC_GetPriorityGrouping() + 1)));
    return __l_irq_rest;
}

SECTION_RAM_CODE static void vRestoreIRQExeptBLE(uint32_t set_pri)
{
      __set_BASEPRI(set_pri);
}

/*
 * WEAK Implementation to support graphics power management
 *****************************************************************************************
 */

SECTION_RAM_CODE void wfe_func(void)
{
    __WFI();
}

SECTION_RAM_CODE static void systick_sleep_timeout_set(uint32_t timeout_tick)
{
    hal_rtc_stop_tick(&s_rtc1_handle);
    // Limit the maximum sleep time
    if( timeout_tick > MAX_SYS_SLEEP_TICKS )
    {
        timeout_tick = MAX_SYS_SLEEP_TICKS;
    }
    if( timeout_tick > s_wakeup_tick_cnt)
    {
        timeout_tick -= s_wakeup_tick_cnt;
    }
    uint32_t rtc_count = RTOSTICK_TO_RTCCOUNT(timeout_tick);
    hal_rtc_set_tick_and_start(&s_rtc1_handle, ONE_TIME, rtc_count);
}

SECTION_RAM_CODE static uint32_t system_sleep_tick_compensate(void)
{
    uint32_t cur_rtc_tick_cnt = ll_rtc_get_read_counter(s_rtc1_handle.p_instance);
    uint32_t cur_sys_tick_cnt = rt_tick_get();
    uint32_t rtos_tick_diff = cur_sys_tick_cnt - s_sys_tick_cnt;
    uint32_t rtc_tick_diff = cur_rtc_tick_cnt - s_rtc_tick_cnt;
    uint32_t rtc_2_rtos_tick_diff = RTCCOUNT_TO_RTOSTICK(rtc_tick_diff);
    // No need to compensate when exit sleep within 1 tick
    if( rtc_2_rtos_tick_diff <= rtos_tick_diff )
    {
        return 0;
    }
    uint32_t step_tick = rtc_2_rtos_tick_diff - rtos_tick_diff;
    // Limit the compensate tick within normal range
    if( step_tick > s_expect_sleep_ticks )
    {
        s_wakeup_tick_cnt = step_tick - s_expect_sleep_ticks;
        step_tick = s_expect_sleep_ticks;
    }
    else
    {
        s_wakeup_tick_cnt = 0;
    }
    rt_tick_set(cur_sys_tick_cnt + step_tick);
    // Update the tick reference every 100 seconds to reduce deviation
    // Notice the RTC frequency may change with the temperature change
    if( rtos_tick_diff > MAX_SYS_SLEEP_TICKS )
    {
        s_rtc_tick_cnt = cur_rtc_tick_cnt;
        s_sys_tick_cnt = rt_tick_get();
    }
    return step_tick;
}

SECTION_RAM_CODE static void systick_compensate_restart(void)
{
    system_sleep_tick_compensate();
    hal_rtc_set_tick_and_start(&s_rtc1_handle, AUTO_RELOAD, RTC_COUNT_PERIOD_TICK);
}

SECTION_RAM_CODE static uint8_t pwr_mgmt_system_sleep(void)
{
    uint8_t ret = pwr_mgmt_sleep();

    return ret;
}

SECTION_RAM_CODE static void pwr_mgmt_enter_sleep_with_cond(uint32_t pwr_mgmt_expected_time)
{
    s_expect_sleep_ticks = pwr_mgmt_expected_time;
    uint32_t pwr_locker = vPortLocker();
    uint32_t ble_unlocker;

    systick_sleep_timeout_set(s_expect_sleep_ticks);
    if ((s_expect_sleep_ticks < 5) ||
        (PMR_MGMT_SLEEP_MODE != pwr_mgmt_mode_get()) ||
        (DEVICE_BUSY == pwr_mgmt_dev_suspend()) )
    {
        wfe_func();
        //Enable BLE IRQ during systick compensate for ble schedule on time
        ble_unlocker = vDisableIRQExeptBLE();
        vPortUnLocker(pwr_locker);
        systick_compensate_restart();
        vRestoreIRQExeptBLE(ble_unlocker);
        return ;
    }

    uint8_t ret = pwr_mgmt_system_sleep();
    if (ret != PMR_MGMT_SUCCESS)
    {
        wfe_func();
        //Enable BLE IRQ during systick compensate for ble schedule on time
        ble_unlocker = vDisableIRQExeptBLE();
        vPortUnLocker(pwr_locker);
        systick_compensate_restart();
        vRestoreIRQExeptBLE(ble_unlocker);
    }
    else // Wakeup from deep sleep mode
    {
        uint32_t _local_lock = vPortLocker();
        systick_compensate_restart();
        vPortUnLocker(_local_lock);
    }
}
#endif
/**
 *****************************************************************************************
 * @brief port_enter_deep_sleep
 *
 * @param[in] expected_idle_time: next task resume time to work
 *
 * @return void
 *****************************************************************************************
 */
SECTION_RAM_CODE void port_enter_deep_sleep(uint32_t expected_idle_time)
{
#if( configOVERRIDE_DEFAULT_TICK_CONFIGURATION == 1 )
	  if(expected_idle_time < 5)
		{
				wfe_func();
			  return;
		}
		
    pwr_mgmt_enter_sleep_with_cond(expected_idle_time);
#else
    __WFI();
#endif		
}

void rt_system_power_manager(void)
{
    rt_tick_t timeout_tick;

    timeout_tick = rt_timer_next_timeout_tick();
    timeout_tick -= rt_tick_get();
	
    port_enter_deep_sleep(timeout_tick);
}
