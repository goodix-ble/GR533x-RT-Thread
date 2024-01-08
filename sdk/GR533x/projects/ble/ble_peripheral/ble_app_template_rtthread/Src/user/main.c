/**
 *****************************************************************************************
 *
 * @file main.c
 *
 * @brief main function Implementation.
 *
 *****************************************************************************************
 * @attention
  #####Copyright (c) 2019 GOODIX
  All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
  * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
  * Neither the name of GOODIX nor the names of its contributors may be used
    to endorse or promote products derived from this software without
    specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************************
 */

/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include "user_app.h"
#include "user_periph_setup.h"
#include "gr_includes.h"
#include "scatter_common.h"
#include "flash_scatter_config.h"
#include "custom_config.h"
#include "patch.h"
#include "app_log.h"
#include "app_rtc.h"

#include <board.h>

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
/**@brief Stack global variables for Bluetooth protocol stack. */
STACK_HEAP_INIT(heaps_table);
calendar_time_t g_calendar_time;
char *const     weeday_str[7] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

static void uart_thread_entry(void* parameter)
{	
    while (1)
    {
        app_rtc_get_time(&g_calendar_time);
        rt_kprintf("Time: %02d/%02d %02d:%02d:%02d.%03d\r\n",
                    g_calendar_time.mon, g_calendar_time.date,
                    g_calendar_time.hour, g_calendar_time.min,
                    g_calendar_time.sec, g_calendar_time.ms);

        app_log_flush();
        rt_thread_delay(1000);   
    }
}

static void app_calendar_init(void)
{
    g_calendar_time.year = 21;
    g_calendar_time.mon  = 12;
    g_calendar_time.date = 1;
    g_calendar_time.hour = 1;
    g_calendar_time.min  = 00;
    g_calendar_time.sec  = 00;
    app_rtc_init(NULL);
    app_rtc_init_time(&g_calendar_time);
}

void rt_sample_application_init(void)
{
    rt_thread_t uart_thread = NULL;

    app_calendar_init();

    uart_thread = rt_thread_create( "uart_task", uart_thread_entry, NULL, 1024, RT_THREAD_PRIORITY_MAX - 10, 8);
    if (uart_thread != RT_NULL)
    {
        rt_thread_startup(uart_thread);
    }
    else
    {
        printf("rt thread create fail");
    }
}

int main (void)
{
    // Initialize ble stack.
    ble_stack_init(ble_evt_handler, &heaps_table);

    rt_sample_application_init();
}
