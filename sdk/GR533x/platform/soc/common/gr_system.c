/**************************************************************************//**
 * @file     gr_system.c
 * @brief    CMSIS Device System Source File for
 *           Device GR55xx
 * @version  V1.00
 * @date     12. June 2018
 ******************************************************************************/
/*
 * Copyright (c) 2009-2016 ARM Limited. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdint.h>
#include "grx_sys.h"

/*----------------------------------------------------------------------------
  Global Functions
 *----------------------------------------------------------------------------*/


__WEAK void SystemInit(void)
{
    #if (__FPU_USED == 1)
    SCB->CPACR |= ((3UL << 10*2) |                 /* set CP10 Full Access */
                    (3UL << 11*2)  );               /* set CP11 Full Access */
    #endif

    #ifdef UNALIGNED_SUPPORT_DISABLE
    SCB->CCR |= SCB_CCR_UNALIGN_TRP_Msk;
    #endif

    return;
}
