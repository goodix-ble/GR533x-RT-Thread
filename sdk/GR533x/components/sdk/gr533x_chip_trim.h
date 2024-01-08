/**
 *******************************************************************************
 *
 * @file gr533x_chip_trim.h
 *
 * @brief GR533X chip trim
 *
 *******************************************************************************
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

/**
 @addtogroup SYSTEM
 @{
 */

/**
 * @addtogroup SYS System SDK
 * @{
 * @brief Definitions and prototypes for the system SDK interface.
*/

#ifndef __GR533X_CHIP_TRIM_H__
#define __GR533X_CHIP_TRIM_H__

#include<stdio.h>
#include<stdint.h>

#define CHIP_TRIM_PATTERN (0x4744)

typedef struct
{
    uint16_t ate_version;
    uint16_t hw_version;
    uint16_t chip_id;
    uint16_t package;
    uint16_t flash_size;
    uint16_t ram_size;
    uint8_t  unused[12];
} __attribute__ ((packed)) production_t;

typedef struct
{
    uint8_t tx_power;
    uint8_t rssi_cali;
    uint8_t hp_gain;
    uint8_t unused[1];
} __attribute__ ((packed)) rf_trim_t;

typedef struct
{
    uint8_t dcdc_1p05;
    uint8_t dcdc_1p15;
    uint8_t dig_ldo_0p9_coarse;
    uint8_t dig_ldo_0P9_fine;
    uint8_t dig_ldo_1p05_coarse;
    uint8_t dig_ldo_1p05_fine;
    uint8_t sys_ldo_1p05;
    uint8_t sys_ldo_1p15;
    uint8_t io_ldo_1p8;
    uint8_t io_ldo_3p0;
    uint8_t stb_io_ldo_1P8;
    uint8_t stb_io_ldo_3p0;
    uint8_t unused[4];
} __attribute__ ((packed)) pmu_trim_t;

typedef struct
{
    uint16_t offset_int_0p8;  /* Offset based on interanl reference(0.85V) */
    uint16_t slope_int_0p8;   /* Slope based on interanl reference(0.85V)  */
    uint16_t offset_int_1p2;  /* Offset based on interanl reference(1.28V) */
    uint16_t slope_int_1p2;   /* Slope based on interanl reference(1.28V)  */
    uint16_t offset_int_1p6;  /* Offset based on interanl reference(0.85V) */
    uint16_t slope_int_1p6;   /* Offset based on interanl reference(0.85V) */
    uint16_t offset_ext_1p0;  /* Offset based on interanl reference(1.6V)  */
    uint16_t slope_ext_1p0;   /* Slope based on interanl reference(1.6V)   */
    uint16_t temp;            /* Offset based on interanl reference(0.85V) */
    uint16_t temp_ref;        /* Chip temperature sensor reference temperature. E.g. Decimal 2618: 26.18��C */
    uint8_t  unused[4];
} __attribute__ ((packed)) sadc_trim_t;

typedef struct
{
    uint16_t hf_osc_192m;
    uint16_t hf_osc_192m_fine;
    uint8_t  unused[4];
} __attribute__ ((packed)) clk_trim_t;

typedef struct
{
    uint8_t flash_manufacturer;  /* flash manufacturer type PUYA: P25Q40SU: 0x11                                      */
    uint8_t feature;             /* 0bit: 1 ~ support 512byte write, 0 ~ not support 512 byte write                   */
    uint8_t flash_tVSL;          /* VCC(min.) to device operation. Uint: 10us                                         */
    uint8_t flash_tESL;          /* Erase suspend latency. Uint: 5us                                                  */
    uint8_t flash_tPSL;          /* Program suspend latency. Uint: 5us                                                */
    uint8_t flash_tPRS;          /* Latency between program resume and next suspend. Uint: 5us                        */
    uint8_t flash_tERS;          /* Latency between erase resume and next suspend. Uint: 5us                          */
    uint8_t flash_tDP;           /* CS# High to Deep Power-down Mode. Uint: 5us                                       */
    uint8_t flash_tRES2;         /* CS# High To Standby Mode With Electronic Signature Read. Uint: 5us                */
    uint8_t flash_tRDINT;        /* Read status register interval when wait busy. Uint: 5us                           */
    uint8_t unused[2];
} __attribute__ ((packed)) flash_timing_t;

typedef struct {
    uint16_t       pattern;
    uint16_t       item_end;
    uint32_t       check_sum;
    production_t   prod;
    rf_trim_t      rf;
    pmu_trim_t     pmu;
    sadc_trim_t    sadc;
    clk_trim_t     clk;
    flash_timing_t flash;
//    uint8_t        unused[168];
} __attribute__ ((packed)) chip_trim0_t;

typedef struct {
    uint8_t  bt_addr[6];
    uint16_t xo_offset;
//    uint8_t  reserved[24];
//    uint8_t  user[224];
} __attribute__ ((packed)) chip_trim1_t;


/** @} */
#endif

/** @} */
