/*
 * Copyright (c) 2012, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

// File: iomux_config.h

/* ------------------------------------------------------------------------------
 * <auto-generated>
 *     This code was generated by a tool.
 *     Runtime Version:3.4.0.0
 *
 *     Changes to this file may cause incorrect behavior and will be lost if
 *     the code is regenerated.
 * </auto-generated>
 * ------------------------------------------------------------------------------
*/

#ifndef _IOMUX_CONFIG_H_
#define _IOMUX_CONFIG_H_

// Board and Module IOMUXC configuration function prototypes.

#if defined(__cplusplus)
extern "C" {
#endif

// Board IOMUXC configuration function.
void iomux_config(void);

// Module IOMUXC configuration functions.
void arm_iomux_config(void);
void asrc_iomux_config(void);
void audmux_iomux_config(void);
void ccm_iomux_config(void);
void dcic_iomux_config(int instance);
void dcic1_iomux_config(void);
void dcic2_iomux_config(void);
void ecspi_iomux_config(int instance);
void ecspi1_iomux_config(void);
void ecspi2_iomux_config(void);
void ecspi3_iomux_config(void);
void ecspi4_iomux_config(void);
void ecspi5_iomux_config(void);
void eim_iomux_config(void);
void enet_iomux_config(void);
void epit_iomux_config(int instance);
void epit1_iomux_config(void);
void epit2_iomux_config(void);
void esai_iomux_config(void);
void flexcan_iomux_config(int instance);
void flexcan1_iomux_config(void);
void flexcan2_iomux_config(void);
void gpio_iomux_config(int instance);
void gpio1_iomux_config(void);
void gpio2_iomux_config(void);
void gpio3_iomux_config(void);
void gpio4_iomux_config(void);
void gpio5_iomux_config(void);
void gpio6_iomux_config(void);
void gpio7_iomux_config(void);
void gpmi_iomux_config(void);
void gpt_iomux_config(void);
void hdmi_iomux_config(void);
void i2c_iomux_config(int instance);
void i2c1_iomux_config(void);
void i2c2_iomux_config(void);
void i2c3_iomux_config(void);
void ipu_iomux_config(int instance);
void ipu1_iomux_config(void);
void ipu2_iomux_config(void);
void kpp_iomux_config(void);
void ldb_iomux_config(void);
void mipi_csi_iomux_config(void);
void mipi_dsi_iomux_config(void);
void mipi_hsi_iomux_config(void);
void mlb_iomux_config(void);
void mmdc_iomux_config(void);
void pcie_iomux_config(void);
void pmu_iomux_config(void);
void pwm_iomux_config(int instance);
void pwm1_iomux_config(void);
void pwm2_iomux_config(void);
void pwm3_iomux_config(void);
void pwm4_iomux_config(void);
void sata_phy_iomux_config(void);
void sdma_iomux_config(void);
void sjc_iomux_config(void);
void snvs_iomux_config(void);
void spdif_iomux_config(void);
void src_iomux_config(void);
void uart_iomux_config(int instance);
void uart1_iomux_config(void);
void uart2_iomux_config(void);
void uart3_iomux_config(void);
void uart4_iomux_config(void);
void uart5_iomux_config(void);
void usb_iomux_config(void);
void usdhc_iomux_config(int instance);
void usdhc1_iomux_config(void);
void usdhc2_iomux_config(void);
void usdhc3_iomux_config(void);
void usdhc4_iomux_config(void);
void wdog_iomux_config(int instance);
void wdog1_iomux_config(void);
void wdog2_iomux_config(void);
void xtalosc_iomux_config(void);

#if defined(__cplusplus)
}
#endif

#endif // _IOMUX_CONFIG_H_
