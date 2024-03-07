#include "gr_soc.h"
#include "grx_hal.h"

#ifndef DRIVER_TEST
#include "gr_includes.h"
#endif

#include "platform_sdk.h"
#include "custom_config.h"
#include "hal_flash.h"
#include "pmu_calibration.h"
#include "app_pwr_mgmt.h"

#define PUYA_FLASH_HP_CMD               (0xA3)
#define PUYA_FLASH_HP_END_DUMMY         (2)

#define FALSH_HP_MODE                   LL_XQSPI_HP_MODE_EN
#define FLASH_HP_CMD                    PUYA_FLASH_HP_CMD
#define FLASH_HP_END_DUMMY              PUYA_FLASH_HP_END_DUMMY

#if (BLE_SUPPORT == 1)
#ifndef DRIVER_TEST
static mesh_config_dev_num_t mesh_config_dev_mun =
{
    .mesh_net_key_list_num = 0,
    .mesh_app_key_list_num = 0,
    .mesh_piblic_subscr_list_num = 0,
    .mesh_friend_num = 0,
    .mesh_LPN_num = 0
};
#endif
#endif

__ALIGNED(0x400) FuncVector_t FuncVector_table[MAX_NUMS_IRQn + NVIC_USER_IRQ_OFFSET] = {
    0,
    Reset_Handler,
    NMI_Handler,
    HardFault_Handler,
    MemManage_Handler,
    BusFault_Handler,
    UsageFault_Handler,
    0,
    0,
    0,
    0,
    SVC_Handler,
    DebugMon_Handler,
    0,
    PendSV_Handler,
    SysTick_Handler,
};

void soc_register_nvic(IRQn_Type indx, uint32_t func)
{
    FuncVector_table[indx + NVIC_USER_IRQ_OFFSET] = (FuncVector_t)func;
}

static fun_t svc_user_func = NULL;

void svc_func_register(uint8_t svc_num, uint32_t user_func)
{
    svc_user_func = (fun_t)user_func;
}

void svc_user_handler(uint8_t svc_num)
{
    if (svc_user_func)
        svc_user_func();
}

#if (BLE_SUPPORT == 1)
__WEAK void nvds_init_error_handler(uint8_t err_code)
{
#ifdef NVDS_START_ADDR
    nvds_deinit(NVDS_START_ADDR, NVDS_NUM_SECTOR);
    nvds_init(NVDS_START_ADDR, NVDS_NUM_SECTOR);
#else
    /* nvds_deinit will erase the flash area and old data will be lost */
    nvds_deinit(0, NVDS_NUM_SECTOR);
    nvds_init(0, NVDS_NUM_SECTOR);
#endif
}

static void nvds_setup(void)
{
#ifndef DRIVER_TEST
    nvds_retention_size(CFG_MAX_BOND_DEVS, mesh_config_dev_mun);
#endif

#ifdef NVDS_START_ADDR
    uint8_t err_code = nvds_init(NVDS_START_ADDR, NVDS_NUM_SECTOR);
#else
    uint8_t err_code = nvds_init(0, NVDS_NUM_SECTOR);
#endif
    uint8_t init_err_code = nvds_get_init_error_info();

    switch(err_code)
    {
        case NVDS_SUCCESS:
            break;
        default:
            /* Nvds initialization other errors.
             * For more information, please see NVDS_INIT_ERR_CODE. */
            nvds_init_error_handler(err_code);
            break;
    }
}
#endif

void first_class_task(void)
{
    ll_xqspi_hp_init_t hp_init;

    platform_exflash_env_init();

    hp_init.xqspi_hp_enable    = FALSH_HP_MODE;
    hp_init.xqspi_hp_cmd       = FLASH_HP_CMD;
    hp_init.xqspi_hp_end_dummy = FLASH_HP_END_DUMMY;
    hal_exflash_enable_quad(hp_init);

#if (BLE_SUPPORT == 1)
    /* set sram power state. */
    mem_pwr_mgmt_mode_set(MEM_POWER_AUTO_MODE);

    /* nvds module init process. */
    nvds_setup();

    /* platform init process. */
    platform_sdk_init();
#endif
}

void second_class_task(void)
{
#if (BLE_SUPPORT == 1)
    /* To choose the System clock source and set the accuracy of OSC. */
#if CFG_LPCLK_INTERNAL_EN
    platform_clock_init((mcu_clock_type_t)SYSTEM_CLOCK, RC_OSC_CLK, CFG_LF_ACCURACY_PPM, 0);
#else
    platform_clock_init((mcu_clock_type_t)SYSTEM_CLOCK, RTC_OSC_CLK, CFG_LF_ACCURACY_PPM, 0);
#endif

#if PMU_CALIBRATION_ENABLE && !defined(DRIVER_TEST)
    /* Enable auto pmu calibration function. */
    if(!CHECK_IS_ON_FPGA())
    {
        system_pmu_calibration_init(30000);
    }
#endif
    system_pmu_init((mcu_clock_type_t)SYSTEM_CLOCK);
#endif

    // pmu shall be init before clock set
    system_power_mode((sys_power_t)SYSTEM_POWER_MODE);
    SetSerialClock(SERIAL_S64M_CLK);
    SystemCoreSetClock((mcu_clock_type_t)SYSTEM_CLOCK);

#if (BLE_SUPPORT == 1)
    // recover the default setting by temperature, should be called in the end
    if(!CHECK_IS_ON_FPGA())
    {
        pmu_calibration_handler(NULL);
    }
    /* Init peripheral sleep management */
    
    /* nix: Comment following Call to avoid sleep issue */
    //app_pwr_mgmt_init();
#endif
}

void otp_trim_init(void)
{
#if (BLE_SUPPORT == 1)
    if(SDK_SUCCESS != sys_trim_info_sync())
    {
        if(!CHECK_IS_ON_FPGA())
        {
            /* do nothing for not calibrated chips */
            while(1);
        }
    }
#endif
}

/* bug fix for #C.493 */

/* digcore volt init info */
typedef struct
{
    uint8_t     dcore_method;  /* enum SYS_PMU_CONFIG_DCORE_METHOD */
    uint32_t    tt_fix_tgt_mv; /* fixed target digcore volt of TT/FF/SF/FS chip */
    uint32_t    ss_fix_tgt_mv; /* fixed target digcore volt of SS chip */
    uint32_t    tgt_ringo;     /* target dvs ringo for finding suitable digcore volt */
    uint32_t    min_dcore_mv;  /* the min digcore volt for mcu work ok, in case the volt finding by ringo too low */
}pmu_dcore_init_para_st;

extern pmu_dcore_init_para_st g_pmu_dcore_init_para_64M;
extern pmu_dcore_init_para_st g_pmu_dcore_init_para_16M;

void platform_init(void)
{
    gr5xx_fpb_init(FPB_MODE_PATCH_AND_DEBUG);
    otp_trim_init();

    /* bug fix for #C.493 */
    {
        g_pmu_dcore_init_para_64M.dcore_method  = 2;
        g_pmu_dcore_init_para_64M.tt_fix_tgt_mv = 950;
        g_pmu_dcore_init_para_64M.ss_fix_tgt_mv = 1050;
        g_pmu_dcore_init_para_64M.tgt_ringo     = 1600;
        g_pmu_dcore_init_para_64M.min_dcore_mv  = 950;

        g_pmu_dcore_init_para_16M.dcore_method  = 2;
        g_pmu_dcore_init_para_16M.tt_fix_tgt_mv = 950;
        g_pmu_dcore_init_para_16M.ss_fix_tgt_mv = 1050;
        g_pmu_dcore_init_para_16M.tgt_ringo     = 1600;
        g_pmu_dcore_init_para_16M.min_dcore_mv  = 950;

        AON_CTL->MEM_MARGIN = 0xDD;
    }

    first_class_task();
    second_class_task();
}

void vector_table_init(void)
{
    __DMB(); // Data Memory Barrier
    FuncVector_table[0] = *(FuncVector_t *)(SCB->VTOR);
    SCB->VTOR = (uint32_t)FuncVector_table; // Set VTOR to the new vector table location
    __DSB(); // Data Synchronization Barrier to ensure all
}

void warm_boot_process(void)
{
#if (BLE_SUPPORT == 1)
    vector_table_init();
    pwr_mgmt_warm_boot();
#endif
}

void soc_init(void)
{
    /* Disable WDT */
    ll_aon_wdt_unlock();
    ll_aon_wdt_disable();
    while(ll_aon_wdt_is_busy());
    ll_aon_wdt_lock();

    platform_init();
}

__WEAK void sdk_init(void){};
