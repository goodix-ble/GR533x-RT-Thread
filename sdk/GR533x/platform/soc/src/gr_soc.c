#include "gr_soc.h"
#include "grx_hal.h"

#ifndef DRIVER_TEST
#include "gr_includes.h"
#endif

#include "platform_sdk.h"
#include "custom_config.h"
#include "hal_flash.h"
#include "pmu_calibration.h"
#include "patch_tab.h"
#include "app_pwr_mgmt.h"

#define PUYA_FLASH_HP_CMD               (0xA3)
#define PUYA_FLASH_HP_END_DUMMY         (2)

#define FALSH_HP_MODE                   LL_XQSPI_HP_MODE_EN
#define FLASH_HP_CMD                    PUYA_FLASH_HP_CMD
#define FLASH_HP_END_DUMMY              PUYA_FLASH_HP_END_DUMMY

#if (EM_BUFF_ENABLE == 1)
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

#if (EM_BUFF_ENABLE == 1)
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
        case NVDS_STORAGE_ACCESS_FAILED:
                /* Operation flash failed.
                 * Please check if the flash is damaged. */
                while(1);
        case NVDS_SUCCESS:
            break;
        default:
            /* Nvds initialization other errors.
             * For more information, please see NVDS_INIT_ERR_CODE. */
            while(1);
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

#if (EM_BUFF_ENABLE == 1)
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
#if (EM_BUFF_ENABLE == 1)
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

#if (EM_BUFF_ENABLE == 1)
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
#if (EM_BUFF_ENABLE == 1)
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

void platform_init(void)
{
    otp_trim_init();
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
    vector_table_init();
    pwr_mgmt_warm_boot();
}

extern void sniffer_func_init(void);
void ble_feature_init(void)
{
#if (EM_BUFF_ENABLE == 1)
    #if (CFG_MAX_ADVS)
    adv_func_init();
    #endif

    #if (CFG_MAX_SCAN)
    scan_func_init();
    #endif

    #if (CFG_MASTER_SUPPORT)
    master_func_init();
    #endif

    #if (CFG_SLAVE_SUPPORT)
    slave_func_init();
    #endif

    #if (CFG_LEGACY_PAIR_SUPPORT)
    legacy_pair_func_init();
    #endif

    #if (CFG_SC_PAIR_SUPPORT)
    sc_pair_func_init();
    #endif

    #if (CFG_COC_SUPPORT)
    coc_func_init();
    #endif

    #if (CFG_GATTS_SUPPORT)
    gatts_func_init();
    #endif

    #if (CFG_GATTC_SUPPORT)
    gattc_func_init();
    #endif

    #if (CFG_CONN_AOA_AOD_SUPPORT)
    conn_aoa_aod_func_init();
    #endif

    #if (CFG_CONNLESS_AOA_AOD_SUPPORT)
    connless_aoa_aod_func_init();
    #endif

    #if (CFG_RANGING_SUPPORT)
    ranging_func_init();
    #endif

    #if (CFG_MESH_SUPPORT)
    mesh_func_init();
    #endif

    #if (RF_TX_PA_SELECT)
    ble_rf_tx_mode_set((ble_rf_tx_mode_t)RF_TX_PA_SELECT);
    #endif
#endif	
}

void soc_init(void)
{
    ll_aon_wdt_unlock();
    ll_aon_wdt_disable();
    while(ll_aon_wdt_is_busy());
    ll_aon_wdt_lock();

    platform_init();
}

__WEAK void sdk_init(void){};

void ble_sdk_patch_env_init(void)
{
#if (EM_BUFF_ENABLE == 1)

    // register the msg handler for patch
    uint16_t msg_cnt = sizeof(msg_tab) / sizeof(msg_tab_item_t);
    reg_msg_patch_tab(msg_tab, msg_cnt);

    // register the llm hci cmd handler for patch
    uint16_t llm_hci_cmd_cnt = sizeof(llm_hci_cmd_tab) / sizeof(llm_hci_cmd_tab_item_t);
    reg_llm_hci_cmd_patch_tab(llm_hci_cmd_tab, llm_hci_cmd_cnt);

    // register the gapm hci evt handler for patch
    uint16_t gapm_hci_evt_cnt = sizeof(gapm_hci_evt_tab) / sizeof(gapm_hci_evt_tab_item_t);
    reg_gapm_hci_evt_patch_tab(gapm_hci_evt_tab, gapm_hci_evt_cnt);

    ble_common_env_init();

    #if CFG_MAX_CONNECTIONS
    ble_con_env_init();
    #endif

    #if DTM_TEST_ENABLE
    ble_test_evn_init();
    #endif

    #if CFG_MUL_LINK_WITH_SAME_DEV
    ble_mul_link_env_init();
    #endif

    #if CFG_CAR_KEY_SUPPORT
    ble_car_key_env_init();
    #endif

    #if (CFG_BT_BREDR)
    ble_bt_bredr_env_init();
    #endif

    #if (CFG_SNIFFER_SUPPORT)
    sniffer_func_init();
    #endif

#endif // EM_BUFF_ENABLE
}
