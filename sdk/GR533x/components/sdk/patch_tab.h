#ifndef __PATCH_TAB_H_
#define __PATCH_TAB_H_

#include <stdint.h>
#include <stdio.h>

#if EM_BUFF_ENABLE

typedef uint16_t ke_task_id_t;
typedef uint16_t ke_msg_id_t;

typedef int (*ke_msg_func_t)(ke_msg_id_t const msgid, void const *param,
    ke_task_id_t const dest_id, ke_task_id_t const src_id);

typedef int (*llm_hci_cmd_hdl_func_t)(void const *param, uint16_t opcode);

typedef int (*gapm_hci_evt_hdl_func_t)(uint16_t opcode, void const *param);

typedef struct
{
    ke_msg_func_t ori_func_addr;
    ke_msg_func_t new_func_addr;
} msg_tab_item_t;

typedef struct
{
    llm_hci_cmd_hdl_func_t ori_func_addr;
    llm_hci_cmd_hdl_func_t new_func_addr;
} llm_hci_cmd_tab_item_t;

typedef struct
{
    gapm_hci_evt_hdl_func_t ori_func_addr;
    gapm_hci_evt_hdl_func_t new_func_addr;
} gapm_hci_evt_tab_item_t;

extern void ble_common_env_init(void);
extern void ble_mul_link_env_init(void);
extern void ble_con_env_init(void);
extern void ble_car_key_env_init(void);
extern void ble_bt_bredr_env_init(void);
extern void ble_test_evn_init(void);

extern int hci_command_llm_handler_patch(ke_msg_id_t const msgid, void const *param,
    ke_task_id_t const dest_id,ke_task_id_t const src_id);

// gapm cfg
extern int gapm_set_dev_config_cmd_handler_patch(ke_msg_id_t const msgid, void *param,
    ke_task_id_t const dest_id, ke_task_id_t const src_id);

extern int gapm_hci_handler_patch(ke_msg_id_t const msgid, void const* event, ke_task_id_t dest_id, ke_task_id_t opcode);

#if CFG_MUL_LINK_WITH_SAME_DEV
// gapm hci event for multiple link
extern int hci_le_adv_set_term_evt_handler_patch(uint16_t opcode, void const *p_event);

// gapc task for multiple link
extern int gapc_bond_cfm_handler_patch(ke_msg_id_t const msgid, void *cfm,
    ke_task_id_t const dest_id, ke_task_id_t const src_id);

extern int lld_adv_end_ind_handler_patch(ke_msg_id_t const msgid, void const *param,
    ke_task_id_t const dest_id, ke_task_id_t const src_id);
#endif

#if CFG_CAR_KEY_SUPPORT
extern int llm_pub_key_gen_ind_handler_patch(ke_msg_id_t const msgid, void const *param,
    ke_task_id_t const dest_id, ke_task_id_t const src_id);
#endif

msg_tab_item_t msg_tab[] =
{
    {(ke_msg_func_t)0x000105a1, (ke_msg_func_t)gapm_set_dev_config_cmd_handler_patch},
    {(ke_msg_func_t)0x00011f3d, (ke_msg_func_t)hci_command_llm_handler_patch},
    {(ke_msg_func_t)0x0000f22d, (ke_msg_func_t)gapm_hci_handler_patch},

    #if CFG_MUL_LINK_WITH_SAME_DEV
    {(ke_msg_func_t)0x0000a1e9, (ke_msg_func_t)gapc_bond_cfm_handler_patch},
    {(ke_msg_func_t)0x0001f829, (ke_msg_func_t)lld_adv_end_ind_handler_patch},
    #endif

    #if CFG_CAR_KEY_SUPPORT
    {(ke_msg_func_t)0x0002ddb5, (ke_msg_func_t)llm_pub_key_gen_ind_handler_patch},
    #endif
};

extern int hci_le_add_dev_to_rslv_list_cmd_handler_patch(void const *param, uint16_t opcode);

extern int hci_le_rmv_dev_from_rslv_list_cmd_handler_patch(void const *param, uint16_t opcode);

extern int hci_le_clear_rslv_list_cmd_handler_patch(void const *param, uint16_t opcode);

extern int hci_le_set_addr_resol_en_cmd_handler_patch(void const *param, uint16_t opcode);

extern int hci_le_set_priv_mode_cmd_handler_patch(void const *param, uint16_t opcode);

#if CFG_CAR_KEY_SUPPORT
extern int hci_le_rd_local_p256_public_key_cmd_handler_patch(void const *param, uint16_t opcode);
#endif

llm_hci_cmd_tab_item_t llm_hci_cmd_tab[] =
{
     // hci cmd for common
    {(llm_hci_cmd_hdl_func_t)0x000126fd, (llm_hci_cmd_hdl_func_t)hci_le_add_dev_to_rslv_list_cmd_handler_patch},
    {(llm_hci_cmd_hdl_func_t)0x00014ced, (llm_hci_cmd_hdl_func_t)hci_le_rmv_dev_from_rslv_list_cmd_handler_patch},
    {(llm_hci_cmd_hdl_func_t)0x00012b1d, (llm_hci_cmd_hdl_func_t)hci_le_clear_rslv_list_cmd_handler_patch},
    {(llm_hci_cmd_hdl_func_t)0x00014e21, (llm_hci_cmd_hdl_func_t)hci_le_set_addr_resol_en_cmd_handler_patch},
    {(llm_hci_cmd_hdl_func_t)0x00016355, (llm_hci_cmd_hdl_func_t)hci_le_set_priv_mode_cmd_handler_patch},

    #if CFG_CAR_KEY_SUPPORT
    {(llm_hci_cmd_hdl_func_t)0x00014641, (llm_hci_cmd_hdl_func_t)hci_le_rd_local_p256_public_key_cmd_handler_patch},
    #endif
};

gapm_hci_evt_tab_item_t gapm_hci_evt_tab[] =
{
    {NULL, NULL},

    #if CFG_MUL_LINK_WITH_SAME_DEV
    {(gapm_hci_evt_hdl_func_t)0x00012991, (gapm_hci_evt_hdl_func_t)hci_le_adv_set_term_evt_handler_patch},
    #endif
};

extern void reg_msg_patch_tab(msg_tab_item_t *msg_tab, uint16_t msg_cnt);
extern void reg_gapm_hci_evt_patch_tab(gapm_hci_evt_tab_item_t *gapm_hci_evt_tab, uint16_t gapm_hci_evt_cnt);
extern void reg_llm_hci_cmd_patch_tab(llm_hci_cmd_tab_item_t *hci_cmd_tab, uint16_t hci_cmd_cnt);
#endif  // EM_BUFF_ENABLE
#endif  // __PATCH_TAB_H_
