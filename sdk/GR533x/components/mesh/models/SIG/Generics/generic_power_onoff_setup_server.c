/**
 *****************************************************************************************
 *
 * @file generic_power_onoff_setup_server.c
 *
 * @brief Generic Power OnOff Setup Server API Implementation.
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
 ****************************************************************************************
 */
#include "app_log.h"
#include "generic_power_onoff_message.h"
#include "generic_power_onoff_setup_server.h"

/*
 * LOCAL FUNCTIONS DECLARATION
 ****************************************************************************************
 */

static void handle_set_cb(const mesh_model_msg_ind_t *p_rx_msg,  void *p_args);

static void generic_power_onoff_setup_server_rx_cb(mesh_model_msg_ind_t *p_model_msg, void *p_args);
static void generic_power_onoff_setup_server_sent_cb(mesh_model_msg_sent_ind_t *p_sent, void *p_args, void *p_buf);


/*
 * LOCAL VARIABLES
 ****************************************************************************************
 */

static const uint16_t generic_power_onoff_setup_server_opcode_list[] =
{
    GENERIC_POWER_ONOFF_OPCODE_SET,
    GENERIC_POWER_ONOFF_OPCODE_SET_UNACKNOWLEDGED,
};

static const mesh_opcode_handler_t m_opcode_handlers[] =
{
    {GENERIC_POWER_ONOFF_OPCODE_SET, handle_set_cb},
    {GENERIC_POWER_ONOFF_OPCODE_SET_UNACKNOWLEDGED, handle_set_cb},
};

static const mesh_model_cb_t generic_power_onoff_setup_server_msg_cb = {
    .cb_rx             = generic_power_onoff_setup_server_rx_cb,
    .cb_sent           = generic_power_onoff_setup_server_sent_cb,
    .cb_publish_period = NULL,
};

static mesh_model_register_info_t generic_power_onoff_server_setup_register_info = 
{
    .model_id = MESH_MODEL_SIG(MODEL_ID_GENS_POOS),
    .element_offset = 0,
    .publish = false,
    .p_opcodes = (uint16_t *)generic_power_onoff_setup_server_opcode_list,
    .num_opcodes = sizeof(generic_power_onoff_setup_server_opcode_list) / sizeof(uint16_t),
    .p_cb = &generic_power_onoff_setup_server_msg_cb,
    .p_args = NULL,
};

/*
 * LOCAL FUNCTIONS
 ****************************************************************************************
 */

static uint32_t status_send(generic_power_onoff_setup_server_t * p_server,
                            const mesh_model_msg_ind_t *p_rx_msg,
                            const generic_power_onoff_status_params_t * p_params)
{
    generic_power_onoff_status_msg_pkt_t msg_pkt;
    uint8_t tx_hdl = (NULL == p_rx_msg) ? GENERIC_POWER_ONOFF_SERVER_PUBLISH_SEND_TX_HDL + p_server->model_instance_index * GENERIC_POWER_ONOFF_SERVER_TX_HDL_TOTAL
                                        : GENERIC_POWER_ONOFF_SERVER_RSP_SEND_TX_HDL + p_server->model_instance_index * GENERIC_POWER_ONOFF_SERVER_TX_HDL_TOTAL;

    if (p_params->on_power_up > GENERIC_ON_POWER_UP_MAX)
    {
        return MESH_ERROR_SDK_INVALID_PARAM;
    }

    msg_pkt.on_power_up = p_params->on_power_up;    

    mesh_model_send_info_t msg_send =
    {
        .model_lid = p_server->power_server->model_lid,
        .opcode = MESH_ACCESS_OPCODE_SIG(GENERIC_POWER_ONOFF_OPCODE_STATUS),
        .tx_hdl = tx_hdl,
        .p_data_send = (uint8_t *) &msg_pkt,
        .data_send_len = GENERIC_POWER_ONOFF_MSG_LEN,
        .dst = (NULL == p_rx_msg) ? MESH_INVALID_ADDR : p_rx_msg->src,
        .appkey_index = (NULL == p_rx_msg) ? MESH_INVALID_KEY_INDEX : p_rx_msg->appkey_index,
    };

    APP_LOG_INFO("SERVER[%d] -- on-power-up = %d.", p_server->model_instance_index, msg_pkt.on_power_up);


    if (NULL == p_rx_msg)
    {
        APP_LOG_INFO("SERVER[%d] -- to publish. ", p_server->model_instance_index);
        return mesh_model_publish(&msg_send, NULL);
    }
    else
    {
        APP_LOG_INFO("SERVER[%d] -- to response.", p_server->model_instance_index);
        return mesh_model_rsp_send(&msg_send);
    }
}

static inline bool set_params_validate(const mesh_model_msg_ind_t * p_rx_msg, const generic_power_onoff_set_msg_pkt_t * p_params)
{
    return ((p_rx_msg->msg_len == GENERIC_POWER_ONOFF_MSG_LEN) && (p_params->on_power_up <= GENERIC_ON_POWER_UP_MAX));
}

static void handle_set_cb(const mesh_model_msg_ind_t *p_rx_msg,  void *p_args)
{
    generic_power_onoff_setup_server_t  * p_server = (generic_power_onoff_setup_server_t  *) p_args;

    APP_LOG_INFO("[%s] enter.", __func__);
    APP_LOG_INFO("SERVER[%d] -- Receive message, want to set on-power-up state!!!", p_server->model_instance_index);
    
    generic_power_onoff_set_params_t in_data = {0};
    generic_power_onoff_status_params_t out_data = {0};
    generic_power_onoff_set_msg_pkt_t * p_msg_params_packed = (generic_power_onoff_set_msg_pkt_t *) p_rx_msg->msg;

    if (set_params_validate(p_rx_msg, p_msg_params_packed))
    {
        in_data.on_power_up = p_msg_params_packed->on_power_up;

        p_server->settings.p_callbacks->power_onoff_cbs.set_cb(p_server,
                                                               p_rx_msg,
                                                               &in_data,
                                                               (p_rx_msg->opcode.company_opcode == GENERIC_POWER_ONOFF_OPCODE_SET) ? &out_data : NULL);

        if (p_rx_msg->opcode.company_opcode == GENERIC_POWER_ONOFF_OPCODE_SET)
        {
            uint32_t send_status = status_send(p_server, p_rx_msg, &out_data);
            if (MESH_ERROR_NO_ERROR != send_status)
            {
                APP_LOG_ERROR("SERVER[%d] -- Response status faild, error code: 0x%04x", p_server->model_instance_index, send_status);
            }
        }
    }
}


static void generic_power_onoff_setup_server_rx_cb(mesh_model_msg_ind_t *p_model_msg, void *p_args)
{  
    uint16_t company_opcode = p_model_msg->opcode.company_opcode;

    mesh_opcode_handler_cb_t handler = m_opcode_handlers[company_opcode - GENERIC_POWER_ONOFF_OPCODE_SET].handler;

    if (NULL != handler)
    {
        handler(p_model_msg, p_args);
    }
}

static void generic_power_onoff_setup_server_sent_cb(mesh_model_msg_sent_ind_t *p_sent, void *p_args, void *p_buf)
{
    generic_power_onoff_setup_server_t * p_server = (generic_power_onoff_setup_server_t *) p_args;
    
    switch(p_sent->tx_hdl - p_server->model_instance_index * GENERIC_ONOFF_SERVER_TX_HDL_TOTAL)
    {
        case GENERIC_POWER_ONOFF_SERVER_RSP_SEND_TX_HDL:
            if(MESH_ERROR_NO_ERROR == p_sent->status)
            {
                APP_LOG_INFO("SERVER[%d] -- Responsed get or set message!!!", p_server->model_instance_index);
            }
            else
            {
                APP_LOG_INFO("SERVER[%d] -- Failed to responsed get or set message, status = %x!!!", p_server->model_instance_index, p_sent->status);
            }
            break;
        case GENERIC_POWER_ONOFF_SERVER_PUBLISH_SEND_TX_HDL:
            if(MESH_ERROR_NO_ERROR == p_sent->status)
            {
                APP_LOG_INFO("SERVER[%d] -- Published message!!!", p_server->model_instance_index);
            }
            else
            {
                APP_LOG_INFO("SERVER[%d] -- Failed to published message, status = %x!!!", p_server->model_instance_index, p_sent->status);
            }
            break;
        default:
            APP_LOG_INFO("Never here!!!");
            break;
    }
}



/*
 * GLOBAL FUNCTIONS
 ****************************************************************************************
 */

uint16_t generic_power_onoff_setup_server_init(generic_power_onoff_setup_server_t *p_server, uint8_t element_offset)
{
    if (NULL == p_server
        || NULL == p_server->settings.p_callbacks->power_onoff_cbs.set_cb
        || GENERIC_ONOFF_SERVER_INSTANCE_COUNT_MAX <= p_server->model_instance_index)
    {
        return MESH_ERROR_SDK_INVALID_PARAM;
    }
    
        generic_power_onoff_server_setup_register_info.p_args = p_server;
        generic_power_onoff_server_setup_register_info.element_offset = element_offset;
        return mesh_model_register(&generic_power_onoff_server_setup_register_info, &p_server->model_lid);
    
}

uint16_t generic_power_onoff_setup_server_status_publish(generic_power_onoff_setup_server_t * p_server, 
                                                         const generic_power_onoff_status_params_t * p_params)
{
    if (NULL == p_server || NULL == p_params)
    {
        return MESH_ERROR_SDK_INVALID_PARAM;
    }

    return status_send(p_server, NULL, p_params);
}

