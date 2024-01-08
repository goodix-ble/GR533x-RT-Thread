/**
 ****************************************************************************************
 *
 * @file generic_user_property_server.c
 *
 * @brief Generic User Property Server API Implementation.
 *
 ****************************************************************************************
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
#include "generic_property_message.h"
#include "generic_user_property_server.h"
#include "common_utils.h"
#include "grx_sys.h"
#include "mesh_property.h"

/*
 * LOCAL FUNCTIONS DECLARATION
 ****************************************************************************************
 */


static void handle_user_get_cb(const mesh_model_msg_ind_t *p_rx_msg,  void *p_args);
static void handle_user_set_cb(const mesh_model_msg_ind_t *p_rx_msg,  void *p_args);

static void generic_user_property_server_rx_cb(mesh_model_msg_ind_t *p_model_msg, void *p_args);
static void generic_user_property_server_sent_cb(mesh_model_msg_sent_ind_t *p_sent, void *p_args);

/*
 * LOCAL VARIABLES
 ****************************************************************************************
 */
static const uint16_t generic_user_property_server_opcode_list[] =
{
    GENERIC_PROPERTY_OPCODE_USER_GET,
    GENERIC_PROPERTY_OPCODE_USER_SET,
    GENERIC_PROPERTY_OPCODE_USER_SET_UNACKNOWLEDGED,
    GENERIC_PROPERTIES_OPCODE_USER_GET,
};

static const mesh_opcode_handler_t m_opcode_handlers[] =
{
    {GENERIC_PROPERTY_OPCODE_USER_GET, handle_user_get_cb},
    {GENERIC_PROPERTY_OPCODE_USER_SET, handle_user_set_cb},
    {GENERIC_PROPERTY_OPCODE_USER_SET_UNACKNOWLEDGED, handle_user_set_cb},
    {GENERIC_PROPERTIES_OPCODE_USER_GET, handle_user_get_cb},
};


static const mesh_model_cb_t generic_user_property_server_msg_cb = {
    .cb_rx             = generic_user_property_server_rx_cb,
    .cb_sent           = generic_user_property_server_sent_cb,
    .cb_publish_period = NULL,
};


static mesh_model_register_info_t generic_user_property_server_register_info = 
{
    .model_id = MESH_MODEL_SIG(MODEL_ID_GENS_UPROP),
    .element_offset = 0,
    .publish = true,
    .p_opcodes = (uint16_t *)generic_user_property_server_opcode_list,
    .num_opcodes = sizeof(generic_user_property_server_opcode_list) / sizeof(uint16_t),
    .p_cb = &generic_user_property_server_msg_cb,
    .p_args = NULL,
};
/*
 * LOCAL FUNCTIONS
 ****************************************************************************************
 */

static uint32_t user_property_status_send(generic_user_property_server_t * p_server,
                                          const mesh_model_msg_ind_t *p_rx_msg,
                                          const generic_property_uam_status_params_t * p_params)
{
    APP_LOG_INFO("[%s] enter.", __func__);
    
    uint8_t tx_hdl = (NULL == p_rx_msg) ? GENERIC_PROPERTY_SERVER_PUBLISH_SEND_TX_HDL + p_server->model_instance_index * GENERIC_PROPERTY_SERVER_TX_HDL_TOTAL
                                        : GENERIC_PROPERTY_SERVER_RSP_SEND_TX_HDL + p_server->model_instance_index * GENERIC_PROPERTY_SERVER_TX_HDL_TOTAL;
    uint16_t status;
    uint8_t* p_buffer = NULL;
    uint16_t send_len = 0;
    
    if (p_params->value_length == 0 && p_params->property_value == NULL)
    {
        //Access not allow
        if (p_params->access != not_user_property)
        {
            /**to do:
            The page 83 of the Spec(v1.0.1) note that if the client has not enough access to get/set the value, 
            the Access should be reply to client,but Property Value should be omitted.
            this is conflict with the note(C.1) which on the page 61 of the Spec(v1.0.1).
            */
            if (REPLY_ACCESS_OMIT_VALUE)
            {
                send_len = GENERIC_PROPERTY_UAM_STATUS_MINLEN + 1;
                p_buffer = sys_malloc(sizeof(uint8_t) * send_len);
                gx_write16p(p_buffer, p_params->property_id);
                gx_write8(p_buffer + 2, p_params->access);
            }
            else
            {
                send_len = GENERIC_PROPERTY_UAM_STATUS_MINLEN;
                p_buffer = sys_malloc(sizeof(uint8_t) * send_len);
                gx_write16p(p_buffer, p_params->property_id);
            }
        }
        else /*not exist or it is not an user property.*/
        {
            send_len = GENERIC_PROPERTY_UAM_STATUS_MINLEN;
            p_buffer = sys_malloc(sizeof(uint8_t) * send_len);
            gx_write16p(p_buffer, p_params->property_id);
        }
        
    }
    else/*set/get successful*/
    {
        send_len = GENERIC_PROPERTY_UAM_STATUS_MINLEN + 1 + p_params->value_length;
        p_buffer = sys_malloc(sizeof(uint8_t) * send_len);
        gx_write16p(p_buffer, p_params->property_id);
        gx_write8(p_buffer + 2, p_params->access);
        memcpy(p_buffer + 3, p_params->property_value, p_params->value_length);
    }

    mesh_model_send_info_t msg_send =
    {
        .model_lid = p_server->model_lid,
        .opcode = MESH_ACCESS_OPCODE_SIG(GENERIC_PROPERTY_OPCODE_USER_STATUS),
        .tx_hdl = tx_hdl,
        .p_data_send = p_buffer,
        .data_send_len = send_len,
        .dst = (NULL == p_rx_msg) ? MESH_INVALID_ADDR : p_rx_msg->src,
        .appkey_index = (NULL == p_rx_msg) ? MESH_INVALID_KEY_INDEX : p_rx_msg->appkey_index,
    };

    if (NULL == p_rx_msg)
    {
        APP_LOG_INFO("SERVER[%d] -- to publish.", p_server->model_instance_index);
        status =  mesh_model_publish(&msg_send, NULL);
    }
    else
    {
        APP_LOG_INFO("SERVER[%d] -- to response.", p_server->model_instance_index);
        status =  mesh_model_rsp_send(&msg_send);
    }
    /*printf("[%s]debug:send property buffer 0x", __func__);
    for (uint8_t i = 0; i < send_len; i++)
    {
        printf("%02x",*(p_buffer+i));
    }
    printf("\n");*/
    sys_free(p_buffer);
    p_buffer = NULL;
    return status;
}

static uint32_t user_properties_status_send(generic_user_property_server_t * p_server,
                                          const mesh_model_msg_ind_t *p_rx_msg,
                                          const generic_properties_uamc_status_params_t * p_params)
{
    APP_LOG_INFO("[%s] enter.", __func__);
    
    uint8_t tx_hdl = (NULL == p_rx_msg) ? GENERIC_PROPERTY_SERVER_PUBLISH_SEND_TX_HDL + p_server->model_instance_index * GENERIC_PROPERTY_SERVER_TX_HDL_TOTAL
                                        : GENERIC_PROPERTY_SERVER_RSP_SEND_TX_HDL + p_server->model_instance_index * GENERIC_PROPERTY_SERVER_TX_HDL_TOTAL;
    uint16_t status;
    uint8_t* p_buffer = NULL;
    uint16_t send_len = 0;
    device_property_t *device_property = NULL;
    if (p_params->id_number != 0)
    {
        int send_number = 0;

        p_buffer = (uint8_t *)sys_malloc(p_params->id_number *  sizeof(uint16_t));
        for(int i = 0; i < p_params->id_number; i++)
        {
            device_property = property_search(p_server->model_instance_index, p_params->property_id[i]);
            if ((device_property != NULL ) && (device_property->access != not_user_property))
            {
                gx_write16p(p_buffer + 2*send_number, p_params->property_id[i]);
                send_number ++;
            }
        }
        send_len = send_number * sizeof(uint16_t);
    }
    
    mesh_model_send_info_t msg_send =
    {
        .model_lid = p_server->model_lid,
        .opcode = MESH_ACCESS_OPCODE_SIG(GENERIC_PROPERTIES_OPCODE_USER_STATUS),
        .tx_hdl = tx_hdl,
        .p_data_send = p_buffer,
        .data_send_len = send_len,
        .dst = (NULL == p_rx_msg) ? MESH_INVALID_ADDR : p_rx_msg->src,
        .appkey_index = (NULL == p_rx_msg) ? MESH_INVALID_KEY_INDEX : p_rx_msg->appkey_index,
    };

    if (NULL == p_rx_msg)
    {
        APP_LOG_INFO("SERVER[%d] -- to publish.", p_server->model_instance_index);
        status = mesh_model_publish(&msg_send, NULL);
    }
    else
    {
        APP_LOG_INFO("SERVER[%d] -- to response.", p_server->model_instance_index);
        status = mesh_model_rsp_send(&msg_send);
    }
    sys_free(p_buffer);
    p_buffer = NULL;
    return status;
}
 

 
static void handle_user_get_cb(const mesh_model_msg_ind_t *p_rx_msg,  void *p_args)
{
    generic_user_property_server_t * p_server = (generic_user_property_server_t *) p_args;

    APP_LOG_INFO("[%s] enter.", __func__);   
    APP_LOG_INFO("SERVER[%d] -- Receive message, want to get user property/properties status!!!", p_server->model_instance_index);
    if (p_rx_msg->opcode.company_opcode == GENERIC_PROPERTY_OPCODE_USER_GET)
    {
        generic_property_uam_get_params_t in_data = {0};
        generic_property_uam_status_params_t out_data = {0};
        if (p_rx_msg->msg_len == GENERIC_PROPERTY_UAM_GET_LEN)
        {
            in_data.property_id = gx_read16(p_rx_msg->msg);

            if (INVILID_PROPERTY_ID == in_data.property_id)
            {
                return ;
            }
            p_server->settings.p_callbacks->property_cbs.user_property_get_cb(p_server,p_rx_msg,&in_data,&out_data);
            uint32_t send_status = user_property_status_send(p_server, p_rx_msg, &out_data);
            if (MESH_ERROR_NO_ERROR != send_status)
            {
                APP_LOG_ERROR("SERVER[%d] -- Publish status faild, error code: 0x%04x", p_server->model_instance_index, send_status);
            }
        }
    }
    else
    {
        generic_properties_uamc_status_params_t out_data = {0};
        if (p_rx_msg->msg_len == 0)
        {
            p_server->settings.p_callbacks->property_cbs.user_properties_get_cb(p_server,p_rx_msg,&out_data);
            uint32_t send_status = user_properties_status_send(p_server, p_rx_msg, &out_data);
            if (MESH_ERROR_NO_ERROR != send_status)
            {
                APP_LOG_ERROR("SERVER[%d] -- Publish status faild, error code: 0x%04x", p_server->model_instance_index, send_status);
            }
        }
    }
}



static void handle_user_set_cb(const mesh_model_msg_ind_t *p_rx_msg,  void *p_args)
{
    generic_user_property_server_t  * p_server = (generic_user_property_server_t  *) p_args;

    APP_LOG_INFO("[%s] enter.", __func__);
    APP_LOG_INFO("SERVER[%d] -- Receive message, want to set user property state!!!", p_server->model_instance_index);
    
    generic_property_user_set_params_t in_data = {0};
    generic_property_uam_status_params_t out_data = {0};
    if (p_rx_msg->msg_len >= GENERIC_PROPERTY_USER_SET_MINLEN)
    {
        uint8_t* p_buffer = NULL;
        in_data.property_id = gx_read16p(p_rx_msg->msg);
        in_data.value_length = p_rx_msg->msg_len - GENERIC_PROPERTY_USER_SET_MINLEN;

        if (UNKNOW_PROPERTY_ID == in_data.property_id)
        {
            out_data.property_id = UNKNOW_PROPERTY_ID;
            out_data.value_length = 0;
            out_data.property_value = NULL;
        }
        else
        {
            if(is_property_msg_length_valid(in_data.property_id, in_data.value_length))
            {
                p_buffer = sys_malloc(sizeof(uint8_t) * in_data.value_length);
                memcpy(p_buffer, p_rx_msg->msg + 2, in_data.value_length);
                in_data.property_value = p_buffer;
            }
            else
            {
                APP_LOG_INFO("[%s] Receive message length is invalid.", __func__);
                return;
            }
            p_server->settings.p_callbacks->property_cbs.user_property_set_cb(p_server,
                                                                 p_rx_msg,
                                                                 &in_data,
                                                                 (p_rx_msg->opcode.company_opcode == GENERIC_PROPERTY_OPCODE_USER_SET) ? &out_data : NULL);
        }

        if (p_rx_msg->opcode.company_opcode == GENERIC_PROPERTY_OPCODE_USER_SET)
        {
            uint32_t send_status = user_property_status_send(p_server, p_rx_msg, &out_data);
            if (MESH_ERROR_NO_ERROR != send_status)
            {
                APP_LOG_ERROR("SERVER[%d] -- Publish status faild, error code: 0x%04x", p_server->model_instance_index, send_status);
            }
        }
        sys_free(p_buffer);
        p_buffer = NULL;
    }
}



static void generic_user_property_server_rx_cb(mesh_model_msg_ind_t *p_model_msg, void *p_args)
{  
    uint16_t company_opcode = p_model_msg->opcode.company_opcode;
    uint8_t index = 0;
    switch(company_opcode)
    {
        case GENERIC_PROPERTY_OPCODE_USER_GET:
            index = 0;
            break;
        case GENERIC_PROPERTY_OPCODE_USER_SET:
            index = 1;
            break;
        case GENERIC_PROPERTY_OPCODE_USER_SET_UNACKNOWLEDGED:
            index = 2;
            break;
        case GENERIC_PROPERTIES_OPCODE_USER_GET:
            index = 3;
            break;
    }

    mesh_opcode_handler_cb_t handler = m_opcode_handlers[index].handler;

    if (NULL != handler)
    {
        handler(p_model_msg, p_args);
    }
}

static void generic_user_property_server_sent_cb(mesh_model_msg_sent_ind_t *p_sent, void *p_args)
{
    generic_user_property_server_t * p_server = (generic_user_property_server_t *) p_args;
    
    switch(p_sent->tx_hdl - p_server->model_instance_index * GENERIC_PROPERTY_SERVER_TX_HDL_TOTAL)
    {
        case GENERIC_PROPERTY_SERVER_RSP_SEND_TX_HDL:
            if(MESH_ERROR_NO_ERROR == p_sent->status)
            {
                APP_LOG_INFO("SERVER[%d] -- Responsed message!!!", p_server->model_instance_index);
            }
            else
            {
                APP_LOG_INFO("SERVER[%d] -- Failed to responsed message, status = %x!!!", p_server->model_instance_index, p_sent->status);
            }
            break;
        case GENERIC_PROPERTY_SERVER_PUBLISH_SEND_TX_HDL:
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

uint16_t generic_user_property_server_init(generic_user_property_server_t * p_server, uint8_t element_offset)
{
    if (p_server == NULL ||
        p_server->settings.p_callbacks == NULL ||
        p_server->settings.p_callbacks->property_cbs.user_properties_get_cb == NULL ||
        p_server->settings.p_callbacks->property_cbs.user_property_get_cb == NULL ||
        p_server->settings.p_callbacks->property_cbs.user_property_set_cb == NULL ||
        GENERIC_PROPERTY_SERVER_INSTANCE_COUNT_MAX <= p_server->model_instance_index)
    {
        return MESH_ERROR_SDK_INVALID_PARAM;
    }

    generic_user_property_server_register_info.p_args = p_server;
    generic_user_property_server_register_info.element_offset = element_offset;
    
    return mesh_model_register(&generic_user_property_server_register_info, &p_server->model_lid);
}

uint16_t generic_user_property_server_status_publish(generic_user_property_server_t * p_server, const generic_property_uam_status_params_t * p_params)
{
    if (NULL == p_server || NULL == p_params)
    {
        return MESH_ERROR_SDK_INVALID_PARAM;
    }

    return user_property_status_send(p_server, NULL, p_params);
}

uint16_t generic_user_properties_server_status_publish(generic_user_property_server_t * p_server, const generic_properties_uamc_status_params_t * p_params)
{
    if (NULL == p_server || NULL == p_params)
    {
        return MESH_ERROR_SDK_INVALID_PARAM;
    }

    return user_properties_status_send(p_server, NULL, p_params);
}
