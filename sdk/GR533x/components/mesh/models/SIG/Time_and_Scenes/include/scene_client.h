/**
 *****************************************************************************************
 *
 * @file scene_client.h
 *
 * @brief Mesh Scene Client API.
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

 /**
 * @addtogroup MESH
 * @{
 */
#ifndef __TSCNS_SCENE_CLIENT_H__
#define __TSCNS_SCENE_CLIENT_H__


/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include <stdint.h>
#include "mesh_common.h"
#include "mesh_model.h"
#include "model_common.h"
#include "scene_common.h"

/*
 * DEFINES
 ****************************************************************************************
 */
//#define MESH_MODEL_BQB_TEST
/*
 * ENUMERATIONS
 ****************************************************************************************
 */


/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/* Forward declaration */
typedef struct __mesh_scene_client_t  mesh_scene_client_t;

/**
 * Callback type for Scene state related transactions
 *
 * @param[in]     p_self                   Pointer to the mesh time client model structure
 * @param[in]     p_rx_msg             Pointer to the the received message info.
 * @param[in]     p_in                     Pointer to the Status message info.
 */
typedef void (*mesh_scene_state_status_cb_t)(const mesh_scene_client_t * p_self,
                                                    const mesh_model_msg_ind_t * p_rx_msg,
                                                    mesh_scene_status_params_t * p_in);

/**
 * Callback type for Scene Register state related transactions
 *
 * @param[in]     p_self                   Pointer to the mesh time client model structure
 * @param[in]     p_rx_msg             Pointer to the the received message info.
 * @param[in]     p_in                     Pointer to the Status message info.
 */
typedef void (*mesh_scene_register_status_cb_t)(const mesh_scene_client_t * p_self,
                                                    const mesh_model_msg_ind_t * p_rx_msg,
                                                    mesh_scene_recv_register_status_params_t * p_in);

/**
 * Mesh Scene Client callback list.
 */
typedef struct
{
    mesh_scene_state_status_cb_t scene_status_cb;                             /**< Callback for Mesh Scene Status message. */
    mesh_scene_register_status_cb_t scene_register_status_cb;          /**< Callback for Mesh Scene Register Status message. */
    mesh_model_reliable_trans_cb_t ack_transaction_status_cb;          /**< Callback for reliable message timeout. */
} mesh_scene_client_callbacks_t;

/**
 * User provided settings and callbacks for this model instance.
 */
typedef struct
{
    uint32_t timeout_ms;                                    /**< Reliable message timeout time. */
    const mesh_scene_client_callbacks_t *p_callbacks;    /**< User provided callback for this model. */
} mesh_scene_client_settings_t;

/**
 * Mesh Scene Client model information.
 */
struct __mesh_scene_client_t
{
    mesh_lid_t model_lid;                           /**< Model local identifier. */
    mesh_scene_client_settings_t settings;   /**< Settings for this model instance. */
    uint8_t model_instance_index;            /**< Model instance index. */
};

/**
 * Initializes Mesh Scene Client model.
 *
 * @param[in]     p_client                     Mesh Scene client information pointer.
 * @param[in]     element_offset           Element address offset from primary element address.
 *
 * @retval ::MESH_ERROR_NO_ERROR                Operation is Success.
 * @retval ::MESH_ERROR_COMMAND_DISALLOWED      Command Disallowed.
 * @retval ::MESH_ERROR_SDK_INVALID_PARAM       Invalid parameter.
 */
uint16_t mesh_scene_client_init(mesh_scene_client_t *p_client, uint8_t element_offset);

/**
 * Sends a Store message to the server.
 *
 * @param[in]     p_client                 Client model information pointer.
 * @param[in]     p_params              Message parameters.
 *
 * @retval ::MESH_ERROR_NO_ERROR                  Operation is Success.
 * @retval ::MESH_ERROR_INVALID_PARAM             Invalid Parameter.
 * @retval ::MESH_ERROR_INSUFFICIENT_RESOURCES    Insufficient Resources.
 * @retval ::MESH_ERROR_INVALID_MODEL             Invalid Model.
 * @retval ::MESH_ERROR_INVALID_PUBLISH_PARAMS    Invalid Publish Parameters.
 * @retval ::MESH_ERROR_INVALID_BINDING           Invalid Binding.
 * @retval ::MESH_ERROR_INVALID_APPKEY_ID         Invalid AppKey Index.
 * @retval ::MESH_ERROR_COMMAND_DISALLOWED        Command Disallowed.
 * @retval ::MESH_ERROR_NOT_FOUND                 Requested resource not found.
 * @retval ::MESH_ERROR_SDK_RELIABLE_TRANS_ON     Reliable message transfer procedure is on.
 * @retval ::MESH_ERROR_SDK_INVALID_PARAM         Invalid Parameter.
 */
uint16_t mesh_scene_client_store(mesh_scene_client_t * p_client, const mesh_scene_store_params_t * p_params);

/**
 * Sends a Store Unacknowledged message to the server.
 *
 * @param[in]     p_client                 Client model information pointer.
 * @param[in]     p_params                 Message parameters.
 *
 * @retval ::MESH_ERROR_NO_ERROR                  Operation is Success.
 * @retval ::MESH_ERROR_INVALID_PARAM             Invalid Parameter.
 * @retval ::MESH_ERROR_INSUFFICIENT_RESOURCES    Insufficient Resources.
 * @retval ::MESH_ERROR_INVALID_MODEL             Invalid Model.
 * @retval ::MESH_ERROR_INVALID_PUBLISH_PARAMS    Invalid Publish Parameters.
 * @retval ::MESH_ERROR_INVALID_BINDING           Invalid Binding.
 * @retval ::MESH_ERROR_INVALID_APPKEY_ID         Invalid AppKey Index.
 * @retval ::MESH_ERROR_COMMAND_DISALLOWED        Command Disallowed.
 * @retval ::MESH_ERROR_NOT_FOUND                 Requested resource not found.
 * @retval ::MESH_ERROR_SDK_RELIABLE_TRANS_ON     Reliable message transfer procedure is on.
 * @retval ::MESH_ERROR_SDK_INVALID_PARAM         Invalid Parameter.
 */
uint16_t mesh_scene_client_store_unack(mesh_scene_client_t * p_client, const mesh_scene_store_params_t * p_params);

/**
 * Sends a Delete message to the server.
 *
 * @param[in]     p_client                 Client model information pointer.
 * @param[in]     p_params                 Message parameters.
 *
 * @retval ::MESH_ERROR_NO_ERROR                  Operation is Success.
 * @retval ::MESH_ERROR_INVALID_PARAM             Invalid Parameter.
 * @retval ::MESH_ERROR_INSUFFICIENT_RESOURCES    Insufficient Resources.
 * @retval ::MESH_ERROR_INVALID_MODEL             Invalid Model.
 * @retval ::MESH_ERROR_INVALID_PUBLISH_PARAMS    Invalid Publish Parameters.
 * @retval ::MESH_ERROR_INVALID_BINDING           Invalid Binding.
 * @retval ::MESH_ERROR_INVALID_APPKEY_ID         Invalid AppKey Index.
 * @retval ::MESH_ERROR_COMMAND_DISALLOWED        Command Disallowed.
 * @retval ::MESH_ERROR_NOT_FOUND                 Requested resource not found.
 * @retval ::MESH_ERROR_SDK_RELIABLE_TRANS_ON     Reliable message transfer procedure is on.
 * @retval ::MESH_ERROR_SDK_INVALID_PARAM         Invalid Parameter.
 */
uint16_t mesh_scene_client_delete(mesh_scene_client_t * p_client, const mesh_scene_delete_params_t * p_params);

/**
 * Sends a Delete Unacknowledged message to the server.
 *
 * @param[in]     p_client                 Client model information pointer.
 * @param[in]     p_params                 Message parameters.
 *
 * @retval ::MESH_ERROR_NO_ERROR                  Operation is Success.
 * @retval ::MESH_ERROR_INVALID_PARAM             Invalid Parameter.
 * @retval ::MESH_ERROR_INSUFFICIENT_RESOURCES    Insufficient Resources.
 * @retval ::MESH_ERROR_INVALID_MODEL             Invalid Model.
 * @retval ::MESH_ERROR_INVALID_PUBLISH_PARAMS    Invalid Publish Parameters.
 * @retval ::MESH_ERROR_INVALID_BINDING           Invalid Binding.
 * @retval ::MESH_ERROR_INVALID_APPKEY_ID         Invalid AppKey Index.
 * @retval ::MESH_ERROR_COMMAND_DISALLOWED        Command Disallowed.
 * @retval ::MESH_ERROR_NOT_FOUND                 Requested resource not found.
 * @retval ::MESH_ERROR_SDK_RELIABLE_TRANS_ON     Reliable message transfer procedure is on.
 * @retval ::MESH_ERROR_SDK_INVALID_PARAM         Invalid Parameter.
 */
uint16_t mesh_scene_client_delete_unack(mesh_scene_client_t * p_client, const mesh_scene_delete_params_t * p_params);

/**
 * Sends a Recall message to the server.
 *
 * @param[in]     p_client                 Client model information pointer.
 * @param[in]     p_params                 Message parameters.
 * @param[in]     p_transition             Optional transition parameters
 *
 * @retval ::MESH_ERROR_NO_ERROR                  Operation is Success.
 * @retval ::MESH_ERROR_INVALID_PARAM             Invalid Parameter.
 * @retval ::MESH_ERROR_INSUFFICIENT_RESOURCES    Insufficient Resources.
 * @retval ::MESH_ERROR_INVALID_MODEL             Invalid Model.
 * @retval ::MESH_ERROR_INVALID_PUBLISH_PARAMS    Invalid Publish Parameters.
 * @retval ::MESH_ERROR_INVALID_BINDING           Invalid Binding.
 * @retval ::MESH_ERROR_INVALID_APPKEY_ID         Invalid AppKey Index.
 * @retval ::MESH_ERROR_COMMAND_DISALLOWED        Command Disallowed.
 * @retval ::MESH_ERROR_NOT_FOUND                 Requested resource not found.
 * @retval ::MESH_ERROR_SDK_RELIABLE_TRANS_ON     Reliable message transfer procedure is on.
 * @retval ::MESH_ERROR_SDK_INVALID_PARAM         Invalid Parameter.
 */
uint16_t mesh_scene_client_recall(mesh_scene_client_t * p_client, const mesh_scene_recall_params_t * p_params,
                                                const model_transition_t * p_transition);

/**
 * Sends a Recall Unacknowledged message to the server.
 *
 * @param[in]     p_client                 Client model information pointer.
 * @param[in]     p_params                 Message parameters.
 * @param[in]     p_transition             Optional transition parameters
 *
 * @retval ::MESH_ERROR_NO_ERROR                  Operation is Success.
 * @retval ::MESH_ERROR_INVALID_PARAM             Invalid Parameter.
 * @retval ::MESH_ERROR_INSUFFICIENT_RESOURCES    Insufficient Resources.
 * @retval ::MESH_ERROR_INVALID_MODEL             Invalid Model.
 * @retval ::MESH_ERROR_INVALID_PUBLISH_PARAMS    Invalid Publish Parameters.
 * @retval ::MESH_ERROR_INVALID_BINDING           Invalid Binding.
 * @retval ::MESH_ERROR_INVALID_APPKEY_ID         Invalid AppKey Index.
 * @retval ::MESH_ERROR_COMMAND_DISALLOWED        Command Disallowed.
 * @retval ::MESH_ERROR_NOT_FOUND                 Requested resource not found.
 * @retval ::MESH_ERROR_SDK_RELIABLE_TRANS_ON     Reliable message transfer procedure is on.
 * @retval ::MESH_ERROR_SDK_INVALID_PARAM         Invalid Parameter.
 */
uint16_t mesh_scene_client_recall_unack(mesh_scene_client_t * p_client, const mesh_scene_recall_params_t * p_params,
                                                const model_transition_t * p_transition);

/**
 * Sends a Get message to the server.
 *
 * @param[in]     p_client                 Client model information pointer.
 *
 * @retval ::MESH_ERROR_NO_ERROR                  Operation is Success.
 * @retval ::MESH_ERROR_INVALID_PARAM             Invalid Parameter.
 * @retval ::MESH_ERROR_INSUFFICIENT_RESOURCES    Insufficient Resources.
 * @retval ::MESH_ERROR_INVALID_MODEL             Invalid Model.
 * @retval ::MESH_ERROR_INVALID_PUBLISH_PARAMS    Invalid Publish Parameters.
 * @retval ::MESH_ERROR_INVALID_BINDING           Invalid Binding.
 * @retval ::MESH_ERROR_INVALID_APPKEY_ID         Invalid AppKey Index.
 * @retval ::MESH_ERROR_COMMAND_DISALLOWED        Command Disallowed.
 * @retval ::MESH_ERROR_NOT_FOUND                 Requested resource not found.
 * @retval ::MESH_ERROR_SDK_RELIABLE_TRANS_ON     Reliable message transfer procedure is on.
 * @retval ::MESH_ERROR_SDK_INVALID_PARAM         Invalid Parameter.
 */
uint16_t mesh_scene_client_get(mesh_scene_client_t * p_client);

/**
 * Sends a Register Get message to the server.
 *
 * @param[in]     p_client                 Client model information pointer.
 *
 * @retval ::MESH_ERROR_NO_ERROR                  Operation is Success.
 * @retval ::MESH_ERROR_INVALID_PARAM             Invalid Parameter.
 * @retval ::MESH_ERROR_INSUFFICIENT_RESOURCES    Insufficient Resources.
 * @retval ::MESH_ERROR_INVALID_MODEL             Invalid Model.
 * @retval ::MESH_ERROR_INVALID_PUBLISH_PARAMS    Invalid Publish Parameters.
 * @retval ::MESH_ERROR_INVALID_BINDING           Invalid Binding.
 * @retval ::MESH_ERROR_INVALID_APPKEY_ID         Invalid AppKey Index.
 * @retval ::MESH_ERROR_COMMAND_DISALLOWED        Command Disallowed.
 * @retval ::MESH_ERROR_NOT_FOUND                 Requested resource not found.
 * @retval ::MESH_ERROR_SDK_RELIABLE_TRANS_ON     Reliable message transfer procedure is on.
 * @retval ::MESH_ERROR_SDK_INVALID_PARAM         Invalid Parameter.
 */
uint16_t mesh_scene_client_register_get(mesh_scene_client_t * p_client);

/**
 * Cancel the sending Set/Get message to the server.
 *
 * @param[in]     p_client                 Client model information pointer.
 *
 * @retval ::MESH_ERROR_NO_ERROR                  Operation is Success.
 * @retval ::MESH_ERROR_SDK_RELIABLE_TRANS_OFF    Reliable message transfer procedure is off.
 * @retval ::MESH_ERROR_SDK_INVALID_PARAM         Invalid Parameter.
 */
uint16_t mesh_scene_client_setget_cancel(mesh_scene_client_t * p_client);

#endif /* __TSCNS_SCENE_CLIENT_H__ */

/** @} */

