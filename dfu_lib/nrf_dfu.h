/**
 * Copyright (c) 2016 - 2019, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/**@file
 *
 * @defgroup sdk_nrf_dfu_req_handler Request handling
 * @{
 * @ingroup  nrf_dfu
 */

#ifndef NRF_DFU_REQ_HANDLER_H__
#define NRF_DFU_REQ_HANDLER_H__

#ifdef __cplusplus
extern "C"
{
#endif

#define DFU_DEBUG
//#define DFU_DEBUG_DUMP

/**
 * @brief DFU object types.
 */
typedef enum
{
    NRF_DFU_OBJ_TYPE_INVALID = 0x00,                   //!< Invalid object type.
    NRF_DFU_OBJ_TYPE_COMMAND,                   //!< Command object.
    NRF_DFU_OBJ_TYPE_DATA,                      //!< Data object.
} nrf_dfu_obj_type_t;

/**
 * @brief DFU protocol operation.
 */
typedef enum
{
    NRF_DFU_OP_PROTOCOL_VERSION     = 0x00,     //!< Retrieve protocol version. - not used
    NRF_DFU_OP_OBJECT_CREATE        = 0x01,     //!< Create selected object.
    NRF_DFU_OP_RECEIPT_NOTIF_SET    = 0x02,     //!< Set receipt notification.
    NRF_DFU_OP_CRC_GET              = 0x03,     //!< Request CRC of selected object.
    NRF_DFU_OP_OBJECT_EXECUTE       = 0x04,     //!< Execute selected object.
    NRF_DFU_OP_OBJECT_SELECT        = 0x06,     //!< Select object.
    NRF_DFU_OP_MTU_GET              = 0x07,     //!< Retrieve MTU size.
    NRF_DFU_OP_OBJECT_WRITE         = 0x08,     //!< Write selected object.  - not used
    NRF_DFU_OP_PING                 = 0x09,     //!< Ping.  - not used
    NRF_DFU_OP_HARDWARE_VERSION     = 0x0A,     //!< Retrieve hardware version. - not used
    NRF_DFU_OP_FIRMWARE_VERSION     = 0x0B,     //!< Retrieve firmware version. - not used
    NRF_DFU_OP_ABORT                = 0x0C,     //!< Abort the DFU procedure.
    NRF_DFU_OP_RESPONSE             = 0x60,     //!< Response.
    NRF_DFU_OP_INVALID              = 0xFF,
} nrf_dfu_op_t;



typedef enum
{
    NRF_DFU_EXT_ERROR_NO_ERROR                  = 0x00, /**< No extended error code has been set. This error indicates an implementation problem. */
    NRF_DFU_EXT_ERROR_INVALID_ERROR_CODE        = 0x01, /**< Invalid error code. This error code should never be used outside of development. */
    NRF_DFU_EXT_ERROR_WRONG_COMMAND_FORMAT      = 0x02, /**< The format of the command was incorrect. This error code is not used in the
                                                             current implementation, because @ref NRF_DFU_RES_CODE_OP_CODE_NOT_SUPPORTED
                                                             and @ref NRF_DFU_RES_CODE_INVALID_PARAMETER cover all
                                                             possible format errors. */
    NRF_DFU_EXT_ERROR_UNKNOWN_COMMAND           = 0x03, /**< The command was successfully parsed, but it is not supported or unknown. */
    NRF_DFU_EXT_ERROR_INIT_COMMAND_INVALID      = 0x04, /**< The init command is invalid. The init packet either has
                                                             an invalid update type or it is missing required fields for the update type
                                                             (for example, the init packet for a SoftDevice update is missing the SoftDevice size field). */
    NRF_DFU_EXT_ERROR_FW_VERSION_FAILURE        = 0x05, /**< The firmware version is too low. For an application or SoftDevice, the version must be greater than
                                                             or equal to the current version. For a bootloader, it must be greater than the current version.
                                                             to the current version. This requirement prevents downgrade attacks.*/
    NRF_DFU_EXT_ERROR_HW_VERSION_FAILURE        = 0x06, /**< The hardware version of the device does not match the required
                                                             hardware version for the update. */
    NRF_DFU_EXT_ERROR_SD_VERSION_FAILURE        = 0x07, /**< The array of supported SoftDevices for the update does not contain
                                                             the FWID of the current SoftDevice or the first FWID is '0' on a
                                                             bootloader which requires the SoftDevice to be present. */
    NRF_DFU_EXT_ERROR_SIGNATURE_MISSING         = 0x08, /**< The init packet does not contain a signature. This error code is not used in the
                                                             current implementation, because init packets without a signature
                                                             are regarded as invalid. */
    NRF_DFU_EXT_ERROR_WRONG_HASH_TYPE           = 0x09, /**< The hash type that is specified by the init packet is not supported by the DFU bootloader. */
    NRF_DFU_EXT_ERROR_HASH_FAILED               = 0x0A, /**< The hash of the firmware image cannot be calculated. */
    NRF_DFU_EXT_ERROR_WRONG_SIGNATURE_TYPE      = 0x0B, /**< The type of the signature is unknown or not supported by the DFU bootloader. */
    NRF_DFU_EXT_ERROR_VERIFICATION_FAILED       = 0x0C, /**< The hash of the received firmware image does not match the hash in the init packet. */
    NRF_DFU_EXT_ERROR_INSUFFICIENT_SPACE        = 0x0D, /**< The available space on the device is insufficient to hold the firmware. */
} nrf_dfu_ext_error_code_t;


/**
 * @brief DFU operation result code.
 */
typedef enum
{
    NRF_DFU_RES_CODE_INVALID                 = 0x00,    //!< Invalid opcode.
    NRF_DFU_RES_CODE_SUCCESS                 = 0x01,    //!< Operation successful.
    NRF_DFU_RES_CODE_OP_CODE_NOT_SUPPORTED   = 0x02,    //!< Opcode not supported.
    NRF_DFU_RES_CODE_INVALID_PARAMETER       = 0x03,    //!< Missing or invalid parameter value.
    NRF_DFU_RES_CODE_INSUFFICIENT_RESOURCES  = 0x04,    //!< Not enough memory for the data object.
    NRF_DFU_RES_CODE_INVALID_OBJECT          = 0x05,    //!< Data object does not match the firmware and hardware requirements, the signature is wrong, or parsing the command failed.
    NRF_DFU_RES_CODE_UNSUPPORTED_TYPE        = 0x07,    //!< Not a valid object type for a Create request.
    NRF_DFU_RES_CODE_OPERATION_NOT_PERMITTED = 0x08,    //!< The state of the DFU process does not allow this operation.
    NRF_DFU_RES_CODE_OPERATION_FAILED        = 0x0A,    //!< Operation failed.
    NRF_DFU_RES_CODE_EXT_ERROR               = 0x0B,    //!< Extended error. The next byte of the response contains the error code of the extended error (see @ref nrf_dfu_ext_error_code_t.
    NRF_DFU_RES_CODE_MAX                      
} nrf_dfu_result_t;

typedef enum
{
    NRF_DFU_FIRMWARE_TYPE_SOFTDEVICE    = 0x00,
    NRF_DFU_FIRMWARE_TYPE_APPLICATION   = 0x01,
    NRF_DFU_FIRMWARE_TYPE_BOOTLOADER    = 0x02,
    NRF_DFU_FIRMWARE_TYPE_UNKNOWN       = 0xFF,
} nrf_dfu_firmware_type_t;

typedef enum{
    NRF_DFU_CHAR_TYPE_CTRL_POINT             = 0x00,
    NRF_DFU_CHAR_TYPE_PACKET                 = 0x01,
} nrf_dfu_char_type_t;

/**
 * @brief @ref NRF_DFU_OP_PROTOCOL_VERSION response details.
 */
typedef struct
{
    uint8_t version;                    //!< Protocol version.
} nrf_dfu_response_protocol_t;

/**
 * @brief @ref NRF_DFU_OP_HARDWARE_VERSION response details.
 */
typedef struct
{
    uint32_t part;                      //!< Hardware part, from FICR register.
    uint32_t variant;                   //!< Hardware variant, from FICR register.
    struct
    {
        uint32_t rom_size;              //!< ROM size, in bytes.
        uint32_t ram_size;              //!< RAM size, in bytes.
        uint32_t rom_page_size;         //!< ROM flash page size, in bytes.
    } memory;
} nrf_dfu_response_hardware_t;

/**
 * @brief @ref NRF_DFU_OP_FIRMWARE_VERSION response details.
 */
typedef struct
{
    nrf_dfu_firmware_type_t type;       //!< Firmware type.
    uint32_t                version;    //!< Firmware version.
    uint32_t                addr;       //!< Firmware address in flash.
    uint32_t                len;        //!< Firmware length in bytes.
} nrf_dfu_response_firmware_t;

/**
 * @brief @ref NRF_DFU_OP_OBJECT_SELECT response details.
 */
typedef struct
{
    uint32_t offset;                    //!< Current offset.
    uint32_t crc;                       //!< Current CRC.
    uint32_t max_size;                  //!< Maximum size of selected object.
} nrf_dfu_response_select_t;

/**
 * @brief @ref NRF_DFU_OP_OBJECT_CREATE response details.
 */
typedef struct
{
    uint32_t offset;                    //!< Current offset
    uint32_t crc;                       //!< Current CRC.
} nrf_dfu_response_create_t;

/**
 * @brief @ref NRF_DFU_OP_OBJECT_WRITE response details.
 */
typedef struct
{
    uint32_t offset;                    //!< Used only when packet receipt notification is used.
    uint32_t crc;                       //!< Used only when packet receipt notification is used.
} nrf_dfu_response_write_t;

/**
 * @brief @ref NRF_DFU_OP_CRC_GET response details.
 */
typedef struct
{
    uint32_t offset;                    //!< Current offset.
    uint32_t crc;                       //!< Current CRC.
} nrf_dfu_response_crc_t;

/**
 * @brief @ref NRF_DFU_OP_PING response details.
 */
typedef struct
{
    uint8_t id;                         //!< The received ID which is echoed back.
} nrf_dfu_response_ping_t;

/**
 * @brief @ref NRF_DFU_OP_MTU_GET response details.
 */
typedef struct
{
    uint16_t size;                      //!< The MTU size as specified by the local transport.
} nrf_dfu_response_mtu_t;

/**
 * @brief DFU response message.
 */
typedef struct
{
    nrf_dfu_op_t     request;                      //!< Requested operation.
    nrf_dfu_result_t result;                       //!< Result of the operation.
    union
    {
        nrf_dfu_response_protocol_t protocol;      //!< Protocol version response.
        nrf_dfu_response_hardware_t hardware;      //!< Hardware version response.
        nrf_dfu_response_firmware_t firmware;      //!< Firmware version response.
        nrf_dfu_response_select_t   select;        //!< Select object response..
        nrf_dfu_response_create_t   create;        //!< Create object response..
        nrf_dfu_response_write_t    write;         //!< Write object response.
        nrf_dfu_response_crc_t      crc;           //!< CRC response.
        nrf_dfu_response_ping_t     ping;          //!< Ping response.
        nrf_dfu_response_mtu_t      mtu;           //!< MTU response.
    };
} nrf_dfu_response_t;

/**
 * @brief @ref NRF_DFU_OP_FIRMWARE_VERSION request details.
 */
typedef struct
{
    uint8_t image_number;  //!< Index of the firmware.
} nrf_dfu_request_firmware_t;

/**
 * @brief @ref NRF_DFU_OP_OBJECT_SELECT request details.
 */
typedef struct
{
    uint32_t object_type;  //!< Object type. See @ref nrf_dfu_obj_type_t.
} nrf_dfu_request_select_t;

/**
 * @brief @ref NRF_DFU_OP_OBJECT_CREATE request details.
 */
typedef struct
{
    uint32_t object_type;  //!< Object type. See @ref nrf_dfu_obj_type_t.
    uint32_t object_size;  //!< Object size in bytes.
} nrf_dfu_request_create_t;

/**
 * @brief @ref NRF_DFU_OP_OBJECT_WRITE request details.
 */
typedef struct
{
    uint8_t  const * p_data; //!< Data.
    uint16_t         len;    //!< Length of data in @ref nrf_dfu_request_write_t::p_data.
} nrf_dfu_request_write_t;

/**
 * @brief @ref NRF_DFU_OP_PING request details.
 */
typedef struct
{
    uint8_t id;             //!< Ping ID that will be returned in response.
} nrf_dfu_request_ping_t;

/**
 * @brief @ref NRF_DFU_OP_MTU_GET request details.
 */
typedef struct
{
    uint16_t size;          //!< Transport MTU size in bytes.
} nrf_dfu_request_mtu_t;

/**
 * @brief @ref NRF_DFU_OP_RECEIPT_NOTIF_SET request details.
 */
typedef struct
{
    uint32_t target;        //!< Target PRN.
} nrf_dfu_request_prn_t;

typedef  struct  {
    uint32_t obj_size;
    uint32_t offset;
    uint32_t crc;
} object_select_param_t;

typedef  struct  {
    uint32_t offset;
    uint32_t crc;
} object_crc_param_t;


typedef  struct  __attribute__((packed)) {
    uint8_t ext_error;
} object_error_param_t;


typedef  struct  __attribute__((packed)) {
	uint8_t response_flag;
    uint8_t cmd;
    uint8_t status;
    union
    {
        uint8_t rawdata[16];
        object_error_param_t  object_error;
        object_select_param_t object_select;
        object_crc_param_t    object_crc;
    };
    uint16_t len;
} ble_dfu_resp_t;


typedef struct  __attribute__((packed)) {
    uint8_t obj_type;
    uint32_t obj_len;
} object_create_req_t;

typedef struct  __attribute__((packed)) {
    uint8_t is_enable;
} object_notification_req_t;

typedef struct  __attribute__((packed)) {
    nrf_dfu_obj_type_t obj_type;
} object_select_req_t;

typedef  struct  __attribute__((packed)) {	
    uint8_t cmd;
    union
    {
        uint8_t rawdata[32];
        object_create_req_t       object_create;
        object_select_req_t       object_select;
        object_notification_req_t object_notification;
    };
    uint8_t len;
} ble_dfu_ctrl_point_req_t;

typedef enum {
    NRF_DFU_STATUS_FAIL = 0x00,
    NRF_DFU_STATUS_SUCCESS
} nrf_dfu_status_t;

typedef void (*disconnect_handler_t)();
typedef void (*status_handler_t)(nrf_dfu_status_t state);
typedef void (*ctrl_point_write_handler_t)(uint8_t *value, uint16_t value_len);
typedef void (*packet_write_handler_t)(uint8_t *value, uint16_t value_len);

typedef struct {
    disconnect_handler_t              disconnect_handler;       
    status_handler_t                  status_handler;           
    ctrl_point_write_handler_t        ctrl_point_write_handler; 
    packet_write_handler_t            packet_write_handler;     
    uint16_t                          mtu_size;
    uint32_t                          transmission_interval;
} nrf_dfu_handler_t;

#if 0
typedef void (*nrf_dfu_response_callback_t)(nrf_dfu_response_t * p_res, void * p_context);

/**
 *@brief DFU request.
 */
typedef struct
{
    nrf_dfu_op_t   request;     //!< Requested operation.
    void         * p_context;
    struct
    {
        nrf_dfu_response_callback_t response; //!< Callback to call to send the response.
        nrf_dfu_flash_callback_t    write;
    } callback;
    union
    {
        nrf_dfu_request_firmware_t firmware;    //!< Firmware version request.
        nrf_dfu_request_select_t   select;      //!< Select object request.
        nrf_dfu_request_create_t   create;      //!< Create object request.
        nrf_dfu_request_write_t    write;       //!< Write object request.
        nrf_dfu_request_ping_t     ping;        //!< Ping.
        nrf_dfu_request_mtu_t      mtu;         //!< MTU size request.
        nrf_dfu_request_prn_t      prn;         //!< Set receipt notification request.
    };
} nrf_dfu_request_t;
#endif


void nrf_dfu_start(nrf_dfu_handler_t* handler);
void nrf_dfu_update_mtu(uint16_t mtu);
void nrf_dfu_receive(nrf_dfu_char_type_t mode, uint8_t *value, uint16_t value_len);
void nrf_dfu_stop();
#ifdef __cplusplus
}
#endif



#endif // NRF_DFU_REQ_HANDLER_H__

/** @} */
