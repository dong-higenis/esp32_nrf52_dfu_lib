
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "nrf_dfu.h"
#include "test_firmware.h"

static const char *TAG = "nrf_dfu";

#define MIN(X, Y) (((X) < (Y)) ? (X) : (Y))

const uint8_t config_file[] = {0x12 ,0x8A ,0x01 ,0x0A ,0x44 ,0x08 ,0x01 ,0x12 ,0x40 ,0x08 ,0x01 ,0x10 ,0x34 ,0x1A ,0x02 ,0xB7 ,0x01 ,0x20 ,0x00 ,0x28 ,0x00 ,0x30 ,0x00 ,0x38 ,0xDC ,0xF6 ,0x05 ,0x42 ,0x24 ,0x08 ,0x03 ,0x12 ,0x20 ,0xE8 ,0x0F ,0x35 ,0xEF ,0x37 ,0x1C ,0xEA ,0x7B ,0xD4 ,0x63 ,0x17 ,0xAE ,0x07 ,0x58 ,0x97 ,0x5A ,0x4E ,0xC0 ,0xE3 ,0xD6 ,0xC9 ,0x08 ,0x64 ,0x43 ,0x42 ,0x16 ,0x85 ,0xEF ,0x8F ,0x92 ,0x4B ,0xF9 ,0x48 ,0x00 ,0x52 ,0x04 ,0x08 ,0x01 ,0x12 ,0x00 ,0x10 ,0x00 ,0x1A ,0x40 ,0x87 ,0x64 ,0xAE ,0xDB ,0xDC ,0x65 ,0xDC ,0x91 ,0x2C ,0xCB ,0xBB ,0x57 ,0x84 ,0xFF ,0x98 ,0xE2 ,0x8D ,0x50 ,0x11 ,0x75 ,0x11 ,0x49 ,0x1D ,0x80 ,0xE3 ,0x60 ,0xFF ,0xBE ,0x5F ,0x52 ,0x06 ,0x59 ,0x1D ,0x35 ,0x86 ,0xAD ,0x69 ,0x0D ,0x08 ,0xA1 ,0x7D ,0x0F ,0x1C ,0xFD ,0x1D ,0xE8 ,0x5B ,0x07 ,0xC4 ,0xC0 ,0xE3 ,0x8E ,0x95 ,0xC7 ,0x98 ,0x97 ,0xD9 ,0xCE ,0xD5 ,0x42 ,0xD0 ,0xE1 ,0xB8 ,0x0A};

static uint32_t total_file_size    = 0;
static uint8_t  packet_total_count = 0;
static uint16_t max_packet_size    = 0; // maximum packet size sent at one command cycle
static uint32_t packet_crc         = 0;  

/* Task handler */
TaskHandle_t h_nrf_dfu_task = NULL;

/* callback handler */
nrf_dfu_handler_t nrf_dfu_handler;

/* dfu control packet receive queue */
QueueHandle_t dfu_queue = NULL;

static uint8_t check_ble_dfu_error(ble_dfu_resp_t *ble_dfu_resp);

void nrf_dfu_init(nrf_dfu_handler_t *handler) {
    memset((uint8_t*)&nrf_dfu_handler, 0, sizeof(nrf_dfu_handler_t)/sizeof(uint8_t));
    nrf_dfu_handler.disconnect_handler       = handler->disconnect_handler;
    nrf_dfu_handler.status_handler           = handler->status_handler;
    nrf_dfu_handler.ctrl_point_write_handler = handler->ctrl_point_write_handler;
    nrf_dfu_handler.packet_write_handler     = handler->packet_write_handler;
    nrf_dfu_handler.mtu_size                 = handler->mtu_size;        
    nrf_dfu_handler.transmission_interval    = handler->transmission_interval;        
}


static void send_packet(uint8_t *dat, uint16_t len){   
    if(nrf_dfu_handler.packet_write_handler != NULL){
        nrf_dfu_handler.packet_write_handler(dat, len);
    }
}

static void send_ctrl_point(uint8_t *dat, uint16_t len) {   
    if(nrf_dfu_handler.ctrl_point_write_handler != NULL){
        nrf_dfu_handler.ctrl_point_write_handler(dat, len);
    }
}

static void nrf_dfu_disconnect(){
    // disconnect ble device
    if(nrf_dfu_handler.disconnect_handler != NULL){
        nrf_dfu_handler.disconnect_handler();
    }    
}
static void nrf_dfu_set_status(bool result){    
    // update dfu status
    if(nrf_dfu_handler.status_handler != NULL) {                                
        nrf_dfu_handler.status_handler(result?NRF_DFU_STATUS_SUCCESS:NRF_DFU_STATUS_FAIL);
    }
}


static uint32_t crc32_compute(uint8_t const * p_data, uint32_t size, uint32_t const * p_crc) {
    uint32_t crc;
    crc = (p_crc == NULL) ? 0xFFFFFFFF : ~(*p_crc);
    for (uint32_t i = 0; i < size; i++)
    {
        crc = crc ^ p_data[i];
        for (uint32_t j = 8; j > 0; j--)
        {
            crc = (crc >> 1) ^ (0xEDB88320U & ((crc & 1) ? 0xFFFFFFFF : 0));
        }
    }
    return ~crc;
}

static char* error_to_string(uint8_t err_code) {   
    switch(err_code){        
    case NRF_DFU_RES_CODE_INVALID:
        return "Invalid";
        break;
    case NRF_DFU_RES_CODE_SUCCESS:
        return "Success";
        break;
    case NRF_DFU_RES_CODE_OP_CODE_NOT_SUPPORTED:
        return "op code not supported";
        break;
    case NRF_DFU_RES_CODE_INVALID_PARAMETER:
        return "invalid parameter";
        break;
    case NRF_DFU_RES_CODE_INSUFFICIENT_RESOURCES:
        return "insufficient resources";
        break;
    case NRF_DFU_RES_CODE_INVALID_OBJECT:
        return "invaild object";
        break;
    case NRF_DFU_RES_CODE_UNSUPPORTED_TYPE:
        return "unsupported type";
        break;
    case NRF_DFU_RES_CODE_OPERATION_NOT_PERMITTED:
        return "op not permitted";
        break;
    case NRF_DFU_RES_CODE_OPERATION_FAILED:
        return "op failed";
        break;
    case NRF_DFU_RES_CODE_EXT_ERROR:
        return "ext error";
        break;
    default:    
        return "unknown error";
    }    
}

static uint8_t check_ble_dfu_error(ble_dfu_resp_t *ble_dfu_resp) {    
    if(ble_dfu_resp->status == NRF_DFU_RES_CODE_SUCCESS) {
        return true;
    }
    if(ble_dfu_resp->len == 3) {
        ESP_LOGE(TAG, "[0x%02x] %s failed! errcode: 0x%02x",  ble_dfu_resp->cmd , error_to_string(ble_dfu_resp->status), ble_dfu_resp->status);    
    }else if(ble_dfu_resp->len > 3){        
        ESP_LOGE(TAG, "[0x%02x] failed! extend errcode: 0x%02x, 0x%02x", ble_dfu_resp->cmd, ble_dfu_resp->status, ble_dfu_resp->object_error.ext_error);
    } else  {
        ESP_LOGE(TAG, "[0x%02x] failed! unknown errcode", ble_dfu_resp->cmd);
        esp_log_buffer_hex(TAG, &ble_dfu_resp, ble_dfu_resp->len);
    }
    return false;
}

static bool send_ctrl_point_wait_response_command(ble_dfu_ctrl_point_req_t *req, ble_dfu_resp_t *ble_dfu_resp) {        
    send_ctrl_point((uint8_t*)req, req->len);
    while(true){          
        if(dfu_queue == NULL){
            break;
        }
        if(xQueueReceive(dfu_queue, ble_dfu_resp, 0) == pdTRUE) {
            if(ble_dfu_resp->response_flag == NRF_DFU_OP_RESPONSE) {
                if(check_ble_dfu_error(ble_dfu_resp)) {
                    if(req->cmd == ble_dfu_resp->cmd){
                        return true;
                    }
                }
            }
            break;
        }     
        vTaskDelay(1 / portTICK_RATE_MS);
    }
    return false;
}


/*
* request object select
*/
static bool send_obj_select(nrf_dfu_obj_type_t type) {     
    uint8_t result = false;      
    ble_dfu_ctrl_point_req_t req;
    req.cmd = NRF_DFU_OP_OBJECT_SELECT;
    req.object_select.obj_type = type;
    req.len = 2;

    ble_dfu_resp_t ble_dfu_resp;    
    if(send_ctrl_point_wait_response_command(&req, &ble_dfu_resp)) {        
        uint32_t obj_size = ble_dfu_resp.object_select.obj_size;
#ifdef DFU_DEBUG
        uint32_t offset = ble_dfu_resp.object_select.offset;
        uint32_t crc = ble_dfu_resp.object_select.crc;
        ESP_LOGI(TAG, ">>>>>>>>>>>>>> NRF_DFU_OP_OBJECT_SELECT obj_size: %d, offset 0x%x, crc: 0x%x", obj_size, offset, crc);
#endif   
        packet_crc = 0;
        max_packet_size = obj_size;
        packet_total_count = (total_file_size / max_packet_size) + ((total_file_size % max_packet_size)>0?1:0);        
        result = true;
    }else{
        //fail
        result = false;
    }
    return result;
}

/*
* request object execute
*/
static bool send_obj_execute(){    
    uint8_t result = false;      
    ble_dfu_ctrl_point_req_t req;
    req.cmd = NRF_DFU_OP_OBJECT_EXECUTE;
    req.len = 1;

    ble_dfu_resp_t ble_dfu_resp;    
    if(send_ctrl_point_wait_response_command(&req, &ble_dfu_resp)){
        result = true;
    }else{
        //fail
        result = false;
    }
    return result;
}


/*
* request object get crc
*/
static bool send_get_crc(uint32_t crc) {         
    uint8_t result = false;      
    ble_dfu_ctrl_point_req_t req;
    req.cmd = NRF_DFU_OP_CRC_GET;
    req.len = 1;

    ble_dfu_resp_t ble_dfu_resp;    
    if(send_ctrl_point_wait_response_command(&req, &ble_dfu_resp)){
#ifdef DFU_DEBUG_DUMP
       ESP_LOGI(TAG, "raw data");
       esp_log_buffer_hex(TAG, &ble_dfu_resp, 11);
#endif       
       if(ble_dfu_resp.object_crc.crc == crc) {
            result = true;
        } else{
            result = false;
        }    
    }else{
        //fail
        result = false;
    }
    return result;
}

/*
* request object create
*/
static bool send_obj_create(nrf_dfu_obj_type_t obj_type, uint32_t wlen) {        
    uint8_t result = false;    
    ble_dfu_ctrl_point_req_t req;
    req.cmd = NRF_DFU_OP_OBJECT_CREATE;
    req.object_create.obj_type = obj_type;
    req.object_create.obj_len = wlen;
    req.len = 6;
    
    ble_dfu_resp_t ble_dfu_resp;    
    if(send_ctrl_point_wait_response_command(&req, &ble_dfu_resp)){
        result = true;
    }else{
        //fail
        result = false;
    }
    return result;
}


/*
* request object notification set
*/
static bool send_obj_notification(uint8_t is_on){    
    uint8_t result = false;
    ble_dfu_ctrl_point_req_t req;
    req.cmd = NRF_DFU_OP_RECEIPT_NOTIF_SET;
    req.object_notification.is_enable = is_on;
    req.len = 2;
    
    ble_dfu_resp_t ble_dfu_resp;    
    if(send_ctrl_point_wait_response_command(&req, &ble_dfu_resp)){
        result = true;
    }else{
        //fail
        result = false;
    }
    return result;
}

/*
* send packets
*/
static uint32_t send_rawdata(uint8_t* dat, uint16_t dat_size, uint32_t* packet_crc) {
    uint8_t* ptr = dat;
    uint16_t mtu_size = nrf_dfu_handler.mtu_size;
    uint32_t interval = nrf_dfu_handler.transmission_interval;
    uint16_t remaining_size = dat_size % mtu_size;
#ifdef DFU_DEBUG
    ESP_LOGI(TAG, "send_rawdata counter %d + (%d), interval: %dms", mtu_size, remaining_size?1:0, interval);
#endif    
    for(int i=0;i<(dat_size / mtu_size);i++) {
        send_packet(ptr, mtu_size);        
        ptr += mtu_size;
        // when the mtu is default write char failed(BTA_GATT_CONGESTED 0x8F)
        vTaskDelay(interval / portTICK_RATE_MS);
    }
    
    if(remaining_size != 0) {
        send_packet(ptr, remaining_size);
    }
    *packet_crc = crc32_compute((uint8_t*)dat, dat_size, packet_crc);    
    return dat_size;
}

void nrf_dfu_update_mtu(uint16_t mtu){
    nrf_dfu_handler.mtu_size = mtu;
}

static void nrf_dfu_ctrl_point_receive(uint8_t *value, uint16_t value_len) {    
    ble_dfu_resp_t ble_dfu_resp;
    if(dfu_queue != NULL) {    
        memcpy((uint8_t*)&ble_dfu_resp, value, value_len);
        ble_dfu_resp.len = value_len;    
        xQueueSend(dfu_queue, &ble_dfu_resp, 0);
    }
}

static void nrf_dfu_packet_receive(uint8_t *value, uint16_t value_len) {
    //not used
}

void nrf_dfu_receive(nrf_dfu_char_type_t mode, uint8_t *value, uint16_t value_len) {
    if(mode == NRF_DFU_CHAR_TYPE_CTRL_POINT){
        nrf_dfu_ctrl_point_receive(value, value_len);
    }else{
        nrf_dfu_packet_receive(value, value_len);
    }    
}

static bool send_config_file(uint8_t *config, uint32_t config_len) {
    bool result;    
    result = send_obj_create(NRF_DFU_OBJ_TYPE_COMMAND, config_len);
    if(!result) {         
        ESP_LOGE(TAG, "send_obj_create error");
        goto NRF_DFU_SEND_CONFIG_EXIT;
    }
    
    packet_crc = 0;
    send_rawdata((uint8_t*)config, config_len, &packet_crc);

#ifdef DFU_DEBUG    
    ESP_LOGI(TAG, "send packet crc is %x ", packet_crc);
#endif    

    result = send_get_crc(packet_crc);
    if(!result) {         
        ESP_LOGE(TAG, "send_get_crc error");
        goto NRF_DFU_SEND_CONFIG_EXIT;
    }
    result = send_obj_execute();
    if(!result) {         
        ESP_LOGE(TAG, "send_obj_execute error");
        goto NRF_DFU_SEND_CONFIG_EXIT;
    }
NRF_DFU_SEND_CONFIG_EXIT:
    return result;
}

static bool send_fw_file(uint8_t *fw, uint32_t fw_len){    
    bool result;
     
    uint32_t offset = 0;
    uint16_t packet_size = 0;
    uint16_t data_packet_count = 0;

    packet_crc = 0;
    total_file_size = fw_len;
    result = send_obj_select(NRF_DFU_OBJ_TYPE_DATA);
    if(!result) {
        ESP_LOGE(TAG, "send_obj_select error");
        goto NRF_DFU_SEND_FW_EXIT;
    }
    
    while(total_file_size != 0) {
#ifdef DFU_DEBUG            
        ESP_LOGI(TAG, ">>>>>>>>>>>>>> Data object (%d/%d) created", (data_packet_count+1), packet_total_count);
#endif        
        packet_size = MIN(total_file_size, max_packet_size);

        result = send_obj_create(NRF_DFU_OBJ_TYPE_DATA, packet_size);
        if(!result){
            ESP_LOGE(TAG, "send_get_crc error");
            goto NRF_DFU_SEND_FW_EXIT;
        }        
        
        send_rawdata((uint8_t*)&fw[offset], packet_size, &packet_crc);

        result = send_get_crc(packet_crc);
        if(!result) {         
            ESP_LOGE(TAG, "send_get_crc error");
            goto NRF_DFU_SEND_FW_EXIT;
        }

        result = send_obj_execute();
        if(!result) {         
            ESP_LOGE(TAG, "send_obj_execute error");
            goto NRF_DFU_SEND_FW_EXIT;
        }

        total_file_size -= packet_size;
        offset += packet_size;
        data_packet_count++;
    }        
NRF_DFU_SEND_FW_EXIT:
    return result;
}

static void nrf_dfu_task(void *param){
    // notification disable!
    bool result;

    //create queue    
    dfu_queue = xQueueCreate( 1, sizeof( ble_dfu_resp_t ) );     
    if(dfu_queue == NULL){
        ESP_LOGE(TAG, "Error creating the ble_msg_queue");
    }

    //disable notification
    result = send_obj_notification(0x00);    
    if(!result) {
        ESP_LOGE(TAG, "send_obj_notification error");
        goto NRF_DFU_EXIT;
    }

    // send config file
    result = send_config_file((uint8_t*)config_file, sizeof(config_file));
    if(!result) {       
        goto NRF_DFU_EXIT;
    }   

    // send firmware file
    result = send_fw_file((uint8_t*)ddl_bin, sizeof(ddl_bin));
    if(!result) {       
        goto NRF_DFU_EXIT;
    }

NRF_DFU_EXIT:
#ifdef DFU_DEBUG    
    ESP_LOGI(TAG, "done");
#endif   
    nrf_dfu_set_status(result);
    nrf_dfu_disconnect();

    nrf_dfu_stop();    
}


/*
    nrf_dfu task start
*/
void nrf_dfu_start(nrf_dfu_handler_t* handler) {
    nrf_dfu_init(handler);
    xTaskCreate(&nrf_dfu_task, "nrf_dfu_task", 2048, NULL, tskIDLE_PRIORITY, &h_nrf_dfu_task);
}


void nrf_dfu_stop(){
    if(dfu_queue != NULL) {
        vQueueDelete(dfu_queue);
        dfu_queue = NULL;
    }
    if(h_nrf_dfu_task != NULL) {
        vTaskDelete(h_nrf_dfu_task);
    }
}
