# esp32_nrf52_dfu_lib


## ESP32 프로젝트에 데이터 입력 받는 부분 처리

    static void gattc_receive_data(struct gattc_notify_evt_param notify {           
        if(notify.handle == ctrl_point_char_handle){
            //if the receive data is Control Point characteristic
            nrf_dfu_receive(NRF_DFU_CHAR_TYPE_CTRL_POINT, notify.value, notify.value_len);
        } else if(notify.handle == packet_char_handle) {
            // if the receive data is Packet characteristic, but not currently in use
            nrf_dfu_receive(NRF_DFU_CHAR_TYPE_PACKET, notify.value, notify.value_len);
        }
        ...
    }

    ...
    ...


## ESP32 프로젝트에 nrf_dfu_lib에 필요한 콜백함수 구현

    //ble 연결 해제 처리 함수
    void ble_disconnect() {
    esp_ble_gattc_close(
        gattc_profile.gattc_if,
        gattc_profile.conn_id);
    }

    // dfu 완료 상태 콜백 함수
    void ble_dfu_status(nrf_dfu_status_t state) {
        if(state == NRF_DFU_STATUS_SUCCESS){
            ESP_LOGI(TAG, "nrf dfu update done");
        }else{
            ESP_LOGI(TAG, "nrf dfu update failed");
        }
    }

    // control_point 캐릭터리스틱 전송용 콜백 함수
    void ble_dfu_control_write(uint8_t *data, uint16_t len) {
        esp_ble_gattc_write_char( 
            gattc_profile.gattc_if,
            gattc_profile.conn_id,
            gattc_profile.ctrl_point_char_handle,
            len,
            data,
            ESP_GATT_WRITE_TYPE_RSP,
            ESP_GATT_AUTH_REQ_NONE
        );
    }

    // pakcet 캐릭터리스틱 전송용 콜백 함수
    void ble_dfu_packet_write(uint8_t *data, uint16_t len) {   
        esp_ble_gattc_write_char( 
            gattc_profile.gattc_if,
            gattc_profile.conn_id,
            gattc_profile.packet_char_handle,
            len,
            data,
            ESP_GATT_WRITE_TYPE_NO_RSP,
            ESP_GATT_AUTH_REQ_NONE
        ); 
    }

## nRF DFU 모드 시작
    
    ...
    ...
    // when the connection setup with nrf52 is completed
    // connect, discover services, notify enable..
    nrf_dfu_handler_t nrf_dfu_handler = {
        .disconnect_handler       = ble_disconnect,
        .status_handler           = ble_dfu_status,
        .ctrl_point_write_handler = ble_dfu_control_write,
        .packet_write_handler     = ble_dfu_packet_write,
        .mtu_size                 = (mtu-3),
        .transmission_interval    = (connect_max_interval * 1.25),
    };          

    //Start dfu      
    nrf_dfu_start(&nrf_dfu_handler);   


## dat 파일과 bin 파일 변경
dfu_lib/nrf_dfu.c file 을 수정한다.

추후 변경 예정 

    static void nrf_dfu_task(void *param){
        ...
        ...        
        // send config file (xxx.dat)
        result = send_config_file((uint8_t*)config_file, sizeof(config_file));
        if(!result) {       
            goto NRF_DFU_EXIT;
        }   

        // send firmware file (xxx.bin)
        result = send_fw_file((uint8_t*)ddl_bin, sizeof(ddl_bin));
        if(!result) {       
            goto NRF_DFU_EXIT;
        }

        ...
        ...


        