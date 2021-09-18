# esp32_nrf52_dfu_lib


the receive data function

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


Start Dfu mode
    
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





You need to modify the code below.

in the dfu_lib/nrf_dfu.c file

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


        