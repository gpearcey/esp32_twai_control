#include <stdio.h>
#include "twaiCanController.h"
#include "driver/gpio.h"
#include "NMEA_msg.h"
#include "esp_log.h"

static const char* TAG = "test.cpp";

extern "C" void app_main(void)
{
    twaiCANController twai1(GPIO_NUM_32, GPIO_NUM_34);

    twai1.init();
/**
    //Transmit messages
    for (int i = 0; i < 100; i++){
        NMEA_msg msg;
        msg.PGN = 128259;
        msg.src = 3;
        msg.priority = 6;
        msg.length = 8;
        msg.data[0] = 0xAA;
        msg.data[1] = 0xAA;
        msg.data[2] = 0xAA;
        msg.data[3] = 0xAA;
        msg.data[4] = 0xAA;
        msg.data[5] = 0xAA;
        msg.data[6] = 0xAA;
        msg.data[7] = 0xAA;
        twai1.transmit(msg);
    }
    
    //Receive Messages
    for (int i = 0; i < 100; i++){
        NMEA_msg rx_msg;
        twai1.receive(rx_msg);
        ESP_LOGD(TAG,"PGN: %u", rx_msg.PGN);
        ESP_LOGD(TAG,"Data: %x %x %x %x %x %x %x %x ", rx_msg.data[0], rx_msg.data[1], rx_msg.data[2], rx_msg.data[3], rx_msg.data[4], rx_msg.data[5], rx_msg.data[6], rx_msg.data[7]);
        ESP_LOGD(TAG,"Data Length: %u", rx_msg.length);    
    }*/

    //Transmit fast packet messages
    for (int i = 0; i < 100; i++){
        NMEA_msg msg;
        msg.PGN = 126996;
        msg.src = 5;
        msg.priority = 6;
        msg.length = 134;
        for(int i = 0; i<msg.length; i++){
            msg.data[i] = i;
        }
        twai1.transmit(msg);
    }


    twai1.deinit();
}
