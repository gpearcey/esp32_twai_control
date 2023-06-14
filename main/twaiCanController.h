#ifndef TWAI_CAN_CONTROLLER_H
#define TWAI_CAN_CONTROLLER_H

#include <string>
#include <bitset>
#include "CAN_controller.h"
#include "esp_log.h"
#include "driver/twai.h"

class twaiCANController : public CANController
{
    public:
        twaiCANController(gpio_num_t txPin, gpio_num_t rxPin);
        void init() override;
        void deinit() override;
        void receive(NMEA_msg& msg) override;
        void transmit(NMEA_msg msg) override;

    private:
        void TransmitNormal(twai_message_t t_msg);
        void TransmitFastPacket(NMEA_msg msg);
        twai_message_t NMEAtoCAN(NMEA_msg msg);
        NMEA_msg CANtoNMEA(twai_message_t t_msg);
        int GetCanId(NMEA_msg msg);

        //set timing to 250kbits for NMEA
        static constexpr twai_timing_config_t t_config_ = TWAI_TIMING_CONFIG_250KBITS();
        //Filter for incoming messages
        static constexpr twai_filter_config_t f_config_ = TWAI_FILTER_CONFIG_ACCEPT_ALL();
        //Configure TX and RX pins, set twai mode (set in constructor)
        const twai_general_config_t g_config_; 

        //constants needed for fast packet calculations
        static const int can_frame_size_ = 8;
        static const int first_fp_data_frame_size_ = 6;
        static const int fp_data_frame_size_ = 7;
        unsigned int seq_count_ : 3;
        int prev_id_;



};

#endif //TWAI_CAN_CONTROLLER_H