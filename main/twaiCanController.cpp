#include "twaiCanController.h"
#include <cmath>
#include <inttypes.h>

static const char* TAG = "twaiCanController";

/* --------------------- Functions -------------------------------------------*/

twaiCANController::twaiCANController(gpio_num_t txPin, gpio_num_t rxPin) : g_config_(TWAI_GENERAL_CONFIG_DEFAULT(txPin, rxPin, TWAI_MODE_NORMAL)), seq_count_(0){}

void twaiCANController::init(){
   
    //Install TWAI driver
    if (twai_driver_install(&g_config_, &t_config_, &f_config_) == ESP_OK) {
        ESP_LOGI(TAG, "Driver installed\n");
    } else {
        ESP_LOGI(TAG, "Failed to install driver\n");
        return;
    }

    //Start TWAI driver
    if (twai_start() == ESP_OK) {
        ESP_LOGI(TAG, "Driver started\n");
    } else {
        ESP_LOGI(TAG,"Failed to start driver\n");
    }

    return;
}

void twaiCANController::deinit(){
    //Stop the TWAI driver
    if (twai_stop() == ESP_OK) {
    ESP_LOGI(TAG, "Driver stopped\n");
    } else {
        ESP_LOGI(TAG,"Failed to stop driver\n");
        return;
    }

    //Uninstall the TWAI driver
    if (twai_driver_uninstall() == ESP_OK) {
        ESP_LOGI(TAG,"Driver uninstalled\n");
    } else {
        ESP_LOGI(TAG,"Failed to uninstall driver\n");
    }
    return;
}

void twaiCANController::receive(NMEA_msg& msg){
    //Wait for message to be received
    twai_message_t message;
    if (twai_receive(&message, pdMS_TO_TICKS(1000)) == ESP_OK) {
        ESP_LOGI(TAG,"Message received\n");
    } else {
        ESP_LOGI(TAG,"Failed to receive message\n");
        return;
    }
    //convert message to NMEA format
    msg = CANtoNMEA(message);
    return;
}

void twaiCANController::transmit(NMEA_msg msg){
    if (msg.length > 8){
        //message is larger than 8 bytes, transmit using fast packet
        TransmitFastPacket(msg);
    }
    else{
        //message fits in single CAN frame, transmit as normal
        twai_message_t t_msg = NMEAtoCAN(msg);
        TransmitNormal(t_msg);
    }
    return; 
}

void twaiCANController::TransmitNormal(twai_message_t t_msg){

    if (twai_transmit(&t_msg, pdMS_TO_TICKS(1000)) == ESP_OK) {
        ESP_LOGI(TAG,"Message queued for transmission\n");
    } else {
        ESP_LOGI(TAG,"Failed to queue message for transmission\n");
    }
    return;
}

void twaiCANController::TransmitFastPacket(NMEA_msg msg){
    twai_message_t t_msg;
    
    //Set fields that will be constant for all frames (except data_length_code in last frame) in message
    t_msg.identifier = GetCanId(msg);
    t_msg.rtr = 0;
    t_msg.data_length_code = 8;
    t_msg.extd = true;

    //Increase sequence count if message has the same ID as last sent message
    if (t_msg.identifier == this->prev_id_){
        this->seq_count_++;
        this->seq_count_ = this->seq_count_ % can_frame_size_;
    }

    int frame_count = 0;
    int total_frames = std::ceil((double)(msg.length-first_fp_data_frame_size_) / fp_data_frame_size_) + 1;

    //transmit the first frame
    //the first byte of the first frame is the packet ID, the second byte is total message length in bytes
    std::string str_seq_count = std::bitset<3>(seq_count_).to_string();
    std::string str_frame_count = std::bitset<5>(frame_count).to_string();
    std::string str_packet_id = str_seq_count + str_frame_count;
    int packet_id = std::stoi(str_packet_id, nullptr, 2);

    t_msg.data[0] = packet_id;
    t_msg.data[1] = msg.length;
    t_msg.data[2] = msg.data[0];
    t_msg.data[3] = msg.data[1];
    t_msg.data[4] = msg.data[2];
    t_msg.data[5] = msg.data[3];
    t_msg.data[6] = msg.data[4];
    t_msg.data[7] = msg.data[5];
    ESP_LOGD(TAG, "About to transmit fast packet frame %d of %d with id: %" PRIx32 " and data: %x %x %x %x %x %x %x %x ", (frame_count+1), total_frames, t_msg.identifier, t_msg.data[0], t_msg.data[1], t_msg.data[2], t_msg.data[3], t_msg.data[4], t_msg.data[5], t_msg.data[6], t_msg.data[7]);
    TransmitNormal(t_msg);
    frame_count++;

    int data_idx = first_fp_data_frame_size_;

    //Transmit remaining frames
    while(data_idx < msg.length-1){
        int i = 0;
        std::string str_frame_count = std::bitset<5>(frame_count).to_string();
        std::string str_packet_id = str_seq_count + str_frame_count;
        int packet_id = std::stoi(str_packet_id, nullptr, 2);
        t_msg.data[0] = packet_id;
        i++;

        for(; i < 8; i++){ 
            if (data_idx >= msg.length){
                //fill remaining bytes with 0xFF if reached end of data
                t_msg.data[i] = 0xFF;
                //calculate data length code for last frame
                t_msg.data_length_code = msg.length - first_fp_data_frame_size_ - ((frame_count-1) * fp_data_frame_size_);
            }
            else{
                t_msg.data[i] = msg.data[data_idx];
                data_idx++;
            }
        }
        ESP_LOGD(TAG, "About to transmit fast packet frame %d of %d with id: %" PRIx32 " and data: %x %x %x %x %x %x %x %x ", (frame_count+1), total_frames, t_msg.identifier, t_msg.data[0], t_msg.data[1], t_msg.data[2], t_msg.data[3], t_msg.data[4], t_msg.data[5], t_msg.data[6], t_msg.data[7]);
        TransmitNormal(t_msg);
        frame_count++;
    }
    prev_id_ = t_msg.identifier;
    return; 
}

twai_message_t twaiCANController::NMEAtoCAN(NMEA_msg msg){
    twai_message_t t_msg;

    //set remote transmission request to 0 to send standard data frame
    t_msg.rtr = 0;    

    //create CAN ID
    t_msg.identifier = GetCanId(msg);

    //all NMEA messages have extended 29 bt ID
    t_msg.extd = true;

    //message length in bytes
    t_msg.data_length_code = msg.length;
    
    t_msg.data[0] = msg.data[0];
    t_msg.data[1] = msg.data[1];
    t_msg.data[2] = msg.data[2];
    t_msg.data[3] = msg.data[3];
    t_msg.data[4] = msg.data[4];
    t_msg.data[5] = msg.data[5];
    t_msg.data[6] = msg.data[6];
    t_msg.data[7] = msg.data[7];

    return t_msg;

}

int twaiCANController::GetCanId(NMEA_msg msg){
    //create CAN ID
    std::string strPriority = std::bitset<3>(msg.priority).to_string();
    std::string strPGN = std::bitset<18>(msg.PGN).to_string();
    std::string strSrc = std::bitset<8>(msg.src).to_string();
    std::string strID = strPriority + strPGN + strSrc;    
    int id = std::stoi(strID, nullptr, 2);

    ESP_LOGD(TAG, "Message ID Created: %x \n", id);

    return id;
}

NMEA_msg twaiCANController::CANtoNMEA(twai_message_t t_msg){
    NMEA_msg msg;
    if (!(t_msg.rtr) && (t_msg.extd)){        
    
        ESP_LOGD(TAG,"ID to be converted: %lx \n",t_msg.identifier);

        //parse the ID to get priority, PGN and source
        std::string strID = std::bitset<32>(t_msg.identifier).to_string();    
        std::string strSrc = strID.substr(24,8);
        std::string strPGN = strID.substr(6,18);
        std::string strPriority = strID.substr(3,3);

        msg.PGN = std::stoi(strPGN, nullptr, 2);
        msg.src = std::stoi(strSrc, nullptr, 2);
        msg.priority = std::stoi(strPriority);
        msg.length = t_msg.data_length_code;

        msg.data[0] = t_msg.data[0];
        msg.data[1] = t_msg.data[1]; 
        msg.data[2] = t_msg.data[2]; 
        msg.data[3] = t_msg.data[3]; 
        msg.data[4] = t_msg.data[4]; 
        msg.data[5] = t_msg.data[5]; 
        msg.data[6] = t_msg.data[6]; 
        msg.data[7] = t_msg.data[7];

    }
    else{
        ESP_LOGI(TAG, "Incomming message is not in NMEA format");
    }
    return msg;
}

