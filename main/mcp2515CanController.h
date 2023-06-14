#ifndef MCP2515_CAN_CONTROLLER_H
#define MCP2515_CAN_CONTROLLER_H

#include "CAN_controller.h"

class mcp2515CANController : public class CANController
{
    public:
        void init();
        void receive(&NMEA_msg msg);
        void transmit(int PGN, uint_8[] data );
        void transmit(NMEA_msg msg);
};

#endif //MCP2515_CAN_CONTROLLER