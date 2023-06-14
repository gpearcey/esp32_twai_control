#ifndef CAN_CONTROLLLER_H
#define CAN_CONTROLLER_H

#include "NMEA_msg.h"
#include "driver/gpio.h"
/**
 * Abstract CAN Controller 
 * Implemented for each specific CAN controller used
*/
class CANController
{
    public:
        virtual ~CANController() {}
        virtual void init() = 0;
        virtual void deinit() = 0;
        virtual void receive(NMEA_msg& msg) = 0;
        virtual void transmit(NMEA_msg msg) = 0;
};

#endif // CAN_CONTROLLER_H
