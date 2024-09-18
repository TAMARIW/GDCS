#ifndef TMW_DATALINK_HPP_
#define TMW_DATALINK_HPP_

#include "rodos.h"


/**
 * Setup for everything needed for the datalink between this and target stm32 and rpi zero 2. (HAL, Link, Gateway)
 */

//Settings for the datalink
#define DATALINK_WIFI_CONNECT_DISTANCE 10 //crossing under this distance will trigger wifi to connect (In Meters)
#define DATALINK_WIFI_DISCONNECT_DISTANCE 12 //crossing over this distance will trigger wifi to disconnect (In Meters)


//Gateway setup
extern HAL_UART datalinkUART;
extern LinkinterfaceUART uartLinkinterface;
extern Gateway datalinkGateway;


//Topic to update if the datalink has connection to opposite satellite
extern Topic<bool> datalinkConnected;
//Topic to update if the opposite datalink has connection to this satellite
extern Topic<bool> oppositeDatalinkConnected;

//Topic to control the WIFI mode of operation. 0 = OFF, 1 = Connect, 2 = Automatic (Default, will connect if within distance)
extern Topic<int> datalinkWiFiMode;

#endif
