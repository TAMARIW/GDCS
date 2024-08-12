#ifndef TMW_DATALINK_HPP_
#define TMW_DATALINK_HPP_

#include "rodos.h"


/**
 * Setup for everything needed for the datalink between this and target stm32 and rpi zero 2. (HAL, Link, Gateway)
 */

//Gateway setup
extern HAL_UART datalinkUART;
extern LinkinterfaceUART uartLinkinterface;
extern Gateway datalinkGateway;


//Topic to update if the datalink has connection to opposite satellite
extern Topic<bool> datalinkConnected;

//Topic to control the WIFI mode of operation. 0 = OFF, 1 = AP, 2 = CONNECT, 3 = Automatic (Default)
extern Topic<int> datalinkWiFiMode;

#endif
