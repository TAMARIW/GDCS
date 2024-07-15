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

#endif
