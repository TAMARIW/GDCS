#include "rodos.h"

#include "Datastruct.h"

#include "Datalink.hpp"


//Gateway setup
HAL_UART datalinkUART(UART_IDX::UART_IDX2);
LinkinterfaceUART uartLinkinterface(&datalinkUART, 115200);
Gateway datalinkGateway(&uartLinkinterface, true);


//Class to setup and control the datalink stuff
class DatalinkManagment : StaticThread<> {
private:



public:

    void init() override {

    }


    void run() override {

        suspendCallerUntil(END_OF_TIME);

    }

};// datalinkManagment;
