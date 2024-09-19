#include "rodos.h"

#include "../datalink/Datalink.hpp"


//Class to simply print the state of the datalink
class DatalinkMonitor : StaticThread<> {
private:

    CommBuffer<bool> datalinkHeartbeatBuf_;
    Subscriber datalinkHeartbeatSub_;

    CommBuffer<bool> oppositeDatalinkHeartbeatBuf_;
    Subscriber oppositeDatalinkHeartbeatSub_;


public:

    DatalinkMonitor() : 
        StaticThread<>("ImAlive", 5000),
        datalinkHeartbeatSub_(datalinkConnected, datalinkHeartbeatBuf_),
        oppositeDatalinkHeartbeatSub_(oppositeDatalinkConnected, oppositeDatalinkHeartbeatBuf_)
    {

    }

    void init() override {

    }


    void run() override {

        while (1) {

            bool connected = false;
            if (datalinkHeartbeatBuf_.getOnlyIfNewData(connected)) {

                PRINTF("Datalink connected: %d\n", connected);

            }

            if (oppositeDatalinkHeartbeatBuf_.getOnlyIfNewData(connected)) {

                PRINTF("Opposite datalink connected: %d\n", connected);

            }

            suspendCallerUntil(NOW() + 1000 * MILLISECONDS);

        }

    }

} datalinkMonitor;
