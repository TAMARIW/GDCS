#include "rodos.h"
#include "random.h"

#include "Datastruct.h"

#include "Sub_Recv_Object.hpp"

#include "Datalink.hpp"


//Settings for the datalink topics
#define DATALINK_ENABLE_WIFI_AP                 2100 //Topic used to enable or disable the wifi access point
#define DATALINK_ENABLE_WIFI_CONNECT            2101 //Topic used to connect or disconnect wifi connection    

#define DATALINK_HEARTBEAT                      2202 //Topic used to send the datalink heartbeat


//Topics for controlling the datalink WIFI
Topic<bool> datalinkEnableWifiAP(DATALINK_ENABLE_WIFI_AP, "Enable or disable the wifi access point");
Topic<bool> datalinkEnableWifiConnect(DATALINK_ENABLE_WIFI_CONNECT, "Enable or disable the wifi connection");

//Topic to update if the datalink has connection to opposite satellite
Topic<bool> datalinkConnected(-1, "Datalink connection");
Topic<bool> oppositeDatalinkConnected(-1, "Opposite Datalink connection");

//Topic to control the WIFI mode of operation. 0 = OFF, 1 = AP, 2 = CONNECT, 3 = Automatic (Default)
Topic<int> datalinkWiFiMode(-1, "Datalink WIFI mode");

//Topic to send the datalink heartbeat
Topic<bool> datalinkHeartbeat(DATALINK_HEARTBEAT, "Datalink heartbeat");


//Gateway setup
HAL_UART datalinkUART(UART_IDX::UART_IDX3, GPIO_026, GPIO_027);
LinkinterfaceUART uartLinkinterface(&datalinkUART, 115200);
Gateway datalinkGateway(&uartLinkinterface);






//Class to setup and control the datalink stuff
class DatalinkManagment : StaticThread<> {
private:

    int64_t nextHeartbeatSend_ = 0;
    int64_t lastHeartbeatRecv_ = 0;
    int64_t wifiConnectionWaitEnd_ = 0;

    bool datalinkConnected_ = false;
    bool oppositeDatalinkConnected_ = false;

    CommBuffer<int64_t> datalinkHeartbeatBuf_;
    Subscriber datalinkHeartbeatSub_;

    SubscriberObjRecv<int, DatalinkManagment> datalinkWiFiModeSub_;


    enum WiFiControlMode_t {
        WiFiControlMode_OFF, //Turn off wifi
        WiFiControlMode_AP, //Enable wifi access point
        WiFiControlMode_CONNECT, //Enable wifi connection
        WiFiControlMode_AUTOMATIC, //Automatically control wifi to negotiate connection
    } wifiControlMode_ = WiFiControlMode_OFF;
    Atomic<WiFiControlMode_t> wifiControlModeSet_ = WiFiControlMode_AUTOMATIC;


    enum WiFiControlState_t {
        WiFiControlState_OFF, //WiFi is off
        WiFiControlState_CONNECTED, //WiFi is connected
        WiFiControlState_CONNECT_WAIT, //Waiting for connection in mode connect
        WiFiControlState_ACCESSPOINT_WAIT, //Waiting for connection in mode access point
    } wifiControlState_ = WiFiControlState_CONNECT_WAIT;

public:

    DatalinkManagment() : 
        StaticThread<>("DatalinkManagment", 2000), 
        datalinkHeartbeatSub_(datalinkHeartbeat, datalinkHeartbeatBuf_),
        datalinkWiFiModeSub_(datalinkWiFiMode, &DatalinkManagment::wifiModeReceiver, this)
    {}

    void init() override {

        datalinkGateway.addTopicsToForward(&datalinkHeartbeat);
        datalinkGateway.addTopicsToForward(&datalinkEnableWifiAP);
        datalinkGateway.addTopicsToForward(&datalinkEnableWifiConnect);

    }


    void run() override {

        while (1) {
            
            handleDatalinkHeartbeat();

            handleWiFiControl();

            suspendCallerUntil(NOW() + 200*MILLISECONDS);

        }

    }

    void wifiModeReceiver(int &mode) {

        switch (mode)
        {
        case 0:
            wifiControlModeSet_ = WiFiControlMode_OFF;
            break;

        case 1:
            wifiControlModeSet_ = WiFiControlMode_AP;
            break;

        case 2:
            wifiControlModeSet_ = WiFiControlMode_CONNECT;
            break;

        case 3:
            wifiControlModeSet_ = WiFiControlMode_AUTOMATIC;
            break;
        
        default:
            wifiControlModeSet_ = WiFiControlMode_AUTOMATIC;
            break;
        }

    }

    void handleWiFiControl() {

        switch (wifiControlModeSet_)
        {
        case WiFiControlMode_t::WiFiControlMode_OFF:
            if (wifiControlMode_ != wifiControlModeSet_) {
                datalinkEnableWifiAP.publish(false);
                datalinkEnableWifiConnect.publish(false);
                wifiControlMode_ = wifiControlModeSet_.load();
            }
            break;

        case WiFiControlMode_t::WiFiControlMode_AP:
            if (wifiControlMode_ != wifiControlModeSet_) {
                datalinkEnableWifiAP.publish(true);
                datalinkEnableWifiConnect.publish(false);
                wifiControlMode_ = wifiControlModeSet_.load();
            }
            break;

        case WiFiControlMode_t::WiFiControlMode_CONNECT:
            if (wifiControlMode_ != wifiControlModeSet_) {
                datalinkEnableWifiAP.publish(false);
                datalinkEnableWifiConnect.publish(true);
                wifiControlMode_ = wifiControlModeSet_.load();
            }
            break;
        
        default: //Both do automatic as a fallback if the mode is not recognized
        case WiFiControlMode_t::WiFiControlMode_AUTOMATIC:
            if (wifiControlMode_ != wifiControlModeSet_) {
                wifiControlState_ = WiFiControlState_t::WiFiControlState_OFF; //Make sure we start from off
            }
            wifiAutoStateMachine();
            break;
        
        }

        if (wifiControlModeSet_ != wifiControlMode_) {
            wifiControlMode_ = wifiControlModeSet_;
        }

    }

    void wifiAutoStateMachine() {

        switch (wifiControlState_)
        {
        case WiFiControlState_OFF:

            PRINTF("Starting wifi connection process.\n Searching for AP...\n");

            datalinkEnableWifiAP.publish(false);
            datalinkEnableWifiConnect.publish(true);
            wifiConnectionWaitEnd_ = NOW() + 10*SECONDS; //Wait for 10 seconds before trying to connect by providing AP
            wifiControlState_ = WiFiControlState_CONNECT_WAIT;
            break;

        case WiFiControlState_CONNECT_WAIT:

            if (datalinkConnected_) {

                wifiControlState_ = WiFiControlState_CONNECTED;

                PRINTF("Connected to AP!\n");

            } else if (NOW() > wifiConnectionWaitEnd_) { //Timeout on connecting via connect mode. Switching to AP and waiting again.

                PRINTF("No AP found. Starting access point...\n");
                
                datalinkEnableWifiAP.publish(true);
                datalinkEnableWifiConnect.publish(false);
                wifiConnectionWaitEnd_ = NOW() + (10 + drandPositive(10))*SECONDS; //Wait for a random time between 0 and 5 seconds before trying to connect
                wifiControlState_ = WiFiControlState_ACCESSPOINT_WAIT;

            }

            break;

        case WiFiControlState_ACCESSPOINT_WAIT:

            if (datalinkConnected_) {

                wifiControlState_ = WiFiControlState_CONNECTED;

                PRINTF("Connected while providing AP!\n");

            } else if (NOW() > wifiConnectionWaitEnd_) { //Timeout on connecting via access point mode. Switching to connect and waiting again.

                PRINTF("No connection found. Starting connect mode...\n");
                
                datalinkEnableWifiAP.publish(false);
                datalinkEnableWifiConnect.publish(true);
                wifiConnectionWaitEnd_ = NOW() + (10 + drandPositive(10))*SECONDS; //Wait for a random time between 0 and 5 seconds before trying to connect
                wifiControlState_ = WiFiControlState_CONNECT_WAIT;

            }

            break;

        case WiFiControlState_CONNECTED:
            //Do nothing. Just wait for the connection to be lost
            if (!datalinkConnected_) {
                wifiControlState_ = WiFiControlState_OFF;
                PRINTF("Datalink connection lost! Restarting connection process...\n");
            }
            break;
        
        default:
            wifiControlState_ = WiFiControlState_OFF;
            break;
        }

    }

    void handleDatalinkHeartbeat() {
        
        if (NOW() > nextHeartbeatSend_) {
            nextHeartbeatSend_ = NOW() + (drandPositive(1) + 0.5)*SECONDS; //Send a heartbeat every 0.5 to 1 seconds. Randomized to avoid collisions when synchronized.

            datalinkHeartbeatSub_.enable(false); //Disable so we dont receive our own heartbeat
            datalinkHeartbeat.publish(datalinkConnected_);
            datalinkHeartbeatSub_.enable(true);
            
        }

        int64_t heartbeatRecv = 0;
        if (datalinkHeartbeatBuf_.getOnlyIfNewData(heartbeatRecv)) {

            lastHeartbeatRecv_ = NOW();

            if (!datalinkConnected_) {
                datalinkConnected_ = true;
                datalinkConnected.publish(true);
            }

        }

        if (NOW() - lastHeartbeatRecv_ > 5*SECONDS) { //If no heartbeat has been received in 5 seconds then assume connection is lost

            if (datalinkConnected_) {
                datalinkConnected_ = false;
                datalinkConnected.publish(false);
            }
            
        }

    }

} datalinkManagment;
