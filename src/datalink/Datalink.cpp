#include "rodos.h"
#include "random.h"

#include "Datastruct.h"

#include "../Sub_Recv_Object.hpp"
#include "../filter/Pose_Filter.hpp"

#include "Datalink.hpp"


//Settings for the datalink topics
#define DATALINK_ENABLE_WIFI_AP                 2100 //Topic used to enable or disable the wifi access point
#define DATALINK_ENABLE_WIFI_CONNECT            2101 //Topic used to connect or disconnect wifi connection    

#define DATALINK_HEARTBEAT                      2202 //Topic used to send the datalink heartbeat


//Topics for controlling the datalink WIFI
Topic<bool> datalinkEnableWifiConnect(DATALINK_ENABLE_WIFI_CONNECT, "Enable or disable the wifi connection");

//Topic to update if the datalink has connection to opposite satellite
Topic<bool> datalinkConnected(-1, "Datalink connection");
Topic<bool> oppositeDatalinkConnected(-1, "Opposite Datalink connection");

//Topic to control the WIFI mode of operation. 0 = OFF, 1 = Connect, 2 = Automatic (Default)
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

    CommBuffer<bool> datalinkHeartbeatBuf_;
    Subscriber datalinkHeartbeatSub_;

    CommBuffer<Vector3D_F> sensorFusionPositionBuf_;
    Subscriber sensorFusionPositionSub_;

    SubscriberObjRecv<int, DatalinkManagment> datalinkWiFiModeSub_;


    enum WiFiControlMode_t {
        WiFiControlMode_OFF, //Turn off wifi
        WiFiControlMode_CONNECT, //Enable wifi connection
        WiFiControlMode_AUTOMATIC, //Automatically control wifi to negotiate connection
    } wifiControlMode_ = WiFiControlMode_OFF;

    //If the wifi is currently enabled
    bool wifiEnabled_ = false;


public:

    DatalinkManagment() : 
        StaticThread<>("DatalinkManagment", 2000), 
        datalinkHeartbeatSub_(datalinkHeartbeat, datalinkHeartbeatBuf_),
        sensorFusionPositionSub_(filterPositionTopic, sensorFusionPositionBuf_),
        datalinkWiFiModeSub_(datalinkWiFiMode, &DatalinkManagment::wifiModeReceiver, this)
    {}


    void init() override {

        datalinkGateway.addTopicsToForward(&datalinkHeartbeat);
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
            wifiControlMode_ = WiFiControlMode_OFF;
            break;

        case 1:
            wifiControlMode_ = WiFiControlMode_CONNECT;
            break;

        case 2:
            wifiControlMode_ = WiFiControlMode_AUTOMATIC;
            break;
        
        default:
            wifiControlMode_ = WiFiControlMode_OFF;
            break;
        }

    }

    void handleWiFiControl() {

        switch (wifiControlMode_)
        {
        case WiFiControlMode_t::WiFiControlMode_OFF:

            if (wifiEnabled_) {
                wifiEnabled_ = false;
                datalinkEnableWifiConnect.publish(false);
            }

            break;

        case WiFiControlMode_t::WiFiControlMode_CONNECT:

            if (!wifiEnabled_) {
                wifiEnabled_ = true;
                datalinkEnableWifiConnect.publish(true);
            }

            break;
        
        default: //Both do automatic as a fallback if the mode is not recognized
        case WiFiControlMode_t::WiFiControlMode_AUTOMATIC:

            wifiAutoStateMachine();

            break;
        
        }

    }

    void wifiAutoStateMachine() {

        Vector3D_F position;
        if (sensorFusionPositionBuf_.getOnlyIfNewData(position)) {

            auto distance = position.getLen();
            if (wifiEnabled_ && distance > DATALINK_WIFI_DISCONNECT_DISTANCE) {
                wifiEnabled_ = false;
                datalinkEnableWifiConnect.publish(false);
            } else if (!wifiEnabled_ && distance < DATALINK_WIFI_CONNECT_DISTANCE) {
                wifiEnabled_ = true;
                datalinkEnableWifiConnect.publish(true);
            }

        }

    }

    void handleDatalinkHeartbeat() {
        
        if (NOW() > nextHeartbeatSend_) {
            nextHeartbeatSend_ = NOW() + (drandPositive(1) + 0.5)*SECONDS; //Send a heartbeat every 0.5 to 1 seconds. Randomized to avoid collisions when synchronized.

            datalinkHeartbeatSub_.enable(false); //Disable so we dont receive our own heartbeat
            datalinkHeartbeat.publish(datalinkConnected_);
            datalinkHeartbeatSub_.enable(true);
            
        }

        bool heartbeatRecv = 0;
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
