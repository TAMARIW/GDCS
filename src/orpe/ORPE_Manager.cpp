#include "rodos.h"

#include "Datastruct.h"

#include "ORPE_Manager.hpp"


namespace ORPETMW {

//Topics used by ORPE datalink
#ifndef SWITCH_ORPE_COMMS
Topic<OrpeTelemetry> orpeSelfTmtTopic(DATALINK_ORPETELEMETRY_SELF_TOPICID, "ORPE Self telemetry");
Topic<ORPECommand> orpeSelfCmdTopic(DATALINK_ORPETELECOMMAND_SELF_TOPICID, "ORPE Self telecommand");
Topic<ORPEState_t> orpeSelfSttTopic(DATALINK_ORPESTATE_SELF_TOPICID, "ORPE Self state");

Topic<OrpeTelemetry> orpeTgtTmtTopic(DATALINK_ORPETELEMETRY_TGT_TOPICID, "ORPE TGT telemetry");
Topic<ORPECommand> orpeTgtCmdTopic(DATALINK_ORPETELECOMMAND_TGT_TOPICID, "ORPE TGT telecommand");
Topic<ORPEState_t> orpeTgtSttTopic(DATALINK_ORPESTATE_TGT_TOPICID, "ORPE TGT state");
#else
Topic<OrpeTelemetry> orpeSelfTmtTopic(DATALINK_ORPETELEMETRY_TGT_TOPICID, "ORPE Self telemetry");
Topic<ORPECommand> orpeSelfCmdTopic(DATALINK_ORPETELECOMMAND_TGT_TOPICID, "ORPE Self telecommand");
Topic<ORPEState_t> orpeSelfSttTopic(DATALINK_ORPESTATE_TGT_TOPICID, "ORPE Self state");

Topic<OrpeTelemetry> orpeTgtTmtTopic(DATALINK_ORPETELEMETRY_SELF_TOPICID, "ORPE TGT telemetry");
Topic<ORPECommand> orpeTgtCmdTopic(DATALINK_ORPETELECOMMAND_SELF_TOPICID, "ORPE TGT telecommand");
Topic<ORPEState_t> orpeTgtSttTopic(DATALINK_ORPESTATE_SELF_TOPICID, "ORPE TGT state");
#endif


//ORPE Topics
//Topic<bool> orpeTakeImage; //Publishing to this will make ORPE send an image back.
Topic<bool> enableORPE(ORPE_ENABLE, "ORPE Enable manager"); //Publish to this to enable for disable ORPE.
Topic<ORPEState_t> orpeState(ORPE_STATE, "ORPE State manager"); //This is where new information of ORPEs state will be published. The state will be updated periodically and async for sudden changes.
Topic<HTransform_F> orpeRelativePose(ORPE_RELATIVEPOSE, "ORPE relative pose estimations manager"); //This is where pose estimations will be published once valid.
Topic<OrpeTelemetry> orpeTelemetry(ORPE_TELEMETRY, "ORPE telemetry.");


//ORPE buffers and subscribers
CommBuffer<ORPEState_t> orpeSelfState;

Subscriber orpeSelfStateSub(orpeSelfSttTopic, orpeSelfState);


//Callback functions for control logic

void callbackFuncEnableORPE(bool& enable) {

    ORPECommand cmd;
    cmd.command = enable ? ORPECommandType_t::ORPECommandType_Start : ORPECommandType_t::ORPECommandType_Stop;
    orpeSelfCmdTopic.publish(cmd);

}
SubscriberReceiver<bool> enableORPERecv(enableORPE, callbackFuncEnableORPE);

void callbackFuncSelfORPEState(ORPEState_t& state) {

    orpeState.publish(state);

}
SubscriberReceiver<ORPEState_t> selfORPEStateRecv(orpeSelfSttTopic, callbackFuncSelfORPEState);

void callbackFuncSelfORPETelemetry(OrpeTelemetry& telemetry) {

    orpeTelemetry.publish(telemetry); //Forward for the rest of the system

    if (telemetry.valid) {

        //Tranform from rotation vector to angleaxis
        Vector3D_F rotVec(telemetry.ax, telemetry.ay, telemetry.az); //sqrtf(telemetry.ax*telemetry.ax + telemetry.ay*telemetry.ay + telemetry.az*telemetry.az);

        HTransform_F pose (
            AngleAxis_F(rotVec.getLen(), rotVec),
            Vector3D_F(telemetry.px/1000, telemetry.py/1000, telemetry.pz/1000)
        );

        orpeRelativePose.publish(pose);
        
    }

}
SubscriberReceiver<OrpeTelemetry> selfORPETmtRecv(orpeSelfTmtTopic, callbackFuncSelfORPETelemetry);



//Class to setup, communicate and control ORPE via datalink.
class ORPEManager : StaticThread<> {
private:

    CommBuffer<ORPEState_t> orpeStateBuf_;
    CommBuffer<HTransform_F> orpePoseBuf_;

    Subscriber orpeStateSubscriber_;
    Subscriber orpePoseSubscriber_;


public:

    ORPEManager() : 
        StaticThread<>("ORPE Manager"),
        orpeStateSubscriber_(orpeState, orpeStateBuf_),
        orpePoseSubscriber_(orpeRelativePose, orpePoseBuf_)
    {}


    void init() override {

        //Add all topics to gateway for communication with ORPE
        datalinkGateway.addTopicsToForward(&orpeSelfTmtTopic);
        datalinkGateway.addTopicsToForward(&orpeSelfCmdTopic);
        datalinkGateway.addTopicsToForward(&orpeSelfSttTopic);

        datalinkGateway.addTopicsToForward(&orpeTgtTmtTopic);
        datalinkGateway.addTopicsToForward(&orpeTgtCmdTopic);
        datalinkGateway.addTopicsToForward(&orpeTgtSttTopic);

    }


    void run() override {

        ORPECommand cmd;
        cmd.command = ORPECommandType_t::ORPECommandType_Startup;
        orpeSelfCmdTopic.publish(cmd);

        ORPEState_t state;
        HTransform_F pose;

        int64_t lastORPEInfo = NOW();

        while (1) { 
            
            if (orpeStateBuf_.getOnlyIfNewData(state)) {

                lastORPEInfo = NOW();
                
                if (state != ORPEState_t::ORPEState_Running) {
                    enableORPE.publish(true);
                }

            }

            if (orpePoseBuf_.getOnlyIfNewData(pose)) {

                lastORPEInfo = NOW();

            }
                

            if (NOW() - lastORPEInfo > 2*SECONDS) { //attempt to startup ORPE if we cant see any state updates
                lastORPEInfo = NOW();

                cmd.command = ORPECommandType_t::ORPECommandType_Startup;
                orpeSelfCmdTopic.publish(cmd);      

            }

            suspendCallerUntil(NOW() + 1000*MILLISECONDS);

        }

    }

};

//Instance of the ORPE manager to control ORPE and receive telemetry.
ORPEManager orpeManager;

}