#ifndef ORPE_MANAGER_HPP
#define ORPE_MANAGER_HPP


#include "rodos.h"
#include "matlib.h"

#include "Datastruct.h"

#include "Datalink.hpp"


//Settings for the ORPE datalink.
#define SWITCH_ORPE_COMMS //Uncomment to switch the SELF and TGT ORPE for communication

#define DATALINK_ORPETELEMETRY_SELF_TOPICID     1300 //ORPE pose estimations from the same satellite. (From itsself)
#define DATALINK_ORPETELECOMMAND_SELF_TOPICID   1301 //ORPE telecommands to ORPE on same satellite. (To itsself)
#define DATALINK_ORPESTATE_SELF_TOPICID         1302 //ORPE state from the same satellite. (From itsself)

#define DATALINK_ORPETELEMETRY_TGT_TOPICID      1310 //ORPE pose estimations from the target satellite. (From other satellite)
#define DATALINK_ORPETELECOMMAND_TGT_TOPICID    1311 //ORPE telecommands to the other satellite. (To other satellite) 
#define DATALINK_ORPESTATE_TGT_TOPICID          1312 //ORPE pose estimations from the target satellite. (From other satellite)

#define ORPE_ENABLE         1200 //ORPE enable commands from manager
#define ORPE_STATE          1201 //ORPE state publishes from manager
#define ORPE_RELATIVEPOSE   1202 //ORPE relative pose estimations from manager
#define ORPE_TELEMETRY      1203 //ORPE telemetry direct from ORPE


// Namespace for encapsulation
namespace ORPETMW {

//ORPE Topics
//extern Topic<bool> orpeTakeImage; //Publishing to this will make ORPE send an image back.
extern Topic<bool> enableORPE; //Publish to this to enable for disable ORPE.
extern Topic<ORPEState_t> orpeState; //This is where new information of ORPEs state will be published. The state will be updated periodically and async for sudden changes.
extern Topic<HTransform_F> orpeRelativePose; //This is where pose estimations will be published once valid.
extern Topic<OrpeTelemetry> orpeTelemetry; //This is where the raw direct telemetry packets are published from ORPE. They contain advanced info on ORPE.

}

#endif