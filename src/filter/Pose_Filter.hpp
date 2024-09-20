#ifndef TMW_POSE_FILTER_HPP_
#define TMW_POSE_FILTER_HPP_

#include "rodos.h"
#include "matlib.h"
#include "Datastruct.h"

#include "../Sub_Recv_Object.hpp"



/**
 * @author Christopher Steffen
 * @date 07.2024 
 * 
 * @brief This filter takes relative position/attitude measurements from ORPE and the radio based sensor systems and combines them to a single best output.
 */

namespace RODOS {



/// @brief The topic to where the filter attitude estimations are published.
extern Topic<Quaternion_F> filterAttitudeTopic;
/// @brief The topic to where the filter attitude estimations are published.
extern Topic<Vector3D_F> filterPositionTopic;


class PoseFilter : StaticThread<> {
private:



    Vector3D_F processPositionCov_ = Vector3D_F(0.01, 0.01, 0.1);  //The covariance of the model. Usually the same in all axis'.
    Vector3D_F processAttitudeCov_ = Vector3D_F(1, 1, 1);   //The covariance of the model. Usually the same in all axis'.
    
    Vector3D_F orpePositionCovPerc_ = Vector3D_F(0.1, 0.1, 2);   //The covariance of orpes position estimation. Usually higher in the direction of the target satellite (simplified by saying its higher in the z axis) 
    Vector3D_F orpeAttitudeCovPerc_ = Vector3D_F(1, 1, 1);    //The covariance of orpes attitude estimation.

    Vector3D_F uwbPositionCov_ = Vector3D_F(1, 1, 0.1);  //The covariance of uwb radio based position estimations. Usually higher than orpe position estimation.


    SubscriberObjRecv<OrpeTelemetry, PoseFilter> orpePoseRecv_;
    SubscriberObjRecv<OrpeTelemetry, PoseFilter> orpePoseRecvOpposite_;
    SubscriberObjRecv<Vector3D_F, PoseFilter> uwbPosRecv_;


    Semaphore newDataSem_;

    OrpeTelemetry orpeData_;
    OrpeTelemetry orpeDataOpposite_;
    Vector3D_F uwbData_;

    bool orpeNewData_ = false;
    bool orpeNewDataOpposite_ = false;
    bool uwbNewData_ = false;

    Quaternion_F attitude_;
    Vector3D_F position_;

    bool firstPosition_ = true;
    bool firstAttitude_ = true;


public:

    PoseFilter(Topic<OrpeTelemetry>& orpePoseTopic, Topic<OrpeTelemetry>& orpeOppositePoseTopic, Topic<Vector3D_F>& uwbPositionTopic);


    void init() override;

    /// @brief Where the actual processing happens.
    void run() override;

    void setPose(Vector3D_F position, Quaternion_F attitude) {
        newDataSem_.enter();    
        position_ = position;
        attitude_ = attitude;
        newDataSem_.leave();
    }


private:

    /// @brief Fuses the orpe telemetry data into the filter.
    /// @param orpeEst 
    void fuseOrpeData(OrpeTelemetry& orpeEst);

    /// @brief Fuses the uwb data into the filter.
    /// @param uwbEst
    void fuseUwbData(Vector3D_F& uwbEst);

    /// @brief To be called when new data is available. Does all filter calculations.
    void processNewData();

    /// @brief Receives the pose estmiations from orpe from topic.
    /// @param orpeEst 
    void orpeEstRecv(OrpeTelemetry& orpeEst);

    /// @brief Receives the pose estmiations from opposite orpe from topic.
    /// @param orpeEst 
    void orpeEstRecvOpposite(OrpeTelemetry& orpeEst);
    
    /// @brief Receives the position estimations from uwb from topic. 
    /// @param uwbEst 
    void uwbEstRecv(Vector3D_F& uwbEst);


};


extern PoseFilter globalEstimationFilter;

}


#endif
