#include "rodos.h"

#include "../orpe/ORPE_Manager.hpp"
#include "../radio_pose/decaWaveTopics.hpp"

#include "Pose_Filter.hpp"



namespace RODOS {


/// @brief The topic to where the filter attitude estimations are published.
Topic<Quaternion_F> filterAttitudeTopic(-1, "Pose Filter Output Attitude");
/// @brief The topic to where the filter attitude estimations are published.
Topic<Vector3D_F> filterPositionTopic(-1, "Pose Filter Output Position");


/// @brief Global filter object for use by other systems
PoseFilter globalEstimationFilter(ORPETMW::orpeTelemetry, uwbPositionTopic);



PoseFilter::PoseFilter(Topic<OrpeTelemetry>& orpePoseTopic, Topic<Vector3D_F>& uwbPositionTopic) :
    StaticThread<>("PoseFilter", 1000),
    orpePoseRecv_(orpePoseTopic, &PoseFilter::orpeEstRecv, this),
    uwbPosRecv_(uwbPositionTopic, &PoseFilter::uwbEstRecv, this)
{}


void PoseFilter::init() {

    attitude_ = Quaternion_F(1, 0, 0, 0);
    position_ = Vector3D_F(0, 0, 0);

}

void PoseFilter::run() {

    while (1) {

        processNewData();

        suspendCallerUntil(END_OF_TIME);

    }

}

void PoseFilter::processNewData() {

    //First we gather new information from the sensors
    newDataSem_.enter();
    bool orpeNew = orpeNewData_;
    bool uwbNew = uwbNewData_;

    OrpeTelemetry orpeTele = orpeData_;
    Vector3D_F uwbPosition = uwbData_;

    orpeNewData_ = false;
    uwbNewData_ = false;
    newDataSem_.leave();

    //Next we do a simple check to see if the data is valid.
    orpeNew &= orpeTele.valid; //If the data is not valid, we ignore it.

    if (uwbPosition.getLen() > 500 || uwbPosition.getLen() < 2)  //If the data is out of bounds, we ignore it.
        uwbNew = false;
    

    //If we have new data, we can start the filter calculations.
    if (orpeNew) {

        Vector3D_F orpeRot(orpeTele.ax, orpeTele.ay, orpeTele.az); 
        Vector3D_F orpePos(orpeTele.px/1000, orpeTele.py/1000, orpeTele.pz/1000);
        
        //Determine the covariance of the sensors.
        //This is modelled as a value for the distance and tangent to the target satellite, as ORPE has good lateral accuracy but poor depth accuracy and UWB has good depth accuracy but poor lateral accuracy.
        //This is a simplification but will greatly improve the filter efficiency.
        float orpeDepthCov = orpePositionCov_.z;
        float orpeLateralCov = sqrt(orpePositionCov_.x * orpePositionCov_.y);

        float orpeAttCov = sqrt(orpeAttitudeCov_.x * orpeAttitudeCov_.y * orpeAttitudeCov_.z); //Simplified to a single value for the attitude covariance.

        float processDepthCov = processPositionCov_.z;
        float processLateralCov = sqrt(processPositionCov_.x * processPositionCov_.y);

        float attitudeNoiseCov_ = sqrt(processAttitudeCov_.x * processAttitudeCov_.y * processAttitudeCov_.z);


        //Next we calculate the weighted average.
        position_ = Vector3D_F(
            (orpePos.x * 1/orpeLateralCov + position_.x * 1/processLateralCov) / (1/orpeLateralCov + 1/processLateralCov),
            (orpePos.y * 1/orpeLateralCov + position_.y * 1/processLateralCov) / (1/orpeLateralCov + 1/processLateralCov),
            (orpePos.z * 1/orpeDepthCov + position_.z * 1/processDepthCov) / (1/orpeDepthCov + 1/processDepthCov)
        );


        auto currentRot = attitude_.getVec() * attitude_.getAngle();
        auto newRot = (currentRot*(1/processAttitudeCov_.getLen()) + orpeRot*(1/orpeAttCov)) / (1/attitudeNoiseCov_ + 1/orpeAttCov);
        attitude_ = Quaternion_F(newRot.getLen(), newRot);

    }


    //Next we do the same for the UWB sensor.
    if (uwbNew) {

        float uwbDepthCov = uwbPositionCov_.z;
        float uwbLateralCov = sqrt(uwbPositionCov_.x * uwbPositionCov_.y);

        float processDepthCov = processPositionCov_.z;
        float processLateralCov = sqrt(processPositionCov_.x * processPositionCov_.y);

        //Next we calculate the weighted average.
        position_ = Vector3D_F(
            (uwbPosition.x * 1/uwbLateralCov + position_.x * 1/processLateralCov) / (1/uwbLateralCov + 1/processLateralCov),
            (uwbPosition.y * 1/uwbLateralCov + position_.y * 1/processLateralCov) / (1/uwbLateralCov + 1/processLateralCov),
            (uwbPosition.z * 1/uwbDepthCov + position_.z * 1/processDepthCov) / (1/uwbDepthCov + 1/processDepthCov)
        );

    }


    //Lastly we publish the new data.
    if (orpeNew) {
        filterAttitudeTopic.publish(attitude_);
        filterPositionTopic.publish(position_);
    } else if (uwbNew) {
        filterPositionTopic.publish(position_);
    }

}


void PoseFilter::orpeEstRecv(OrpeTelemetry& orpeEst) {
    newDataSem_.enter();
    orpeData_ = orpeEst;
    orpeNewData_ = true;
    newDataSem_.leave();
    this->resume();
}

void PoseFilter::uwbEstRecv(Vector3D_F& uwbEst) {
    newDataSem_.enter();
    uwbData_ = uwbEst;
    uwbNewData_ = true;
    newDataSem_.leave();
    this->resume();
}





}
