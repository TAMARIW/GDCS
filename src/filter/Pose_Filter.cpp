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
PoseFilter globalEstimationFilter(ORPETMW::orpeTelemetry, ORPETMW::oppositeORPETelemetry, uwbPositionTopic);



PoseFilter::PoseFilter(Topic<OrpeTelemetry>& orpePoseTopic, Topic<OrpeTelemetry>& orpeOppositePoseTopic, Topic<Vector3D_F>& uwbPositionTopic) :
    StaticThread<>("PoseFilter", 1000),
    orpePoseRecv_(orpePoseTopic, &PoseFilter::orpeEstRecv, this),
    orpePoseRecvOpposite_(orpeOppositePoseTopic, &PoseFilter::orpeEstRecvOpposite, this),
    uwbPosRecv_(uwbPositionTopic, &PoseFilter::uwbEstRecv, this)
{}


void PoseFilter::init() {

    attitude_ = Quaternion_F(1, 0, 0, 0);
    position_ = Vector3D_F(0, 0, 0);

}

void PoseFilter::run() {

    while (1) {

        processNewData();

        suspendCallerUntil(NOW() + 10 * MILLISECONDS);

    }

}

void PoseFilter::processNewData() {

    //First we gather new information from the sensors
    newDataSem_.enter();
    bool orpeNew = orpeNewData_;
    bool orpeOppositeNew = orpeNewDataOpposite_;
    bool uwbNew = uwbNewData_;

    OrpeTelemetry orpeTele = orpeData_;
    OrpeTelemetry orpeTeleOpposite = orpeDataOpposite_;
    Vector3D_F uwbPosition = uwbData_;

    orpeNewData_ = false;
    orpeNewDataOpposite_ = false;
    uwbNewData_ = false;
    newDataSem_.leave();

    //Next we do a simple check to see if the data is valid.
    orpeNew &= orpeTele.valid; //If the data is not valid, we ignore it.
    orpeOppositeNew &= orpeTeleOpposite.valid; //If the data is not valid, we ignore it.

    if (orpeOppositeNew) { //Due to the opposite orpe estimating the inverted coodinate transformation, we need to invert it completely.
        
        //Get the homogeneous transformation matrix from the opposite ORPE.
        Vector3D_F rotVec(orpeTeleOpposite.ax, orpeTeleOpposite.ay, orpeTeleOpposite.az); 
        HTransform_F pose (
            AngleAxis_F(rotVec.getLen(), rotVec),
            Vector3D_F(orpeTeleOpposite.px, orpeTeleOpposite.py, orpeTeleOpposite.pz)
        );

        //Now invert the transformation matrix.
        pose = pose.invert();

        //Now we extract the rotation vector and position from the inverted matrix and place it back in the ORPE data.
        auto pos = pose.getTranslation();
        auto rot = pose.getRotation().toAngleAxis();
        orpeTeleOpposite.px = pos.x;
        orpeTeleOpposite.py = pos.y;
        orpeTeleOpposite.pz = pos.z;

        orpeTeleOpposite.ax = -rot.u.x * rot.phi;
        orpeTeleOpposite.ay = -rot.u.y * rot.phi;
        orpeTeleOpposite.az = -rot.u.z * rot.phi;

    }

    if (uwbPosition.getLen() > 500 || uwbPosition.getLen() < 2)  //If the data is out of bounds, we ignore it.
        uwbNew = false;
    
    if (orpeNew && firstAttitude_) {

        Vector3D_F orpeRot(orpeTele.ax, orpeTele.ay, orpeTele.az); 
        Vector3D_F rotAxis = orpeRot.normalize();
        float rotAngle = orpeRot.getLen();
        attitude_ = Quaternion_F(rotAngle, rotAxis);

        firstAttitude_ = false;

    } else if (orpeOppositeNew && firstAttitude_) {

        Vector3D_F orpeRot(orpeTeleOpposite.ax, orpeTeleOpposite.ay, orpeTeleOpposite.az); 
        Vector3D_F rotAxis = orpeRot.normalize();
        float rotAngle = orpeRot.getLen();
        attitude_ = Quaternion_F(rotAngle, rotAxis);

        firstAttitude_ = false;

    }

    if (orpeNew && firstPosition_) {

        Vector3D_F orpePos(orpeTele.px/1000, orpeTele.py/1000, orpeTele.pz/1000);
        position_ = orpePos;

        firstPosition_ = false;

    } else if (orpeOppositeNew && firstPosition_) {

        Vector3D_F orpePos(orpeTeleOpposite.px/1000, orpeTeleOpposite.py/1000, orpeTeleOpposite.pz/1000);
        position_ = orpePos;

        firstPosition_ = false;

    }

    if (uwbNew && firstPosition_) {

        position_ = uwbPosition;

        firstPosition_ = false;

    }

    //If we have new data, we can start the filter calculations.
    if (orpeNew) {

        //PRINTF("ORPE data: %f, %f, %f, %f, %f, %f\n", orpeTele.px, orpeTele.py, orpeTele.pz, orpeTele.ax, orpeTele.ay, orpeTele.az);

        fuseOrpeData(orpeTele);

    }

    if (orpeOppositeNew) {

        //PRINTF("ORPE data opposite: %f, %f, %f, %f, %f, %f\n", orpeTeleOpposite.px, orpeTeleOpposite.py, orpeTeleOpposite.pz, orpeTeleOpposite.ax, orpeTeleOpposite.ay, orpeTeleOpposite.az);

        fuseOrpeData(orpeTeleOpposite);

    }


    //Next we do the same for the UWB sensor.
    if (uwbNew) {

        //PRINTF("UWB data: %f, %f, %f\n", uwbPosition.x, uwbPosition.y, uwbPosition.z);

        fuseUwbData(uwbPosition);

    }

    //Do a quick check to see if the data is valid. If any position is NaN, we set it to zero. Same for the attitude.
    if (isnan(position_.x) || isnan(position_.y) || isnan(position_.z)) {
        position_ = Vector3D_F(0, 0, 0);
    }
    if (isnan(attitude_.q0) || isnan(attitude_.q.x) || isnan(attitude_.q.y) || isnan(attitude_.q.z)) {
        attitude_ = Quaternion_F(1, 0, 0, 0);
    }


    //Lastly we publish the new data.
    if (orpeNew || orpeOppositeNew) {
        filterAttitudeTopic.publish(attitude_);
        filterPositionTopic.publish(position_);
    } else if (uwbNew) {
        filterPositionTopic.publish(position_);
    }

}


void PoseFilter::fuseOrpeData(OrpeTelemetry& orpeTele) {

    Vector3D_F orpeRot(orpeTele.ax, orpeTele.ay, orpeTele.az); 
    Vector3D_F orpePos(orpeTele.px/1000, orpeTele.py/1000, orpeTele.pz/1000);
    
    //Determine the covariance of the sensors.
    //This is modelled as a value for the distance and tangent to the target satellite, as ORPE has good lateral accuracy but poor depth accuracy and UWB has good depth accuracy but poor lateral accuracy.
    //This is a simplification but will greatly improve the filter efficiency.

    //For ORPE the covariance is modelled as percentage of the distance.
    auto dist = orpePos.getLen();
    Vector3D_F orpePositionCov_ = dist * orpePositionCovPerc_;   //The covariance of orpes position estimation calculated from the percentage of the distance.
    Vector3D_F orpeAttitudeCov_ = Vector3D_F(1, 1, 1);    //The covariance of orpes attitude estimation. Remains constant.
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

    //attitude_.print();
    Vector3D_F rotAxis = orpeRot.normalize();
    float rotAngle = orpeRot.getLen();
    Quaternion_F orpeRotQuat = Quaternion_F(AngleAxis_F(rotAngle, rotAxis));

    //Simply average the two quaternions.
    auto newAttitude = (orpeRotQuat * 1/orpeAttCov + attitude_ * 1/attitudeNoiseCov_) / (1/orpeAttCov + 1/attitudeNoiseCov_);
    newAttitude = newAttitude.normalize();

    attitude_ = newAttitude;

}

void PoseFilter::fuseUwbData(Vector3D_F& uwbPosition) {

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

void PoseFilter::orpeEstRecv(OrpeTelemetry& orpeEst) {
    newDataSem_.enter();
    orpeData_ = orpeEst;
    orpeNewData_ = true;
    newDataSem_.leave();
    this->resume();
}

void PoseFilter::orpeEstRecvOpposite(OrpeTelemetry& orpeEst) {
    newDataSem_.enter();
    orpeDataOpposite_ = orpeEst;
    orpeNewDataOpposite_ = true;
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
