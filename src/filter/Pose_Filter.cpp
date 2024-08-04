#include "rodos.h"

#include "../orpe/ORPE_Manager.hpp"

#include "Pose_Filter.hpp"



namespace RODOS {


/// @brief The topic to where the filter attitude estimations are published.
Topic<Quaternion_F> filterAttitudeTopic(-1, "Pose Filter Output Attitude");
/// @brief The topic to where the filter attitude estimations are published.
Topic<Vector3D_F> filterPositionTopic(-1, "Pose Filter Output Position");


Topic<Vector3D_F> uwbDummyTopic(-1, "UWB dummy topic so it compiles");


/// @brief Global filter object for use by other systems
extern PoseFilter globalEstimationFilter(ORPETMW::orpeRelativePose, uwbDummyTopic);



PoseFilter::PoseFilter(Topic<HTransform_F>& orpePoseTopic, Topic<Vector3D_F>& uwbPositionTopic) :
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

    //First we check which sensor has new data.
    bool orpeNew = false;
    auto orpePosition = orpeData_.getTranslation();
    auto orpeAttitude = Quaternion_F(orpeData_.getRotation());
    if (orpePosition.getLen() > 1.0f/100) // ORPE sensor does not work at 1cm, so we use this as a threshold to determine if the data is not zero.
        orpeNew = true;

    bool uwbNew = false;
    auto uwbPosition = uwbData_;
    if (uwbPosition.getLen() > 1.0f/100) //Same as orpe, below 1cm does not matter anymore.
        uwbNew = true;

    
    //Next we extract the better of the two sensors and use them for the filter output and publish them
    if (orpeNew) { //Better and therefore overrides the uwb sensor.

        attitude_ = orpeAttitude;
        position_ = orpePosition;

        filterAttitudeTopic.publish(attitude_);
        filterPositionTopic.publish(position_);

    } else if (uwbNew) {
        
        position_ = uwbPosition;

        filterPositionTopic.publish(position_);

    }


    //End by settings datas to zero to flag them as not new. Not possible for position of either sensor to become zero.
    orpeData_ = HTransform_F();
    uwbData_ = Vector3D_F();

}


void PoseFilter::orpeEstRecv(HTransform_F& orpeEst) {
    orpeData_ = orpeEst;
    this->resume();
}

void PoseFilter::uwbEstRecv(Vector3D_F& uwbEst) {
    uwbData_ = uwbEst;
    this->resume();
}





}
