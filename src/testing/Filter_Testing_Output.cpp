#include "rodos.h"

#include "../Sub_Recv_Object.hpp"

#include "../filter/Pose_Filter.hpp"



namespace RODOS {


class FilterOutputTesting : StaticThread<> {
private:

    //SubscriberObjRecv<Quaternion_F, FilterOutputTesting> attitudeSubr_;
    //SubscriberObjRecv<Vector3D_F, FilterOutputTesting> positionSubr_;

    CommBuffer<Quaternion_F> attitudeBuf_;
    CommBuffer<Vector3D_F> positionBuf_;

    Subscriber attitudeSubr_;
    Subscriber positionSubr_;

public:

    FilterOutputTesting(Topic<Quaternion_F>& attitudeTopic, Topic<Vector3D_F>& positionTopic) :
        StaticThread<>("FilterOutputTesting"),
        attitudeSubr_(attitudeTopic, attitudeBuf_, "Attitude Subr for Filter output testing"),
        positionSubr_(positionTopic, positionBuf_, "Position Subr for Filter output testing")
    {}

    void init() override {



    }

    void run() override {

        Quaternion_F attitude;
        Vector3D_F position;

        bool attNew = false;
        bool posNew = false;

        while (true) {
            
            attNew = attitudeBuf_.getOnlyIfNewData(attitude);
            posNew = positionBuf_.getOnlyIfNewData(position);

            //Print the attitude and position and if they are new
            if (attNew || posNew)
                PRINTF("Att: new %d, %f %f %f %f \t Pos %d, %f, %f, %f \n", attNew, attitude.q0, attitude.q.x, attitude.q.y, attitude.q.z, posNew, position.x, position.y, position.z);

            suspendCallerUntil(NOW() + 10 * MILLISECONDS);

        }

    }

};


FilterOutputTesting filterOutputTesting (filterAttitudeTopic, filterPositionTopic);


}
