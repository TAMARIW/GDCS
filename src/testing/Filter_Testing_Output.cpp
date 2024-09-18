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
            
            attNew |= attitudeBuf_.getOnlyIfNewData(attitude);
            posNew |= positionBuf_.getOnlyIfNewData(position);

            if (posNew && attNew) {

                PRINTF("Both position and attitude are new.\n");
                PRINTF("Attiude: ");
                attitude.print();
                PRINTF("Position: ");
                position.print();
                PRINTF("\n");

            } else if (posNew) {

                PRINTF("Position is new.\n");
                PRINTF("Position: ");
                position.print();
                PRINTF("\n");

            } else if (attNew) {

                PRINTF("Attitude is new.\n");
                PRINTF("Attitude: ");
                attitude.print();
                PRINTF("\n");

            }

        }

    }

};


//FilterOutputTesting filterOutputTesting (filterAttitudeTopic, filterPositionTopic);


}
