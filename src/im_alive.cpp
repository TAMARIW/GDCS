#include "rodos.h"

#include "Datastruct.h"

#include "Datalink.hpp"


//Class to setup and control the datalink stuff
class ImAlive : StaticThread<> {
private:



public:

    ImAlive() : StaticThread<>("ImAlive", 5000) {

    }

    void init() override {

    }


    void run() override {

        while (1) {
            PRINTF("I'm alive!. Time %.1f\n", SECONDS_NOW());
            suspendCallerUntil(NOW() + 1000 * MILLISECONDS);
        }

    }

} imAlive;
