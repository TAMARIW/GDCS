#include "rodos.h"


//Class to simply print that the program is still running
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

};// imAlive;
