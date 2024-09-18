#include "rodos.h"
#include "Datastruct.h"

#include "../Datalink.hpp"

#include "LED_Thread.hpp"

using namespace ORPE;


ORPE_LEDControl::ORPE_LEDControl(int64_t frameInterval_ns) : 
    StaticThread<>("LED Thread", 1000)
{

	frameInterval_ns_ = frameInterval_ns;
    currentFrame_ = 0;
  
}

void ORPE_LEDControl::setupAllLEDs() {

#ifndef LED_IDENT_TESTING

    this->setupLED(0, LEDPIN_1, 1, 0);
    this->setupLED(1, LEDPIN_2, 2, 10);
    this->setupLED(2, LEDPIN_3, 3, 20);
    this->setupLED(3, LEDPIN_4, 4, 30);
    this->setupLED(4, LEDPIN_5, 5, 40);
    this->setupLED(5, LEDPIN_6, 6, 50);

    this->setupLED(6, LEDPIN_7, 7, 0);
    this->setupLED(7, LEDPIN_8, 8, 10);
    this->setupLED(8, LEDPIN_9, 9, 20);
    this->setupLED(9, LEDPIN_10, 10, 30);
    this->setupLED(10, LEDPIN_11, 11, 40);
    this->setupLED(11, LEDPIN_12, 12, 50);

    setLEDMode(LEDCodingMode_t::LEDCodingMode_Continuous, 0);
    setLEDMode(LEDCodingMode_t::LEDCodingMode_Illuminate, 1);
    setLEDMode(LEDCodingMode_t::LEDCodingMode_Illuminate, 2);
    setLEDMode(LEDCodingMode_t::LEDCodingMode_Continuous, 3);
    setLEDMode(LEDCodingMode_t::LEDCodingMode_Continuous, 4);
    setLEDMode(LEDCodingMode_t::LEDCodingMode_Continuous, 5);

    setLEDMode(LEDCodingMode_t::LEDCodingMode_Continuous, 6);
    setLEDMode(LEDCodingMode_t::LEDCodingMode_Continuous, 7);
    setLEDMode(LEDCodingMode_t::LEDCodingMode_Continuous, 8);
    setLEDMode(LEDCodingMode_t::LEDCodingMode_Continuous, 9);
    setLEDMode(LEDCodingMode_t::LEDCodingMode_Continuous, 10);
    setLEDMode(LEDCodingMode_t::LEDCodingMode_Off, 11);

    /*setLEDMode(LEDCodingMode_t::LEDCodingMode_Off, 6);
    setLEDMode(LEDCodingMode_t::LEDCodingMode_Off, 7);
    setLEDMode(LEDCodingMode_t::LEDCodingMode_Off, 8);
    setLEDMode(LEDCodingMode_t::LEDCodingMode_Off, 9);
    setLEDMode(LEDCodingMode_t::LEDCodingMode_Off, 10);
    setLEDMode(LEDCodingMode_t::LEDCodingMode_Off, 11);*/
    

#else 

    // LED Ident testing
    this->setupLED(0, LEDPIN_1, 0b10000000, 0);
    this->setupLED(1, LEDPIN_2, 0b11000000, 0);
    this->setupLED(2, LEDPIN_3, 0b11100000, 0);
    this->setupLED(3, LEDPIN_4, 0b11110000, 0);
    this->setupLED(4, LEDPIN_5, 0b11111000, 0);
    this->setupLED(5, LEDPIN_6, 0b11111100, 0);
    this->setupLED(6, LEDPIN_7, 0b11111110, 0);
    this->setupLED(7, LEDPIN_8, 0b11111111, 0);

    frameInterval_ns_ = 1*SECONDS;
    codeRepeat = 15;

#endif

}

void ORPE_LEDControl::setupLED(size_t ledIndex, GPIO_PIN ledGPIO, uint8_t ledID, size_t startFrame) {
    
    //Set GPIO pin
	gpio_[ledIndex] = HAL_GPIO(ledGPIO);
    gpio_[ledIndex].init(true, 1, 0);

    //Run semaphore constructor
    ledSemaphore_[ledIndex] = Semaphore();

    //Set State and mode to default, also lock the semaphore
    ledSemaphore_[ledIndex].enter();
    ledState_[ledIndex] = LEDCodingState_t::LEDCodingState_Idle;
    ledModeBuffer_[ledIndex] = ledMode_[ledIndex] = LEDCodingMode_t::LEDCodingMode_Continuous;
    ledSemaphore_[ledIndex].leave();

    //Set start frame
    ledStartFrame_[ledIndex] = startFrame%codeRepeat; //Make sure the start frame is within the code repeat range

    //Set binary bits and calculate parity bits
    /*size_t parity0 = 0;
	size_t parity1 = 0;
    ledCode_[ledIndex] = ledID;
	for (size_t i = 0; i < 8; i++) { //Count the number of even and odd bits

        if (i%2 == 1) //Even or odd bit?
            parity0 += 0b00000001 & (ledID>>i);
        else
            parity1 += 0b00000001 & (ledID>>i);

	}

    //Set parity bits
    ledCode_[ledIndex] |= (parity0%2) << 8;
    ledCode_[ledIndex] |= (parity1%2) << 9;*/

    //Use hamming library to generate hamming code from ledID.
    ledCode_[ledIndex] = dataToHamming(ledID); //Hamming code is 7 bits long.

    uint8_t ledCode = ledCode_[ledIndex];
    auto ret = hammingToData(ledCode);

    PRINTF("Index %d, ID %d: hammingcode %d, decode %d, ret %d\n", ledIndex, ledID, int(ledCode_[ledIndex]), int(ledCode), ret);

}


void ORPE_LEDControl::ledModeStateMachine(size_t ledIndex) {

    switch (ledMode_[ledIndex])
    {
    case LEDCodingMode_t::LEDCodingMode_Off:
        {
            ledCodingStateMachine(ledIndex, false, true);
        }
        break;

    case LEDCodingMode_t::LEDCodingMode_Illuminate:
        {
            ledCodingStateMachine(ledIndex, false, false);
        }
        break;

    case LEDCodingMode_t::LEDCodingMode_Continuous:
        {
            
            if (ledState_[ledIndex] == LEDCodingState_t::LEDCodingState_Idle && ledStartFrame_[ledIndex] == currentFrame_) {
                ledCodingStateMachine(ledIndex, true, false);
            } else {
                ledCodingStateMachine(ledIndex, false, false);
            }

        }
        break;

    case LEDCodingMode_t::LEDCodingMode_Single:
        {
            if (ledState_[ledIndex] == LEDCodingState_t::LEDCodingState_Idle && ledStartFrame_[ledIndex] == currentFrame_) {
                ledCodingStateMachine(ledIndex, true, false);
                ledMode_[ledIndex] = LEDCodingMode_t::LEDCodingMode_Illuminate;
            } else {
                ledCodingStateMachine(ledIndex, false, false);
            }
        }
        break;
    
    default:
        ledMode_[ledIndex] = LEDCodingMode_t::LEDCodingMode_Off;
        break;
    }

}


void ORPE_LEDControl::ledCodingStateMachine(size_t ledIndex, bool beginCoding, bool ledOff) {

    switch (ledState_[ledIndex])
    {
    case LEDCodingState_t::LEDCodingState_Idle :
        {

            if (beginCoding) {
                gpio_[ledIndex].setPins(0);
                ledState_[ledIndex] = LEDCodingState_t::LEDCodingState_Binary;
                ledFrame_[ledIndex] = 0;
            } else {
                gpio_[ledIndex].setPins(!ledOff);
            }

        }
        break;

    case LEDCodingState_t::LEDCodingState_Binary :
        {

            gpio_[ledIndex].setPins(0b00000001 & (ledCode_[ledIndex] >> (ledFrame_[ledIndex])));
            ledFrame_[ledIndex]++;

            if (ledFrame_[ledIndex] >= 8) { //Binary coding is finished. Switch to idle
                ledState_[ledIndex] = LEDCodingState_t::LEDCodingState_Idle;
            }
            
        }
        break;
    
    default:
        break;
    }

}


void ORPE_LEDControl::runtimeIDLogic() {

    /*bool connected = false;
    if (datalinkConnected_.getOnlyIfNewData(connected)) {

        for (int i = 0; i < 12; i++) 
            setLEDMode(connected ? LEDCodingMode_t::LEDCodingMode_Continuous : LEDCodingMode_t::LEDCodingMode_Off, i);

    }*/

    /*OrpeTelemetry orpeTele;
    if (teleBuf_.getOnlyIfNewData(orpeTele) && orpeTele.valid) {

        lastORPETeleRecvTime_ = NOW();

        if (ledMode_[0] != LEDCodingMode_t::LEDCodingMode_Illuminate) {
            for (int i = 0; i < 12; i++) 
                setLEDMode(LEDCodingMode_t::LEDCodingMode_Illuminate, i);
        }

    }

    if (NOW() - lastORPETeleRecvTime_ > 1*SECONDS && ledMode_[0] != LEDCodingMode_t::LEDCodingMode_Continuous) {
        for (int i = 0; i < 12; i++) 
            setLEDMode(LEDCodingMode_t::LEDCodingMode_Continuous, i);
    }*/

}


void ORPE_LEDControl::init() {

    setupAllLEDs();

}


void ORPE_LEDControl::run() {

    //setupAllLEDs();

	int64_t deadline = NOW();

	while(1) {

        runtimeIDLogic();

		for (size_t i = 0; i < NUMLEDS; i++) {
            
            // Update the led mode
            //ledSemaphore_[i].enter();
            ledMode_[i] = ledModeBuffer_[i];
            //ledSemaphore_[i].leave();

            // Run the state machine
			ledModeStateMachine(i);

		}

        //Increment the frame counter and reset if necessary.
		currentFrame_++;
		if (currentFrame_ >= codeRepeat) currentFrame_ = 0;

        //Wait until the next frame interval. Skip if we are already late
		deadline = NOW() - NOW() % frameInterval_ns_ + frameInterval_ns_;
		suspendCallerUntil(deadline);

	}

}


void ORPE_LEDControl::setLEDMode(LEDCodingMode_t mode, size_t ledIndex) {

    ledSemaphore_[ledIndex].enter();
    ledModeBuffer_[ledIndex] = mode;
    ledSemaphore_[ledIndex].leave();

}

bool ORPE_LEDControl::isLEDCoding(size_t ledIndex) {

    return ledState_[ledIndex] == LEDCodingState_t::LEDCodingState_Binary;

}


/**
 * @todo set correct frameInterval 
*/
ORPE_LEDControl LED_THREAD(SECONDS/LED_FPS);
