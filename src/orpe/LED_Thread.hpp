#ifndef ORPE_LED_THREAD_HPP_
#define ORPE_LED_THREAD_HPP_

#include "rodos.h"

#include "Datastruct.h"

#include "../hamming/HammingCodes.h"

/**
 * Pins of the LEDs
 * The outer LEDs have the following shape:
 *  1              5
 *      3  []  4
 *
 *  2              6
 *
 * The inner LEDS have the following shape:
 * 7         11
 *      9
 *
 * 8         12
 *      10
 *
 */
#define LEDPIN_1 GPIO_050
#define LEDPIN_2 GPIO_048
#define LEDPIN_3 GPIO_049
#define LEDPIN_4 GPIO_019
#define LEDPIN_5 GPIO_051
#define LEDPIN_6 GPIO_015

#define LEDPIN_7 GPIO_014
#define LEDPIN_8 GPIO_055
#define LEDPIN_9 GPIO_059
#define LEDPIN_10 GPIO_077
#define LEDPIN_11 GPIO_031
#define LEDPIN_12 GPIO_079


/**
 * Coding settings
*/
#define LED_FPS 10
//#define LED_IDENT_TESTING


/**
 * @brief Namespace used by ORPE (Optical relative pose estimation)
 */
namespace ORPE {

//Number of LEDs to control.
constexpr size_t NUMLEDS = 12;

/**
 * @brief This class will control the LEDs for Optical Relative Pose Estimation.
 */
class ORPE_LEDControl : public RODOS::StaticThread<> {
public:

    enum LEDCodingMode_t {
        /// @brief Will keep the LED powered Off
        LEDCodingMode_Off,
        /// @brief Will repeat the LED coding continuously according to setup coding pattern
        LEDCodingMode_Continuous,
        /// @brief Will trigger the LED to do its coding once according to setup coding pattern
        LEDCodingMode_Single,
        /// @brief Will keep the led powered on.
        LEDCodingMode_Illuminate
    };

private:

    enum LEDCodingState_t {
        LEDCodingState_Idle,
        LEDCodingState_Binary
    };

	///@brief Number of frames until the code repeats.
	size_t codeRepeat = 60;

    /// @brief The frame at which this LED starts coding.
    size_t ledStartFrame_[NUMLEDS];
    /// @brief The 10 bit binary code including the parity bits.
    uint16_t ledCode_[NUMLEDS];
    /// @brief The set mode for the led.
    LEDCodingMode_t ledMode_[NUMLEDS];
    /// @brief The current state of the LED coding.
    LEDCodingState_t ledState_[NUMLEDS];
    /// @brief The current frame of the led.
    size_t ledFrame_[NUMLEDS];

    /// @brief What the led mode was changed to.
    LEDCodingMode_t ledModeBuffer_[NUMLEDS];
    /// @brief The semaphore to protect the led modes during setLEDMode calls.
    Semaphore ledSemaphore_[NUMLEDS];

    /// @brief The current coding frame for synchronisation with settings.
    size_t currentFrame_ = 0;

	/// @brief Each led GPIO.
	HAL_GPIO gpio_[NUMLEDS];

	/// @brief The time between each frame.
	int64_t frameInterval_ns_ = END_OF_TIME;

    /// @brief Sets up the leds
    void setupAllLEDs();

public:

	ORPE_LEDControl(int64_t frameInterval_ns);

	/**
	 * @brief Sets up an led using the given GPIO pin and LEDID.
     * @note ONLY CALL ONCE FOR EACH LED. Not thread safe.
	 * @param ledIndex The LED index, from 0 to NUMLED.
	 * @param ledPin The GPIO pin to use for this led.
	 * @param ledID What ID to give this LED. Only values from 0 to 15 are valid.
	 * @param startFrame How many frames to wait before starting first code.
	 */
	void setupLED(size_t ledIndex, GPIO_PIN ledGPIO, uint8_t ledID, size_t startFrame);

    /**
     * @brief Sets the mode for the LED. Can be used to change the led coding system according to external parameters. (e.g. distance, no tracking, etc.)
     * @note This function is thread safe.
     * @param mode The mode to set for the LED.
     * @param ledIndex The index of which LED to set the mode for.
     */
    void setLEDMode(LEDCodingMode_t mode, size_t ledIndex);

    /**
     * @returns true if the LED is currently coding.
     */
    bool isLEDCoding(size_t ledIndex);

private:

    /**
     * @brief Does the state machine logic for LED modes.
     * @param ledIndex The index of which LED this state machine belongs to.
    */
    void ledModeStateMachine(size_t ledIndex);

    /**
     * @brief Does the state machine logic for the led coding.
     * @param ledIndex The index of which LED this state machine belongs to.
     * @param beginCoding If set to true, the led will begin its coding on this function call (Led will turn off signaling coding start)
     * @param ledPower Will force LED to turn off if set to true.
    */
    void ledCodingStateMachine(size_t ledIndex, bool beginCoding, bool ledOff);

    void runtimeIDLogic();

	void init() override;

	void run() override;

};

}

#endif
