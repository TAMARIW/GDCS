/*! ----------------------------------------------------------------------------
 * @file	deca_device_api.h
 * @brief	DW1000 API Functions
 *
 * @attention
 *
 * Copyright 2013 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 */

#ifndef _DECA_DEVICE_API_H_
#define _DECA_DEVICE_API_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DWT_SUCCESS (0)
#define DWT_ERROR   (-1)

#define DWT_TIME_UNITS          (1.0/499.2e6/128.0) //!< = 15.65e-12 s

#define DWT_DEVICE_ID   (0xDECA0130) 		//!< DW1000 MP device ID

//! constants for selecting the bit rate for data TX (and RX)
//! These are defined for write (with just a shift) the TX_FCTRL register
#define DWT_BR_110K		0	//!< UWB bit rate 110 kbits/s
#define DWT_BR_850K		1	//!< UWB bit rate 850 kbits/s
#define DWT_BR_6M8		2	//!< UWB bit rate 6.8 Mbits/s

//! constants for specifying the (Nominal) mean Pulse Repetition Frequency
//! These are defined for direct write (with a shift if necessary) to CHAN_CTRL and TX_FCTRL regs
#define DWT_PRF_16M		1	//!< UWB PRF 16 MHz
#define DWT_PRF_64M		2	//!< UWB PRF 64 MHz

//! constants for specifying Preamble Acquisition Chunk (PAC) Size in symbols
#define DWT_PAC8		0	//!< PAC  8 (recommended for RX of preamble length  128 and below
#define DWT_PAC16		1	//!< PAC 16 (recommended for RX of preamble length  256
#define DWT_PAC32		2	//!< PAC 32 (recommended for RX of preamble length  512
#define DWT_PAC64		3	//!< PAC 64 (recommended for RX of preamble length 1024 and up

//! constants for specifying TX Preamble length in symbols
//! These are defined to allow them be directly written into byte 2 of the TX_FCTRL register
//! (i.e. a four bit value destined for bits 20..18 but shifted left by 2 for byte alignment)
#define DWT_PLEN_4096	0x0C	//! Standard preamble length 4096 symbols
#define DWT_PLEN_2048	0x28	//! Non-standard preamble length 2048 symbols
#define DWT_PLEN_1536	0x18	//! Non-standard preamble length 1536 symbols
#define DWT_PLEN_1024	0x08	//! Standard preamble length 1024 symbols
#define DWT_PLEN_512	0x34	//! Non-standard preamble length 512 symbols
#define DWT_PLEN_256	0x24	//! Non-standard preamble length 256 symbols
#define DWT_PLEN_128	0x14	//! Non-standard preamble length 128 symbols
#define DWT_PLEN_64		0x04	//! Standard preamble length 64 symbols

//! callback events
#define DWT_SIG_RX_NOERR			0
#define DWT_SIG_TX_DONE             1		// Frame has been sent
#define DWT_SIG_RX_OKAY             2       // Frame Received with Good CRC
#define DWT_SIG_RX_ERROR            3       // Frame Received but CRC is wrong
#define DWT_SIG_RX_TIMEOUT          4       // Timeout on receive has elapsed
#define DWT_SIG_TX_AA_DONE			6		// ACK frame has been sent (as a result of auto-ACK)

#define DWT_SIG_RX_PHR_ERROR        8       // Error found in PHY Header
#define DWT_SIG_RX_SYNCLOSS         9       // Un-recoverable error in Reed Solomon Decoder
#define DWT_SIG_RX_SFDTIMEOUT       10      // Saw preamble but got no SFD within configured time
#define DWT_SIG_RX_PTOTIMEOUT		11		// Got preamble detection timeout (no preamble detected)
#define DWT_SIG_TX_PENDING			12		// Delayed TX is pending
#define DWT_SIG_TX_ERROR			13      // TX failed
#define DWT_SIG_RX_PENDING 			14 		// RX has been re-enabled

#define DWT_SFDTOC_DEF				0x1041	// default SFD timeout value

#define DWT_PHRMODE_STD				0x0		// standard PHR mode
#define DWT_PHRMODE_EXT				0x3		// DW proprietary extended frames PHR mode

// Defined constants for "mode" bitmask parameter passed into dwt_starttx() function.
#define DWT_START_TX_IMMEDIATE      0
#define DWT_START_TX_DELAYED        1
#define DWT_RESPONSE_EXPECTED		2


//frame filtering configuration options
#define DWT_FF_NOTYPE_EN			0x000			// no frame types allowed (FF disabled)
#define DWT_FF_COORD_EN				0x002			// behave as coordinator (can receive frames with no dest address (PAN ID has to match))
#define DWT_FF_BEACON_EN			0x004			// beacon frames allowed
#define DWT_FF_DATA_EN				0x008			// data frames allowed
#define DWT_FF_ACK_EN				0x010			// ack frames allowed
#define DWT_FF_MAC_EN				0x020			// mac control frames allowed
#define DWT_FF_RSVD_EN				0x040			// reserved frame types allowed

//DW1000 interrupt events
#define DWT_INT_TFRS			0x00000080			// frame sent
#define DWT_INT_LDED            0x00000400			// micro-code has finished execution
#define DWT_INT_RFCG			0x00004000			// frame received with good CRC
#define DWT_INT_RPHE			0x00001000			// receiver PHY header error
#define DWT_INT_RFCE			0x00008000			// receiver CRC error
#define DWT_INT_RFSL			0x00010000			// receiver sync loss error
#define DWT_INT_RFTO			0x00020000			// frame wait timeout
#define DWT_INT_RXOVRR			0x00100000			// receiver overrun
#define DWT_INT_RXPTO			0x00200000			// preamble detect timeout
#define DWT_INT_SFDT			0x04000000			// SFD timeout
#define DWT_INT_ARFE			0x20000000			// frame rejected (due to frame filtering configuration)


//DW1000 SLEEP and WAKEUP configuration parameters
#define DWT_PRESRV_SLEEP 0x0100                      // PRES_SLEEP - on wakeup preserve sleep bit
#define DWT_LOADOPSET    0x0080						 // ONW_L64P - on wakeup load operating parameter set for 64 PSR
#define DWT_CONFIG       0x0040						 // ONW_LDC - on wakeup restore (load) the saved configurations (from AON array into HIF)
#define DWT_TANDV        0x0001                      // ONW_RADC - on wakeup run ADC to sample temperature and voltage sensor values

#define DWT_XTAL_EN		 0x10						// keep XTAL running during sleep
#define DWT_WAKE_SLPCNT  0x8						// wake up after sleep count
#define DWT_WAKE_CS      0x4						// wake up on chip select
#define DWT_WAKE_WK      0x2						// wake up on WAKEUP PIN
#define DWT_SLP_EN       0x1						// enable sleep/deep sleep functionality

//DW1000 INIT configuration parameters
#define DWT_LOADUCODE     0x1
#define DWT_LOADNONE	  0x0

//DW1000 OTP operating parameter set selection
#define DWT_OPSET_64LEN   0x0
#define DWT_OPSET_TIGHT   0x1
#define DWT_OPSET_DEFLT   0x2

typedef struct{

	uint32_t status;      //initial value of register as ISR is entered
	uint8_t  event;		//event type
	uint8_t  aatset;		//auto ACK TX bit is set
	uint16_t datalength;	//length of frame
	uint8_t  fctrl[2];	//frame control bytes
	uint8_t  dblbuff ;	//set if double buffer is enabled

}dwt_callback_data_t;


typedef enum {
	 CHAN_CTRL_TXCHAN_1 = 0x01,	/* Selects the transmit channel 1 */
	 CHAN_CTRL_TXCHAN_2 = 0x02,	/* Selects the transmit channel 2 */
	 CHAN_CTRL_TXCHAN_3 = 0x03,	/* Selects the transmit channel 3 */
	 CHAN_CTRL_TXCHAN_4 = 0x04,	/* Selects the transmit channel 4 */
	 CHAN_CTRL_TXCHAN_5 = 0x05,	/* Selects the transmit channel 5 */
	 CHAN_CTRL_TXCHAN_7 = 0x07	/* Selects the transmit channel 7 */
}eCHAN;


/*! ------------------------------------------------------------------------------------------------------------------
 * Structure typedef: dwt_config_t
 *
 * Structure for setting device configuration via dwt_configure() function
 *
 */

#pragma pack(1)
typedef struct
{
    uint8_t chan ;           //!< channel number {1, 2, 3, 4, 5, 7 }
    uint8_t prf ;            //!< Pulse Repetition Frequency {DWT_PRF_16M or DWT_PRF_64M}
    uint8_t txPreambLength ; //!< DWT_PLEN_64..DWT_PLEN_4096
    uint8_t rxPAC ;          //!< Acquisition Chunk Size (Relates to RX preamble length)
    uint8_t txCode ;         //!< TX preamble code
    uint8_t rxCode ;         //!< RX preamble code
    uint8_t nsSFD ;          //!< Boolean should we use non-standard SFD for better performance
    uint8_t dataRate ;       //!< Data Rate {DWT_BR_110K, DWT_BR_850K or DWT_BR_6M8}
    uint8_t phrMode ;        //!< PHR mode {0x0 - standard DWT_PHRMODE_STD, 0x3 - extended frames DWT_PHRMODE_EXT}
    uint16_t sfdTO ;         //!< SFD timeout value (in symbols)
}__attribute__ ((packed))  dwt_config_t ;
#pragma pack()


typedef struct
{
	uint8_t	PGdly;
	//TX POWER
	//31:24		BOOST_0.125ms_PWR
	//23:16		BOOST_0.25ms_PWR-TX_SHR_PWR
	//15:8		BOOST_0.5ms_PWR-TX_PHR_PWR
	//7:0		DEFAULT_PWR-TX_DATA_PWR
	uint32_t	power;
}
dwt_txconfig_t ;


typedef struct
{

	uint16_t		maxNoise ;			// LDE max value of noise
	uint16_t		firstPathAmp1 ;	    // Amplitude at floor(index FP) + 1
	uint16_t      stdNoise ;			// Standard deviation of noise
	uint16_t		firstPathAmp2 ;		// Amplitude at floor(index FP) + 2
	uint16_t		firstPathAmp3 ;		// Amplitude at floor(index FP) + 3
	uint16_t		maxGrowthCIR ;		// Channel Impulse Response max growth CIR
    uint16_t      rxPreamCount ;      // Count of preamble symbols accumulated
    uint16_t      firstPath ;         // First path index (10.6 bits fixed point integer)
}dwt_rxdiag_t ;


typedef struct
{
	//all of the below are mapped to a 12-bit register in DW1000
    uint16_t PHE ;					//number of received header errors
	uint16_t RSL ;					//number of received frame sync loss events
    uint16_t CRCG ;					//number of good CRC received frames
    uint16_t CRCB ;					//number of bad CRC (CRC error) received frames
	uint16_t ARFE ;					//number of address filter errors
	uint16_t OVER ;					//number of receiver overflows (used in double buffer mode)
    uint16_t SFDTO ;					//SFD timeouts
    uint16_t PTO ;					//Preamble timeouts
    uint16_t RTO ;					//RX frame wait timeouts
    uint16_t TXF ;					//number of transmitted frames
	uint16_t HPW ;					//half period warn
    uint16_t TXW ;					//power up warn

} dwt_deviceentcnts_t ;


/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_getpartid()
 *
 * @brief This is used to return the read part ID of the device
 *
 * input parameters
 *
 * output parameters
 *
 * returns the 32 bit part ID value as programmed in the factory
 */
uint32_t dwt_getpartid(void);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_getlotid()
 *
 * @brief This is used to return the read lot ID of the device
 *
 * input parameters
 *
 * output parameters
 *
 * returns the 32 bit lot ID value as programmed in the factory
 */
uint32_t dwt_getlotid(void);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_readdevid()
 *
 * @brief This is used to return the read device type and revision information of the DW1000 device (MP part is 0xDECA0130)
 *
 * input parameters
 *
 * output parameters
 *
 * returns the read value which for DW1000 is 0xDECA0130
 */
uint32_t dwt_readdevid(uint8_t spi_num);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_otprevision()
 *
 * @brief This is used to return the read OTP revision
 *
 * input parameters
 *
 * output parameters
 *
 * returns the read OTP revision value
 */
uint8_t dwt_otprevision(void);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_setGPIOforEXTTRX()
 *
 * @brief This is used to enable GPIO for external LNA or PA functionality - HW dependent, consult the DW1000 User Manual
 *
 * input parameters
 * @param spi_num - ID for spi interface (equal to used DW1000 device entity) ( 0 / 1 )
 *
 * output parameters
 *
 * no return value
 */
void dwt_setGPIOforEXTTRX(uint8_t spi_num);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_setGPIOdirection()
 *
 * @brief This is used to set GPIO direction as an input (1) or output (0)
 *
 * input parameters
 * @param spi_num    -   ID for spi interface (equal to used DW1000 device entity) ( 0 / 1 )
 * @param gpioNum    -   this is the GPIO to configure - see GxM0... GxM8 in the deca_regs.h file
 * @param direction  -   this sets the GPIO direction - see GxP0... GxP8 in the deca_regs.h file
 *
 * output parameters
 *
 * no return value
 */
void dwt_setGPIOdirection(uint8_t spi_num, uint32_t gpioNum, uint32_t direction);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_setGPIOvalue()
 *
 * @brief This is used to set GPIO value as (1) or (0) only applies if the GPIO is configured as output
 *
 * input parameters
 * @param spi_num    -  ID for spi interface (equal to used DW1000 device entity) ( 0 / 1 )
 * @param gpioNum    -   this is the GPIO to configure - see GxM0... GxM8 in the deca_regs.h file
 * @param value  -   this sets the GPIO value - see GDP0... GDP8 in the deca_regs.h file
 *
 * output parameters
 *
 * no return value
 */
void dwt_setGPIOvalue(uint8_t spi_num, uint32_t gpioNum, uint32_t value);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_initialise()
 *
 * @brief This function initiates communications with the DW1000 transceiver
 * and reads its DEV_ID register (address 0x00) to verify the IC is one supported
 * by this software (e.g. DW1000 32-bit device ID value is 0xDECA0130).  Then it
 * does any initial once only device configurations needed for use and initialises
 * as necessary any static data items belonging to this low-level driver.
 *
 * NOTES:
 * 1.this function needs to be run before dwt_configuresleep, also the SPI frequency has to be < 3MHz
 * 2.it also reads and applies LDO tune and crystal trim values from OTP memory
 *
 * input parameters
 * @param spi_num   - ID of the spi interface (equal to the used DW1000 device entity) ( 0 / 1 )
 * @param config    -   specifies what configuration to load
 *                  DWT_LOADUCODE     0x1 - load the LDE microcode from ROM - enabled accurate RX timestamp
 *                  DWT_LOADNONE      0x0 - do not load any values from OTP memory
 *
 * output parameters
 *
 * returns DWT_SUCCESS for success, or DWT_ERROR for error
 */
int dwt_initialise(uint8_t spi_num, uint16_t config) ;

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_configure()
 *
 * @brief This function provides the main API for the configuration of the
 * DW1000 and this low-level driver.  The input is a pointer to the data structure
 * of type dwt_config_t that holds all the configurable items.
 * The dwt_config_t structure shows which ones are supported
 *
 * input parameters
 * @param spi_num   -   ID for spi interface (equal to used DW1000 device entity) ( 0 / 1 )
 * @param config    -   pointer to the configuration structure, which contains the device configuration data.
 *
 * output parameters
 *
 * returns DWT_SUCCESS for success, or DWT_ERROR for error
 */
int dwt_configure(uint8_t spi_num, dwt_config_t *config) ;

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_configuretxrf()
 *
 * @brief This function provides the API for the configuration of the TX spectrum
 * including the power and pulse generator delay. The input is a pointer to the data structure
 * of type dwt_txconfig_t that holds all the configurable items.
 *
 * input parameters
 * @param spi_num   -   ID for spi interface (equal to used DW1000 device entity) ( 0 / 1 )
 * @param config    -   pointer to the txrf configuration structure, which contains the tx rf config data
 *
 * output parameters
 *
 * no return value
 */
void dwt_configuretxrf(uint8_t spi_num, dwt_txconfig_t *config) ;

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_setrxantennadelay()
 *
 * @brief This API function writes the antenna delay (in time units) to RX registers
 *
 * input parameters:
 * @param spi_num   -   ID for spi interface (equal to used DW1000 device entity) ( 0 / 1 )
 * @param rxDelay - this is the total (RX) antenna delay value, which
 *                          will be programmed into the RX register
 *
 * output parameters
 *
 * no return value
 */
void dwt_setrxantennadelay(uint8_t spi_num, uint16_t antennaDly);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_settxantennadelay()
 *
 * @brief This API function writes the antenna delay (in time units) to TX registers
 *
 * input parameters:
 * @param spi_num   -   ID for spi interface (equal to used DW1000 device entity) ( 0 / 1 )
 * @param txDelay - this is the total (TX) antenna delay value, which
 *                          will be programmed into the TX delay register
 *
 * output parameters
 *
 * no return value
 */
void dwt_settxantennadelay(uint8_t spi_num, uint16_t antennaDly);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_setsmarttxpower()
 *
 * @brief This call enables or disables the smart TX power feature.
 *
 * input parameters
 * @param spi_num   -   ID for spi interface (equal to used DW1000 device entity) ( 0 / 1 )
 * @param enable - this enables or disables the TX smart power (1 = enable, 0 = disable)
 *
 * output parameters
 *
 * no return value
 */
void dwt_setsmarttxpower(uint8_t spi_num, int enable);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_writetxdata()
 *
 * @brief This API function writes the supplied TX data into the DW1000's
 * TX buffer.  The input parameters are the data length in bytes and a pointer
 * to those data bytes.
 *
 * input parameters
 * @param spi_num   -   ID for spi interface (equal to used DW1000 device entity) ( 0 / 1 )
 * @param txFrameLength  - This is the total frame length, including the two byte CRC.
 *                         Note: this is the length of TX message (including the 2 byte CRC) - max is 1023
 *                         standard PHR mode allows up to 127 bytes
 *                         if > 127 is programmed, DWT_PHRMODE_EXT needs to be set in the phrMode configuration
 *                         see dwt_configure function
 * @param txFrameBytes   - Pointer to the user�s buffer containing the data to send.
 * @param txBufferOffset - This specifies an offset in the DW1000�s TX Buffer at which to start writing data.
 *
 * output parameters
 *
 * returns DWT_SUCCESS for success, or DWT_ERROR for error
 */
int dwt_writetxdata(uint8_t spi_num, uint16_t txFrameLength, uint8_t *txFrameBytes, uint16_t txBufferOffset) ;

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_writetxfctrl()
 *
 * @brief This API function configures the TX frame control register before the transmission of a frame
 *
 * input parameters:
 * @param spi_num   -   ID for spi interface (equal to used DW1000 device entity) ( 0 / 1 )
 * @param txFrameLength - this is the length of TX message (including the 2 byte CRC) - max is 1023
 *                              NOTE: standard PHR mode allows up to 127 bytes
 *                              if > 127 is programmed, DWT_PHRMODE_EXT needs to be set in the phrMode configuration
 *                              see dwt_configure function
 * @param txBufferOffset - the offset in the tx buffer to start writing the data
 *
 * output parameters
 *
 * returns DWT_SUCCESS for success, or DWT_ERROR for error
 */
int dwt_writetxfctrl(uint8_t spi_num, uint16_t txFrameLength, uint16_t txBufferOffset) ;

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_starttx()
 *
 * @brief This call initiates the transmission, input parameter indicates which TX mode is used see below
 *
 * input parameters:
 * @param spi_num   -   ID for spi interface (equal to used DW1000 device entity) ( 0 / 1 )
 * @param mode - if 0 immediate TX (no response expected)
 *               if 1 delayed TX (no response expected)
 *               if 2 immediate TX (response expected - so the receiver will be automatically turned on after TX is done)
 *               if 3 delayed TX (response expected - so the receiver will be automatically turned on after TX is done)
 *
 * output parameters
 *
 * returns DWT_SUCCESS for success, or DWT_ERROR for error (e.g. a delayed transmission will fail if the delayed time has passed)
 */
int dwt_starttx(uint8_t spi_num, uint8_t mode) ;

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_setdelayedtrxtime()
 *
 * @brief This API function configures the delayed transmit time or the delayed RX on time
 *
 * input parameters
 * @param spi_num   -   ID for spi interface (equal to used DW1000 device entity) ( 0 / 1 )
 * @param starttime - the TX/RX start time (the 32 bits should be the high 32 bits of the system time at which to send the message,
 * or at which to turn on the receiver)
 *
 * output parameters none
 *
 * no return value
 */
void dwt_setdelayedtrxtime(uint8_t spi_num, uint32_t starttime) ;

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_readtxtimestamp()
 *
 * @brief This is used to read the TX timestamp (adjusted with the programmed antenna delay)
 *
 * input parameters
 * @param spi_num   -   ID for spi interface (equal to used DW1000 device entity) ( 0 / 1 )
 * @param timestamp - a pointer to a 5-byte buffer which will store the read TX timestamp time
 *
 * output parameters - the timestamp buffer will contain the value after the function call
 *
 * no return value
 */
void dwt_readtxtimestamp(uint8_t spi_num, uint8_t * timestamp);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_readtxtimestamphi32()
 *
 * @brief This is used to read the high 32-bits of the TX timestamp (adjusted with the programmed antenna delay)
 *
 * input parameters
 * @param spi_num   -   ID for spi interface (equal to used DW1000 device entity) ( 0 / 1 )
 *
 * output parameters
 *
 * returns high 32-bits of TX timestamp
 */
uint32_t dwt_readtxtimestamphi32(uint8_t spi_num);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_readtxtimestamplo32()
 *
 * @brief This is used to read the low 32-bits of the TX timestamp (adjusted with the programmed antenna delay)
 *
 * input parameters
 * @param spi_num   -   ID for spi interface (equal to used DW1000 device entity) ( 0 / 1 )
 *
 * output parameters
 *
 * returns low 32-bits of TX timestamp
 */
uint32_t dwt_readtxtimestamplo32(uint8_t spi_num);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_readrxtimestamp()
 *
 * @brief This is used to read the RX timestamp (adjusted time of arrival)
 *
 * input parameters
 * @param spi_num   -   ID for spi interface (equal to used DW1000 device entity) ( 0 / 1 )
 * @param timestamp - a pointer to a 5-byte buffer which will store the read RX timestamp time
 *
 * output parameters - the timestamp buffer will contain the value after the function call
 *
 * no return value
 */
void dwt_readrxtimestamp(uint8_t spi_num, uint8_t * timestamp);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_readrxtimestamphi32()
 *
 * @brief This is used to read the high 32-bits of the RX timestamp (adjusted with the programmed antenna delay)
 *
 * input parameters
 * @param spi_num   -   ID for spi interface (equal to used DW1000 device entity) ( 0 / 1 )
 *
 * output parameters
 *
 * returns high 32-bits of RX timestamp
 */
uint32_t dwt_readrxtimestamphi32(uint8_t spi_num);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_readrxtimestamplo32()
 *
 * @brief This is used to read the low 32-bits of the RX timestamp (adjusted with the programmed antenna delay)
 *
 * input parameters
 * @param spi_num   -   ID for spi interface (equal to used DW1000 device entity) ( 0 / 1 )
 *
 * output parameters
 *
 * returns low 32-bits of RX timestamp
 */
uint32_t dwt_readrxtimestamplo32(uint8_t spi_num);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_readsystimestamphi32()
 *
 * @brief This is used to read the high 32-bits of the system time
 *
 * input parameters
 * @param spi_num   -   ID for spi interface (equal to used DW1000 device entity) ( 0 / 1 )
 *
 * output parameters
 *
 * returns high 32-bits of system time timestamp
 */
uint32_t dwt_readsystimestamphi32(uint8_t spi_num);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_readsystime()
 *
 * @brief This is used to read the system time
 *
 * input parameters
 * @param spi_num   -   ID for spi interface (equal to used DW1000 device entity) ( 0 / 1 )
 * @param timestamp - a pointer to a 5-byte buffer which will store the read system time
 *
 * output parameters
 * @param timestamp - the timestamp buffer will contain the value after the function call
 *
 * no return value
 */
void dwt_readsystime(uint8_t spi_num, uint8_t * timestamp);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_checkoverrun()
 *
 * @brief This is used to check if the overrun condition is set in DW1000
 *
 * input parameters
 * @param spi_num   -   ID for spi interface (equal to used DW1000 device entity) ( 0 / 1 )
 *
 * output parameters
 *
 * returns 1 if the RXOVERR bit is set, else 0
 */
int dwt_checkoverrun(uint8_t spi_num);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_forcetrxoff()
 *
 * @brief This is used to turn off the transceiver
 *
 * input parameters
 * @param spi_num   -   ID for spi interface (equal to used DW1000 device entity) ( 0 / 1 )
 *
 * output parameters
 *
 * no return value
 */
void dwt_forcetrxoff(uint8_t spi_num);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_syncrxbufptrs()
 *
 * @brief this function synchronizes rx buffer pointers
 * need to make sure that the host/IC buffer pointers are aligned before starting RX
 *
 * input parameters:
 * @param spi_num   -   ID for spi interface (equal to used DW1000 device entity) ( 0 / 1 )
 *
 * output parameters
 *
 * no return value
 */
void dwt_syncrxbufptrs(uint8_t spi_num);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_rxenable()
 *
 * @brief This call turns on the receiver, can be immediate or delayed.
 * The receiver will stay turned on, listening to any messages until
 * it either receives a good frame, an error (CRC, PHY header, Reed Solomon) or  it times out (SFD, Preamble or Frame).
 *
 * input parameters
 * @param spi_num   -   ID for spi interface (equal to used DW1000 device entity) ( 0 / 1 )
 * @param delayed - TRUE the receiver is turned on after some delay (as programmed with dwt_setdelayedtime())
 *
 * returns DWT_SUCCESS for success, or DWT_ERROR for error (e.g. a delayed receive enable will be too far in the future if delayed time has passed (if delayed time is > 8s from now))
 */
int dwt_rxenable(uint8_t spi_num, int delayed) ;

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_setrxmode()
 *
 *  @brief enable different RX modes, e.g.:
 *   a) "snooze" mode, the receiver only listens periodically for preamble
 *   b) the RX PPDM "sniff" mode - receiver cycles through ON/OFF periods
 *
 * input parameters:
 * @param spi_num   -   ID for spi interface (equal to used DW1000 device entity) ( 0 / 1 )
 * @param mode      - DWT_RX_NORMAL = 0x0
 *                    DWT_RX_SNIFF  = 0x1      enable the rx PPDM "sniff" mode
 * @param rxON      - SNIFF mode ON period in PACs
 * @param rxOFF     - SNIFF mode OFF period in us (actually in 1.0256 micro second intervals)
 *
 * output parameters
 *
 * no return value
 */

#define DWT_RX_NORMAL (0x0)
#define DWT_RX_SNIFF  (0x1)

void dwt_setrxmode(uint8_t spi_num, int mode, uint8_t rxON, uint8_t rxOFF);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_setautorxreenable()
 *
 * @brief This call enables the auto RX re-enable feature
 *
 * input parameters
 * @param spi_num   -   ID for spi interface (equal to used DW1000 device entity) ( 0 / 1 )
 * @param enable - 1 to enable, 0 to disable the feature
 *
 * output parameters
 *
 * no return value
 */
void dwt_setautorxreenable(uint8_t spi_num, int enable);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_setdblrxbuffmode()
 *
 * @brief This call enables the double receive buffer mode
 *
 * input parameters
 * @param spi_num   -   ID for spi interface (equal to used DW1000 device entity) ( 0 / 1 )
 * @param enable - 1 to enable, 0 to disable the double buffer mode
 *
 * output parameters
 *
 * no return value
 */
void dwt_setdblrxbuffmode(uint8_t spi_num, int enable);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_setrxtimeout()
 *
 * @brief This call enables RX timeout (SY_STAT_RFTO event)
 *
 * input parameters
 * @param spi_num   -   ID for spi interface (equal to used DW1000 device entity) ( 0 / 1 )
 * @param time - how long the receiver remains on from the RX enable command
 *               The time parameter used here is in 1.0256 us (512/499.2MHz) units
 *               If set to 0 the timeout is disabled.
 *
 * output parameters
 *
 * no return value
 */
void dwt_setrxtimeout(uint8_t spi_num, uint16_t time);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_setpreambledetecttimeout()
 *
 * @brief This call enables preamble timeout (SY_STAT_RXPTO event)
 *
 * input parameters
 * @param spi_num   -   ID for spi interface (equal to used DW1000 device entity) ( 0 / 1 )
 * @param  timeout - in PACs
 *
 * output parameters
 *
 * no return value
 */
void dwt_setpreambledetecttimeout(uint8_t spi_num, uint16_t timeout);


/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_calibratesleepcnt()
 *
 * @brief calibrates the local oscillator as its frequency can vary between 7 and 13kHz depending on temp and voltage
 *
 * NOTE: this function needs to be run before dwt_configuresleepcnt, so that we know what the counter units are
 *
 * input parameters
 * @param spi_num   -   ID for spi interface (equal to used DW1000 device entity) ( 0 / 1 )
 *
 * output parameters
 *
 * returns the number of XTAL/2 cycles per low-power oscillator cycle. LP OSC frequency = 19.2 MHz/return value
 */
uint16_t dwt_calibratesleepcnt(uint8_t spi_num);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_configuresleepcnt()
 *
 * @brief sets the sleep counter to new value, this function programs the high 16-bits of the 28-bit counter
 *
 * NOTE: this function needs to be run before dwt_configuresleep, also the SPI frequency has to be < 3MHz
 *
 * input parameters
 * @param spi_num   -   ID for spi interface (equal to used DW1000 device entity) ( 0 / 1 )
 * @param sleepcnt - this it value of the sleep counter to program
 *
 * output parameters
 *
 * no return value
 */
 void dwt_configuresleepcnt(uint8_t spi_num, uint16_t sleepcnt);

 /*! ------------------------------------------------------------------------------------------------------------------
  * @fn dwt_configuresleep()
  *
  * @brief configures the device for both DEEP_SLEEP and SLEEP modes, and on-wake mode
  * i.e. before entering the sleep, the device should be programmed for TX or RX, then upon "waking up" the TX/RX settings
  * will be preserved and the device can immediately perform the desired action TX/RX
  *
  * NOTE: e.g. Tag operation - after deep sleep, the device needs to just load the TX buffer and send the frame
  *
  *
  *      mode: the array and LDE code (OTP/ROM) and LDO tune, and set sleep persist
  *      DWT_PRESRV_SLEEP 0x0100 - preserve sleep
  *      DWT_LOADOPSET    0x0080 - load operating parameter set on wakeup
  *      DWT_CONFIG       0x0040 - download the AON array into the HIF (configuration download)
  *      DWT_LOADEUI      0x0008
  *      DWT_GOTORX       0x0002
  *      DWT_TANDV        0x0001
  *
  *      wake: wake up parameters
  *      DWT_XTAL_EN      0x10 - keep XTAL running during sleep
  *      DWT_WAKE_SLPCNT  0x8 - wake up after sleep count
  *      DWT_WAKE_CS      0x4 - wake up on chip select
  *      DWT_WAKE_WK      0x2 - wake up on WAKEUP PIN
  *      DWT_SLP_EN       0x1 - enable sleep/deep sleep functionality
  *
  * input parameters
  * @param spi_num   -   ID for spi interface (equal to used DW1000 device entity) ( 0 / 1 )
  * @param mode - config on-wake parameters
  * @param wake - config wake up parameters
  *
  * output parameters
  *
  * no return value
  */
void dwt_configuresleep(uint8_t spi_num, uint16_t mode, uint8_t wake);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_entersleep()
 *
 * @brief This function puts the device into deep sleep or sleep. dwt_configuredeepsleep should be called first
 * to configure the sleep and on-wake/wake-up parameters
 *
 * input parameters
 * @param spi_num   -   ID for spi interface (equal to used DW1000 device entity) ( 0 / 1 )
 *
 * output parameters
 *
 * no return value
 */
void dwt_entersleep(uint8_t spi_num);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_entersleepaftertx(int enable)
 *
 * @brief sets the auto TX to sleep bit. This means that after a frame
 * transmission the device will enter deep sleep mode. The dwt_setdeepsleep() function
 * needs to be called before this to configure the on-wake settings
 *
 * NOTE: the IRQ line has to be low/inactive (i.e. no pending events)
 *
 * input parameters
 * @param spi_num   -   ID for spi interface (equal to used DW1000 device entity) ( 0 / 1 )
 * @param enable - 1 to configure the device to enter deep sleep after TX, 0 - disables the configuration
 *
 * output parameters
 *
 * no return value
 */
void dwt_entersleepaftertx(uint8_t spi_num, int enable);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_spicswakeup()
 *
 * @brief wake up the device from sleep mode using the SPI read,
 * the device will wake up on chip select line going low if the line is held low for at least 500us.
 * To define the length depending on the time one wants to hold
 * the chip select line low, use the following formula:
 *
 *      length (bytes) = time (s) * byte_rate (Hz)
 *
 * where fastest byte_rate is spi_rate (Hz) / 8 if the SPI is sending the bytes back-to-back.
 * To save time and power, a system designer could determine byte_rate value more precisely.
 *
 * NOTE: Alternatively the device can be waken up with WAKE_UP pin if configured for that operation
 *
 * input parameters
 * @param spi_num   -   ID for spi interface (equal to used DW1000 device entity) ( 0 / 1 )
 * @param buff   - this is a pointer to the dummy buffer which will be used in the SPI read transaction used for the WAKE UP of the device
 * @param length - this is the length of the dummy buffer
 *
 * output parameters
 *
 * returns DWT_SUCCESS for success, or DWT_ERROR for error
 */
int dwt_spicswakeup(uint8_t spi_num, uint8_t *buff, uint16_t length);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_setcallbacks()
 *
 * @brief This is the devices interrupt handler function, it will process/report status events
 *
 * input parameters
 * @param txcallback - the pointer to the TX call-back function
 * @param rxcallback - the pointer to the RX call-back function
 *
 * output parameters
 *
 * no return value
 */
void dwt_setcallbacks(void (*txcallback)(const dwt_callback_data_t *), void (*rxcallback)(const dwt_callback_data_t *));

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_checkIRQ()
 *
 * @brief This function checks if the IRQ line is active - this is used instead of interrupt handler
 *
 * input parameters
 * @param spi_num   -   ID for spi interface (equal to used DW1000 device entity) ( 0 / 1 )
 *
 * output parameters
 *
 * return value is 1 if the IRQS bit is set and 0 otherwise
 */
uint8_t dwt_checkIRQ(uint8_t spi_num);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_isr()
 *
 * @brief This is the devices interrupt handler function, it will process/report status events
 * Notes:  In PC based system using (Cheetah or ARM) USB to SPI converter there can be no interrupts, however we still need something
 *         to take the place of it and operate in a polled way.
 *         In an embedded system this function should be configured to launch on an interrupt, then it will process the interrupt trigger event and
 *         call a TX or RX call-back function depending on whether the event is a TX or RX event.
 *         The TX call-back will be called when a frame has been sent and the RX call-back when a frame has been received.
 *
 * input parameters
 * @param spi_num   -   ID for spi interface (equal to used DW1000 device entity) ( 0 / 1 )
 *
 * output parameters
 *
 * no return value
 */
void dwt_isr(uint8_t spi_num);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn void dwt_setinterrupt()
 *
 * @brief This function enables the specified events to trigger an interrupt.
 * The following events can be enabled:
 * DWT_INT_TFRS         0x00000080          // frame sent
 * DWT_INT_RFCG         0x00004000          // frame received with good CRC
 * DWT_INT_RPHE         0x00001000          // receiver PHY header error
 * DWT_INT_RFCE         0x00008000          // receiver CRC error
 * DWT_INT_RFSL         0x00010000          // receiver sync loss error
 * DWT_INT_RFTO         0x00020000          // frame wait timeout
 * DWT_INT_RXPTO        0x00200000          // preamble detect timeout
 * DWT_INT_SFDT         0x04000000          // SFD timeout
 * DWT_INT_ARFE         0x20000000          // frame rejected (due to frame filtering configuration)
 *
 *
 * input parameters:
 * @param spi_num   -   ID for spi interface (equal to used DW1000 device entity) ( 0 / 1 )
 * @param bitmask - sets the events which will generate interrupt
 * @param enable - if set the interrupts are enabled else they are cleared
 *
 * output parameters
 *
 * no return value
 */
void dwt_setinterrupt(uint8_t spi_num, uint32_t bitmask, uint8_t enable);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_setpanid()
 *
 * @brief This is used to set the PAN ID
 *
 * input parameters
 * @param spi_num   -   ID for spi interface (equal to used DW1000 device entity) ( 0 / 1 )
 * @param panID - this is the PAN ID
 *
 * output parameters
 *
 * no return value
 */
void dwt_setpanid(uint8_t spi_num, uint16_t panID);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_setaddress16()
 *
 * @brief This is used to set 16-bit (short) address
 *
 * input parameters
 * @param spi_num   -   ID for spi interface (equal to used DW1000 device entity) ( 0 / 1 )
 * @param shortAddress - this sets the 16 bit short address
 *
 * output parameters
 *
 * no return value
 */
void dwt_setaddress16(uint8_t spi_num, uint16_t shortAddress);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_seteui()
 *
 * @brief This is used to set the EUI 64-bit (long) address
 *
 * input parameters
 * @param spi_num   -   ID for spi interface (equal to used DW1000 device entity) ( 0 / 1 )
 * @param eui64 - this is the pointer to a buffer that contains the 64bit address
 *
 * output parameters
 *
 * no return value
 */
void dwt_seteui(uint8_t spi_num, uint8_t *eui64);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_geteui()
 *
 * @brief This is used to get the EUI 64-bit from the DW1000
 *
 * input parameters
 * @param spi_num   -   ID for spi interface (equal to used DW1000 device entity) ( 0 / 1 )
 * @param eui64 - this is the pointer to a buffer that will contain the read 64-bit EUI value
 *
 * output parameters
 *
 * no return value
 */
void dwt_geteui(uint8_t spi_num, uint8_t *eui64);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_otpread()
 *
 * @brief This is used to read the OTP data from given address into provided array
 *
 * input parameters
 * @param spi_num   -   ID for spi interface (equal to used DW1000 device entity) ( 0 / 1 )
 * @param address - this is the OTP address to read from
 * @param array - this is the pointer to the array into which to read the data
 * @param length - this is the number of 32 bit words to read (array needs to be at least this length)
 *
 * output parameters
 *
 * no return value
 */
void dwt_otpread(uint8_t spi_num, uint32_t address, uint32_t *array, uint8_t length);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_enableframefilter()
 *
 * @brief This is used to enable the frame filtering - (the default option is to
 * accept any data and ACK frames with correct destination address
 *
 * input parameters
 * @param spi_num   -   ID for spi interface (equal to used DW1000 device entity) ( 0 / 1 )
 * @param - bitmask - enables/disables the frame filtering options according to
 *      DWT_FF_NOTYPE_EN        0x000   no frame types allowed
 *      DWT_FF_COORD_EN         0x002   behave as coordinator (can receive frames with no destination address (PAN ID has to match))
 *      DWT_FF_BEACON_EN        0x004   beacon frames allowed
 *      DWT_FF_DATA_EN          0x008   data frames allowed
 *      DWT_FF_ACK_EN           0x010   ack frames allowed
 *      DWT_FF_MAC_EN           0x020   mac control frames allowed
 *      DWT_FF_RSVD_EN          0x040   reserved frame types allowed
 *
 * output parameters
 *
 * no return value
 */
void dwt_enableframefilter(uint8_t spi_num, uint16_t bitmask);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_enableautoack()
 *
 * @brief This call enables the auto-ACK feature. If the responseDelayTime (parameter) is 0, the ACK will be sent a.s.a.p.
 * otherwise it will be sent with a programmed delay (in symbols), max is 255.
 * NOTE: needs to have frame filtering enabled as well
 *
 * input parameters
 * @param spi_num   -   ID for spi interface (equal to used DW1000 device entity) ( 0 / 1 )
 * @param responseDelayTime - if non-zero the ACK is sent after this delay, max is 255.
 *
 * output parameters
 *
 * no return value
 */
void dwt_enableautoack(uint8_t spi_num, uint8_t responseDelayTime);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_setrxaftertxdelay()
 *
 * @brief This sets the receiver turn on delay time after a transmission of a frame
 *
 * input parameters
 * @param spi_num   -   ID for spi interface (equal to used DW1000 device entity) ( 0 / 1 )
 * @param rxDelayTime - (20 bits) - the delay is in UWB microseconds
 *
 * output parameters
 *
 * no return value
 */
void dwt_setrxaftertxdelay(uint8_t spi_num, uint32_t rxDelayTime);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_rxreset()
 *
 * @brief this function resets the receiver of the DW1000
 *
 * input parameters:
 * @param spi_num   -   ID for spi interface (equal to used DW1000 device entity) ( 0 / 1 )
 *
 * output parameters
 *
 * no return value
 */
void dwt_rxreset(uint8_t spi_num);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_softreset()
 *
 * @brief this function resets the DW1000
 *
 * input parameters:
 * @param spi_num   -   ID for spi interface (equal to used DW1000 device entity) ( 0 / 1 )
 *
 * output parameters
 *
 * no return value
 */
void dwt_softreset(uint8_t spi_num) ;

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_readrxdata()
 *
 * @brief This is used to read the data from the RX buffer, from an offset location give by offset parameter
 *
 * input parameters
 * @param spi_num   -   ID for spi interface (equal to used DW1000 device entity) ( 0 / 1 )
 * @param buffer - the buffer into which the data will be read
 * @param length - the length of data to read (in bytes)
 * @param rxBufferOffset - the offset in the rx buffer from which to read the data
 *
 * output parameters
 *
 * no return value
 */
void dwt_readrxdata(uint8_t spi_num, uint8_t *buffer, uint16_t length, uint16_t rxBufferOffset);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_readaccdata()
 *
 * @brief This is used to read the data from the Accumulator buffer, from an offset location give by offset parameter
 *
 * input parameters
 * @param spi_num   -   ID for spi interface (equal to used DW1000 device entity) ( 0 / 1 )
 * @param buffer - the buffer into which the data will be read
 * @param length - the length of data to read (in bytes)
 * @param accOffset - the offset in the acc buffer from which to read the data
 *
 * output parameters
 *
 * no return value
 */
void dwt_readaccdata(uint8_t spi_num, uint8_t *buffer, uint16_t length, uint16_t rxBufferOffset);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_readdiagnostics()
 *
 * @brief this function reads the RX signal quality diagnostic data
 *
 * input parameters
 * @param spi_num   -   ID for spi interface (equal to used DW1000 device entity) ( 0 / 1 )
 * @param diagnostics - diagnostic structure pointer, this will contain the diagnostic data read from the DW1000
 *
 * output parameters
 *
 * no return value
 */
void dwt_readdiagnostics(uint8_t spi_num, dwt_rxdiag_t * diagnostics);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_loadopsettabfromotp()
 *
 * @brief This is used to select which Operational Parameter Set table to load from OTP memory
 *
 * input parameters
 * @param spi_num   -   ID for spi interface (equal to used DW1000 device entity) ( 0 / 1 )
 * @param - opset table selection
 *                  DWT_OPSET_64LEN = 0x0 - load the operational parameter set table for 64 length preamble configuration
 *                  DWT_OPSET_TIGHT = 0x1 - load the operational parameter set table for tight xtal offsets (<1ppm)
 *                  DWT_OPSET_DEFLT = 0x2 - load the default operational parameter set table (this is loaded from reset)
 *
 * output parameters
 *
 * no return value
 */
void  dwt_loadopsettabfromotp(uint8_t spi_num, uint8_t gtab_sel);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_configeventcounters()
 *
 * @brief This is used to enable/disable the event counter in the IC
 *
 * input parameters
 * @param spi_num   -   ID for spi interface (equal to used DW1000 device entity) ( 0 / 1 )
 * @param - enable - 1 enables (and reset), 0 disables the event counters
 * output parameters
 *
 * no return value
 */
void dwt_configeventcounters(uint8_t spi_num, int enable);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_readeventcounters()
 *
 * @brief This is used to read the event counters in the IC
 *
 * input parameters
 * @param spi_num   -   ID for spi interface (equal to used DW1000 device entity) ( 0 / 1 )
 * @param counters - pointer to the dwt_deviceentcnts_t structure which will hold the read data
 *
 * output parameters
 *
 * no return value
 */
void dwt_readeventcounters(uint8_t spi_num, dwt_deviceentcnts_t *counters);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_otpwriteandverify()
 *
 * @brief This is used to program 32-bit value into the DW1000 OTP memory.
 *
 * input parameters
 * @param spi_num   -   ID for spi interface (equal to used DW1000 device entity) ( 0 / 1 )
 * @param value - this is the 32-bit value to be programmed into OTP
 * @param address - this is the 16-bit OTP address into which the 32-bit value is programmed
 *
 * output parameters
 *
 * returns DWT_SUCCESS for success, or DWT_ERROR for error
 */
uint32_t dwt_otpwriteandverify(uint8_t spi_num, uint32_t value, uint16_t address);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_setleds()
 *
 * @brief This is used to set up Tx/Rx GPIOs which could be used to control LEDs
 * Note: not completely IC dependent, also needs board with LEDS fitted on right I/O lines
 *       this function enables GPIOs 2 and 3 which are connected to LED3 and LED4 on EVB1000
 *
 * input parameters
 * @param spi_num   -   ID for spi interface (equal to used DW1000 device entity) ( 0 / 1 )
 * @param test - if 1 the LEDs will be enabled, if 0 the LED control is disabled.
 *             - if value is 2 the LEDs will flash once after enable.
 *
 * output parameters
 *
 * no return value
 */
void dwt_setleds(uint8_t spi_num, uint8_t test);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_xtaltrim()
 *
 * @brief This is used adjust the crystal frequency
 *
 * input parameters:
 * @param spi_num - ID for spi interface (equal to used DW1000 device entity) ( 0 / 1 )
 * @param   value - crystal trim value (in range 0x0 to 0x1F) 31 steps (~1.5ppm per step)
 *
 * @output
 *
 * no return value
 */
void dwt_xtaltrim(uint8_t spi_num, uint8_t value);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_configcwmode()
 *
 * @brief this function sets the DW1000 to transmit cw signal at specific channel frequency
 *
 * input parameters:
 * @param spi_num   -   ID for spi interface (equal to used DW1000 device entity) ( 0 / 1 )
 * @param chan - specifies the operating channel (e.g. 1, 2, 3, 4, 5, 6 or 7)
 *
 * output parameters
 *
 * returns DWT_SUCCESS for success, or DWT_ERROR for error
 */
int dwt_configcwmode(uint8_t spi_num, uint8_t chan);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_configcontinuousframemode()
 *
 * @brief this function sets the DW1000 to continuous tx frame mode for regulatory approvals testing.
 *
 * input parameters:
 * @param spi_num   -   ID for spi interface (equal to used DW1000 device entity) ( 0 / 1 )
 * @param framerepetitionrate - This is a 32-bit value that is used to set the interval between transmissions.
*  The minimum value is 4. The units are approximately 8 ns. (or more precisely 512/(499.2e6*128) seconds)).
 *
 * output parameters
 *
 * no return value
 */
void dwt_configcontinuousframemode(uint8_t spi_num, uint32_t framerepetitionrate);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_readtempvbat()
 *
 * @brief this function reads the battery voltage and temperature of the MP
 * The values read here will be the current values sampled by DW1000 AtoD converters.
 * Note on Temperature: the temperature value needs to be converted to give the real temperature
 * the formula is: 1.13 * reading - 113.0
 * Note on Voltage: the voltage value needs to be converted to give the real voltage
 * the formula is: 0.0057 * reading + 2.3
 *
 * NB: To correctly read the temperature this read should be done with xtal clock
 * however that means that the receiver will be switched off, if receiver needs to be on then
 * the timer is used to make sure the value is stable before reading
 *
 * input parameters:
 * @param spi_num   -   ID for spi interface (equal to used DW1000 device entity) ( 0 / 1 )
 * @param fastSPI - set to 1 if SPI rate > than 3MHz is used
 *
 * output parameters
 *
 * returns  (temp_raw<<8)|(vbat_raw)
 */
uint16_t dwt_readtempvbat(uint8_t spi_num, uint8_t fastSPI);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_readwakeuptemp()
 *
 * @brief this function reads the temperature of the DW1000 that was sampled
 * on waking from Sleep/Deepsleep. They are not current values, but read on last
 * wakeup if DWT_TANDV bit is set in mode parameter of dwt_configuresleep
 *
 * input parameters:
 * @param spi_num   -   ID for spi interface (equal to used DW1000 device entity) ( 0 / 1 )
 *
 * output parameters:
 *
 * returns: 8-bit raw temperature sensor value
 */
uint8_t dwt_readwakeuptemp(uint8_t spi_num) ;

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_readwakeupvbat()
 *
 * @brief this function reads the battery voltage of the DW1000 that was sampled
 * on waking from Sleep/Deepsleep. They are not current values, but read on last
 * wakeup if DWT_TANDV bit is set in mode parameter of dwt_configuresleep
 *
 * input parameters:
 * @param spi_num   -   ID for spi interface (equal to used DW1000 device entity) ( 0 / 1 )
 *
 * output parameters:
 *
 * returns: 8-bit raw battery voltage sensor value
 */
uint8_t dwt_readwakeupvbat(uint8_t spi_num) ;

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_writetodevice()
 *
 * @brief  this function is used to write to the DW1000 device registers
 * Notes:
 *        1. Firstly we create a header (the first byte is a header byte)
 *        a. check if sub index is used, if subindexing is used - set bit-6 to 1 to signify that the sub-index address follows the register index byte
 *        b. set bit-7 (or with 0x80) for write operation
 *        c. if extended sub address index is used (i.e. if index > 127) set bit-7 of the first sub-index byte following the first header byte
 *
 *        2. Write the header followed by the data bytes to the DW1000 device
 *
 *
 * input parameters:
 * @param spi_num       - ID of spi interface (equal to the used DW1000 device entity) ( 0 / 1 )
 * @param recordNumber  - ID of register file or buffer being accessed
 * @param index         - byte index into register file or buffer being accessed
 * @param length        - number of bytes being written
 * @param buffer        - pointer to buffer containing the 'length' bytes to be written
 *
 * output parameters
 *
 * returns DWT_SUCCESS for success, or DWT_ERROR for error
 */
int dwt_writetodevice             // returns 0 for success, or, -1 for error.
(
    uint8_t       spi_num,        // input parameter - ID of spi interface (equal to the used DW1000 device entity)
    uint16_t      recordNumber,   // input parameter - ID of register file or buffer being accessed
    uint16_t      index,          // input parameter - byte index into register file or buffer being accessed
    uint32_t      length,         // input parameter - number of bytes being written
    const uint8_t *buffer         // input parameter - pointer to buffer containing the 'length' bytes to be written
) ;

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_readfromdevice()
 *
 * @brief  this function is used to read from the DW1000 device registers
 * Notes:
 *        1. Firstly we create a header (the first byte is a header byte)
 *        a. check if sub index is used, if subindexing is used - set bit-6 to 1 to signify that the sub-index address follows the register index byte
 *        b. set bit-7 (or with 0x80) for write operation
 *        c. if extended sub address index is used (i.e. if index > 127) set bit-7 of the first sub-index byte following the first header byte
 *
 *        2. Write the header followed by the data bytes to the DW1000 device
 *        3. Store the read data in the input buffer
 *
 * input parameters:
 * @param spi_num       - ID of the spi interface (equal to the used DW1000 device entity) ( 0 / 1 )
 * @param recordNumber  - ID of register file or buffer being accessed
 * @param index         - byte index into register file or buffer being accessed
 * @param length        - number of bytes being read
 * @param buffer        - pointer to buffer in which to return the read data.
 *
 * output parameters
 *
 * returns DWT_SUCCESS for success, or DWT_ERROR for error
 */
int dwt_readfromdevice            // returns offset where requested data begins in supplied buffer, or, -1 for error.
(
    uint8_t   spi_num,            // input parameter - ID of the spi interface (equal to the used DW1000 device entity) ( 0 / 1 )
    uint16_t  recordNumber,       // input parameter - ID of register file or buffer being accessed
    uint16_t  index,              // input parameter - byte index into register file or buffer being accessed
    uint32_t  length,             // input parameter - number of bytes being read
    uint8_t   *buffer             // input parameter - pointer to buffer in which to return the read data.
) ;

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_read32bitoffsetreg()
 *
 * @brief  this function is used to read 32-bit value from the DW1000 device registers
 *
 * input parameters:
 * @param spi_num   - ID of the spi interface (equal to the used DW1000 device entity)
 * @param regFileID - ID of register file or buffer being accessed
 * @param regOffset - the index into register file or buffer being accessed
 *
 * output parameters
 *
 * returns 32 bit register value (success), or DWT_ERROR for error
 */
uint32_t dwt_read32bitoffsetreg(uint8_t spi_num, int regFileID, int regOffset) ;

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_write32bitoffsetreg()
 *
 * @brief  this function is used to write 32-bit value to the DW1000 device registers
 *
 * input parameters:
 * @param spi_num   - ID of the spi interface (equal to the used DW1000 device entity)
 * @param regFileID - ID of register file or buffer being accessed
 * @param regOffset - the index into register file or buffer being accessed
 * @param regval    - the value to write
 *
 * output parameters
 *
 * returns DWT_SUCCESS for success, or DWT_ERROR for error
 */
int dwt_write32bitoffsetreg(uint8_t spi_num, int regFileID, int regOffset, uint32_t regval);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_read16bitoffsetreg()
 *
 * @brief  this function is used to read 16-bit value from the DW1000 device registers
 *
 * input parameters:
 * @param spi_num   - ID of the spi interface (equal to the used DW1000 device entity) ( 0 / 1 )
 * @param regFileID - ID of register file or buffer being accessed
 * @param regOffset - the index into register file or buffer being accessed
 *
 * output parameters
 *
 * returns 16 bit register value (success), or DWT_ERROR for error
 */
uint16_t dwt_read16bitoffsetreg(uint8_t spi_num, int regFileID, int regOffset);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_write16bitoffsetreg()
 *
 * @brief  this function is used to write 16-bit value to the DW1000 device registers
 *
 * input parameters:
 * @param spi_num   - ID of the spi interface (equal to the used DW1000 device entity) ( 0 / 1 )
 * @param regFileID - ID of register file or buffer being accessed
 * @param regOffset - the index into register file or buffer being accessed
 * @param regval    - the value to write
 *
 * output parameters
 *
 * returns DWT_SUCCESS for success, or DWT_ERROR for error
 */
int dwt_write16bitoffsetreg(uint8_t spi_num, int regFileID, int regOffset, uint16_t regval) ;

#define dwt_write32bitreg(a,x,y)	dwt_write32bitoffsetreg(a,x,0,y) // a )) spi_num
#define dwt_read32bitreg(a,x)		dwt_read32bitoffsetreg(a,x,0) // a == spi_num

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn writetospi()
 *
 * @brief
 * NB: In porting this to a particular microprocessor, the implementer needs to define the two low
 * level abstract functions to write to and read from the SPI the definitions should be in deca_spi.c file.
 * Low level abstract function to write to the SPI
 * Takes two separate byte buffers for write header and write data
 * returns 0 for success, or -1 for error
 *
 * Note: The body of this function is defined in deca_spi.c and is platform specific
 *
 * input parameters:
 * @param spi_num       - ID of the spi interface (equal to the used DW1000 device entity) ( 0 / 1 )
 * @param headerLength  - number of bytes header being written
 * @param headerBuffer  - pointer to buffer containing the 'headerLength' bytes of header to be written
 * @param bodylength    - number of bytes data being written
 * @param bodyBuffer    - pointer to buffer containing the 'bodylength' bytes od data to be written
 *
 * output parameters
 *
 * returns DWT_SUCCESS for success, or DWT_ERROR for error
 */
extern int writetospi                          // returns 0 for success, or, -1 for error.
(
    uint8_t        spi_num,
    uint16_t       headerLength,          // input parameter - number of bytes header being written
    const uint8_t *headerBuffer,          // input parameter - pointer to buffer containing the 'headerLength' bytes of header to be written
    uint32_t       bodylength,            // input parameter - number of bytes data being written
    const uint8_t *bodyBuffer             // input parameter - pointer to buffer containing the 'bodylength' bytes od data to be written
) ;

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn readfromspi()
 *
 * @brief
 * NB: In porting this to a particular microprocessor, the implementer needs to define the two low
 * level abstract functions to write to and read from the SPI the definitions should be in deca_spi.c file.
 * Low level abstract function to write to the SPI
 * Takes two separate byte buffers for write header and write data
 * returns 0 for success, or -1 for error
 *
 * Note: The body of this function is defined in deca_spi.c and is platform specific
 *
 * input parameters:
 * @param spi_num       - ID of the spi interface (equal to the used DW1000 device entity) ( 0 / 1 )
 * @param headerLength  - number of bytes header to write
 * @param headerBuffer  - pointer to buffer containing the 'headerLength' bytes of header to write
 * @param bodylength    - number of bytes data being read
 * @param bodyBuffer    - pointer to buffer containing to return the data (NB: size required = headerLength + readlength)
 *
 * output parameters
 *
 * returns DWT_SUCCESS for success (and the position in the buffer at which data begins), or DWT_ERROR for error
 */
extern int readfromspi                         // returns offset where requested data begins in supplied buffer, or, -1 for error.
(
    uint8_t        spi_num,               // input parameter - ID of the spi interface (equal to the used DW1000 device entity)
    uint16_t       headerLength,          // input parameter - number of bytes header to write
    const uint8_t *headerBuffer,          // input parameter - pointer to buffer containing the 'headerLength' bytes of header to write
    uint32_t       readlength,            // input parameter - number of bytes data being read
    uint8_t       *readBuffer             // input parameter - pointer to buffer containing to return the data (NB: size required = headerLength + readlength)
) ;

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_getrangebias()
 *
 * @brief This function is used to return the range bias correction need for TWR with DW1000 units.
 *
 * input parameters:
 * @param chan  - specifies the operating channel (e.g. 1, 2, 3, 4, 5, 6 or 7)
 * @param range - the calculated distance before correction
 * @param prf   - this is the PRF e.g. DWT_PRF_16M or DWT_PRF_64M
 *
 * output parameters
 *
 * returns correction needed in meters
 */
double dwt_getrangebias(uint8_t chan, float range, uint8_t prf);


// ---------------------------------------------------------------------------
//
// NB: The purpose of the deca_mutex.c file is to provide for microprocessor interrupt enable/disable, this is used for
//     controlling mutual exclusion from critical sections in the code where interrupts and background
//     processing may interact.  The code using this is kept to a minimum and the disabling time is also
//     kept to a minimum, so blanket interrupt disable may be the easiest way to provide this.  But at a
//     minimum those interrupts coming from the decawave device should be disabled/re-enabled by this activity.
//
//     In porting this to a particular microprocessor, the implementer may choose to use #defines here
//     to map these calls transparently to the target system.  Alternatively the appropriate code may
//     be embedded in the functions provided in the deca_irq.c file.
//
// ---------------------------------------------------------------------------

typedef int decaIrqStatus_t ; // Type for remembering IRQ status
// TODO: Is it necessary to have a second one here too ? ( for the other DW1000 device )


/*! ------------------------------------------------------------------------------------------------------------------
 * @fn decamutexon()
 *
 * @brief This function should disable interrupts. This is called at the start of a critical section
 * It returns the IRQ state before disable, this value is used to re-enable in decamutexoff call
 *
 * Note: The body of this function is defined in deca_mutex.c and is platform specific
 *
 * input parameters:
 *
 * output parameters
 *
 * returns the state of the DW1000 interrupt
 */
decaIrqStatus_t decamutexon(void) ;

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn decamutexoff()
 *
 * @brief This function should re-enable interrupts, or at least restore their state as returned(&saved) by decamutexon
 * This is called at the end of a critical section
 *
 * Note: The body of this function is defined in deca_mutex.c and is platform specific
 *
 * input parameters:
 * @param s - the state of the DW1000 interrupt as returned by decamutexon
 *
 * output parameters
 *
 * returns the state of the DW1000 interrupt
 */
void decamutexoff(decaIrqStatus_t s) ;


#ifdef __cplusplus
}
#endif

#endif /* _DECA_DEVICE_API_H_ */


