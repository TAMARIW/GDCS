#include <stdint.h>
#include <stddef.h>

#include "HammingCodes.h"


/**
 * @brief This file contains the impementation of the helper functions for producing hamming codes and decoding and correcting errors.
*/
namespace ORPE {


/**
 * @brief Calculates the parity for a given data value. Bit positions in the data param where at the same bit position in positions is set to 1 will be used for the parity calc.
*/
bool parityCalc(uint32_t data, uint32_t positions) {

    uint8_t counter = 0;
    uint32_t result = data & positions; //Result contains only the binary 1 values of interest in data param.
    for (size_t i = 0; i < 32; i++) //We count the number of binary 1s inside result value.
        counter += 0x01&(result>>i);

    return counter % 2 == 1;

}


/**
 * @brief Will produce the (7,4) Hamming code inclusing 1 extra bit for double error detection. Works with only 4 data bits.
 * @param data The data bits. Max 4 bits long.
 * 
 * @returns hamming code. 8 Bits long.
*/
uint8_t dataToHamming(uint8_t data) {

    uint8_t hamming = 0;

    //Data Bits
    hamming |= (0x01&(data))   <<2; //Bit 2
    hamming |= (0x01&(data>>1))<<4; //Bit 4
    hamming |= (0x01&(data>>2))<<5; //Bit 5
    hamming |= (0x01&(data>>3))<<6; //Bit 6
    //Parity bits
    hamming |= parityCalc(hamming, 0b01010100)<<0; //Bit 0
    hamming |= parityCalc(hamming, 0b01100100)<<1; //Bit 1
    hamming |= parityCalc(hamming, 0b01110000)<<3; //Bit 3
    hamming |= parityCalc(hamming, 0b01111111)<<7; //Bit 7, extra parity bit including all bits.
    
    return hamming;

}


/**
 * @brief Takes the given hamming code and decodes into data bits. Will also correct errors. Works with only 4 bits.
 * @note Returns multiple cases. Even after correction or no error detection the data might be incorrect!
 * @param code Input hamming code. Will be replaced by data bits.
 * 
 * @returns 1 if decoding had no errors, 2 if error was corrected, 0 if failed to decode and correct.
*/
int hammingToData(uint8_t& code) {

    // Calculate parity checks
    uint8_t errorPosition = 0;
    errorPosition |= parityCalc(code, 0b01010101) << 0; //Bit 0
    errorPosition |= parityCalc(code, 0b01100110) << 1; //Bit 1
    errorPosition |= parityCalc(code, 0b01111000) << 2; //Bit 2

    // Correct the error if there's a single-bit error
    if (errorPosition > 0 & errorPosition <= 7) {
        code ^= 1<<(errorPosition - 1);
    } 
    
    bool parity = parityCalc(code, 0b11111111);
    if (parity) { // There is more than one error. Unable to correct. Return failure.
        return 0;
    }

    // Extract the original data bits
    uint8_t data = 0;
    data |= (0x01&(code>>2)) << 0;
    data |= (0x01&(code>>4)) << 1;
    data |= (0x01&(code>>5)) << 2;
    data |= (0x01&(code>>6)) << 3;
    code = data;
    
    return errorPosition == 0 ? 1 : 2; // Return 1 if no error, 2 if error was corrected.

}


}