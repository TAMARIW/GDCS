#ifndef ORPE_HAMMINGCODES_H
#define ORPE_HAMMINGCODES_H


#include <stdint.h>


/**
 * @brief This file contains helper functions for producing hamming codes and decoding and correcting errors.
*/
namespace ORPE {


/**
 * @brief Will produce the Hamming code. Works with only 4 bits.
 * @param data The data bits
 * 
 * @returns hamming code
*/
uint8_t dataToHamming(uint8_t data);


/**
 * @brief Takes the given hamming code and decodes into data bits. Will also correct errors. Works with only 4 bits.
 * @note Returns multiple cases. Even after correction or no error detection the data might be incorrect!
 * @param code Input hamming code. Will be replaced by data bits.
 * 
 * @returns 1 if decoding had no errors, 2 if error was corrected, 0 if failed to decode and correct.
*/
int hammingToData(uint8_t& code);


}

#endif 