/*
 * deca_spi.cpp
 *
 *  Created on: 17.03.2016
 *      Author: thomas
 */

#include "rodos.h"
HAL_SPI spi0(SPI_IDX1, GPIO_005, GPIO_006, GPIO_007); // Added by SL -- Is there a reason why first SPI was SPI2 and not SPI1 ? Maybe keep in mind and if errors occur switch to SPI3...
HAL_SPI spi1(SPI_IDX2, GPIO_029 , GPIO_030, GPIO_031); // TODO: Hier müsste was geändert werden damit das passt mit der Zuordnung, ziemlich sicher
HAL_GPIO cs0 = GPIO_004;// Version 1
HAL_GPIO cs1 = GPIO_028;// Version 2

HAL_GPIO UWBirq1(GPIO_040);//irq = GPIO_035;
HAL_GPIO UWBirq2(GPIO_039); // added by SL
uint32_t baudrate = 1000000;

void spiWrite(uint8_t spi_num, uint32_t headerLength, const uint8_t* headerBuffer,uint32_t  bodylength, const uint8_t* bodyBuffer){
	if (spi_num == 0) { /* spi 0 */
		cs0.setPins(0);
		while(!spi0.isWriteFinished()){};
		spi0.write(headerBuffer,headerLength);
		while(!spi0.isWriteFinished()){};
		spi0.write(bodyBuffer,bodylength);
		while(!spi0.isWriteFinished()){};
		cs0.setPins(1);
	} else { /* spi 1 */
		cs1.setPins(0);
		while(!spi1.isWriteFinished()){};
		spi1.write(headerBuffer,headerLength);
		while(!spi1.isWriteFinished()){};
		spi1.write(bodyBuffer,bodylength);
		while(!spi1.isWriteFinished()){};
		cs1.setPins(1);		
	}
    return ;
}
int spiRead(uint8_t spi_num, const uint8_t* sendBuf, uint32_t len, uint8_t* recBuf, uint32_t maxLen){
	if (spi_num == 0) {
		cs0.setPins(0);
		while(!spi0.isWriteFinished()){};
		spi0.write(sendBuf,len);
		while(!spi0.isWriteFinished()){};
		spi0.read(recBuf,maxLen);
		while(!spi0.isReadFinished()){};
		cs0.setPins(1);
	} else {
		cs1.setPins(0);
		while(!spi1.isWriteFinished()){};
		spi1.write(sendBuf,len);
		while(!spi1.isWriteFinished()){};
		spi1.read(recBuf,maxLen);
		while(!spi1.isReadFinished()){};
		cs1.setPins(1);
	}
	return 0;
}

extern "C" {
	int openspi()
	{
		return 0;
	}

	int closespi(void)
	{
		return 0;
	}

	int writetospi_serial
	(
		uint8_t		   spi_num,
	    uint16_t       headerLength,
	    const uint8_t *headerBuffer,
	    uint32_t       bodylength,
	    const uint8_t *bodyBuffer
	){
		spiWrite(spi_num, headerLength, headerBuffer,bodylength, bodyBuffer);

		return 0;
	}

	int readfromspi_serial
	(
		uint8_t		   spi_num,
	    uint16_t       headerLength,
	    const uint8_t *headerBuffer,
	    uint32_t       readlength,
	    uint8_t       *readBuffer
	){
		spiRead(spi_num, headerBuffer,headerLength,readBuffer,readlength);
		return 0;
	}
}


