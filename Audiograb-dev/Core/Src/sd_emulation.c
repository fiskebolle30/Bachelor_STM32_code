/*
 * sd_emulation.c
 *
 *  Created on: Apr 4, 2025
 *      Author: steinar
 */
#include "sd_emulation.h"


void SD_emulation_init()
{
	MX_SPI2_Init(); //Init SPI.

	//Start reception of SPI. This is continual, and all of the "action" happens in the DMA callbacks.
	while (HAL_SPI_Receive_DMA(&hspi2, &SPI2_initial_data, 1) = HAL_BUSY) {;}
}
