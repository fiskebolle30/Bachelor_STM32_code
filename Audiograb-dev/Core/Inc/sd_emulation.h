/*
 * sd_emulation.h
 *
 *  Created on: Apr 4, 2025
 *      Author: steinar
 */

#ifndef INC_SD_EMULATION_H_
#define INC_SD_EMULATION_H_

#include "gpio.h"
#include "spi.h"
#include <stdbool.h>


//Defines
#define SD_EMUL_SPI SPI2 //SD emulator SPI instance
#define SPI_RX_DMA_INSTANCE DMA1
#define SPI_RX_DMA_STREAM_NUM 0
#define SPI_TX_DMA_INSTANCE DMA1
#define SPI_TX_DMA_STREAM_NUM 1

//Masks used to clear all interrupt flags for a DMA stream, while leaving reserved bits.
#define DMA_STREAM0_INTERRUPTFLAGS_MASK (uint32_t)0b01111101
#define DMA_STREAM1_INTERRUPTFLAGS_MASK (uint32_t)(0b01111101 << 8)

#define SD_START_BITS_MASK 0b11000000 //start bit + transmission bit
#define SD_VALID_START_PATTERN 0b01000000 //start bit 0, transmision bit 1
#define SD_ACMD_BITMASK 0b10000000

#define CMD(cmd_num) (cmd_num) //can't be bothered to write all of them lol
#define ACMD(cmd_num) (cmd_num | SD_ACMD_BITMASK)


//Global variables.


//Public functions
void SD_emulation_init();


#endif /* INC_SD_EMULATION_H_ */
