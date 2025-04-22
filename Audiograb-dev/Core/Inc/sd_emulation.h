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

#define CMD0_CRC 0x94 //Valid CRC value of CMD0, according to https://rjhcoding.com/avrc-sd-interface-1.php. This matches with observations from a real SD card transaction.
#define CMD0_FINALBYTE 0x95 //Final bit is always 1.
#define CRC_MSK 0xFE //The 7 first bits in the final byte contain the CRC.


//Global variables.
extern volatile uint8_t R1_status; //This is the variable containing the status bits of the R1 response.
//Definition of bits in the R1 status register.
//Bit 7 is always 0.
#define R1_PARAM_ERR_MSK (1 << 6)
#define R1_ADDR_ERR_MSK (1 << 5)
#define R1_ERASE_SEQ_ERR_MSK (1 << 4)
#define R1_COM_CRC_ERR_MSK (1 << 3)
#define R1_ILLEGAL_CMD_MSK (1 << 2)
#define R1_ERASE_RST_MSK (1 << 1)
#define R1_IDLE_STATE_MSK (1 << 0)

//Public functions
void SD_emulation_init();


#endif /* INC_SD_EMULATION_H_ */
