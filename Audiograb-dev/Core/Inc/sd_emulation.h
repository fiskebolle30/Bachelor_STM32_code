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
#define SD_EMUL_SPI SPI2 //SPI hardware instance used by the SD emulator
#define SPI_RX_DMA_INSTANCE DMA1 //DMA hardware instance used by SD emulator (when DMA is in use)
#define SPI_RX_DMA_STREAM_NUM 0 //DMA stream number used for SPI RX when DMA is in use
#define SPI_TX_DMA_INSTANCE DMA1
#define SPI_TX_DMA_STREAM_NUM 1

//Masks used to clear all interrupt flags for a DMA stream, while leaving reserved bits.
#define DMA_STREAM0_INTERRUPTFLAGS_MASK (uint32_t)0b01111101
#define DMA_STREAM1_INTERRUPTFLAGS_MASK (uint32_t)(0b01111101 << 8)

#define SD_START_BITS_MASK 0b11000000 //start bit + transmission bit
#define SD_VALID_START_PATTERN 0b01000000 //start bit 0, transmision bit 1
#define SD_ACMD_BITMASK 0b10000000 //This bit being set in command_num indicates an ACMD. This is also done by the AudioMoth in its code (specifically microsd.c, line 348)
#define SD_BLOCK_START_TOKEN 0xFE //This token indicates that the following bytes on the bus are actual data from the SD card.
#define SD_BLOCK_LENGTH 512 //512 bytes in a block, assuming SDHC or greater or whatever

#define CMD(cmd_num) (cmd_num) //can't be bothered to write all of them lol
#define ACMD(cmd_num) (cmd_num | SD_ACMD_BITMASK) //Internally in this project, an ACMD is indicated by the most significant bit being set in the command number.

//Definitions for CMD0
#define CMD0_CRC 0x94 //Valid CRC value of CMD0, according to https://rjhcoding.com/avrc-sd-interface-1.php. This matches with observations from a real SD card transaction.
#define CMD0_EXPECTED_FINALBYTE 0x95 //Final bit is always 1.
#define CRC_MSK 0xFE //The 7 first bits in the final byte contain the CRC.

//Definitions for CMD8
#define CMD8_CRC 0x86
#define CMD8_EXPECTED_FINALBYTE 0x87
#define CMD8_VOLTAGE_ACCEPTED_MSK (0x0F << 8)
#define CMD8_2V7_3V6_ACCEPTED (0x01 << 8) //The first bit of the "voltage accepted" field indicates 2.7V-3.6V.
#define R7_ECHO_BACK_MSK (0xFF << 0) //The echo-back in R7 is the last byte of both R7 and the CMD8 argument.


//Global variable definitions
extern volatile bool card_initialized; //Variable to keep track of if the device is ready to tell the SD card host that it's initialized.

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

extern volatile uint32_t OCR; //The operation conditions register stores the "card"'s accepted voltage range, and some other info.
//The bits in the register are defined in the SD card physical layer spec, under section 5.1.
#define OCR_CCS_MSK (1 << 30) //The card capacity status bit
#define OCR_POWERUP_STATUS_MSK (1 << 31) //This bit is low if the "card" hasn't powered up yet
#define OCR_STATIC_PARAMS (0x1FF << 15) /*voltage between 2.7-3.6 supported.*/

extern volatile bool HCS; //The Host Capacity Support bit indicates if the host supports high capacity SD cards (SDHC & SDUC).
#define HCS_BIT_MSK (1 << 30) //The HCS is located in bit 30 of the command argument for ACMD41 and CMD1.

extern volatile bool SD_card_DMA_read_completed; //flag to indicate that one block has been read by the SD card, and that the buffer is now valid


//Public functions
void SD_emulation_init();

#endif /* INC_SD_EMULATION_H_ */
