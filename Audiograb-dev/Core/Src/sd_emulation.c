/*
 * sd_emulation.c
 *
 *  Created on: Apr 4, 2025
 *      Author: steinar
 */
#include "sd_emulation.h"

#define SD_START_BITS_MASK 0b11000000 //start bit + transmission bit
#define SD_VALID_START_PATTERN 0b01000000 //start bit 0, transmision bit 1
#define SD_ACMD_BITMASK 0b10000000

uint8_t SPI2_rx_buffer[10];
uint8_t SPI2_tx_buffer[10];
unsigned int transmission_length;
uint8_t command_num; //bit #7 being set (0b10xxxxxx) indicates ACMD
uint32_t command_arg;
uint8_t command_CRC;

enum SD_emulator_state {
	awaiting_cmd, receiving_cmd, error
};
enum SD_emulator_state state;


//Private function prototypes:
void SPI_TX_RX_complete_callback(SPI_HandleTypeDef *hspi);
void command_handler(uint8_t num, uint32_t arg, uint8_t crc);
void set_bytes_to_ff(uint8_t *buf, const unsigned int len);

//Function declarations:
void SD_emulation_init()
{
	MX_SPI2_Init(); //Init SPI.

	HAL_SPI_RegisterCallback(&hspi2, HAL_SPI_TX_RX_COMPLETE_CB_ID, &SPI_TX_RX_complete_callback);

	//Start reception of SPI. This is continual, and all of the "action" happens in the DMA callbacks.
	state = awaiting_cmd;
	transmission_length = 1;
	SPI2_tx_buffer[0] = 0xff;
	while (HAL_SPI_TransmitReceive_DMA(&hspi2, SPI2_tx_buffer, SPI2_rx_buffer, transmission_length) == HAL_BUSY) {;}
}


void SPI_TX_RX_complete_callback(SPI_HandleTypeDef *hspi)
{
	if(transmission_length == 1 && state == awaiting_cmd)
	{
		uint8_t received_byte = SPI2_rx_buffer[0];
		if((received_byte & SD_START_BITS_MASK) == SD_VALID_START_PATTERN)
		{
			state = receiving_cmd;
			command_num = (received_byte & ~SD_START_BITS_MASK); //the 6 least significant bits are the command number.

			//Receive the rest of the command (5 more bytes).
			transmission_length = 5;
			set_bytes_to_ff(SPI2_tx_buffer, transmission_length);
			HAL_SPI_TransmitReceive_DMA(hspi, SPI2_tx_buffer, SPI2_rx_buffer, transmission_length);
			return;
		}
	}
	else if(state == receiving_cmd)
	{
		command_arg = SPI2_rx_buffer[3];		//LSBs of argument
		command_arg |= (SPI2_rx_buffer[2] << 8);
		command_arg |= (SPI2_rx_buffer[1] << 16);
		command_arg |= (SPI2_rx_buffer[0] << 24); //MSBs of argument
		command_CRC = SPI2_rx_buffer[4];
		command_handler(command_num, command_arg, command_CRC);
		//Do I place SPI_transfer here or in command_handler?
		return;
	}
}

void command_handler(uint8_t num, uint32_t arg, uint8_t crc)
{
	switch(num)
	{
	case 0: {
		state = awaiting_cmd;
		break;
	}
	default: {
		state = error;
		//send r1 response with error?
		break;
	}
	}
}

void set_bytes_to_ff(uint8_t *buf, const unsigned int len)
{
	if(buf != NULL) {
		for(uint8_t i = 0; i < len; ++i) //Set all bytes to be transmitted to 0xff
		{
			buf[i] = 0xff;
		}
	}
}
