/*
 * sd_emulation.c
 *
 *  Created on: Apr 4, 2025
 *      Author: steinar
 */
#include "sd_emulation.h"

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

volatile uint8_t SD_emulator_rx_buffer[10];
volatile uint8_t SD_emulator_tx_buffer[10];
unsigned int transmission_length;
uint8_t command_num; //bit #7 being set (0b10xxxxxx) indicates ACMD
uint32_t command_arg;
uint8_t command_CRC;

enum SD_emulator_state {
	awaiting_cmd, receiving_cmd, error
};
enum SD_emulator_state state;


//Private function prototypes:
void SPI_TX_RX_complete_callback(/*SPI_HandleTypeDef *hspi*/);
void SD_command_handler(uint8_t num, uint32_t arg, uint8_t crc);
//void set_bytes_to_ff(uint8_t *buf, const unsigned int len);


//Function declarations:
void SD_emulation_init()
{
	MX_SPI2_Init(); //Init SPI.
	LL_SPI_SetUDRPattern(SD_EMUL_SPI, 0xFFFFFF89); //Send only ones on TX buffer underrun, as this means the "SD card" is processing/busy.
	LL_SPI_SetUDRConfiguration(SD_EMUL_SPI, LL_SPI_UDR_CONFIG_REGISTER_PATTERN);
	LL_SPI_SetUDRDetection(SD_EMUL_SPI, LL_SPI_UDR_DETECT_END_DATA_FRAME); //register underrun immediately when the TX FIFO is empty.
	LL_SPI_EnableIOLock(SD_EMUL_SPI); //The SPI IO settings shouldn't change ever anyways (I think lol).

	//Configure DMA interrupts. The implementation is driven by the RX DMA interrupt.
	LL_DMA_EnableIT_TC(SPI_RX_DMA_INSTANCE, SPI_RX_DMA_STREAM_NUM);

	//Start reception of SPI. This is continual, and all of the "action" happens in the DMA callbacks.
	state = awaiting_cmd;
	transmission_length = 1;
	SD_emulator_tx_buffer[0] = 0x11;

}

void transfer_SPI_DMA(uint8_t *txbuf, uint8_t *rxbuf, unsigned int trans_len) //Kind of unfinished, something something surprise tool that will help us later
{
	//Start DMA transfer in accordance to the "cookbook recipe" at ref. manual section 55.4.14.
	LL_SPI_EnableDMAReq_RX(SD_EMUL_SPI);

	//Ensure DMA streams are disabled to make changes to them.
	LL_DMA_DisableStream(SPI_RX_DMA_INSTANCE, SPI_RX_DMA_STREAM_NUM);
	LL_DMA_DisableStream(SPI_TX_DMA_INSTANCE, SPI_TX_DMA_STREAM_NUM);
	while (LL_DMA_IsEnabledStream(SPI_RX_DMA_INSTANCE, SPI_RX_DMA_STREAM_NUM)) {;} //wait until stream is disabled
	while (LL_DMA_IsEnabledStream(SPI_TX_DMA_INSTANCE, SPI_TX_DMA_STREAM_NUM)) {;}
	SPI_RX_DMA_INSTANCE->LIFCR = DMA_STREAM0_INTERRUPTFLAGS_MASK | DMA_STREAM1_INTERRUPTFLAGS_MASK; //Clear all interrupt flags for channel 0 and 1.

	//Set addresses to transfer data from and to.
	LL_DMA_SetPeriphAddress(SPI_RX_DMA_INSTANCE, SPI_RX_DMA_STREAM_NUM, (uint32_t)SD_EMUL_SPI); //Peripheral address should never change, so this line may be unnecessary.
	LL_DMA_SetPeriphAddress(SPI_TX_DMA_INSTANCE, SPI_TX_DMA_STREAM_NUM, (uint32_t)SD_EMUL_SPI); //Cast pointers into uint32_t to write it to DMA register
	LL_DMA_SetMemoryAddress(SPI_RX_DMA_INSTANCE, SPI_RX_DMA_STREAM_NUM, (uint32_t)rxbuf);
	LL_DMA_SetMemoryAddress(SPI_TX_DMA_INSTANCE, SPI_TX_DMA_STREAM_NUM, (uint32_t)txbuf);

	LL_DMA_SetDataLength(SPI_TX_DMA_INSTANCE, SPI_TX_DMA_STREAM_NUM, trans_len);
	LL_DMA_SetDataLength(SPI_RX_DMA_INSTANCE, SPI_RX_DMA_STREAM_NUM, trans_len);

	LL_DMA_EnableStream(SPI_RX_DMA_INSTANCE, SPI_RX_DMA_STREAM_NUM);
	LL_DMA_EnableStream(SPI_TX_DMA_INSTANCE, SPI_TX_DMA_STREAM_NUM);

	LL_SPI_Enable(SD_EMUL_SPI);
}


void SPI_TX_RX_complete_callback(/*SPI_HandleTypeDef *hspi*/)
{
	HAL_GPIO_TogglePin(USER_LED2_GPIO_Port, USER_LED2_Pin); //do something
	if(transmission_length == 1 && state == awaiting_cmd)
	{
		uint8_t received_byte = SD_emulator_rx_buffer[0];
		if((received_byte & SD_START_BITS_MASK) == SD_VALID_START_PATTERN)
		{
			state = receiving_cmd;
			command_num = (received_byte & ~SD_START_BITS_MASK); //the 6 least significant bits are the command number.

			//Receive the rest of the command (5 more bytes).
			transmission_length = 5;
			//set_bytes_to_ff(SD_emulator_tx_buffer, transmission_length);
			//HAL_SPI_TransmitReceive_DMA(hspi, SD_emulator_tx_buffer, SD_emulator_rx_buffer, transmission_length); //replace me
			return;
		}
	}
	else if(state == receiving_cmd)
	{
		command_arg = SD_emulator_rx_buffer[3];		//LSBs of argument
		command_arg |= (SD_emulator_rx_buffer[2] << 8);
		command_arg |= (SD_emulator_rx_buffer[1] << 16);
		command_arg |= (SD_emulator_rx_buffer[0] << 24); //MSBs of argument
		command_CRC = SD_emulator_rx_buffer[4];
		SD_command_handler(command_num, command_arg, command_CRC);
		//Do I place SPI_transfer here or in command_handler?
		return;
	}
}

void SD_command_handler(uint8_t num, uint32_t arg, uint8_t crc)
{
	switch(num)
	{
	case 0: {
		state = awaiting_cmd; //switch back to "idle" state before sending response
		HAL_GPIO_TogglePin(USER_LED1_GPIO_Port, USER_LED1_Pin); //debug to see if this actually happens
		//send response R1 with no errors and "in idle state" true
		//HAL_SPI_TransmitReceive_DMA(&hspi2, SD_emulator_tx_buffer, SPI2_rx_buffer, transmission_length); //replace me
		break;
	}
	default: {
		state = error;
		HAL_GPIO_TogglePin(USER_LED2_GPIO_Port, USER_LED2_Pin);

		//send r1 response with error?
		break;
	}
	}
}

/*void set_bytes_to_ff(uint8_t *buf, const unsigned int len)
{
	if(buf != NULL) {
		for(uint8_t i = 0; i < len; ++i) //Set all bytes to be transmitted to 0xff
		{
			buf[i] = 0xff;
		}
	}
}*/
