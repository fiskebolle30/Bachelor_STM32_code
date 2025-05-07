/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32h7xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32h7xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "sd_emulation.h"
#include "sdmmc.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern SD_HandleTypeDef hsd1;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32H7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32h7xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 stream0 global interrupt.
  */
void DMA1_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream0_IRQn 0 */
	//This is the SD emulator SPI RX stream

  /* USER CODE END DMA1_Stream0_IRQn 0 */
  /* USER CODE BEGIN DMA1_Stream0_IRQn 1 */

  /* USER CODE END DMA1_Stream0_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream1 global interrupt.
  */
void DMA1_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream1_IRQn 0 */
	//This is the SD emulator SPI TX stream

  /* USER CODE END DMA1_Stream1_IRQn 0 */
  /* USER CODE BEGIN DMA1_Stream1_IRQn 1 */

  /* USER CODE END DMA1_Stream1_IRQn 1 */
}

/**
  * @brief This function handles SPI2 global interrupt.
  */
void SPI2_IRQHandler(void)
{
  /* USER CODE BEGIN SPI2_IRQn 0 */
	//SD emulator IRQ.
	/*	This function is expecting compiler optimization to be turned on.
	 *	When debugging, consider changing optimization flags to -Og and
	 *	if things then break, try turning CPU speed up.
	 */

	//Static variables to keep track of SD emulator state.
	static bool cmd_is_ACMD; //Bool to indicate if the current command is an application-specific command (CMD55 before it)

	static uint8_t command_num; //bit #7 being set (0b10xxxxxx) indicates ACMD
	static uint32_t command_arg;
	static uint8_t command_CRC;

	static unsigned int command_arg_index;

	static uint8_t data_block[SD_BLOCK_LENGTH]; //data block for reading from SD card. Kind of loosely defined and liable to change, will be further specified when implementing write. (TODO)
	static int block_index;

	enum SD_emulator_state {
		awaiting_cmd, receiving_cmd_arg, waiting_for_SD_read_DMA, transmitting_single_block, error
	};
	static enum SD_emulator_state state = awaiting_cmd;

	while(SD_EMUL_SPI->SR & SPI_SR_RXP) //Repeat as long as there are packets to be read from rx FIFO:
	{
		uint32_t received_packet = LL_SPI_ReceiveData32(SD_EMUL_SPI); //Receive 4 bytes. Note that this is little-endian, since the smallest byte was received first.
		int packet_index = 0; //Track amount of bytes not handled in packet.

		switch(state) {
		case awaiting_cmd: {

			command_arg_index = 0; //Reset index when start-of-command hasn't been detected

			for(; packet_index < 4; ++packet_index) //Check for start byte as long as there are bytes left in current packet:
			{
				uint8_t received_data = (received_packet >> (8 * packet_index)); //Get one byte from RX FIFO
				if((received_data & SD_START_BITS_MASK) == SD_VALID_START_PATTERN) //If start-of-command pattern detected:
				{
					state = receiving_cmd_arg;
					command_num = (received_data & ~SD_START_BITS_MASK) | (cmd_is_ACMD << 7); //Mask out start bits, and add ACMD indication if applicable.
					cmd_is_ACMD = false; //Reset ACMD bool in preparation for next reception.
					command_arg = 0;
					command_arg_index = 0;
					++packet_index; //Increment index because the for loop won't after the break statement.
					break; //break out of for loop, continue to next case
				}
			}
			if(state == awaiting_cmd) //If the state is still awaiting_cmd:
			{
				break; //Don't go into the receiving_cmd_arg case.
			}
			}

		case receiving_cmd_arg: {

			for(; packet_index < 4; ++packet_index) //As long as there are bytes left in current packet:
			{
				uint8_t received_data = (received_packet >> (8 * packet_index)); //Get one byte from RX FIFO
				if(command_arg_index <= 3)
				{
					command_arg |= (received_data << (8 * (3 - command_arg_index)));
					++command_arg_index; //increment index
				}
				else
				{
					command_CRC = received_data;
					++command_arg_index;
					++packet_index; //Increment index because the for loop won't after the break statement.
					break;
				}
			}
			if(command_arg_index < 5) //If the whole command hasn't been read:
			{
				break; //Prevent commands from being handled before the entire command has been received
			}

			LL_SPI_ReceiveData32(SD_EMUL_SPI); //Throw away any bytes that have come after the end of command but before answer.
			LL_SPI_ReceiveData32(SD_EMUL_SPI);


			switch(command_num) { //This section is where the different commands are handled.

			case CMD(0): { //CMD0: go to idle state.

				R1_status |= R1_IDLE_STATE_MSK; //Enter idle state.

				if(command_CRC != CMD0_EXPECTED_FINALBYTE) //If the CRC isn't the expected value:
				{
					R1_status |= R1_COM_CRC_ERR_MSK; //set CRC error bit in R1 response.
				}
				LL_SPI_TransmitData8(SD_EMUL_SPI, R1_status); //Send R1 response.
				LL_SPI_ClearFlag_UDR(SD_EMUL_SPI); //Clear UDR flag to send response.

				R1_status = (R1_status & R1_IDLE_STATE_MSK); //Clear all error flags when reading them, as described in
				//SD Specifications Part 1 Physical Layer Simplified Specification Version 9.10, section 7.3.4
				break;
			}

			case CMD(8): { //CMD8: SEND_IF_COND

				if(command_CRC != CMD8_EXPECTED_FINALBYTE)
				{
					R1_status |= R1_COM_CRC_ERR_MSK; //set CRC error bit in R1 response.
					LL_SPI_TransmitData8(SD_EMUL_SPI, R1_status); //Send R1 response.
					LL_SPI_ClearFlag_UDR(SD_EMUL_SPI); //Begin transmission (Only R1 on wrong CRC)
					R1_status = (R1_status & R1_IDLE_STATE_MSK); //Clear all error flags when reading them.
					break;
				}

				LL_SPI_TransmitData8(SD_EMUL_SPI, R1_status); //Send R1 response, which is the start of the R7 response
				uint32_t rest_of_R7 = 0;
				if((command_arg & CMD8_VOLTAGE_ACCEPTED_MSK) == CMD8_2V7_3V6_ACCEPTED) //Only send back voltage accepted field if it's the correct voltage for the "card"
				{
					rest_of_R7 |= (command_arg & CMD8_VOLTAGE_ACCEPTED_MSK); //Echo back the voltage accepted field to indicate the SD card accepts it.
				}
				rest_of_R7 |= (command_arg & R7_ECHO_BACK_MSK); //Send back the last byte of the argument.

				LL_SPI_TransmitData8(SD_EMUL_SPI, (rest_of_R7 >> 24)); //Send word big-endian, so that the order the bytes appear in on the bus (and in the simplified spec document)
				LL_SPI_TransmitData8(SD_EMUL_SPI, (rest_of_R7 >> 16)); //are the same as the way they are organized in memory.
				LL_SPI_TransmitData8(SD_EMUL_SPI, (rest_of_R7 >> 8));  //(TransmitData32 is little-endian by necessity).
				LL_SPI_TransmitData8(SD_EMUL_SPI, (rest_of_R7 >> 0));
				LL_SPI_ClearFlag_UDR(SD_EMUL_SPI); //Begin transmission
				break;
			}

			case CMD(55): //CMD55: The next command is an application-specific one (ACMD).
			case ACMD(55): { //ACMD55 has the same functionality as CMD55.

				cmd_is_ACMD = true;

				LL_SPI_TransmitData8(SD_EMUL_SPI, R1_status); //Send R1 response.
				LL_SPI_ClearFlag_UDR(SD_EMUL_SPI); //Begin transmission.
				R1_status = (R1_status & R1_IDLE_STATE_MSK); //Clear all error flags when reading them.
				break;
			}

			case CMD(1): //CMD1: SEND_OP_COND. Not sure what the difference is between this and ACMD41. CMD1 is seemingly not used by AudioMoth.
			case ACMD(41): { //ACMD41: SD_SEND_OP_COND. The host sends the HCS bit, and this command is also supposed to start the "card"'s init process.
				if(card_initialized)
				{
					R1_status &= ~R1_IDLE_STATE_MSK; //Clear idle bit if the card is initialized.
				}
				LL_SPI_TransmitData8(SD_EMUL_SPI, R1_status); //Send R1 response.
				LL_SPI_ClearFlag_UDR(SD_EMUL_SPI); //Begin transmission.
				R1_status = (R1_status & R1_IDLE_STATE_MSK); //Clear all error flags when reading them.

				if(command_arg & HCS_BIT_MSK)
				{
					HCS = true;
				}
				else
				{
					HCS = false;
				}

				break;
			}

			case CMD(58): { //CMD58: SEND_OCR.

				LL_SPI_TransmitData8(SD_EMUL_SPI, R1_status);
				LL_SPI_TransmitData8(SD_EMUL_SPI, (OCR >> 24)); //Transmit OCR big-endian-ly.
				LL_SPI_TransmitData8(SD_EMUL_SPI, (OCR >> 16));
				LL_SPI_TransmitData8(SD_EMUL_SPI, (OCR >> 8));
				LL_SPI_TransmitData8(SD_EMUL_SPI, (OCR >> 0));
				LL_SPI_ClearFlag_UDR(SD_EMUL_SPI);
				break;
			}

			case CMD(17): { //CMD17: Read single block.
				/* DEBUGDEBUGDEBUG, uncomment when debugging is finished and it hopefully works
				if(card_initialized == false) //Not sure if this can ever be the case if the host has "gotten this far"
				{
					R1_status |= R1_ILLEGAL_CMD_MSK; //Set the illegal command bit in R1.
					LL_SPI_TransmitData8(SD_EMUL_SPI, R1_status); //Send R1 response.
					LL_SPI_ClearFlag_UDR(SD_EMUL_SPI); //Clear UDR flag to send response.
					R1_status = (R1_status & R1_IDLE_STATE_MSK); //Clear all error flags when the host reads them.
					break;
				}*/

				LL_SPI_TransmitData8(SD_EMUL_SPI, R1_status); //Send R1 response.
				LL_SPI_ClearFlag_UDR(SD_EMUL_SPI); //Clear UDR flag to send response.
				R1_status = (R1_status & R1_IDLE_STATE_MSK); //Clear all error flags when the host reads them.

				uint32_t block_number = command_arg; //block number/address is given in the argument

				HAL_StatusTypeDef SD_card_read_status = HAL_SD_ReadBlocks_DMA(&hsd1, data_block, block_number, 1);
				if(SD_card_read_status == HAL_ERROR)
				{
					//TODO: Send error token and move on
					Error_Handler();
				}

				state = waiting_for_SD_read_DMA;

				break;
			}

			default: { //Not an implemented/known command
				R1_status |= R1_ILLEGAL_CMD_MSK; //Set the illegal command bit in R1.
				/*LL_SPI_TransmitData8(SD_EMUL_SPI, command_num); //DEBUG
				LL_SPI_TransmitData8(SD_EMUL_SPI, (command_arg >> 24)); //Send word big-endian, so that the order the bytes appear in on the bus (and in the simplified spec document)
				LL_SPI_TransmitData8(SD_EMUL_SPI, (command_arg >> 16)); //are the same as the way they are organized in memory.
				LL_SPI_TransmitData8(SD_EMUL_SPI, (command_arg >> 8));  //(TransmitData32 is little-endian by necessity).
				LL_SPI_TransmitData8(SD_EMUL_SPI, (command_arg >> 0));
				LL_SPI_TransmitData16(SD_EMUL_SPI, command_CRC);*/
				LL_SPI_TransmitData8(SD_EMUL_SPI, R1_status); //Send R1 response.
				LL_SPI_ClearFlag_UDR(SD_EMUL_SPI); //Clear UDR flag to send response.
				R1_status = (R1_status & R1_IDLE_STATE_MSK); //Clear all error flags when the host reads them.
			}
			} //end of switch(command_num)

			//Get ready to receive another command:
			if(state == receiving_cmd_arg) { //If state hasn't been set to something else, then go back to awaiting_cmd
				state = awaiting_cmd; //State is once more awaiting_cmd
			}
			break;
		}

		case waiting_for_SD_read_DMA: {
			if(!SD_card_DMA_read_completed)
			{
				break; //Don't do anything if the SD card isn't finished.
			}
			//Else: (if the SD card has finished reading)
			state = transmitting_single_block;
			SD_card_DMA_read_completed = false; //Reset read_completed flag
			block_index = 0;

			LL_SPI_TransmitData8(SD_EMUL_SPI, SD_BLOCK_START_TOKEN); //Start by sending the block start token.

			LL_SPI_TransmitData8(SD_EMUL_SPI, data_block[block_index++]); //Transmit data from the block, and post-increment block index.
			LL_SPI_TransmitData8(SD_EMUL_SPI, data_block[block_index++]);
			LL_SPI_TransmitData8(SD_EMUL_SPI, data_block[block_index++]);
			LL_SPI_ClearFlag_UDR(SD_EMUL_SPI);
			//No break, we continue to transmitting_single_block.
		}

		case transmitting_single_block: {
			//Don't do anything with received data, it's only 0xFF anyways. TODO: sleep + make DMA automatically put received bytes in a trash register or something
			while(SD_EMUL_SPI->SR & SPI_SR_TXP) //While the TXP flag in the SPI status register is set (There is space for another packet in the TX FIFO):
			{
				LL_SPI_TransmitData8(SD_EMUL_SPI, data_block[block_index++]); //Transmit four bytes of data.
				LL_SPI_TransmitData8(SD_EMUL_SPI, data_block[block_index++]);
				LL_SPI_TransmitData8(SD_EMUL_SPI, data_block[block_index++]);
				LL_SPI_TransmitData8(SD_EMUL_SPI, data_block[block_index++]);
				if(block_index >= (SD_BLOCK_LENGTH - 4)) //If there are less than four bytes left in the data block:
				{
					while(!(SD_EMUL_SPI->SR & SPI_SR_TXP)) {;} //Wait until there is space in the TX FIFO (do nothing while the TXP flag isn't set)

					while(block_index < SD_BLOCK_LENGTH) //Send last data bytes
					{
						LL_SPI_TransmitData8(SD_EMUL_SPI, data_block[block_index++]);
					}

					while(!(SD_EMUL_SPI->SR & SPI_SR_TXP)) {;} //Wait until there is space in the TX FIFO (do nothing while the TXP flag isn't set)

					LL_SPI_TransmitData16(SD_EMUL_SPI, 0x0000); //Send two dummy bytes as "CRC" (potential TODO: actual CRC).

					state = awaiting_cmd; //Data transfer finished.
					break;
				}
			}

			break;
		}
		default: {
			//Should probably throw an error here if the state isn't a known state
			LL_SPI_TransmitData32(SD_EMUL_SPI, 0x0BBB3E12); //will print 123EBB0B on SPI bus
			LL_SPI_ClearFlag_UDR(SD_EMUL_SPI); //Clear UDR flag to send response.
			Error_Handler();
		}
		} //end of switch(state)
	}
	if(LL_SPI_IsActiveFlag_OVR(SD_EMUL_SPI))
	{
		HAL_GPIO_TogglePin(USER_LED1_GPIO_Port, USER_LED1_Pin); //Ideally this shouldn't happen.
		LL_SPI_ClearFlag_OVR(SD_EMUL_SPI);
	}

  /* USER CODE END SPI2_IRQn 0 */
  /* USER CODE BEGIN SPI2_IRQn 1 */

  /* USER CODE END SPI2_IRQn 1 */
}

/**
  * @brief This function handles SDMMC1 global interrupt.
  */
void SDMMC1_IRQHandler(void)
{
  /* USER CODE BEGIN SDMMC1_IRQn 0 */

  /* USER CODE END SDMMC1_IRQn 0 */
  HAL_SD_IRQHandler(&hsd1);
  /* USER CODE BEGIN SDMMC1_IRQn 1 */

  /* USER CODE END SDMMC1_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
