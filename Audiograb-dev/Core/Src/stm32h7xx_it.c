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
	HAL_GPIO_TogglePin(USER_LED1_GPIO_Port, USER_LED1_Pin);

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
	/*	This function is expecting compiler optimization to be turned on.
	 *	When debugging, consider changing optimization flags to -Og and
	 *	if things then break, try turning CPU speed up.
	 */

	//Static variables to keep track of SD emulator state.
	static bool cmd_is_ACMD; //Bool to indicate if the current command is an application-specific command (CMD55 before it)

	static uint8_t command_num; //bit #7 being set (0b10xxxxxx) indicates ACMD
	static uint32_t command_arg;
	static uint8_t command_CRC;

	enum SD_emulator_state {
		awaiting_cmd, receiving_cmd_arg, error
	};
	static enum SD_emulator_state state = awaiting_cmd;

	while(LL_SPI_IsActiveFlag_RXP(SD_EMUL_SPI)) //Repeat as long as there are packets to be read from rx FIFO:
	{
		switch(state) {
		case awaiting_cmd: {
			//HAL_GPIO_TogglePin(USER_LED2_GPIO_Port, USER_LED2_Pin); //Toggle LED for debugging.

			uint8_t received_data = LL_SPI_ReceiveData8(SD_EMUL_SPI); //Get one byte from RX FIFO
			if((received_data & SD_START_BITS_MASK) == SD_VALID_START_PATTERN) //If start-of-command pattern detected:
			{
				LL_SPI_SetFIFOThreshold(SD_EMUL_SPI, LL_SPI_FIFO_TH_05DATA); //Set FIFO threshold to 5, so that the interrupt won't fire again until the rest of the command is received.

				state = receiving_cmd_arg;
				command_num = (received_data & ~SD_START_BITS_MASK) | (cmd_is_ACMD << 7); //Mask out start bits, and add ACMD indication if applicable.
			}
			for(int i = 0; i < 30; ++i) //Delay i*3-ish cycles (see disassembly for accurate number). This is to ensure that enough time
			{							//passes for the RXP flag to reset after changing FIFO threshold. The minimum value at 144MHz F_CPU seems to be 90.
				__NOP();				//TODO: make a more robust solution to this problem.
			}
			break;
			}

		case receiving_cmd_arg: { //When the unit is finished receiving the command arguments and CRC:

			command_arg = (LL_SPI_ReceiveData8(SD_EMUL_SPI) << 24); //Retrieve command argument from RX FIFO
			command_arg |= (LL_SPI_ReceiveData8(SD_EMUL_SPI) << 16); //This is done using receive8 instead of 32 to make it little-endian
			command_arg |= (LL_SPI_ReceiveData8(SD_EMUL_SPI) << 8);
			command_arg |= (LL_SPI_ReceiveData8(SD_EMUL_SPI) << 0);
			command_CRC = LL_SPI_ReceiveData8(SD_EMUL_SPI); //Do the same with the last byte of the command

			switch(command_num) {
			case CMD(0): { //CMD0: go to idle state.
				//Should probably maybe technically be setting the IDLE bit to '1' here, but I'll get there when writing the part clearing the idle bit.
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

				//LL_SPI_TransmitData32(SD_EMUL_SPI, (command_arg));

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
					rest_of_R7 |= (command_arg & CMD8_VOLTAGE_ACCEPTED_MSK); //Send back the voltage accepted field to indicate the SD card accepts it.
				}
				rest_of_R7 |= (command_arg & R7_ECHO_BACK_MSK); //Send back the last byte of the argument.

				LL_SPI_TransmitData8(SD_EMUL_SPI, (rest_of_R7 >> 24)); //Send word big-endian, so that the order the bytes appear in on the bus
				LL_SPI_TransmitData8(SD_EMUL_SPI, (rest_of_R7 >> 16)); //are the same as the way they are organized in memory.
				LL_SPI_TransmitData8(SD_EMUL_SPI, (rest_of_R7 >> 8));  //(TransmitData32 is little-endian by necessity).
				LL_SPI_TransmitData8(SD_EMUL_SPI, (rest_of_R7 >> 0));
				LL_SPI_ClearFlag_UDR(SD_EMUL_SPI); //Begin transmission
				break;
			}

			default: { //Not an implemented/known command
				R1_status |= R1_ILLEGAL_CMD_MSK; //Set the illegal command bit in R1
				LL_SPI_TransmitData8(SD_EMUL_SPI, R1_status); //Send R1 response.
				LL_SPI_ClearFlag_UDR(SD_EMUL_SPI); //Clear UDR flag to send response.
				R1_status = (R1_status & R1_IDLE_STATE_MSK); //Clear all error flags when the host reads them.
			}
			}

			//Get ready to receive another command:
			LL_SPI_SetFIFOThreshold(SD_EMUL_SPI, LL_SPI_FIFO_TH_05DATA); //Set FIFO threshold back to 1,
			state = awaiting_cmd; //State is once more awaiting_cmd
			break;
		}
		default: {
			//Should probably throw an error here if the state isn't a known state
		}
		} //end of switch(state)
	}
	/*if(LL_SPI_IsActiveFlag_UDR(SD_EMUL_SPI))
	{
		HAL_GPIO_TogglePin(USER_LED1_GPIO_Port, USER_LED1_Pin); //Ideally this shouldn't happen.
		LL_SPI_ClearFlag_UDR(SD_EMUL_SPI);
	}*/

  /* USER CODE END SPI2_IRQn 0 */
  /* USER CODE BEGIN SPI2_IRQn 1 */

  /* USER CODE END SPI2_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
