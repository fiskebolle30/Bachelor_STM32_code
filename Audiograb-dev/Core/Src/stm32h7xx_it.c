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

	//Static variables to keep track of SD emulator state.
	static bool cmd_is_ACMD; //Bool to indicate if the current command is an application-specific command (CMD55 before it)

	static uint8_t command_num; //bit #7 being set (0b10xxxxxx) indicates ACMD
	static uint32_t command_arg;
	static uint8_t command_CRC;

	enum SD_emulator_state {
		awaiting_cmd, receiving_cmd_arg, error
	};
	static enum SD_emulator_state state = awaiting_cmd;

	while((SD_EMUL_SPI->SR & SPI_SR_RXP_Msk) == (SPI_SR_RXP)) //Repeat as long as there are packets to be read from rx FIFO:
	{
		switch(state) {
		case awaiting_cmd: {
			//HAL_GPIO_TogglePin(USER_LED2_GPIO_Port, USER_LED2_Pin); //Toggle LED for debugging.

			uint8_t received_data = (*((__IO uint8_t *)&SD_EMUL_SPI->RXDR)); //Get one byte from RX FIFO
			if((received_data & SD_START_BITS_MASK) == SD_VALID_START_PATTERN) //If start-of-command pattern detected:
			{
				MODIFY_REG(SD_EMUL_SPI->CFG1, SPI_CFG1_FTHLV, LL_SPI_FIFO_TH_05DATA); //Set FIFO threshold to 5, so that the interrupt won't fire again until the rest of the command is received.

				state = receiving_cmd_arg;
				command_num = (received_data & ~SD_START_BITS_MASK) | (cmd_is_ACMD << 7); //Mask out start bits, and add ACMD indication if applicable.
			}
			break;
			}

		case receiving_cmd_arg: { //When the unit is finished receiving the command arguments and CRC:

			command_arg = LL_SPI_ReceiveData32(SD_EMUL_SPI); //Retrieve command argument from RX FIFO
			command_CRC = LL_SPI_ReceiveData8(SD_EMUL_SPI); //Do the same with the last byte of the command

			switch(command_num) {
			case CMD(0): { //CMD0: go to idle state.
				*((__IO uint8_t *)&SD_EMUL_SPI->TXDR) = 0x01; //Send dummy "I'm Ok" R1 response. Maybe change to actually transmit errors if applicable?
				SET_BIT(SD_EMUL_SPI->IFCR, SPI_IFCR_UDRC); //Clear UDR flag to send response.
				break;
			}

			default: {

			}
			}

			//Get ready to receive another command:
			MODIFY_REG(SD_EMUL_SPI->CFG1, SPI_CFG1_FTHLV, LL_SPI_FIFO_TH_01DATA); //Set FIFO threshold back to 1,
			state = awaiting_cmd; //State is once more awaiting_cmd
			break;
		}
		default: {
			//Should probably throw an error if the state isn't any of the above handled ones.
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
