/*
 * sd_emulation.c
 *
 *  Created on: Apr 4, 2025
 *      Author: steinar
 */
#include "sd_emulation.h"

#include "sdmmc.h"

//Private function prototypes:
void SD_card_RX_complete_callback(SD_HandleTypeDef *hsd);
void SD_command_handler(uint8_t num, uint32_t arg, uint8_t crc);

//Global variable initializations
volatile bool card_initialized = false; //Variable to keep track of if the device is ready to tell the SD card host that it's initialized.

volatile uint8_t R1_status = R1_IDLE_STATE_MSK; //This is the variable containing the status bits of the R1 response. Initialized to no errors, and in idle state.

volatile uint32_t OCR = (OCR_CCS_MSK | OCR_POWERUP_STATUS_MSK | OCR_STATIC_PARAMS); //The operation conditions register stores the "card"'s accepted voltage range, and some other info.
																					//TODO: figure out if it's any point supporting cards less than 2GB (CCS bit will in that case not be set.)
volatile bool HCS = false;

volatile bool SD_card_DMA_read_completed = false;

//Function declarations:
void SD_emulation_init()
{
	MX_SPI2_Init(); //Init SPI.
	LL_SPI_SetUDRPattern(SD_EMUL_SPI, 0xFFFFFFFF); //Send only ones on TX buffer underrun, as this means the "SD card" is processing/busy.
	LL_SPI_SetUDRConfiguration(SD_EMUL_SPI, LL_SPI_UDR_CONFIG_REGISTER_PATTERN);
	LL_SPI_SetUDRDetection(SD_EMUL_SPI, LL_SPI_UDR_DETECT_END_DATA_FRAME); //register underrun immediately when the TX FIFO is empty.
	LL_SPI_EnableIOLock(SD_EMUL_SPI); //The SPI IO settings shouldn't change ever anyways (I think lol).

	//Configure DMA interrupts.
	LL_DMA_EnableIT_TC(SPI_TX_DMA_INSTANCE, SPI_TX_DMA_STREAM_NUM); //Enable DMA SPI TX interrupt

	//Setup interrupt-based command reception:
	LL_SPI_SetTransferSize(SD_EMUL_SPI, 0); //Transfer length unknown/indefinite.
	LL_SPI_EnableIT_RXP(SD_EMUL_SPI); //Enable "packet received" interrupt

	//Start reception of SPI. This is continual, and all of the "action" happens in the SPI interrupt callbacks. Dummy bytes are provided by the underrun system.
	LL_SPI_Enable(SD_EMUL_SPI);
	LL_SPI_TransmitData8(SD_EMUL_SPI, 0xFF); //Underrun system starts at the second byte, so we provide the first one.

	//init SD card
	while(HAL_GPIO_ReadPin(uSD_Detect_GPIO_Port, uSD_Detect_Pin) == GPIO_PIN_SET) {;} //Wait until SD card detected
	MX_SDMMC1_SD_Init();
	HAL_SD_RegisterCallback(&hsd1, HAL_SD_RX_CPLT_CB_ID, SD_card_RX_complete_callback);
	//TODO: register+handle error callback.
	card_initialized = true; //The card's been initialized if the code has managed to get this far.
}

void SD_card_RX_complete_callback(SD_HandleTypeDef *hsd) //NBNB this is for the actual SD card, not the SD emulator!
{
       HAL_GPIO_WritePin(USER_LED2_GPIO_Port, USER_LED2_Pin, GPIO_PIN_RESET); //debug
       SD_card_DMA_read_completed = true;
}
