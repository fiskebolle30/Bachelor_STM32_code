/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins
     PG15   ------> FMC_SDNCAS
     PK5   ------> LTDC_B6
     PG14   ------> I2S6_SDO
     PG9   ------> OCTOSPIM_P1_IO6
     PD7   ------> OCTOSPIM_P1_IO7
     PD1   ------> FMC_D3_DA3
     PE0   ------> FMC_NBL0
     PK6   ------> LTDC_B7
     PK3   ------> LTDC_B4
     PJ15   ------> LTDC_B3
     PE1   ------> FMC_NBL1
     PK4   ------> LTDC_B5
     PG12   ------> I2S6_SDI
     PJ14   ------> LTDC_B2
     PD0   ------> FMC_D2_DA2
     PK7   ------> LTDC_DE
     PG13   ------> I2S6_CK
     PJ13   ------> LTDC_B1
     PJ12   ------> LTDC_B0
     PA10   ------> USART1_RX
     PA9   ------> USART1_TX
     PG8   ------> FMC_SDCLK
     PF1   ------> FMC_A1
     PF0   ------> FMC_A0
     PA8   ------> RCC_MCO_1
     PG6   ------> OCTOSPIM_P1_NCS
     PG5   ------> FMC_A15_BA1
     PF2   ------> FMC_A2
     PI12   ------> LTDC_HSYNC
     PF4   ------> FMC_A4
     PI14   ------> LTDC_CLK
     PI13   ------> LTDC_VSYNC
     PG4   ------> FMC_A14_BA0
     PK2   ------> LTDC_G7
     PK1   ------> LTDC_G6
     PF3   ------> FMC_A3
     PF5   ------> FMC_A5
     PF6   ------> OCTOSPIM_P1_IO3
     PF7   ------> OCTOSPIM_P1_IO2
     PJ11   ------> LTDC_G4
     PK0   ------> LTDC_G5
     PJ10   ------> LTDC_G3
     PJ9   ------> LTDC_G2
     PJ8   ------> LTDC_G1
     PF9   ------> OCTOSPIM_P1_IO1
     PD13   ------> I2C4_SDA
     PD14   ------> FMC_D0_DA0
     PD15   ------> FMC_D1_DA1
     PJ6   ------> LTDC_R7
     PJ7   ------> LTDC_G0
     PE12   ------> FMC_D9_DA9
     PD8   ------> FMC_D13_DA13
     PD10   ------> FMC_D15_DA15
     PD11   ------> OCTOSPIM_P1_IO0
     PD12   ------> I2C4_SCL
     PC1   ------> OCTOSPIM_P1_IO4
     PI15   ------> LTDC_R0
     PF13   ------> FMC_A7
     PE7   ------> FMC_D4_DA4
     PE13   ------> FMC_D10_DA10
     PH6   ------> FMC_SDNE1
     PH3   ------> OCTOSPIM_P1_IO5
     PC5   ------> OCTOSPIM_P1_DQS
     PJ0   ------> LTDC_R1
     PF11   ------> FMC_SDNRAS
     PF15   ------> FMC_A9
     PE14   ------> FMC_D11_DA11
     PE10   ------> FMC_D7_DA7
     PJ5   ------> LTDC_R6
     PD9   ------> FMC_D14_DA14
     PA0   ------> I2S6_WS
     PH5   ------> FMC_SDNWE
     PJ1   ------> LTDC_R2
     PJ4   ------> LTDC_R5
     PF14   ------> FMC_A8
     PG1   ------> FMC_A11
     PE9   ------> FMC_D6_DA6
     PE15   ------> FMC_D12_DA12
     PA3   ------> USB_OTG_HS_ULPI_D0
     PB2   ------> OCTOSPIM_P1_CLK
     PJ2   ------> LTDC_R3
     PJ3   ------> LTDC_R4
     PF12   ------> FMC_A6
     PG0   ------> FMC_A10
     PE8   ------> FMC_D5_DA5
     PE11   ------> FMC_D8_DA8
     PH7   ------> FMC_SDCKE1
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOK_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOJ_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOI, WIFI_BOOT_Pin|WIFI_WKUP_Pin|WIFI_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, USER_LED1_Pin|USER_LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(AUDIO_NRST_GPIO_Port, AUDIO_NRST_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LCD_BL_CTRL_Pin|LCD_ON_OFF_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : WIFI_GPIO_Pin WIFI_DATRDY_Pin */
  GPIO_InitStruct.Pin = WIFI_GPIO_Pin|WIFI_DATRDY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pins : PB9 PB6 PB8 PB7
                           PB5 PB1 PB13 PB12
                           PB0 PB11 PB10 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_6|GPIO_PIN_8|GPIO_PIN_7
                          |GPIO_PIN_5|GPIO_PIN_1|GPIO_PIN_13|GPIO_PIN_12
                          |GPIO_PIN_0|GPIO_PIN_11|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SDNCAS_Pin SDCLK_Pin A15_Pin A14_Pin
                           A11_Pin A10_Pin */
  GPIO_InitStruct.Pin = SDNCAS_Pin|SDCLK_Pin|A15_Pin|A14_Pin
                          |A11_Pin|A10_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_B6_Pin LCD_B7_Pin LCD_B4_Pin LCD_B5_Pin
                           LCD_DE_Pin LCD_G7_Pin LCD_G6_Pin LCD_G5_Pin */
  GPIO_InitStruct.Pin = LCD_B6_Pin|LCD_B7_Pin|LCD_B4_Pin|LCD_B5_Pin
                          |LCD_DE_Pin|LCD_G7_Pin|LCD_G6_Pin|LCD_G5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF14_LTDC;
  HAL_GPIO_Init(GPIOK, &GPIO_InitStruct);

  /*Configure GPIO pins : I2S6_SDO_Pin I2S6_SDI_Pin I2S6_CK_Pin */
  GPIO_InitStruct.Pin = I2S6_SDO_Pin|I2S6_SDI_Pin|I2S6_CK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI6;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : PG10 PG7 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : OCSPI1_IO6_Pin */
  GPIO_InitStruct.Pin = OCSPI1_IO6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_OCTOSPIM_P1;
  HAL_GPIO_Init(OCSPI1_IO6_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OCSPI1_IO7_Pin */
  GPIO_InitStruct.Pin = OCSPI1_IO7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OCTOSPIM_P1;
  HAL_GPIO_Init(OCSPI1_IO7_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PD4 PD6 PD5 PD3 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_5|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : D3_Pin D2_Pin D0_Pin D1_Pin
                           D13_Pin D15_Pin D14_Pin */
  GPIO_InitStruct.Pin = D3_Pin|D2_Pin|D0_Pin|D1_Pin
                          |D13_Pin|D15_Pin|D14_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : WIFI_BOOT_Pin WIFI_WKUP_Pin WIFI_RST_Pin */
  GPIO_InitStruct.Pin = WIFI_BOOT_Pin|WIFI_WKUP_Pin|WIFI_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pins : PE3 PE4 PE5 PE6
                           PE2 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : FMC_NBL0_Pin FMC_NBL1_Pin D9_Pin D4_Pin
                           D10_Pin D11_Pin D7_Pin D6_Pin
                           D12_Pin D5_Pin D8_Pin */
  GPIO_InitStruct.Pin = FMC_NBL0_Pin|FMC_NBL1_Pin|D9_Pin|D4_Pin
                          |D10_Pin|D11_Pin|D7_Pin|D6_Pin
                          |D12_Pin|D5_Pin|D8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : USER_LED1_Pin AUDIO_NRST_Pin USER_LED2_Pin */
  GPIO_InitStruct.Pin = USER_LED1_Pin|AUDIO_NRST_Pin|USER_LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_B3_Pin LCD_B2_Pin LCD_B1_Pin LCD_B0_Pin
                           LCD_G4_Pin LCD_G3_Pin LCD_G2_Pin LCD_G1_Pin
                           LCD_R7_Pin LCD_G0_Pin LCD_R1_Pin LCD_R6_Pin
                           LCD_R2_Pin LCD_R5_Pin LCD_R3_Pin LCD_R4_Pin */
  GPIO_InitStruct.Pin = LCD_B3_Pin|LCD_B2_Pin|LCD_B1_Pin|LCD_B0_Pin
                          |LCD_G4_Pin|LCD_G3_Pin|LCD_G2_Pin|LCD_G1_Pin
                          |LCD_R7_Pin|LCD_G0_Pin|LCD_R1_Pin|LCD_R6_Pin
                          |LCD_R2_Pin|LCD_R5_Pin|LCD_R3_Pin|LCD_R4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF14_LTDC;
  HAL_GPIO_Init(GPIOJ, &GPIO_InitStruct);

  /*Configure GPIO pins : PH15 PH14 PH13 PH12
                           PH4 PH10 PH9 PH8
                           PH11 */
  GPIO_InitStruct.Pin = GPIO_PIN_15|GPIO_PIN_14|GPIO_PIN_13|GPIO_PIN_12
                          |GPIO_PIN_4|GPIO_PIN_10|GPIO_PIN_9|GPIO_PIN_8
                          |GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pin : uSD_Detect_Pin */
  GPIO_InitStruct.Pin = uSD_Detect_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(uSD_Detect_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PI6 PI7 PI9 PI10
                           PI11 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_9|GPIO_PIN_10
                          |GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI2_NSS_Pin LCD_BL_CTRL_Pin LCD_ON_OFF_Pin */
  GPIO_InitStruct.Pin = SPI2_NSS_Pin|LCD_BL_CTRL_Pin|LCD_ON_OFF_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : VCP_RX_Pin VCP_TX_Pin */
  GPIO_InitStruct.Pin = VCP_RX_Pin|VCP_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PC7 PC6 PC2 PC0
                           PC4 PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_6|GPIO_PIN_2|GPIO_PIN_0
                          |GPIO_PIN_4|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : WAKEUP_Pin */
  GPIO_InitStruct.Pin = WAKEUP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(WAKEUP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : A1_Pin A0_Pin A2_Pin A4_Pin
                           A3_Pin A5_Pin A7_Pin SDNRAS_Pin
                           A9_Pin A8_Pin A6_Pin */
  GPIO_InitStruct.Pin = A1_Pin|A0_Pin|A2_Pin|A4_Pin
                          |A3_Pin|A5_Pin|A7_Pin|SDNRAS_Pin
                          |A9_Pin|A8_Pin|A6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : MCO_Pin */
  GPIO_InitStruct.Pin = MCO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(MCO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OCSPI1_NCS_Pin */
  GPIO_InitStruct.Pin = OCSPI1_NCS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OCTOSPIM_P1;
  HAL_GPIO_Init(OCSPI1_NCS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_HSYNC_Pin LCD_CLK_Pin LCD_VSYNC_Pin LCD_R0_Pin */
  GPIO_InitStruct.Pin = LCD_HSYNC_Pin|LCD_CLK_Pin|LCD_VSYNC_Pin|LCD_R0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF14_LTDC;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pins : OCSPI1_IO3_Pin OCSPI1_IO2_Pin OCSPI1_IO1_Pin */
  GPIO_InitStruct.Pin = OCSPI1_IO3_Pin|OCSPI1_IO2_Pin|OCSPI1_IO1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OCTOSPIM_P1;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PF8 PF10 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : I2C4_SDA_Pin I2C4_SCL_Pin */
  GPIO_InitStruct.Pin = I2C4_SDA_Pin|I2C4_SCL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C4;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : OCSPI1_IO0_Pin */
  GPIO_InitStruct.Pin = OCSPI1_IO0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_OCTOSPIM_P1;
  HAL_GPIO_Init(OCSPI1_IO0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OCSPI1_IO4_Pin OCSPI1_DQS_Pin */
  GPIO_InitStruct.Pin = OCSPI1_IO4_Pin|OCSPI1_DQS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OCTOSPIM_P1;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_INT_Pin */
  GPIO_InitStruct.Pin = LCD_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LCD_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SDNE1_Pin SDNWE_Pin SDCKE1_Pin */
  GPIO_InitStruct.Pin = SDNE1_Pin|SDNWE_Pin|SDCKE1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pin : OCSPI1_IO5_Pin */
  GPIO_InitStruct.Pin = OCSPI1_IO5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_OCTOSPIM_P1;
  HAL_GPIO_Init(OCSPI1_IO5_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : I2S6_WS_Pin */
  GPIO_InitStruct.Pin = I2S6_WS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI6;
  HAL_GPIO_Init(I2S6_WS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG1_HS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : OCSPI1_CLK_Pin */
  GPIO_InitStruct.Pin = OCSPI1_CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_OCTOSPIM_P1;
  HAL_GPIO_Init(OCSPI1_CLK_GPIO_Port, &GPIO_InitStruct);

  /*AnalogSwitch Config */
  HAL_SYSCFG_AnalogSwitchConfig(SYSCFG_SWITCH_PA1, SYSCFG_SWITCH_PA1_CLOSE);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
