# Bachelor_STM32_code
All STM32 microcontroller code for the Audiograb bachelor project.

The Audiograb-dev folder contains the STM32CubeIDE project for the STM32H7B3I-DK devkit, which was used for development before the hardware prototype was designed and produced.

The Audiograb folder contains the STM32CubeIDE project for the hardware prototype which was designed during the bachelor. It has mostly been set up using STM32CubeMX, but the code from Audiograb-dev has not yet been ported.

The Pico_SPI_master folder contains the Raspberry Pi Pico SDK project, used with a Raspberry Pi Pico W (or similar) to send SD commands over the SPI bus for testing.

The Test_projects folder contains projects developed to get familiar with the STM32H7 microcontroller family.

## A note on devkit-to-prototype migration
There are some differences between the STM32H7B3I-DK and the hardware prototype. 

The SPI peripheral used for SD emulation was SPI2 on the devkit, but it is SPI1 on the prototype. Thus, the SD_EMUL_SPI define has to be changed from SPI2 to SPI1, and the SPI2 interrupt moved/copied to the SPI1 interrupt.

The NSS pin (called the CS pin in the schematic) was also not connected to the SPI1 peripheral's hardware NSS pin on the prototype, but rather to PE7. This means software management of the NSS pin should be utilised, this is handled in section 55.4.7 of the STM32H7 reference manual. I would recommend looking into GPIO interrupts to manage the SSI bit.

In summary, the only files that should be copied from Audiograb-dev to Audiograb are the main.c, sd_emulation.h and sd_emulation.c. The SPI interrupt should be copied between the stm32h7xx_it.c files.
