# How to flash using STLINK USB Mass Storage

If your STM32 board or STLINK probe has a Mass Storage device, you can flash the device by following these instructions:
 - Plug in the STM32 board STLINK USB cable
 - Wait for the USB Mass Storage device to appear. It should have the name DIS_F723IE (for STM32F723 discovery)
 - Copy either damc_stm32f723disco.hex or damc_stm32f723disco.bin to the USB Mass Storage
 - This will flash the STM32F7 board
 - The board is now ready to use
