# usb_audio_loopback_stm32f723disco
Use a 32F723EDISCOVERY eval board as a USB audio loopback to be able to post-process Windows audio

This project allow the use of a 32F723EDISCOVERY eval board as a way to loopback audio on Windows.
This way, it is possible to make Windows output audio to the eval board, then record back the audio and post-process it.

The audio path is like this:

![image](https://github.com/amurzeau/usb_audio_loopback_stm32f723disco/assets/5435069/b9adb736-e9c5-461f-9e07-33d51fc76fa5)

# Usage

To do this, you must do the following:

- Flash this firmware to a [32F723EDISCOVERY eval board](https://www.st.com/en/evaluation-tools/32f723ediscovery.html).
- Switch the power supply from STLINK USB cable to the USB HS cable by putting the jumper on USB HS on CN8 connector (under the board)
- Connect a micro-USB cable to the USB HS connector of the eval board:
![image](https://github.com/amurzeau/usb_audio_loopback_stm32f723disco/assets/5435069/a30fe00a-61e6-4fd6-afcb-3306aab80789)

4 audio outputs and 4 audio inputs should appear in Windows.
Each output is loopback to one input.

The supported audio format is 48000 Hz, Stereo, 16 bits.
