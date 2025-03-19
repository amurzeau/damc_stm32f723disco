# ISO IN first frame detection

As ISO IN transfer (feedback and mic data) are not occurring at each microframe, we need to find when the USB host is sending IN token.
To do this, either:
- We detect the first IN token using ITTXFE (IN token received when Tx FIFO is empty) interrupt.
- Or by preparing TX transfer on each microframe and checking which frame get the XFR (transfer complete) interrupt (DataIn() in usbd_audio.c).

Using ITTXFE (IN token received when Tx FIFO is empty) to detect the first ISO frame number lead to massive amount of interrupts due to the CDC BULK IN endpoint, as the host is constantly (multiple time per microframe) probing for CDC data using BULK IN tokens (most of them are NAK unless we need to send some OSC data to host).
It seems not possible to reduce this flood according to USB 2.0 specification (this is only specified for BULK OUT when the device is NAKing).

Due to this flood, it is preferable to use IISOIXFR (ISO IN Incomplete) interrupt as it happens at most once per microframe and its handling does not require a Global NAK mode (which is required only for ISO OUT Incomplete case).
So we are preparing a transfer for all microframes until we get a transfer complete interrupt (DataIn() is called). When DataIn() is called, we know which frame it was and when the next IN token will be send by the host.

# ISO OUT first frame detection

For the ISO OUT endpoints, we have the same choices, but things happen differently:
- We detect the first OUT token using OTEPDIS (OUT token received when endpoint disabled) interrupt.
- Or by preparing RX transfer on each microframe and checking which frame get the XFR interrupt.

The OTEPDIS interrupt doesn't have the same issue as ITTXFE. The host only sends OUT tokens when it has data to send. So it is not flooding OUT tokens on CDC BULK endpoint.

Preparing a RX transfer on each microframe could be a solution, but the handling of incomplete transfers is different from ISO IN transfers.
For ISO IN Incomplete interrupt, it is possible to only disable the transfer that was incomplete without touching other ISO IN endpoints, so we don't get glitches due to lost ISO IN transfers.

But for ISO OUT Incomplete interrupt, it is required to set the Global OUT NAK bit (SGONAK) to be able to disable the endpoint where the ISO OUT Incomplete occurred.
See STM32F7 reference manual, chapter 32.16.6 Device Programming Model in 32.16 OTG_FS/OTG_HS programming model
This means all ISO OUT endpoints are NAKed for a short period, leading to glitches on other running ISO OUT endpoints.

So for ISO OUT Incomplete endpoints, the solution is to use OTEPDIS instead.

# Other OS implementations

## Linux

On Linux, the first frame detection is done using:
- For ISO IN: NAK interrupt as an alternative to ITTXFE (https://github.com/torvalds/linux/blob/2d5404caa8c7bb5c4e0435f94b28834ae5456623/drivers/usb/dwc2/gadget.c#L2956)

The NAK interrupt flooding for BULK IN endpoints is probably not an issue as Linux runs on fast CPUs compared to microcontrollers.

- For ISO OUT: OTEPDIS interrupt (the same way as us) (https://github.com/torvalds/linux/blob/2d5404caa8c7bb5c4e0435f94b28834ae5456623/drivers/usb/dwc2/gadget.c#L2888)

Both were implemented in https://github.com/torvalds/linux/commit/5321922cb6fa0f513fdd8ce73b281f4b9957886b

## Zephyr

No first frame handling seems present.
The dwc2 driver only handle ISO incomplete interrupts.
The STM32 driver uses HAL.
Audio class doesn't seem to handle first frame issue.

## STM32 HAL

ISO First frame detection is not implemented.
