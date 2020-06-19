# NiryoConveyor firmware 

Firmware to control a stepper motor for Niryo Conveyor with the custom Niryo Arduino-compatible board.
  
Communication interface: CAN bus (using MCP2515 to make an SPI-CAN interface) to control the conveyor with Niryo One 

Analog Input: potentiometer to control speed and direction. 

Digital input: Read IR Sensor status.

This firmware is used to control Niryo Conveyor Belt.
  
### How to upgrade the firmware on NiryoConveyor boards for Niryo One

Download the latest (recommended) version of the firmware.

You will need to install the Arduino IDE software, and install the “Arduino SAMD boards (32-bits ARM Cortex-M0+)” library from the boards manager (Tools -> Board -> Boards manager).

Choose “Arduino/Genuino Zero (Native USB Port)” in Tools -> Board

If your are going to control the conveyor with Niryo One, make sure that you put the correct ID (6 or 7) on the NiryoConveyor code, before you upload it to your conveyor belt. 

--> You can find a [complete tutorial on Niryo website](https://niryo.com/docs/niryo-one/update-your-robot/update-niryo-steppers/) (with photos and screenshots).

### How to communicate with a NiryoConveyor board using Can Bus

First you need to find a device that can send data on a CAN Bus. We recommend using MCP2515 on both Raspberry Pi 3 and Arduino to communicate between the boards. The MCP2515 allows you to make an SPI-CAN interface.
The robot Niryo One communicate with the NiryoConveyor board with Can Bus to control the speed, the direction and read back the conveyor status. 


### Documentation
For more details, please refer to the [Documentation](https://drive.google.com/uc?export=download&id=1tDDB3jQsZ0oynYOaSj9zD8i5R5m9koYb). 


