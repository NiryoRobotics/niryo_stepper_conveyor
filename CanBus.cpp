/*
    CanBus.cpp
    Copyright (C) 2020 Niryo

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "CanBus.h"

CanBus::CanBus(MCP_CAN* mcp, uint8_t motor_id)
{
  can_driver = mcp;

  this->motor_id = motor_id;
  conveyor_status = 0;
  conveyor_speed = 0;
  conveyor_direction = 1;
  steps_position = 1;

}

void CanBus::setup()
{
  // Initialize MCP2515 running at 16MHz with a baudrate of 1000kb/s
  if (can_driver->begin(MCP_STDEXT, CAN_1000KBPS, MCP_16MHZ) == CAN_OK) {
    SerialUSB.println("MCP2515 Initialized Successfully!");
  }
  else {
    SerialUSB.println("Error Initializing MCP2515...");
  }

  // Set interrupt pin as input
  pinMode(CAN_PIN_INT, INPUT);

  // Set filters - only accept frames for current motor ID, and CAN broadcast ID
  can_driver->init_Mask(0, 0, 0x000F0000);
  can_driver->init_Filt(0, 0, motor_id * 65536);
  can_driver->init_Mask(1, 0, 0x000F0000);
  can_driver->init_Filt(2, 0, CAN_BROADCAST_ID * 65536);

  can_driver->setMode(MCP_NORMAL);
  SerialUSB.println("MCP2515 CAN started");
}


void CanBus::controlConveyor()
{

  if (conveyor_status)
  {
    if (conveyor_speed == 0)
    {
      relaxed_mode_with_resistance();
      fan_LOW();
    }
    else
    {
      fan_HIGH();
      if ( conveyor_speed > 49) {
        delay_steps = (-4.4 * conveyor_speed) + 590;
      }
      else if (conveyor_speed < 49)   {
        delay_steps = (-40.75 * conveyor_speed) + 2407;
      }
      if ( conveyor_direction == 255)
      {
        conveyor_direction = -1;
      }
      // delay_steps = (10 * (100 - conveyor_speed)) + 200;
      output(-1800 * conveyor_direction * steps_position / 8, UMAX_80_PERCENT); // 32 microsteps -> more precision
      steps_position = steps_position + 1;
      delayMicroseconds(delay_steps);
    }
  }
  //  else {
  //    relaxed_mode_with_resistance();
  //      fan_LOW();
  //  }

}

void CanBus::readData()
{
  long unsigned int rxId;
  unsigned char len;
  unsigned char rxBuf[8];
  can_driver->readMsgBuf(&rxId, &len, rxBuf);
  // read data (len 8) : 291 micros, (len : 1) : 224 micros

  // check id is standard, not extended
  if ((rxId & 0x80000000) == 0x80000000) {
    SerialUSB.println("Extended ID, nop nop nop.");
    return;
  }
  // check message is not a remote request frame
  if ((rxId & 0x40000000) == 0x40000000) {
    SerialUSB.print("Remote request frame");
    return;
  }
  if (len < 1) {
    SerialUSB.print("not length");
    return;
  }

  // check cmd
  uint8_t cmd = rxBuf[0];
  if (cmd == CAN_CMD_MODE)
  {
    uint8_t control_mode = rxBuf[1];

    // turn on the conveyor
    if (control_mode == STEPPER_CONVEYOR_ON) {
      conveyor_speed = rxBuf[2];
      conveyor_direction = rxBuf[3];
      conveyor_status = 1;
    }

    // turn off the conveyor
    else if (control_mode == STEPPER_CONVEYOR_OFF)
    {
      relaxed_mode_with_resistance();
      fan_LOW();
      conveyor_status = 0;
      conveyor_speed = 0;
      conveyor_direction = 1;
      steps_position = 0;

    }
    // update Motor ID
    else if (control_mode == CAN_UPDATE_CONVEYOR_ID) {
      // change motor id
      new_motor_id = rxBuf[2];
      updateId(new_motor_id);
      setup();
      conveyor_status = 0;
      conveyor_speed = 0;
      conveyor_direction = 1;
      relaxed_mode_with_resistance();
      fan_LOW();
    }
  }
}
/*
   send conveyor feedback ( status , direction and speed)
*/
void CanBus::writeConveyorfeedback()
{ uint8_t data[] = {CAN_DATA_CONVEYOR_STATE, conveyor_status, conveyor_speed, conveyor_direction};
  can_driver->sendMsgBuf(0x10 + motor_id, 0, 4, data);
}



/*
   Check if a CAN frame has arrived
*/
bool CanBus::available()
{
  return !digitalRead(CAN_PIN_INT);
}

/*
   Read from CAN bus
   - check ID and length
   - get command (first data byte)
   - execute command action with the rest of data
*/
void CanBus::read()
{
  can_driver->readMsgBuf(&rxId, &len, rxBuf);     // read data (len 8) : 291 micros, (len : 1) : 224 micros

  // check id is standard, not extended
  if ((rxId & 0x80000000) == 0x80000000) {
    SerialUSB.println("Extended ID, nop nop nop.");
    return;
  }
  // check message is not a remote request frame
  if ((rxId & 0x40000000) == 0x40000000) {
    SerialUSB.print("Remote request frame");
    return;
  }
  if (len < 1) return;
}
/*
   update conveyor id with the new id
*/
void CanBus::updateId(uint8_t new_motor_id)
{
  this->motor_id = new_motor_id;
}

int CanBus::getConveyorStatus()
{
  return (conveyor_status);
}
