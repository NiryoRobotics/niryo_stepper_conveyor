/*
    CanBus.h
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

#ifndef CAN_BUS_H
#define CAN_BUS_H

#include "config.h"
#include "mcp_can.h"
#include "A4954.h"


class CanBus {

  private:

    // CAN RX Variables
    long unsigned int rxId;
    unsigned char len;
    unsigned char rxBuf[8];
    uint8_t new_motor_id;
    int conveyor_status;
    int conveyor_speed;
    int conveyor_direction;
    long steps_position;
    unsigned long delay_steps;

    uint8_t  motor_id;
    MCP_CAN* can_driver;
    int can_receive ; 


  public:


    CanBus() {}
    CanBus(MCP_CAN* mcp, uint8_t motor_id);

    void setup();
    void controlConveyor();
    void readData();
    void writeFirmwareVersion(uint8_t major, uint8_t minor, uint8_t patch);
    void writeConveyorfeedback();
    void updateId(uint8_t new_motor_id);
    bool available();
    void read();
    int getConveyorStatus(); 
};

#endif
