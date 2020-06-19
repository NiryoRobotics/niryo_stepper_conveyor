/*
    NiryoStepper.ino
    author Sarra EL GHALI
    Version 1.0
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

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#include "config.h"
#include "utils.h"
#include "A4954.h"
#include "CanBus.h"

#define Serial SerialUSB

#define MOTOR_ID   6  // Default ID for every conveyor stepper is 6. additionnal ID 7 is possible 

uint8_t motor_id = MOTOR_ID;

// Can driver
MCP_CAN can_driver(CAN_PIN_CS);

// Can Bus (will transfer commands and data between CAN driver and stepper controller)
CanBus canBus(&can_driver, motor_id);

unsigned long time_last_write_state = micros();
unsigned long write_frequency_state = 40000; // 12.5Hz

// Steppers variables
long steps_position;
unsigned long delay_steps;

// variable to control conveyor for the external control box
int external_conveyor_speed;
int external_conveyor_direction = 1;
int potentiometer_pin = A0;   // external_conveyor_speed potentiometer
int digital_input_pin = 0;  // IR digital input
float vcc = 4.272; // Voltage divider input voltage
int resistor = 10; // 10kohm
int k = 25; //Slope


void autonomeConveyorControl(); // fonction to drive the conveyor with the control box

//////////////////////////////////////
/////////////////SETUP////////////////
//////////////////////////////////////

void setup() {
  SerialUSB.begin(115200);
  delay(2000);
  SerialUSB.println("-------------- START --------------");
  canBus.setup();

  Wire.begin();
  Wire.setClock(1000000); // 1 Mbits
  delay(100);

  // start fan
  setup_fan();
  fan_LOW();
  init_driver(); // init Stepper driver
  relaxed_mode_with_resistance();
  pinMode(potentiometer_pin, INPUT_PULLUP);
  pinMode(digital_input_pin, INPUT);

  SerialUSB.println("-------------- SETUP FINISHED --------------");
}

//////////////////////////////////////
/////////////////LOOP/////////////////
//////////////////////////////////////

void loop() {

  /*
     if the conveyor is not activated on the side of Niryo One --> drive it with the control box
  */
  if (!canBus.getConveyorStatus()) {
    autonomeConveyorControl();

  }

  /* Control the conveyor via Niryo One

  */
  canBus.controlConveyor();
  /* Read data send from the robot
     // - start or stop conevyor
     // - change speed ,direction
     // - update ID
  */
  if (canBus.available()) {
    canBus.readData();
  }

  /*
     // send conveyor data ( status , speed , direction)
  */
  if (micros() - time_last_write_state > write_frequency_state) {
    time_last_write_state += write_frequency_state;
    canBus.writeConveyorfeedback();

  }
}

/*  Read potentiometer value and vary conevyor speed
    + change direction
*/
void autonomeConveyorControl()
{
  int potentiometer_value = analogRead(potentiometer_pin);
  float adc_value = (3.3 * potentiometer_value * resistor * k) / ((1024 * vcc) - (3.3 * potentiometer_value)); // linearizing  ADC value
 // if(potentiometer_value == 1023)  {
   // return; 
  //}
  SerialUSB.println(potentiometer_value);
  SerialUSB.println(digitalRead(digital_input_pin));
  if ( adc_value <= 300) {
    external_conveyor_speed = map((int)adc_value, 0, 300, 100, 0);
    external_conveyor_direction = 1;
  }

  else if (adc_value > 470)
  {
    external_conveyor_speed = map((int)adc_value, 470, 800, 0, 100);
    external_conveyor_direction = -1;
    if (adc_value > 800) external_conveyor_speed = 100;
  }
  else if ((adc_value > 300) and (adc_value <= 470))
  {
    external_conveyor_speed = 0;
  }
  //SerialUSB.println(adc_value);
  // if sensor detect an object or the speed == 0 , stop conveyor
  if ((external_conveyor_speed == 0) or (digitalRead(digital_input_pin) == LOW) or (potentiometer_value == 1023))
  {
    relaxed_mode_with_resistance();
    fan_LOW();
  }
  else
  {
    fan_HIGH();
    delay_steps = (10 * ( 100 - external_conveyor_speed)) + 10;
    //SerialUSB.println(external_conveyor_speed);
    output(-1800 * external_conveyor_direction * steps_position / 3, uMAX);
    steps_position = steps_position + 1;
    delayMicroseconds(delay_steps);
  }
}
