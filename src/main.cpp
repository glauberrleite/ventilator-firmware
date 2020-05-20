/*
 *  Microcontroller module for the ventilator project developed by EASY Engineering and Systems Group
 *  Instituto de Computação - Universidade Federal de Alagoas
 *  @author glauberrleite
 *  @author vangasse
 */

#ifndef ARDUINO_H
#define ARDUINO_H
#include <Arduino.h>
#endif

#include "sensors.h"
#include "valves.h"

int cnt = 1;
Sensors sensors;
Valves valves;

// Setting state times, in milliseconds
#define PIP_TO_PLATEAU      3000
#define PLATEAU_TO_PEEP     3000
#define PEEP_TO_PIP         3000

typedef enum {
    IDLE,
    PIP,
    PLATEAU,
    PEEP
} state;

volatile state current_state;
volatile int timer_counter;

hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// Custom functions

// Timer callback
void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  if (current_state == PIP) {
    if (timer_counter >= PIP_TO_PLATEAU) {
      current_state = PLATEAU;
      timer_counter = 0;
    } else {
      timer_counter++;
    }
  } else if (current_state == PLATEAU) {
    if (timer_counter >= PLATEAU_TO_PEEP) {
      current_state = PEEP;
      timer_counter = 0;
    } else {
      timer_counter++;
    }
  } else if (current_state == PEEP) {
    if (timer_counter >= PEEP_TO_PIP) {
      current_state = PIP;
      timer_counter = 0;
    } else {
      timer_counter++;
    }
  }
  portEXIT_CRITICAL_ISR(&timerMux);
 
}

String readStringSerial(){
  String content = "";
  char character;
  
  // Enquanto receber algo pela serial
  while(Serial.available() > 0) {
    // Lê byte da serial
    character = Serial.read();
    // Ignora caractere de quebra de linha
    if (character != '\n'){
      // Concatena valores
      content.concat(character);
    }
    // Aguarda buffer serial ler próximo caractere
    delay(10);
  }    
  return content;
}

String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length()-1;

  for(int i=0; i<=maxIndex && found<=index; i++){
    if(data.charAt(i)==separator || i==maxIndex){
        found++;
        strIndex[0] = strIndex[1]+1;
        strIndex[1] = (i == maxIndex) ? i+1 : i;
    }
  }

  return found>index ? data.substring(strIndex[0], strIndex[1]) : "";
}

// Main setup and loop
void setup() {
  Serial.begin(9600);

  // Sensors and Valves init
  sensors = Sensors();
  valves = Valves();

  // Stating State
  current_state = IDLE;

  // Timer config
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 1000, true);
  timer_counter = 0;
}

void loop() {

  if (cnt > 100) {
    cnt = 0; 
    sensors.update();

    Serial.print(current_state);
    Serial.print("\t\t");
    Serial.print(sensors.getFL_INT());
    Serial.print("\t\t");
    Serial.print(sensors.getFL_PAC_INS());
    Serial.print("\t\t");
    Serial.print(sensors.getFL_PAC_EXP());
    Serial.print("\t\t");
    Serial.print(sensors.getPRES_PAC());
    Serial.print("\t\t");
    Serial.print(sensors.getPRES_INT());

    Serial.println();
  }

  switch (current_state) {
    case PIP:
      valves.setINS_VALVE(100);
      valves.setEXP_VALVE(0);
      break;
    case PLATEAU:
      valves.setINS_VALVE(0);
      valves.setEXP_VALVE(0);
      break;
    case PEEP:
      valves.setINS_VALVE(0);
      valves.setEXP_VALVE(100);
      break;
    default: break;
  }
  
  if (Serial.available() > 0) {
    // Lê toda string recebida
    String received = readStringSerial();
  
    String part01 = getValue(received, ',', 0);
    String part02 = getValue(received, ',', 1);

    float value = part02.toFloat();

    if (part01.equals("START")) {
      current_state = PIP;
      timerAlarmEnable(timer);
    } else if (part01.equals("AUTO")) {
      valves.setAUTO_SEC_VALVE(bool(value));
    } else if (part01.equals("MANUAL")) {
      valves.setMANUAL_SEC_VALVE(bool(value));
    } else if (part01.equals("INS")) {
      valves.setINS_VALVE(value);
    } else {
      valves.setEXP_VALVE(value);
    }

    Serial.print("Received: ");
    Serial.print(part01);
    Serial.print('\t');
    Serial.println(part02);
  
  }

  cnt++;
  delay(10);
}
