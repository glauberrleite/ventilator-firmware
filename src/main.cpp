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

Sensors sensors;
Valves valves;

// Setting state times, in milliseconds
volatile int PIP_TO_PLATEAU = 1000;
volatile int PLATEAU_TO_PEEP = 1000;
volatile int PEEP_TO_PIP = 2000;

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

// Print debug variables
bool print_state;
bool print_fl_int;
bool print_fl_pac_ins;
bool print_fl_pac_exp;
bool print_pres_pac;
bool print_pres_int;

float VALVE_INS = 0;

// Custom functions

// Timer callback
void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  if (current_state == PIP) {
    if (timer_counter >= PIP_TO_PLATEAU) {
      current_state = PLATEAU;
      timer_counter = 0;
      VALVE_INS = 0;
    } else {
      timer_counter++;
      
      // Soft Starter
      if (timer_counter < (PIP_TO_PLATEAU/2)) {
        VALVE_INS = 2 * 100 * timer_counter * 0.001 / timer_counter;
      } else {
        VALVE_INS = 100;
      }
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

  sensors.update();

  if (print_state) {
    Serial.print(current_state);
    Serial.print("\t");
  }

  if (print_fl_int) {
    Serial.print(sensors.getFL_INT());
    Serial.print("\t");
  }
  
  if (print_fl_pac_ins) {
    Serial.print(sensors.getFL_PAC_INS_cm3H2O());
    Serial.print("\t");
  }
  
  if (print_fl_pac_exp) {
    Serial.print(sensors.getFL_PAC_EXP_cm3H2O());
    Serial.print("\t");
  }
  
  if (print_pres_pac) {
    Serial.print(sensors.getPRES_PAC_cm3H2O());
    Serial.print("\t");
  }
  
  if (print_pres_int) {
    Serial.print(sensors.getPRES_INT_cm3H2O());
  }

  if (print_state || print_fl_int || print_fl_pac_ins || print_fl_pac_exp || print_pres_pac || print_pres_int)
    Serial.println();

  // State machine
  switch (current_state) {
    case PIP:
      valves.setINS_VALVE(VALVE_INS);
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
  
  // Receiving commands via serial
  if (Serial.available() > 0) {
    // Lê toda string recebida
    String received = readStringSerial();
  
    String part01 = getValue(received, ',', 0);
    String part02 = getValue(received, ',', 1);

    float value = part02.toFloat();

    if (part01.equals("START")) {
      current_state = PIP;
      timerAlarmEnable(timer);
    } else if (part01.equals("PRINT")) {
      print_state = bool(getValue(part02, ';', 0).toFloat());
      print_fl_int = bool(getValue(part02, ';', 1).toFloat());
      print_fl_pac_ins = bool(getValue(part02, ';', 2).toFloat());
      print_fl_pac_exp = bool(getValue(part02, ';', 3).toFloat());
      print_pres_pac = bool(getValue(part02, ';', 4).toFloat());
      print_pres_int = bool(getValue(part02, ';', 5).toFloat());
    } else if (part01.equals("AUTO")) {
      valves.setAUTO_SEC_VALVE(bool(value));
    } else if (part01.equals("MANUAL")) {
      valves.setMANUAL_SEC_VALVE(bool(value));
    } else if (part01.equals("INS")) {
      valves.setINS_VALVE(value);
    } else if (part01.equals("TEST")) {
      for (int i = 69; i <= 100; i++) {
        Serial.print(i);
        valves.setINS_VALVE(i);
        Serial.print('\t');
        delay(1000);
        sensors.update();
        Serial.print(sensors.getFL_INT());
        Serial.println();
      }
    } else if (part01.equals("PIP")) {
        PIP_TO_PLATEAU = value;
    } else if (part01.equals("PLATEAU")) {
        PLATEAU_TO_PEEP = value;
    } else if (part01.equals("PEEP")) {
        PEEP_TO_PIP = value;
    } else if (part01.equals("VALVE_INS")) {
        VALVE_INS = value;
    } else {
      valves.setEXP_VALVE(value);
    }
  
  }
  delay(10);
}
