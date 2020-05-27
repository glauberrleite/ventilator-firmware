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
volatile int INHALE_TO_EXHALE;
volatile int EXHALE_TO_PAUSE;
volatile int PAUSE_TO_INHALE = 500;

volatile int bpm = 15;
volatile float ratio = 0.66;

typedef enum {
    IDLE,
    INHALE,
    EXHALE,
    PAUSE
} state;

volatile state current_state;
volatile int timer_counter;

hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// Print debug variables
bool print_state;
bool print_fl_int;
bool print_fl_pac;
bool print_pres_int;
bool print_pres_pac;
bool print_pres_ext;

bool print_valve_in;

volatile float SOFT_INS = 0;
volatile float VALVE_INS = 100;

// Custom functions

// Timer callback
void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  if (current_state == INHALE) {
    if (timer_counter >= INHALE_TO_EXHALE) {
      current_state = EXHALE;
      timer_counter = 0;
      SOFT_INS = 0;
    } else {
      timer_counter++;      
      
      SOFT_INS =  10 * 100 * timer_counter * 0.1 / INHALE_TO_EXHALE;
    }
  } else if (current_state == EXHALE) {
    if (timer_counter >= EXHALE_TO_PAUSE) {
      current_state = PAUSE;
      timer_counter = 0;
    } else {
      timer_counter++;
    }
  } else if (current_state == PAUSE) {
    if (timer_counter >= PAUSE_TO_INHALE) {
      current_state = INHALE;
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

int calculateInhale(float bpm, float ratio) {
  return (60000 / bpm) * (1 - ratio);
}

int calculateExhale(float bpm, float ratio) {
  return (60000 / bpm) * ratio;
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

  INHALE_TO_EXHALE = calculateInhale(bpm, ratio);
  EXHALE_TO_PAUSE = calculateExhale(bpm, ratio);
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
  
  if (print_pres_int) {
    Serial.print(sensors.getPRES_INT_cm3H2O());
    Serial.print("\t");
  }

    if (print_pres_ext) {
    Serial.print(sensors.getPRES_EXT_cm3H2O());
    Serial.print("\t");
  }

  if (print_pres_pac) {
    Serial.print(sensors.getPRES_PAC_cm3H2O());
    Serial.print("\t");
  }
  
  if (print_fl_pac) {
    Serial.print(sensors.getFL_PAC());
    //Serial.print(sensors.getDIFF_PRES_PAC_cm3H2O());
    Serial.print("\t");
  }

  if (print_valve_in) {
    Serial.print(SOFT_INS);
  }

  if (print_state || print_fl_int || print_fl_pac || print_pres_pac || print_pres_int || print_pres_ext || print_valve_in)
    Serial.println();

  // State machine
  switch (current_state) {
    case INHALE:
      valves.setINS_VALVE(SOFT_INS);
      valves.setEXP_VALVE(0);   
      break;
    case PAUSE:
      valves.setINS_VALVE(0);
      valves.setEXP_VALVE(0);
      break;
    case EXHALE:
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
      current_state = INHALE;
      timerAlarmEnable(timer);} 
      else if (part01.equals("PRINT")) {
        switch (int(value)) {
          case 0: print_state = true;
                  break;
          case 1: print_fl_int = true;
                  break;
          case 2: print_pres_int = true;
                  break;
          case 3: print_pres_ext = true;
                  break;
          case 4: print_pres_pac = true;
                  break;
          case 5: print_fl_pac = true;
                  break;
          default: break; 
        }
    } else if (part01.equals("AUTO")) {
      valves.setAUTO_SEC_VALVE(bool(value));
    } else if (part01.equals("MANUAL")) {
      valves.setMANUAL_SEC_VALVE(bool(value));
    } else if (part01.equals("VALVE_INS")) {
        VALVE_INS = value;
    } else if (part01.equals("BPM")) {
        INHALE_TO_EXHALE = calculateInhale(getValue(part02, ';', 0).toFloat(), getValue(part02, ';', 1).toFloat());
        EXHALE_TO_PAUSE = calculateExhale(getValue(part02, ';', 0).toFloat(), getValue(part02, ';', 1).toFloat());
    } else if (part01.equals("PAUSE")) {
        PAUSE_TO_INHALE = value;
    } else if (part01.equals("TEST")) {
        print_valve_in = true;
    } else {
      valves.setEXP_VALVE(value);
    }
  
  }
  delay(10);
}
