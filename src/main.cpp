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

#define Ts    0.05
#define BPM   15
#define RATIO 0.66


Sensors sensors;
Valves valves;

volatile float plateau = 15;
volatile float plateau_ref;

// Setting state times, in milliseconds
volatile int INHALE_TO_EXHALE = 5000;
volatile int PAUSE_TO_INHALE = 500;
volatile int EXHALE_TO_PAUSE = 5000;

typedef enum {
    IDLE,
    INHALE,
    EXHALE,
    PAUSE,
    TEST
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

float VALVE_INS = 0;

// PID variables
float Kp = 0.1;
float Ki = 0.001;
float Kd = 0.001;

float delta_ins = 0;
float error = 0;
float ierror = 0;
float derror = 0;
float prev_error = 0;

// Custom functions

// Timer callback
void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  if (current_state == INHALE) {
    if (timer_counter >= INHALE_TO_EXHALE) {
      current_state = EXHALE;
      timer_counter = 0;
    } else {
      timer_counter++;
    }
  } else if (current_state == EXHALE) {
    if (timer_counter >= EXHALE_TO_PAUSE) {
      current_state = INHALE;
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

  //INHALE_TO_EXHALE = calculateInhale(BPM, RATIO);
  //EXHALE_TO_PAUSE = calculateExhale(BPM, RATIO);
}

int ins_pwm = 0;
void loop() {

  sensors.update();
  
  // Printing variables
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
    Serial.print(VALVE_INS);
  }

  if (print_state || print_fl_int || print_fl_pac || print_pres_pac || print_pres_int || print_pres_ext || print_valve_in) {
    Serial.print(float(ins_pwm)/100);
    Serial.print("\t");
    //Serial.print(plateau_ref);
    Serial.println();
  }

  // State machine

  switch (current_state) {
    case INHALE:
      plateau = 15;

      // Setpoint
      if (timer_counter < (INHALE_TO_EXHALE)) {
        plateau_ref = ( plateau * timer_counter) / INHALE_TO_EXHALE;
      }

      // Using plateau as reference, adjusting PID
      if (sensors.getPRES_PAC_cm3H2O() > 30) {
        current_state = EXHALE;
      }
      error = plateau_ref - sensors.getPRES_PAC_cm3H2O();
      
      ierror += error * Ts;
      derror = (error - prev_error) / Ts;

      delta_ins = Kp * error + Ki * ierror + Kd * derror;

      // Constraint in control action
      //delta_ins = delta_ins > 10 ? 10 : delta_ins;
      //delta_ins = delta_ins < -30 ? -30 : delta_ins;

      VALVE_INS = delta_ins;
    
      // Constraint in control action
      VALVE_INS = VALVE_INS > 100 ? 100 : VALVE_INS;
      VALVE_INS = VALVE_INS < 0 ? 0 : VALVE_INS;

      valves.setINS_VALVE(VALVE_INS);
      valves.setEXP_VALVE(0);

      prev_error = error;
      break;
    case EXHALE:
      // Reset Inhale PID
      plateau = 0;
      plateau_ref = 0;
      ierror = 0;
      prev_error = 0;
      VALVE_INS = 0;

      // Exhale valves configuration
      valves.setINS_VALVE(0);
      valves.setEXP_VALVE(100);
      break;
    case PAUSE:
      valves.setINS_VALVE(0);
      valves.setEXP_VALVE(0);
      break;
    case TEST:
      valves.setINS_VALVE_PWM(ins_pwm);
      ins_pwm++;
      ins_pwm = ins_pwm > 1023 ? 1023 : ins_pwm;
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
        valves.setINS_VALVE(value);
    } else if (part01.equals("BPM")) {
        INHALE_TO_EXHALE = calculateInhale(getValue(part02, ';', 0).toFloat(), getValue(part02, ';', 1).toFloat());
        EXHALE_TO_PAUSE = calculateExhale(getValue(part02, ';', 0).toFloat(), getValue(part02, ';', 1).toFloat());
    } else if (part01.equals("PAUSE")) {
        PAUSE_TO_INHALE = value;
    } else if (part01.equals("PLATEAU")) {
        plateau = value;
    } else if (part01.equals("PRINT_INS")) {
        print_valve_in = true;
    } else if (part01.equals("PID")) {
        Kp = getValue(part02, ';', 0).toFloat();
        Ki = getValue(part02, ';', 1).toFloat();
        Kd = getValue(part02, ';', 2).toFloat();
    } else if (part01.equals("COEF")) {
        sensors.a1 = getValue(part02, ';', 0).toFloat();
        sensors.a2 = getValue(part02, ';', 1).toFloat();
        sensors.a3 = getValue(part02, ';', 2).toFloat();
    } else if (part01.equals("TEST")) {
        current_state = TEST;
        delay(5000);
    } else {
      valves.setEXP_VALVE(value);
    }
  
  }
  delay(Ts * 1000); // Ts is in seconds, but delay is in milliseconds
}
