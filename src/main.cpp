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

#define Ts    0.001
#define BPM   15
#define RATIO 0.66


Sensors sensors;
Valves valves;

volatile float pres_peak = 10;
volatile float pres_ref;
volatile float volume_ref = 30;
float flow;
float volume;

// Setting state times, in milliseconds
volatile int INHALE_TO_EXHALE = 5000;
volatile int EXHALE_TO_INHALE = 5000;
volatile int TRANSITION_TIME = 300;


typedef enum {
    IDLE,
    INHALE,
    TRANSITION,
    PLATEAU,
    EXHALE,
    TEST
} state;

volatile state current_state;
volatile int timer_counter;

hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// Print debug variables
float VALVE_INS = 0;

float pres_init = 0;

// PID variables
float Kp = 25;
float Ki = 2;
float Kd = 2;

float alpha = 0.1;
float pid_out = 0;
float delta_ins = 0;
float error = 0;
float derror = 0;
float pid_prop = 0;
float pid_int = 0;
float pid_der = 0;
float prev_error = 0;
float prev_pid_out = 0;
float prev_ierror = 0;
float prev_derror = 0;
float delta_u = 0;
float tau_aw = 1;

// PEEP control variables
float peep_value = 0;
bool offexpvalve = false;
int timer_peep = 100;
bool PEEP;
float p1 = 0.1;
float p2 = 3;
// Custom functions

// Timer callback
void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  if (current_state == INHALE) {
    if (timer_counter >= INHALE_TO_EXHALE) {
      current_state = TRANSITION;
      timer_counter = 0;
    } else {
      timer_counter++;
    }
  } else if (current_state == TRANSITION){
    if (timer_counter >= TRANSITION_TIME) {
      current_state = EXHALE;
      timer_counter = 0;
    } else {
      timer_counter++;
    }
  } else if (current_state == EXHALE) {
    if (timer_counter >= EXHALE_TO_INHALE) {
      current_state = INHALE;
      timer_counter = 0;
      offexpvalve = false;
    } else {
      timer_counter++;
      if (EXHALE_TO_INHALE - timer_counter <10) offexpvalve = true;
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

  // Other variables initialization
  INHALE_TO_EXHALE = calculateInhale(BPM, RATIO);
  EXHALE_TO_INHALE = calculateExhale(BPM, RATIO);
  tau_aw = sqrt(1/(Ki * Kd));
  flow = 0;
  volume = 0;
}

float ins = 0;
bool limit = false;
void loop() {

  // Read sensors 
  sensors.update();
  
  // calculate flow and volume
  if (current_state == EXHALE) {
    flow = sensors.getFL_PAC();
  } else {
    flow = sensors.getFL_INT();
  }
  volume += flow * (Ts * 60) * 0.001; // Volume in mL

  // Printing variables
  Serial.print(current_state);
  Serial.print(",");

  Serial.print(sensors.getFL_INT());
  Serial.print(",");
  
  Serial.print(sensors.getPRES_INT_cm3H2O());
  Serial.print(",");

  Serial.print(sensors.getPRES_EXT_cm3H2O());
  Serial.print(",");
  
  Serial.print(sensors.getPRES_PAC_cm3H2O());
  Serial.print(",");
  
  Serial.print(flow);
  Serial.print(",");

  Serial.print(volume);
  Serial.print(",");
  
  Serial.print(VALVE_INS);
  Serial.print(",");
  
  Serial.print(pres_ref);
  //Serial.print(ins);
  //Serial.print(pid_prop);
  //Serial.print("\t");

  //Serial.print(pid_int);
  //Serial.print("\t");

  //Serial.print(pid_der);
  //Serial.print("\t");

  Serial.println();

  // State machine
  switch (current_state) {
    case INHALE:
      // Pressure security condition
      if (sensors.getPRES_INT_cm3H2O() > 30) {
        current_state = EXHALE;
        timer_counter = 0;
      }

      // Calculating error from current reading to desired output
      pres_ref = pres_peak;
      error = pres_ref - sensors.getPRES_PAC_cm3H2O();

      // Proportional calc
      pid_prop = Kp * error;

      // Integrative calc with anti-windup filter (coef tau_aw)
      pid_int = pid_int + Ts * (Ki * error + (Kp/tau_aw) * delta_u);

      // Derivative calc with derivative filter (coef alpha)
      derror = (1 - alpha) * prev_error + alpha * derror;
      pid_der = Kd * derror;

      // Control action computation (PID)
      pid_out = pid_prop + pid_int + pid_der;

      // Control effort constraints
      delta_ins = pid_out - prev_pid_out;     
      
      pid_out = delta_ins > 5 ? prev_pid_out + 5 : pid_out;
      pid_out = delta_ins < - 5 ? prev_pid_out - 5 : pid_out;

      // // Control action limits
      VALVE_INS = pid_out;      
      VALVE_INS = VALVE_INS > 100 ? 100 : VALVE_INS;
      VALVE_INS = VALVE_INS < 0 ? 0 : VALVE_INS;

      // Anti-windup component for next iteration
      delta_u = VALVE_INS - pid_out;
      
      // Applying control action to actuators
      valves.setINS_VALVE(VALVE_INS);
      valves.setEXP_VALVE(0);

      // Saving variables for next iteration
      prev_error = error;
      prev_derror = derror;
      prev_pid_out = pid_out;
      break;
    case TRANSITION:
      //valves.setINS_VALVE(0);
      //valves.setEXP_VALVE(0);

      //FECHAR EXP SUAVEMENTE
      if (timer_counter <= TRANSITION_TIME/2){
        
        //valves.setEXP_VALVE((-50*timer_counter)/(TRANSITION_TIME-1)+50);
        //FECHA INS
        valves.setINS_VALVE((-2*VALVE_INS*timer_counter*1.2/(TRANSITION_TIME))+VALVE_INS);
        //ABRE EXP
        valves.setEXP_VALVE(0);
    
        PEEP = false;
        timer_peep = 100;
        
      }
      else if (timer_counter > TRANSITION_TIME/2) {
        valves.setINS_VALVE(0);
        valves.setEXP_VALVE((120*timer_counter/TRANSITION_TIME)-20); 
      }     
      break;
    case PLATEAU:
      valves.setINS_VALVE(0);
      valves.setEXP_VALVE(0);
    case EXHALE:
      // Reset Inhale PID
      pres_ref = peep_value;
      pid_prop = 0;
      pid_int = 0;
      pid_der = 0;
      prev_error = 0;
      prev_pid_out = 0;
      VALVE_INS = 0;
      delta_u = 0;

      // Exhale valves configuration
      //valves.setINS_VALVE(0);
      //valves.setEXP_VALVE(100); 

      if (offexpvalve)
        valves.setEXP_VALVE(0);
      else {
        if (sensors.getPRES_PAC_cm3H2O() <= peep_value + peep_value * p1){
          PEEP = true;          
        }
        if(PEEP){
          //FECHAR SUAVE NA PEEP
          //valves.setEXP_VALVE(0);
          //
          timer_peep = timer_peep-p2;
          valves.setEXP_VALVE(timer_peep);
          

        } else{
            valves.setEXP_VALVE(100);    
        }
      }
      
      //pres_init = sensors.getPRES_PAC_cm3H2O(); // Pressure to compute soft setpoint trajectory
      break;
    case TEST:
      VALVE_INS = ins;
      valves.setINS_VALVE_PWM(ins);
      ins++;
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
      timerAlarmEnable(timer);
    } else if (part01.equals("AUTO")) {
      valves.setAUTO_SEC_VALVE(bool(value));
    } else if (part01.equals("MANUAL")) {
      valves.setMANUAL_SEC_VALVE(bool(value));
    } else if (part01.equals("VALVE_INS")) {
        valves.setINS_VALVE(value);
    } else if (part01.equals("BPM")) {
        INHALE_TO_EXHALE = calculateInhale(getValue(part02, ';', 0).toFloat(), getValue(part02, ';', 1).toFloat());
        EXHALE_TO_INHALE = calculateExhale(getValue(part02, ';', 0).toFloat(), getValue(part02, ';', 1).toFloat());
    } else if (part01.equals("PEAK")) {
        pres_peak = value;
    } else if (part01.equals("PID")) {
        Kp = getValue(part02, ';', 0).toFloat();
        Ki = getValue(part02, ';', 1).toFloat();
        Kd = getValue(part02, ';', 2).toFloat();
        tau_aw = sqrt(1/(Ki * Kd));
    } else if (part01.equals("FILTER")) {
        sensors.setFilterWeight(value);
    } else if (part01.equals("ALPHA")) {
        alpha = value;
    } else if(part01.equals("PEEP")){
      peep_value = value;
    } else if (part01.equals("SET_PEEP")) {
        p1 = getValue(part02, ';', 0).toFloat();
        p2 = getValue(part02, ';', 1).toFloat(); 
    } else if (part01.equals("TEST")) {
      current_state = TEST;
    } else {
      valves.setEXP_VALVE(value);
    }
  
  }
  delay(Ts * 1000); // Ts is in seconds, but delay is in milliseconds
}
