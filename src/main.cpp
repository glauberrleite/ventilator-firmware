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

volatile float plateau = 10;
volatile float plateau_ref;

// Setting state times, in milliseconds
volatile int INHALE_TO_EXHALE = 5000;
volatile int EXHALE_TO_INHALE = 5000;
volatile int PAUSE_TIME = 600;


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

float pres_init = 0;

// PID variables
float Kp = 5.2;
float Ki = 75;
float Kd = 1.25;

float alpha = 0.1;
float pid_out = 0;
float delta_ins = 0;
float error = 0;
float ierror = 0;
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
float volume = 0;
float peep_value = 0;
bool offexpvalve = false;
int timer_peep =1;


bool PEEP;
// Custom functions

// Timer callback
void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  if (current_state == INHALE) {
    if (timer_counter >= INHALE_TO_EXHALE) {
      current_state = PAUSE;
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
  else if (current_state == PAUSE){
    if (timer_counter >= PAUSE_TIME) {
      current_state = EXHALE;
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

  INHALE_TO_EXHALE = calculateInhale(BPM, RATIO);
  EXHALE_TO_INHALE = calculateExhale(BPM, RATIO);
  tau_aw = sqrt(1/(Ki * Kd));
}

// TEST state variables
int ins = 0;
bool flag = false;
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
    //Serial.print(sensors.getFL_PAC()+9.46, 4);
    if (current_state == INHALE) {
      Serial.print(sensors.getFL_INT());
    } else {
      Serial.print(sensors.getFL_PAC()+9.46, 4);
    }
    Serial.print("\t");
  }
  
  if (print_valve_in) {
    Serial.print(VALVE_INS);
    Serial.print("\t");
  }

  if (print_state || print_fl_int || print_fl_pac || print_pres_pac || print_pres_int || print_pres_ext || print_valve_in) {
    //Serial.print(ins);
    //Serial.print(pid_prop);
    //Serial.print("\t");

    //Serial.print(pid_int);
    //Serial.print("\t");

    //Serial.print(pid_der);
    //Serial.print("\t");

    //Serial.print(plateau_ref);
    Serial.println();
  }

  // State machine

  switch (current_state) {
    case INHALE:
 
    

      // Setpoint
      /*if (timer_counter < (INHALE_TO_EXHALE/5)) {
        plateau_ref = ((5 * (plateau - pres_init) * timer_counter) / INHALE_TO_EXHALE) + pres_init;
      } else {
        plateau_ref = plateau;
      }*/

      // Pressure security condition
      if (sensors.getPRES_INT_cm3H2O() > 30) {
        current_state = EXHALE;
        timer_counter = 0;
      }

      // Using plateau as reference, adjusting PID     

      plateau_ref = plateau;

      //error = plateau_ref - sensors.getPRES_PAC_cm3H2O();
      error = plateau_ref -sensors.getPRES_INT_cm3H2O();
      /* Proportional calc with constraints */
      pid_prop = Kp * error;

      /* Integrative calc */
      //ierror += error * Ts;
      //pid_int = Ki * ierror;
      pid_int = pid_int + Ts * (Ki * error + (Kp/tau_aw) * delta_u);

      /* Derivative calc */
      //derror = (error - prev_error) / Ts;
      derror = (1 - alpha) * prev_error + alpha * derror;
      pid_der = Kd * derror;

      // conditional integration
      //pid_int = pid_int >= 30 ? 30 : pid_int;

      pid_out = pid_prop + pid_int + pid_der;

      delta_ins = pid_out - prev_pid_out;
      
      // Constraint in control effort
      pid_out = delta_ins > 5 ? prev_pid_out + 5 : pid_out;
      pid_out = delta_ins < -5 ? prev_pid_out - 5 : pid_out;

      VALVE_INS = pid_out;
    
      // Constraint in control action
      VALVE_INS = VALVE_INS > 100 ? 100 : VALVE_INS;
      VALVE_INS = VALVE_INS < 0 ? 0 : VALVE_INS;

      // Anti windup component
      delta_u = VALVE_INS - pid_out;

      valves.setINS_VALVE(VALVE_INS);
      valves.setEXP_VALVE(0);

      prev_error = error;
      prev_ierror = ierror;
      prev_derror = derror;
      prev_pid_out = pid_out;
      PEEP = false;
      timer_peep =1;
      break;
    case PAUSE:
      //valves.setINS_VALVE(0);
      //valves.setEXP_VALVE(0);

      //FECHAR EXP SUAVEMENTE
      if (timer_counter <=PAUSE_TIME/2){
        
        //valves.setEXP_VALVE((-50*timer_counter)/(PAUSE_TIME-1)+50);
        //FECHA INS
        valves.setINS_VALVE((-2*VALVE_INS*timer_counter*1.2/(PAUSE_TIME))+VALVE_INS);
        //ABRE EXP
        valves.setEXP_VALVE(0);
        
      }
      else if (timer_counter >PAUSE_TIME/2) {
        valves.setINS_VALVE(0);
        valves.setEXP_VALVE((120*timer_counter/PAUSE_TIME)-20); 
      }     
      break;
    case EXHALE:
      // Reset Inhale PID
      plateau_ref = 0;
      ierror = 0;
      pid_prop = 0;
      pid_int = 0;
      pid_der = 0;
      prev_error = 0;
      prev_ierror = 0;
      prev_pid_out = 0;
      VALVE_INS = 0;
      delta_u = 0;

      // Exhale valves configuration
      //valves.setINS_VALVE(0);
      //valves.setEXP_VALVE(100); 

      if (offexpvalve) valves.setEXP_VALVE(0);
      else{
        if (sensors.getPRES_INT_cm3H2O()<=peep_value-peep_value*0.25){
          PEEP = true;          
        }
        if(PEEP){
          //FECHAR SUAVE NA PEEP
          valves.setEXP_VALVE(0);
          //valves.setEXP_VALVE(-10*timer_peep+100);
          timer_peep++;


          /*if (true){          
            valves.setEXP_VALVE(70);
            
          }
          else{
            valves.setEXP_VALVE(0);
          }   */
        }
        else{
          //ABRIR SUAVE NO COMEÇO DA EXPIRAÇÃO
          if (timer_counter<=EXHALE_TO_INHALE/3){
            //float x = 60*timer_counter/EXHALE_TO_INHALE;
            //valves.setEXP_VALVE(x);
            //Serial.println(x);
            valves.setEXP_VALVE(100);

            
            
          }
          else{
            valves.setEXP_VALVE(100);
          }      
        }
      }

      
      //pres_init = sensors.getPRES_PAC_cm3H2O(); // Pressure to compute soft setpoint trajectory
      break;
    case TEST:
      delay(500);
      Serial.print(ins);Serial.print("\t");
      valves.setINS_VALVE(100);
      valves.setEXP_VALVE(ins);
      if (flag) {
        ins--;
      } else {
        ins++;
      }
      //O valor correto é 950;
      if (ins >= 100) {
        ins = 100;
        flag = true;
      }
      if (ins <= 0) {
        flag = false;
      }
      if (sensors.getPRES_PAC_cm3H2O() >= 30) {
        //Serial.println("HEEELP");
        //valves.setINS_VALVE(0);
        //valves.setEXP_VALVE(100);
        //current_state = EXHALE;
        print_fl_int = false;
        print_fl_pac = false;
        print_pres_pac = false;
      }
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
    } else if (part01.equals("PRINT")) {
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
        EXHALE_TO_INHALE = calculateExhale(getValue(part02, ';', 0).toFloat(), getValue(part02, ';', 1).toFloat());
    } else if (part01.equals("PLATEAU")) {
        plateau = value;
    } else if (part01.equals("PRINT_INS")) {
        print_valve_in = true;
    } else if (part01.equals("PRINT_ALL")) {
        print_fl_int = true;
        print_fl_pac = true;
        print_pres_int = true;
        print_pres_pac = true;
        print_pres_ext = true;
    } else if (part01.equals("PID")) {
        Kp = getValue(part02, ';', 0).toFloat();
        Ki = getValue(part02, ';', 1).toFloat();
        Kd = getValue(part02, ';', 2).toFloat();
        tau_aw = sqrt(1/(Ki * Kd));
    } else if (part01.equals("ALPHA")) {
        alpha = value;
    } else if (part01.equals("COEF")) {
        //sensors.a1 = getValue(part02, ';', 0).toFloat();
        //sensors.a2 = getValue(part02, ';', 1).toFloat();
        //sensors.a3 = getValue(part02, ';', 2).toFloat();
    }else if(part01.equals("PEEP")){
      peep_value = part02.toFloat();
    
    }else if (part01.equals("TEST")) {
        current_state = TEST;

        //print_fl_int = true;
        //print_fl_pac = true;
        //print_pres_pac = true;
        //print_pres_int = true; 
        delay(1000);
    } else {
      valves.setEXP_VALVE(value);
    }
  
  }
  delay(Ts * 1000); // Ts is in seconds, but delay is in milliseconds
}
