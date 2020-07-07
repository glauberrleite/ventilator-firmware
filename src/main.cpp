/*
 *  Microcontroller module for the ventilator project developed by EASY Engineering and Systems Group
 *  Instituto de Computação - Universidade Federal de Alagoas
 *  @author glauberrleite
 *  @author vangasse
 */

/*
SOBRE ACELERACAO DO CLOCK DO ADS
https://forum.arduino.cc/index.php?topic=559766.0

*/
#ifndef ARDUINO_H
#define ARDUINO_H
#include <Arduino.h>
#endif


#include "sensors.h"
#include "valves.h"

volatile int BPM =15;
volatile float RATIO =2;

float Ts = 0.07;

Sensors sensors;
Valves valves;

volatile float pres_peak = 10;
volatile float pres_ref;
float flow;
volatile long time_passed = 0;

volatile float est_next_volume;

// Setting state times, in milliseconds
volatile int time_inhale_to_exhale = 5000;
volatile int time_exhale_to_inhale = 5000;
volatile int time_cicle= 60000/BPM;
volatile int time_transition = 200;
volatile int time_plateau = 2000;
volatile int time_exp_pause = 2000;

typedef enum {
    IDLE,
    INHALE_PCV,
    PLATEAU,
    INHALE_TO_EXHALE,
    EXHALE,
    EXP_PAUSE,
    EXHALE_TO_INHALE,
    INHALE_VCV,
    TEST
}
state;

typedef enum {
    PCV,
    VCV
}
resp_mode;

volatile state current_state;
volatile resp_mode mode;
volatile int timer_counter = 0;
volatile bool flag = false;

hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// Print debug variables
volatile float VALVE_INS = 0;
volatile float VALVE_EXP = 0;

float pres_init = 0;

unsigned long prev_timestamp;
unsigned long new_timestamp;

// Ins PID variables
float Kp = 1;
float Ki = 0.01;
float Kd = 0.1;

float alpha = 0.9;
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
bool steady_peep = false;

bool offexpvalve = false;
float p1 = 0;

bool ins_pause = false;
bool exp_pause = false;
float last_ins_pressure;
float last_inhale_pressure;
float peep_error = 0;
bool first_time = true;
bool first_inhale_to_exhale = true;
float save_last_ins_pressure = 0;

// VCV variables
float volume;
float max_flux = 30;
float inlet_flux_ref = max_flux;
float volume_desired = 600;
float volume_ref = volume_desired; // May change over time to avoid error
float volume_peak = 0;
float last_inlet_flux_ref;
float last_inlet_flux;
float last_VALVE_INS;
float adjusted_VALVE_INS;

// Custom functions

// Timer callback
void IRAM_ATTR onTimer() {
    portENTER_CRITICAL_ISR( & timerMux);
    if (current_state == INHALE_PCV) {
        if (!ins_pause) { /// SE O BOTÃO INSP NÃO TIVER SIDO PRESSIONADO (CONDIÇÃO NORMAL)
            if (timer_counter >= time_inhale_to_exhale) {
                current_state = INHALE_TO_EXHALE; //VAI PARA A TRANSIÇÃO INS - EXP
                timer_counter = 0;
                
            } else {
                timer_counter++;
            }
        } else { /// SE O BOTÃO INSP TIVER SIDO PRESSIONADO
            if (timer_counter >= time_inhale_to_exhale) {
                current_state = PLATEAU; //VAI PARA A TRANSIÇÃO PLATEAU
                timer_counter = 0;
            } else {
                timer_counter++;
            }
        }
    } else if (current_state == PLATEAU) {
        if (timer_counter >= time_plateau) {
            current_state = INHALE_TO_EXHALE;
            timer_counter = 0;
            ins_pause = false;
        } else {
            timer_counter++;
        }
    } else if (current_state == INHALE_TO_EXHALE) {
        if (timer_counter >= time_transition) {
            current_state = EXHALE;
            timer_counter = 0;
        } else {
            timer_counter++;
        }
    } else if (current_state == EXHALE) {
        if (!exp_pause) {
            if (timer_counter >= time_exhale_to_inhale) {
                current_state = EXHALE_TO_INHALE;
                timer_counter = 0;
                offexpvalve = false;
            } else {
                if (timer_counter == 0)
                    VALVE_EXP = 100;
                timer_counter++;
            }
        } else {
            if (timer_counter >= time_exhale_to_inhale) {
                current_state = EXP_PAUSE;
                timer_counter = 0;
                offexpvalve = false;
            } else {
                if (timer_counter == 0)
                    VALVE_EXP = 100;
                timer_counter++;
            }
        }
    } else if (current_state == EXP_PAUSE) {
        if (timer_counter >= time_exp_pause) {
            current_state = EXHALE_TO_INHALE;
            timer_counter = 0;
            exp_pause = false;
        } else {
            timer_counter++;
        }
    } else if (current_state == EXHALE_TO_INHALE) {
        if (timer_counter >= 10 && flag) {
            if (mode == PCV) {
                current_state = INHALE_PCV;
                Ts = 0.001;
            } else if (mode == VCV) {
                current_state = INHALE_VCV;
                Ts = 0.07;
            }
            timer_counter = 0;
            flag = false;
            volume = 0;
        } else {
            timer_counter++;
            //if (time_exhale_to_inhale - timer_counter <10) offexpvalve = true;
        }
    } else if (current_state == INHALE_VCV) {
        time_inhale_to_exhale = timer_counter;
        time_exhale_to_inhale = time_cicle - time_inhale_to_exhale;
        timer_counter++;
    }

    portEXIT_CRITICAL_ISR( & timerMux);
}

String readStringSerial() {
    String content = "";
    char character;

    // Enquanto receber algo pela serial
    while (Serial.available() > 0) {
        // Lê byte da serial
        character = Serial.read();
        // Ignora caractere de quebra de linha
        if (character != '\n') {
            // Concatena valores
            content.concat(character);
        }
        // Aguarda buffer serial ler próximo caractere
        delay(10);
    }
    return content;
}

String getValue(String data, char separator, int index) {
    int found = 0;
    int strIndex[] = {
        0,
        -1
    };
    int maxIndex = data.length() - 1;

    for (int i = 0; i <= maxIndex && found <= index; i++) {
        if (data.charAt(i) == separator || i == maxIndex) {
            found++;
            strIndex[0] = strIndex[1] + 1;
            strIndex[1] = (i == maxIndex) ? i + 1 : i;
        }
    }

    return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

int calculateInhale(float bpm, float ratio) {
    float t_cicle= 60000 / bpm;
    return t_cicle * (1 - ratio/(ratio+1));
}

int calculateExhale(float bpm, float ratio) {
    float t_cicle= 60000 / bpm;
    return t_cicle * ratio/(ratio+1);
}

void prints(void * arg) {
    while (1) {
        //long start = millis();

        // Printing variables
        Serial.print(current_state);
        Serial.print(",");

        /*Serial.print(sensors.getFL_INT());
        Serial.print(",");
  
        Serial.print(sensors.getPRES_INT_cm3H2O());
        Serial.print(",");

        Serial.print(sensors.getPRES_EXT_cm3H2O());
        Serial.print(",");*/

        Serial.print(sensors.getPRES_PAC_cm3H2O());
        Serial.print(",");
        Serial.print(sensors.getPRES_INT_cm3H2O());
        Serial.print(",");

        Serial.print(sensors.getPRES_EXT_cm3H2O());
        Serial.print(",");       
    
        Serial.print(sensors.getFL_INT());
        Serial.print(","); 

        Serial.print(sensors.getFi02());
        Serial.print(","); 


        Serial.print(flow);
        Serial.print(",");

        Serial.print(volume);
        Serial.print(",");

        Serial.print(VALVE_INS);
        Serial.print(",");

        Serial.print(VALVE_EXP);
        Serial.print(",");

        Serial.print(pres_ref);
        Serial.print(",");

        Serial.print(inlet_flux_ref);
        Serial.print(",");

        Serial.print(Kp);
        Serial.print(",");

        Serial.print(time_cicle);
        Serial.print(",");
        if (current_state != INHALE_VCV){
            Serial.print(time_inhale_to_exhale);
            Serial.print(",");
            Serial.print(time_exhale_to_inhale);
            Serial.print(","); 

        }
        else{
            Serial.print("0");
            Serial.print(",");
            Serial.print("0");
            Serial.print(","); 

        }
         if (current_state == INHALE_TO_EXHALE)
            Serial.print(timer_counter);
        else
            Serial.print("-1");   

        Serial.println();
        //long end = millis() - start;
        /*if (100-end >0){ 
        vTaskDelay(100-end);
        }
        else {*/
        vTaskDelay(1);
        //}
    }
}
void commands(void * arg) {
    while (1) {
        if (Serial.available() > 0) {
            // Lê toda string recebida
            String received = readStringSerial();

            String part01 = getValue(received, ',', 0);
            String part02 = getValue(received, ',', 1);

            float value = part02.toFloat();

            if (part01.equals("START")) {
                current_state = EXHALE_TO_INHALE;
                timerAlarmEnable(timer);

                if (part02.equals("PCV")) {
                    mode = PCV;
                } else if (part02.equals("VCV")) {
                    mode = VCV;
                } else {
                    current_state = IDLE;
                    timerAlarmDisable(timer);
                }

            } else if (part01.equals("STOP")){
                current_state = IDLE;
            } else if (part01.equals("AUTO")) {
                valves.setAUTO_SEC_VALVE(bool(value));
            } else if (part01.equals("MANUAL")) {
                valves.setMANUAL_SEC_VALVE(bool(value));
            } else if (part01.equals("VALVE_INS")) {
                valves.setINS_VALVE(value);
            } else if (part01.equals("BPM")) {
                BPM = getValue(part02, ';', 0).toFloat();
                time_inhale_to_exhale = calculateInhale(BPM,RATIO);
                time_exhale_to_inhale = calculateExhale(BPM,RATIO);
                time_cicle = 60000/ getValue(part02, ';', 0).toFloat();
            }else if (part01.equals("RATIO")){
                RATIO = getValue(part02, ';', 0).toFloat();
                time_inhale_to_exhale = calculateInhale(BPM,RATIO);
                time_exhale_to_inhale = calculateExhale(BPM,RATIO);
            
            }else if (part01.equals("PEAK")) {
                pres_peak = value;

                // Defining PID adaptive tunning
                if (pres_peak <= 20) {
                    Kp = 2.5;
                    Ki = 3;
                    Kd = 3;
                } else if (peep_value < 2) {
                    Kp = 4;
                    Ki = 3;
                    Kd = 1.25;
                } else if (peep_value < 10) {
                    Kp = 2;
                    Ki = 3;
                    Kd = 2;
                } else {
                    Kp = 0.9;
                    Ki = 3;
                    Kd = 1.25;
                }

                first_time = true;
            } else if (part01.equals("PID")) {
                Kp = getValue(part02, ';', 0).toFloat();
                Ki = getValue(part02, ';', 1).toFloat();
                Kd = getValue(part02, ';', 2).toFloat();
                tau_aw = sqrt(1 / (Ki * Kd));
            } else if (part01.equals("FILTER")) {
                sensors.setFilterWeight(value);
            } else if (part01.equals("ALPHA")) {
                alpha = value;
            } else if (part01.equals("PEEP")) {
                peep_value = value;
                
                // Defining PID adaptive tunning
                if (pres_peak <= 20) {
                    Kp = 2.5;
                    Ki = 3;
                    Kd = 3;
                } else if (peep_value < 2) {
                    Kp = 4;
                    Ki = 3;
                    Kd = 1.25;
                } else if (peep_value < 10) {
                    Kp = 2;
                    Ki = 3;
                    Kd = 2;
                } else {
                    Kp = 0.9;
                    Ki = 3;
                    Kd = 1.25;
                }
                first_time = true;
            } else if (part01.equals("TEST")) {
                current_state = TEST;
            } else if (part01.equals("INS_HOLD")) {
                ins_pause = true;
            } else if (part01.equals("EXP_HOLD")) {
                exp_pause = true;
            } else if (part01.equals("MAX_FLUX")) {
                max_flux = value;
                adjusted_VALVE_INS = 1.3 * max_flux;
            } else if (part01.equals("VOLUME")) {
                volume_desired = value;
                volume_ref = volume_desired;
            } else if (part01.equals("SET_PEEP")) {
                p1 = getValue(part02, ';', 0).toFloat();
            }

            
        }
        vTaskDelay(1);
    }
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
    timerAttachInterrupt(timer, & onTimer, true);
    timerAlarmWrite(timer, 1000, true);
    timer_counter = 0;

    // Other variables initialization
    time_inhale_to_exhale = calculateInhale(BPM, RATIO);
    time_exhale_to_inhale = calculateExhale(BPM, RATIO);
    time_cicle = 60000/BPM;
    tau_aw = sqrt(1 / (Ki * Kd));
    flow = 0;
    volume = 0;

    xTaskCreatePinnedToCore(prints, "prints", 8192, NULL, 1, NULL, PRO_CPU_NUM); //Cria a tarefa "loop2()" com prioridade 1, atribuída ao core 0
    delay(1);
    xTaskCreatePinnedToCore(commands, "commands", 8192, NULL, 1, NULL, PRO_CPU_NUM); //Cria a tarefa "loop2()" com prioridade 1, atribuída ao core 0
    delay(1);

    prev_timestamp = millis();
}

void loop() {

    long init_loop_timestamp = millis();
    // Read sensors
    sensors.update();

    // calculate flow and volume
    flow = sensors.getFL_PAC();
    new_timestamp = millis();
    long delta_timestamp = new_timestamp - prev_timestamp;
    if (delta_timestamp < 0) {
        delta_timestamp = Ts;
    }
    //volume  = delta_timestamp;
    if (current_state != IDLE && !((flow < 0) && (flow > -3)))
        volume += (flow / 60) * delta_timestamp; // Volume in mL

    if (volume < 0) {
        volume = 0;
    }
    est_next_volume = volume + sensors.getFL_PAC() * Ts;

    prev_timestamp = new_timestamp;

    // State machine
    switch (current_state) {
        case IDLE:
            VALVE_INS = 0;
            VALVE_EXP = 100;
            timer_counter = 0;
            break;
        case INHALE_PCV:
            // Pressure security condition
            if (sensors.getPRES_PAC_cm3H2O() > 60) {
                current_state = INHALE_TO_EXHALE;
                timer_counter = 0;
            }

            // Calculating error from current reading to desired output
            error = pres_ref - sensors.getPRES_PAC_cm3H2O();
            error = error - 0.025 * sensors.getFL_PAC();
            // Proportional calc
            pid_prop = Kp * error;
            // Integrative calc with anti-windup filter (coef tau_aw)
            pid_int = pid_int + Ts * (Ki * error + (Kp / tau_aw) * delta_u);
            //pid_int = pid_int + Ki * error * Ts;
            // Derivative calc with derivative filter (coef alpha)
            derror = (1 - alpha) * prev_error + alpha * derror;
            pid_der = Kd * derror;
            // Control action computation (PID)
            pid_out = pid_prop + pid_int + pid_der;
            // Control effort constraints
            delta_ins = pid_out - prev_pid_out;
            pid_out = delta_ins > 10 ? prev_pid_out + 10 : pid_out;
            pid_out = delta_ins < - 3 ? prev_pid_out - 3 : pid_out;
            // // Control action limits
            VALVE_INS = pid_out;
            VALVE_INS = VALVE_INS > 100 ? 100 : VALVE_INS;
            VALVE_INS = VALVE_INS < 0 ? 0 : VALVE_INS;
            // Anti-windup component for next iteration
            delta_u = VALVE_INS - (prev_pid_out + delta_ins);

            // Saving variables for next iteration
            prev_error = error;
            prev_derror = derror;
            prev_pid_out = pid_out;
            last_inhale_pressure = sensors.getPRES_PAC_cm3H2O();
            break;
        case INHALE_VCV:

            if (volume < volume_ref) {
                // If constant flux
                inlet_flux_ref = max_flux;
                // If decreasing flux
                // TODO
                // inlet_flux_ref = ...

                if (first_time) {
                    VALVE_INS = 1.3 * max_flux;
                } else {
                    VALVE_INS = adjusted_VALVE_INS;
                }
            } else {
                if (!ins_pause) {
                    current_state = INHALE_TO_EXHALE;
                } else {
                    current_state = PLATEAU;
                    ins_pause = false;
                }

                timer_counter = 0;
                last_inlet_flux = sensors.getFL_PAC();
                last_inlet_flux_ref = inlet_flux_ref;
                last_VALVE_INS = VALVE_INS;
                VALVE_INS = 0;
            }

            break;
        case PLATEAU:
            VALVE_INS = 0;
            VALVE_EXP = 0;
            break;
        case INHALE_TO_EXHALE:

            if (mode == PCV) {
                // Tunning PID Kp for next cycle
                if (first_inhale_to_exhale) {
                    VALVE_INS = 0;   
                    if (abs(pres_peak - last_inhale_pressure) > 1) {
                        Kp = Kp + 0.08 * (pres_peak - last_inhale_pressure);
                        Kp = Kp > 0.01 ? Kp : 0.01;
                    }
                    first_inhale_to_exhale = false;
                }
                //VALVE_INS = (-2 * VALVE_INS * timer_counter * 1.2 / (time_transition)) + VALVE_INS;
                //VALVE_EXP = (100 * timer_counter / time_transition) + 40;
                VALVE_EXP = (50 * timer_counter / time_transition) + 50;
                VALVE_EXP = VALVE_EXP > 100 ? 100 : VALVE_EXP;
                    //VALVE_EXP = 100;

                if (abs(peep_error)>= 3){
                    p1 = p1 + 0.04 * peep_error;
                }
                else if (abs(peep_error)<3 && abs(peep_error)>=1){
                    p1 = p1 + 0.03 * peep_error;
                }
                else if (abs(peep_error)<1 && abs(peep_error)>=0.5){
                    p1 = p1 + 0.02 * peep_error;
                }
                else if (abs(peep_error)<0.5 && abs(peep_error)>=0.1){
                    p1 = p1 + 0.01 * peep_error;
                }
                /*
                if (timer_counter <= time_transition / 2) {

                    //valves.setEXP_VALVE((-50*timer_counter)/(time_transition-1)+50);
                    //FECHA INS                    
                    VALVE_INS = (-2 * VALVE_INS * timer_counter * 1.2 / (time_transition)) + VALVE_INS;
                    //ABRE EXP
                    VALVE_EXP = 0;
                } else {
                    VALVE_INS = 0;
                    //VALVE_EXP = 100;
                    VALVE_EXP = (120 * timer_counter / time_transition) - 20;
                }*/
            } else if (mode == VCV) {

                if (volume > volume_peak) {
                    volume_peak = volume;
                }

                VALVE_INS = 0;
                VALVE_EXP = (100 * timer_counter / time_transition) + 40;
                VALVE_EXP = VALVE_EXP > 100 ? 100 : VALVE_EXP;
                VALVE_EXP = VALVE_EXP < 0 ? 0 : VALVE_EXP;

                adjusted_VALVE_INS = last_VALVE_INS + 0.3 * (last_inlet_flux_ref - last_inlet_flux);


                if (abs(peep_error)>= 3){
                    p1 = p1 + 0.04 * peep_error;
                }
                else if (abs(peep_error)<3 && abs(peep_error)>=1){
                    p1 = p1 + 0.03 * peep_error;
                }
                else if (abs(peep_error)<1 && abs(peep_error)>=0.5){
                    p1 = p1 + 0.02 * peep_error;
                }
                else if (abs(peep_error)<0.5 && abs(peep_error)>=0.1){
                    p1 = p1 + 0.01 * peep_error;
                }             
            }
            // Cleaning control variables
            pid_prop = 0;
            pid_int = 0;
            pid_der = 0;
            prev_error = 0;
            delta_u = 0;

            // New inlet flux and pressure references
            inlet_flux_ref = 0;
            pres_ref = peep_value;
  
            last_ins_pressure = sensors.getPRES_PAC_cm3H2O();
            break;
        case EXHALE:
            first_inhale_to_exhale = true;

            save_last_ins_pressure = last_ins_pressure;
            if (first_time) {

              p1 = 0;  
              //p1 = -3.294 -0.05674*last_ins_pressure+2.596*peep_value +0.0487* pow(last_ins_pressure,2) -0.3695*last_ins_pressure*peep_value +0.3002*pow(peep_value,2)- 0.001333*pow(last_ins_pressure,3) +0.008985*pow(last_ins_pressure,2)*peep_value-0.009181*last_ins_pressure*pow(peep_value,2)  ; 
              //p1 = 2.131 + 0.09507*last_ins_pressure -0.7746*peep_value -0.0008329*pow(last_ins_pressure,2) -0.01363*last_ins_pressure*peep_value+ 0.07615*pow(peep_value,2);
              //p1 = 0.7116 - 0.03798 * last_ins_pressure + 0.05375 * peep_value - 0.001111 * pow(peep_value, 2) + 0.005175 * last_ins_pressure * peep_value - 0.02419 * pow(peep_value, 2) + 0.001014 * pow(last_ins_pressure, 2) * peep_value - 0.004439 * last_ins_pressure * pow(peep_value, 2) + 0.007044 * pow(peep_value, 3);
              first_time = false;
            }
            VALVE_INS = 0;
            VALVE_EXP = VALVE_EXP - p1;
           
            peep_error = peep_value - sensors.getPRES_PAC_cm3H2O();
            if (timer_counter > time_exhale_to_inhale/2){

            }
            // When peep is in steady state, monitor it to trigger ASSISTED_MODE
            break;

        case EXP_PAUSE:
            VALVE_EXP = 0;
            VALVE_INS = 0;

            break;
        case EXHALE_TO_INHALE:
            // Cleaning control variables
            pid_prop = 0;
            pid_int = 0;
            pid_der = 0;
            prev_error = 0;
            prev_pid_out = 0;
            VALVE_INS = 0;
            VALVE_EXP = 0;
            delta_u = 0;
            steady_peep = false;

            flag = true;

            if (mode == VCV && !first_time) {
                volume_ref = volume_ref + (volume_desired - volume_peak);
                volume_peak = 0;
            }

            pres_ref = pres_peak;

            break;
        default:
            break;
    }

    valves.setINS_VALVE(VALVE_INS);
    valves.setEXP_VALVE(VALVE_EXP);

    // Assert realtime iteration delay (Default operation time is 60 ms)
    long end_loop_timestamp = millis();
    time_passed = end_loop_timestamp - init_loop_timestamp;
    if (end_loop_timestamp - init_loop_timestamp < (Ts * 1000)) {
        delay(Ts * 1000 - (end_loop_timestamp - init_loop_timestamp)); // Ts is in seconds, but delay is in milliseconds
    }
}