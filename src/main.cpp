/*
 *  Microcontroller module for the ventilator project developed by EASY Engineering and Systems Group
 *  Instituto de Computação - Universidade Federal de Alagoas
 *  @author glauberrleite
 *  @author vangasse
 *  @author cesaraugusto
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

Sensors sensors;
Valves valves;

float Ts = 0.07;
volatile int BPM = 15;
volatile float RATIO = 2;


// Setting state times with default values, in milliseconds
volatile int time_inhale_to_exhale = 5000;
volatile int time_exhale_to_inhale = 5000;
volatile int time_cicle = 60000 / BPM;
volatile int time_transition = 200;
volatile int time_plateau = 2000;
volatile int time_exp_pause = 2000;
volatile int timer_counter = 0;

unsigned long prev_timestamp;
unsigned long new_timestamp;
volatile long time_passed = 0;

typedef enum {
    SENDING,
    NOT_SENDING
}
transmission_state;

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
volatile transmission_state transmission;

volatile bool flag = false;

hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// Print debug variables
volatile float VALVE_INS = 0;
volatile float VALVE_EXP = 0;
volatile bool SEC_VALVE = false;

// Ins PID (PCV) variables
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

//PVC variables
volatile float pres_peak = 10;
volatile float pres_ref = 0;
float pres_init = 0;

// PEEP control variables
float peep_value = 0;
bool steady_peep = false;
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
float flow = 0;
float max_flux = 30;
float inlet_flux_ref = max_flux;
float volume_desired = 600;
float volume_ref = volume_desired; // May change over time to minimize uncontrollable error
float volume_peak = 0;
float last_inlet_flux_ref;
float last_inlet_flux;
float last_VALVE_INS;
float adjusted_VALVE_INS;
volatile float est_next_volume;
bool is_square = false;
float ajuste_rampa =0.5;

// Assisted mode variables
bool assisted = false;
float effort = 0;
float total_effort = 0;
float sensibility = 1;

// Peak variables
volatile float ve = 0;
volatile float pe = 0;
float pe_aux = 0;
bool active_valve_sec = false;

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
            //ins_pause = false;
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

            } else {
                if (timer_counter == 0)
                    VALVE_EXP = 100;
                timer_counter++;
            }
        } else {
            if (timer_counter >= time_exhale_to_inhale) {
                current_state = EXP_PAUSE;
                timer_counter = 0;

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
            //exp_pause = false;
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
    float t_cicle = 60000 / bpm;
    return t_cicle * (1 - ratio / (ratio + 1));
}

int calculateExhale(float bpm, float ratio) {
    float t_cicle = 60000 / bpm;
    return t_cicle * ratio / (ratio + 1);
}

void prints_hmi(void * arg) {
    while (1) {
        long start = millis();

        String msg;

        msg = "$";

        if (mode == PCV) {
            msg.concat("paw:");
            msg.concat(sensors.getPRES_PAC_cm3H2O());
            msg.concat(",");
            msg.concat("vtidal:");
            msg.concat(volume);
            msg.concat(",");
            msg.concat("flow:");
            msg.concat(sensors.getFL_PAC());
            msg.concat(",");
            msg.concat("ve:");
            msg.concat(ve);
            msg.concat(",");
            msg.concat("pe:");
            msg.concat(pe);
            msg.concat(",");
            msg.concat("fio2:");
            msg.concat(sensors.getFi02(pe));
        } else if (mode == VCV) {
            msg.concat("paw:");
            msg.concat(sensors.getPRES_PAC_cm3H2O());
            msg.concat(",");
            msg.concat("vtidal:");
            msg.concat(volume);
            msg.concat(",");
            msg.concat("flow:");
            msg.concat(sensors.getFL_PAC());
            msg.concat(",");
            msg.concat("tInsp:");
            msg.concat(time_inhale_to_exhale);
            msg.concat(",");
            msg.concat("ve:");
            msg.concat(ve);
            msg.concat(",");
            msg.concat("pe:");
            msg.concat(pe);
            msg.concat(",");
            msg.concat("fio2:");
            msg.concat(sensors.getFi02(pe));
        }

        msg.concat("%");

        if (transmission == SENDING) {
            Serial.println(msg);
        }
        long end = millis() - start;
        if (166 - end > 0) {
            vTaskDelay(166 - end);
        } else {
            vTaskDelay(1);
        }
    }
}

void prints(void * arg) {
    while (1) {
        //long start = millis();

        // Printing variables
        Serial.print(current_state);
        Serial.print(",");

        Serial.print(sensors.getPRES_PAC_cm3H2O());
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

        /*Serial.print(Kp);
        Serial.print(",");*/

        Serial.print(time_cicle);
        Serial.print(",");
        if (current_state != INHALE_VCV) {
            Serial.print(time_inhale_to_exhale);
            Serial.print(",");
            Serial.print(time_exhale_to_inhale);
            Serial.print(",");

        } else {
            Serial.print("0");
            Serial.print(",");
            Serial.print("0");
            Serial.print(",");

        }
        /*if (current_state == INHALE_TO_EXHALE)
            Serial.print(timer_counter);
        else
            Serial.print("-1"); */
        Serial.print(assisted);
        Serial.print(",");

        Serial.print(effort);
        Serial.print(",");

        Serial.print(ve);
        Serial.print(",");

        Serial.print(p1);
        Serial.print(",");

        Serial.print(sensors.getFi02(pe),4);
        Serial.print(",");

        Serial.print(SEC_VALVE);

        Serial.println();
        //long end = millis() - start;
        //if (166 - end > 0){ 
        //    vTaskDelay(166 - end);
        //}
        //else {
        vTaskDelay(1);
        //}
    }
}
void commands(void * arg) {
    while (1) {
        if (Serial.available() > 0) {
            // Lê toda string recebida
            //String received = readStringSerial();
            String received = Serial.readStringUntil('\n');

            String part01 = getValue(received, '$', 0);
            String part02 = getValue(received, '$', 1);
            part02 = getValue(part02, '%', 0);

            if (part01.equals("START")) {
                current_state = EXHALE_TO_INHALE;
                timerAlarmEnable(timer);
            } else if (part01.equals("STOP")) {
                current_state = IDLE;
                volume = 0;
            } else if (part01.equals("SET")) {
                int i = 0;
                String cmd = getValue(part02, ',', 0);
                while (!cmd.equals("")) {
                    String parameter = getValue(cmd, ':', 0);
                    String value = getValue(cmd, ':', 1);


                    if (parameter.equals("OP")) {
                        if (value.toFloat() == 0) {
                            mode = PCV;
                        } else {
                            mode = VCV;
                        }
                    } else if (parameter.equals("RR")) {
                        BPM = value.toFloat();
                        time_inhale_to_exhale = calculateInhale(BPM, RATIO);
                        time_exhale_to_inhale = calculateExhale(BPM, RATIO);
                        time_cicle = 60000 / BPM;
                    } else if (parameter.equals("IE")) {
                        RATIO = value.toFloat();
                        time_inhale_to_exhale = calculateInhale(BPM, RATIO);
                        time_exhale_to_inhale = calculateExhale(BPM, RATIO);
                    } else if (parameter.equals("PIP")) {
                        pres_peak = value.toFloat();
                        // Defining PID adaptive tunning
                        if (pres_peak <= 20) {
                            Kp = 2.5;
                            Ki = 3;
                            Kd = 3;
                        } else if (peep_value < 10) {
                            Kp = 2;
                            Ki = 3;
                            Kd = 0.5;
                        } else {
                            Kp = 0.9;
                            Ki = 3;
                            Kd = 1.25;
                        }

                        first_time = true;
                    } else if (parameter.equals("PEEP")) {
                        peep_value = value.toFloat();

                        // Defining PID adaptive tunning
                        if (pres_peak <= 20) {
                            Kp = 2.5;
                            Ki = 3;
                            Kd = 3;
                        } else if (peep_value < 10) {
                            Kp = 2;
                            Ki = 3;
                            Kd = 0.5;
                        } else {
                            Kp = 0.9;
                            Ki = 3;
                            Kd = 1.25;
                        }

                        first_time = true;
                    } else if (parameter.equals("VOLUME")) {
                        volume_desired = value.toFloat();
                        volume_ref = volume_desired;
                        ajuste_rampa = (max_flux/volume_desired)*10;
                        first_time = true;
                    } else if (parameter.equals("FLOW")) {
                        max_flux = value.toFloat();
                        adjusted_VALVE_INS = 1.3 * max_flux;
                        ajuste_rampa = (max_flux/volume_desired)*10;
                        first_time = true;
                    } else if (parameter.equals("PINSP")) {
                        if (value.toInt() == 0) {
                            ins_pause = false;
                        } else {
                            ins_pause = true;
                        }
                    } else if (parameter.equals("PEXP")) {
                        if (value.toInt() == 0) {
                            exp_pause = false;
                        } else {
                            exp_pause = true;
                        }
                    } else if (parameter.equals("SENSIT")) {
                        // TODO
                        sensibility = value.toFloat();
                    }

                    // Testing/Debugging parameters
                    // --------------------
                    else if (parameter.equals("KP")) {
                        Kp = value.toFloat();
                    } else if (parameter.equals("KI")) {
                        Ki = value.toFloat();
                    } else if (parameter.equals("KD")) {
                        Kd = value.toFloat();
                    } else if (parameter.equals("FILTER")) {
                        sensors.setFilterWeight(value.toFloat());
                    } else if (parameter.equals("ALPHA")) {
                        alpha = value.toFloat();
                    } else if (parameter.equals("AJUSTE")) {
                        ajuste_rampa = value.toFloat();
                    } else if (parameter.equals("SEC_VALVE")) {
                        if (value.toInt() == 0) {
                            SEC_VALVE = false;
                        } else if (value.toInt() == 1){
                            SEC_VALVE = true;
                        }
                    }else if (parameter.equals("WAVE")) {
                        if (value.toInt() == 0) {
                            is_square = false;
                        } else if (value.toInt() == 1){
                            is_square = true;
                        }
                    }


                    // --------------------

                    i = i + 1;
                    cmd = getValue(part02, ',', i);
                }

            } else if (part01.equals("CONNECT")) {
                Serial.println("OK");
            } else if (part01.equals("SEND_DATA")) {
                transmission = SENDING;
            } else if (part01.equals("STOP_SENDING")) {
                transmission = NOT_SENDING;
            }
    

        }
        vTaskDelay(1);
    }
}

// Main setup and loop

void setup() {
    
    Serial.begin(9600);

    delay(1000);

    // Sensors and Valves init
    sensors = Sensors();
    sensors.onSFM(true);
    delay(1000);
    valves = Valves();
    


    // Stating State
    current_state = IDLE;
    transmission = NOT_SENDING;

    // Timer config
    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &onTimer, true);
    timerAlarmWrite(timer, 1000, true);
    timer_counter = 0;

    // Other variables initialization
    time_inhale_to_exhale = calculateInhale(BPM, RATIO);
    time_exhale_to_inhale = calculateExhale(BPM, RATIO);
    time_cicle = 60000 / BPM;
    tau_aw = sqrt(1 / (Ki * Kd));
    flow = 0;
    volume = 0;

    xTaskCreatePinnedToCore(prints_hmi, "prints", 8192, NULL, 1, NULL, PRO_CPU_NUM); //Cria a tarefa "loop2()" com prioridade 1, atribuída ao core 0
    delay(1);
    xTaskCreatePinnedToCore(commands, "commands", 8192, NULL, 1, NULL, PRO_CPU_NUM); //Cria a tarefa "loop2()" com prioridade 1, atribuída ao core 0
    delay(1);

    prev_timestamp = millis();
}

void loop() {

    long init_loop_timestamp = millis();
    // Read sensors
    sensors.onSFM(true);
    sensors.update();
    //Serial.print("FL: ");Serial.println(sensors.getFL_PAC());
    if (abs(sensors.getFL_PAC())>250){
        sensors.resetSFM();
    }
    

    // Pressure security condition


    if (sensors.getPRES_PAC_cm3H2O() > 60) {
        current_state = INHALE_TO_EXHALE;
        //SEC_VALVE = true;
        timer_counter = 0;
    } else {
        //SEC_VALVE = false;
    }

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

    if (pe_aux < sensors.getPRES_PAC_cm3H2O()) {
        pe_aux = sensors.getPRES_PAC_cm3H2O();
    }

    prev_timestamp = new_timestamp;

    // State machine
    switch (current_state) {
        case IDLE:
            VALVE_INS = 0;
            VALVE_EXP = 100;
            timer_counter = 0;
            break;
        case INHALE_PCV:

            // Testing SEC VALVE
            // -----------
            /*if (sensors.getPRES_PAC_cm3H2O() > pres_ref+1) {
                active_valve_sec = true;                
            } */

            // -----------


            if (assisted) {
                error = pres_ref - save_last_ins_pressure;
                assisted = false;
            } else {
                error = pres_ref - sensors.getPRES_PAC_cm3H2O();

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
            pid_out = delta_ins < -3 ? prev_pid_out - 3 : pid_out;
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
                
                if (is_square){    
                    inlet_flux_ref = max_flux;                
                    if (first_time) {
                        VALVE_INS = 1.3 * max_flux;
                    } else {
                        VALVE_INS = adjusted_VALVE_INS;
                    }
                } else {
                    if (timer_counter< 350){//ainda não chegou no maxflux{
                    inlet_flux_ref = max_flux;
                        if (first_time) {
                            VALVE_INS = 1.3 * max_flux;
                        } else {
                            VALVE_INS = adjusted_VALVE_INS;
                        }
                    last_inlet_flux = sensors.getFL_PAC();
                    last_inlet_flux_ref = inlet_flux_ref;
                    last_VALVE_INS = VALVE_INS;                                      
                    }
                    else{             
                        inlet_flux_ref = inlet_flux_ref-ajuste_rampa > 5 ? inlet_flux_ref-ajuste_rampa: 5;
                        VALVE_INS = inlet_flux_ref*0.9;                           


                    }     
                    

                }                
            } else {
                if (!ins_pause) {
                    current_state = INHALE_TO_EXHALE;
                } else {
                    current_state = PLATEAU;
                    //ins_pause = false;
                }

                timer_counter = 0;
                if (is_square){
                    last_inlet_flux = sensors.getFL_PAC();
                    last_inlet_flux_ref = inlet_flux_ref;
                    last_VALVE_INS = VALVE_INS;

                }

                VALVE_INS = 0;
            }

            break;
        case PLATEAU:
            VALVE_INS = 0;
            VALVE_EXP = 0;
            break;
        case INHALE_TO_EXHALE:
            active_valve_sec = false;
            if (volume > volume_peak) {
                volume_peak = volume;
            }

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

                if (abs(peep_error) >= 3) {
                    p1 = p1 + 0.04 * peep_error;
                } else if (abs(peep_error) < 3 && abs(peep_error) >= 1) {
                    p1 = p1 + 0.03 * peep_error;
                } else if (abs(peep_error) < 1 && abs(peep_error) >= 0.5) {
                    p1 = p1 + 0.02 * peep_error;
                } else if (abs(peep_error) < 0.5 && abs(peep_error) >= 0.1) {
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

                VALVE_INS = 0;
                VALVE_EXP = (50 * timer_counter / time_transition) + 50;
                VALVE_EXP = VALVE_EXP > 100 ? 100 : VALVE_EXP;
                VALVE_EXP = VALVE_EXP < 0 ? 0 : VALVE_EXP;

                adjusted_VALVE_INS = last_VALVE_INS + 0.3 * (last_inlet_flux_ref - last_inlet_flux);


                if (abs(peep_error) >= 3) {
                    p1 = p1 + 0.04 * peep_error;
                } else if (abs(peep_error) < 3 && abs(peep_error) >= 1) {
                    p1 = p1 + 0.03 * peep_error;
                } else if (abs(peep_error) < 1 && abs(peep_error) >= 0.5) {
                    p1 = p1 + 0.02 * peep_error;
                } else if (abs(peep_error) < 0.5 && abs(peep_error) >= 0.1) {
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


            break;
        case EXHALE:
            first_inhale_to_exhale = true;
            if (first_time) {

                p1 = 0;

                first_time = false;
            }
            VALVE_INS = 0;
            VALVE_EXP = VALVE_EXP - p1;
            //peep_error = peep_value - sensors.getPRES_PAC_cm3H2O();

            // When peep is in steady state, monitor it to trigger ASSISTED_MODE

            if (timer_counter > time_exhale_to_inhale / 2) {
                steady_peep = true;
                last_ins_pressure = sensors.getPRES_PAC_cm3H2O();
                effort = save_last_ins_pressure - last_ins_pressure;
                if (save_last_ins_pressure > last_ins_pressure) {
                    total_effort += effort;
                    if (total_effort > sensibility) {
                        assisted = true;
                        current_state = EXHALE_TO_INHALE;
                        timer_counter = 0;
                    }
                }
                save_last_ins_pressure = last_ins_pressure;
            } else {
                save_last_ins_pressure = sensors.getPRES_PAC_cm3H2O();
            } 
            peep_error = peep_value - sensors.getPRES_PAC_cm3H2O();
            
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
            //assisted = false;
            total_effort = 0;

            flag = true;


            // Setpoint adaptation in VCV to change INHALE to another state earlier
            if (mode == VCV && !first_time) {
                volume_ref = volume_ref + (volume_desired - volume_peak);
            }
            // Updating max/peak volume and pressure values
            ve = volume_peak;
            pe = pe_aux;
            pe_aux = 0;
            pres_ref = pres_peak;
            volume_peak = 0;

            break;
        default:
            break;
    }

    valves.setINS_VALVE(VALVE_INS);
    valves.setEXP_VALVE(VALVE_EXP);
    if(active_valve_sec){
        SEC_VALVE = true;
    }
    else {
        SEC_VALVE = false;
    }
    valves.setAUTO_SEC_VALVE(SEC_VALVE);

    // Assert realtime iteration delay (Default operation time is 60 ms)
    long end_loop_timestamp = millis();
    time_passed = end_loop_timestamp - init_loop_timestamp;
    if (end_loop_timestamp - init_loop_timestamp < (Ts * 1000)) {
        delay(Ts * 1000 - (end_loop_timestamp - init_loop_timestamp)); // Ts is in seconds, but delay is in milliseconds
    }
}