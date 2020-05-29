#include "valves.h"

#ifndef ARDUINO_H
#define ARDUINO_H
#include <Arduino.h>
#endif

Valves::Valves()
{
    // Proportional Valves setup
    ledcSetup(INS_PROP_VALVE_CH, PWM_FREQ, PWM_RES);
    ledcSetup(EXP_PROP_VALVE_CH, PWM_FREQ, PWM_RES);

    ledcAttachPin(INS_PROP_VALVE_PIN, INS_PROP_VALVE_CH);
    ledcAttachPin(EXP_PROP_VALVE_PIN, EXP_PROP_VALVE_CH);

    // Security Valve configs
    pinMode(MANUAL_SEC_VALVE_PIN, OUTPUT);
    pinMode(AUTO_SEC_VALVE_PIN, OUTPUT);
}

void Valves::setINS_VALVE(float level)
{
    level = level > 100 ? 100 : level;
    level = level < 0 ? 0 : level;

    int value;
    // It's only a try
    if (level > 2) {
        float z = (level - 62)/32;
        //value = int(-12 * pow(z, 4) + 31 * pow(z, 3) + 43 * pow(z, 2) + 43 * z + 780);
        //value = int(-33 * pow(z, 10) + 11 * pow(z, 9) + 72 * pow(z, 8) - 110 * pow(z, 7) - 28 * pow(z, 6) + 210 * pow(z, 5) - 2.2 * pow(z, 4) - 110 * pow(z, 3) + 16 * pow(z, 2) + 71 * z + 790);
        value = 17 * pow(z, 3) + 37 * pow(z, 2) + 60 * z + 800;
    } else {
        value = 670;
    }

    //int value = int(3*level + 650); // [0,100] signal convertion to [650,950] pwm
    ledcWrite(INS_PROP_VALVE_CH, value);
}

void Valves::setINS_VALVE_PWM(int value)
{
    value = value > 1023 ? 1023 : value;
    value = value < 0 ? 0 : value;
    ledcWrite(INS_PROP_VALVE_CH, value);
}


void Valves::setEXP_VALVE(float level)
{
    level = level > 100 ? 100 : level;
    level = level < 0 ? 0 : level;

    level = -(level - 100); // Normally openned valve requires treatment to behave as expected in system's agreement, especified in valves.h 
    int value = int((level/100) * 1023); // [0,100] signal convertion to [0,1023] pwm    
    ledcWrite(EXP_PROP_VALVE_CH, value);
}

void Valves::setMANUAL_SEC_VALVE(bool on)
{
    digitalWrite(MANUAL_SEC_VALVE_PIN, on);
}

void Valves::setAUTO_SEC_VALVE(bool on)
{
    digitalWrite(AUTO_SEC_VALVE_PIN, on);
}
