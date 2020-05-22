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
    // Valve starts to open around 68 percent, so we need to offset to make level variable work from 0 to 100
    float offset = level*(100 - 68)/100 + 68;
    int value = int((offset/100) * 1023); // [0,100] signal convertion to [0,1023] pwm
    ledcWrite(INS_PROP_VALVE_CH, value);
}

void Valves::setEXP_VALVE(float level)
{
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
