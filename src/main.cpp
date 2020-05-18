/*
 *  Microcontroller module for the ventilator project developed by EASY Engineering and Systems Group
 *  Instituto de Computação - Universidade Federal de Alagoas
 *  @author glauberrleite
 *  @author vangasse
 */

#include <Arduino.h>
#include "sensors.h"

#define   PWM_FREQ              25000
#define   PWM_RES               8
#define   LEFT_PROP_VALVE_PIN   5
#define   RIGHT_PROP_VALVE_PIN  18
#define   LEFT_PROP_VALVE_CH    0
#define   RIGHT_PROP_VALVE_CH   1
#define   LEFT_SEC_VALVE_PIN    33
#define   RIGHT_SEC_VALVE_PIN   25

int cnt = 0;
Sensors sensors;

// Custom functions

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

  sensors = Sensors();

  // Proportional Valves setup
  ledcSetup(LEFT_PROP_VALVE_CH, PWM_FREQ, PWM_RES);
  ledcSetup(RIGHT_PROP_VALVE_CH, PWM_FREQ, PWM_RES);
    
  ledcAttachPin(LEFT_PROP_VALVE_PIN, LEFT_PROP_VALVE_CH);
  ledcAttachPin(RIGHT_PROP_VALVE_PIN, RIGHT_PROP_VALVE_CH);

  // Security Valve configs
  pinMode(LEFT_SEC_VALVE_PIN, OUTPUT);
  pinMode(RIGHT_SEC_VALVE_PIN, OUTPUT);

}

void loop() {

  if (cnt > 100) {
    cnt = 0; 
    sensors.update();

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
  
  // Expiratory Valve Test

  if (Serial.available() > 0) {
    // Lê toda string recebida
    String received = readStringSerial();
  
    String part01 = getValue(received, ',', 0);
    String part02 = getValue(received, ',', 1);

    int value = part02.toInt();

    if (part01.equals("LSV")) {
      digitalWrite(LEFT_SEC_VALVE_PIN, value);
    } else if (part01.equals("RSV")) {
      digitalWrite(RIGHT_SEC_VALVE_PIN, value);
    } else if (part01.equals("LPV")) {
      ledcWrite(LEFT_PROP_VALVE_CH, value);
    } else {
      ledcWrite(RIGHT_PROP_VALVE_CH, value);
    }

    Serial.print(part01);
    Serial.print('\t');
    Serial.println(part02);
  
  }

  cnt++;
  delay(10);
}