/*
 *  Microcontroller module for the ventilator project developed by EASY Engineering and Systems Group
 *  Instituto de Computação - Universidade Federal de Alagoas
 *  @author glauberrleite
 *  @author vangasse
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ADS1015.h>
#include <sensors.h>

#define   ADC_16BIT_MAX         65536

#define   PWM_FREQ              25000
#define   PWM_RES               8
#define   LEFT_PROP_VALVE_PIN   5
#define   RIGHT_PROP_VALVE_PIN  18
#define   LEFT_PROP_VALVE_CH    0
#define   RIGHT_PROP_VALVE_CH   1
#define   LEFT_SEC_VALVE_PIN    33
#define   RIGHT_SEC_VALVE_PIN   25

Adafruit_ADS1115 ads1(0x48);
Adafruit_ADS1115 ads2(0x49);

float ads_bit_Voltage;

int cnt = 0;

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

  // ADS1115 Analog Digital Converter configs
  ads1.setGain(GAIN_TWOTHIRDS);      // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  ads2.setGain(GAIN_TWOTHIRDS);      // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  //  ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  //  ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
  //  ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
  //  ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
  //  ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV

  ads1.begin();
  ads2.begin();

  float ads_InputRange = 6.144f;
  ads_bit_Voltage = (ads_InputRange * 2) / (ADC_16BIT_MAX - 1);

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

  if (cnt > 50) {
    cnt = 0; 

    int16_t ads_ch0 = 0;
    int16_t ads_ch1 = 0;
    int16_t ads_ch2 = 0;
    int16_t ads_ch3 = 0;
    int16_t ads_ch4 = 0;

    // Getting sensors data
    float ads_Voltage_ch0 = 0.0f;
    float ads_Voltage_ch1 = 0.0f;
    float ads_Voltage_ch2 = 0.0f;
    float ads_Voltage_ch3 = 0.0f;
    float ads_Voltage_ch4 = 0.0f;

    // Reading and converting ADC Raw data to Voltage
    ads_ch0 = ads1.readADC_SingleEnded(0);
    ads_ch1 = ads2.readADC_SingleEnded(0);
    ads_ch2 = ads2.readADC_SingleEnded(1);
    ads_ch3 = ads2.readADC_SingleEnded(2);
    ads_ch4 = ads2.readADC_SingleEnded(3);
    
    ads_Voltage_ch0 = ads_ch0 * ads_bit_Voltage;
    ads_Voltage_ch1 = ads_ch1 * ads_bit_Voltage;
    ads_Voltage_ch2 = ads_ch2 * ads_bit_Voltage;
    ads_Voltage_ch3 = ads_ch3 * ads_bit_Voltage;
    ads_Voltage_ch4 = ads_ch4 * ads_bit_Voltage;

    // Printing voltage and pressure
    Serial.print(ads_Voltage_ch0);
    Serial.print("-->");  
    Serial.print(getFlowAWM720P(ads_Voltage_ch0));
    // When working with 5V supply, the sensor gives 2.5 V for 0 cm3 H2O and 5 V for 5 cm3 H2O
    Serial.print("\t\t");
    Serial.print(ads_Voltage_ch1);
    Serial.print("-->");  
    Serial.print(getPressureASDX001PDAA5(ads_Voltage_ch1));
    Serial.print("\t\t");
    Serial.print(ads_Voltage_ch2);
    Serial.print("-->");  
    Serial.print(getPressureASDX001PDAA5(ads_Voltage_ch2));
    Serial.print("\t\t");
    Serial.print(ads_Voltage_ch3);
    Serial.print("-->");  
    Serial.print(getPressureASDX005NDAA5(ads_Voltage_ch3));
    Serial.print("\t\t");
    Serial.print(ads_Voltage_ch4);
    Serial.print("-->");  
    Serial.print(getPressureASDX005NDAA5(ads_Voltage_ch4));

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