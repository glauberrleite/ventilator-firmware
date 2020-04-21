/*
  Microcontroller module for the ventilator project developed by EASY Engineering and Systems Group
  Instituto de Computação - Universidade Federal de Alagoas

  @author glauberrleite
*/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ADS1015.h>

#define   ADC_16BIT_MAX   65536

Adafruit_ADS1115 ads(0x48);

float ads_bit_Voltage;

void setup() {
  Serial.begin(9600);

  ads.setGain(GAIN_TWOTHIRDS);      // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  //  ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  //  ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
  //  ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
  //  ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
  //  ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV

  ads.begin();

  float ads_InputRange = 6.144f;
  ads_bit_Voltage = (ads_InputRange * 2) / (ADC_16BIT_MAX - 1);

  Serial.println("ADS Voltage (V) \tPressure (cm3 H2O)");
}

void loop() {
  int16_t ads_ch0 = 0;
  float ads_Voltage_ch0 = 0.0f;

  // Reading and converting ADC Raw data to Voltage
  ads_ch0 = ads.readADC_SingleEnded(0);
  ads_Voltage_ch0 = ads_ch0 * ads_bit_Voltage;

  // Printing voltage and pressure
  Serial.print(ads_Voltage_ch0);
  Serial.print("\t\t");  
  // When working with 5V supply, the sensor gives 2.5 V for 0 cm3 H2O and 5 V for 5 cm3 H2O
  Serial.print(map(ads_Voltage_ch0, 2.5 * 3.3/5, 3.3, 0, 5)); // Considering Voltage Divider 3.3/5

  Serial.println();

  delay(500);
}