#include "sensors.h"

Sensors::Sensors()
{
  // I2C addresses for Analog-Digital Convertes
  this->ads1 = Adafruit_ADS1115(0x48);
  this->ads2 = Adafruit_ADS1115(0x49);
  
  // ADS1115 Analog Digital Converter configs
  this->ads1.setGain(GAIN_TWOTHIRDS);      // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  this->ads2.setGain(GAIN_TWOTHIRDS);      // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  //  ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  //  ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
  //  ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
  //  ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
  //  ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV

  this->ads1.begin();
  this->ads2.begin();

  float ads_InputRange = 6.144f;
  this->ads_bit_Voltage = (ads_InputRange * 2) / (ADC_16BIT_MAX - 1);

  // Initializing attributes  
  this->fl_pac_ins = 0;
  this->fl_pac_exp = 0;
  this->fl_int = 0;
  this->pres_pac = 0;
  this->pres_int = 0;

}

void Sensors::update()
{
    int16_t ads1_ch0 = 0;
    int16_t ads2_ch0 = 0;
    int16_t ads2_ch1 = 0;
    int16_t ads2_ch2 = 0;
    int16_t ads2_ch3 = 0;

    // Getting sensors data
    float ads1_Voltage_ch0 = 0.0f;
    float ads2_Voltage_ch0 = 0.0f;
    float ads2_Voltage_ch1 = 0.0f;
    float ads2_Voltage_ch2 = 0.0f;
    float ads2_Voltage_ch3 = 0.0f;

    // Reading and converting ADC Raw data to Voltage
    ads1_ch0 = this->ads1.readADC_SingleEnded(0);
    ads2_ch0 = this->ads2.readADC_SingleEnded(0);
    ads2_ch1 = this->ads2.readADC_SingleEnded(1);
    ads2_ch2 = this->ads2.readADC_SingleEnded(2);
    ads2_ch3 = this->ads2.readADC_SingleEnded(3);
    
    ads1_Voltage_ch0 = ads1_ch0 * ads_bit_Voltage;
    ads2_Voltage_ch0 = ads2_ch0 * ads_bit_Voltage;
    ads2_Voltage_ch1 = ads2_ch1 * ads_bit_Voltage;
    ads2_Voltage_ch2 = ads2_ch2 * ads_bit_Voltage;
    ads2_Voltage_ch3 = ads2_ch3 * ads_bit_Voltage;

    
    this->fl_pac_ins = getPressureASDX001PDAA5(ads2_Voltage_ch0);
    this->fl_pac_exp = getPressureASDX001PDAA5(ads2_Voltage_ch1);
    this->fl_int = getFlowAWM720P(ads1_Voltage_ch0);
    this->pres_pac = getPressureASDX005NDAA5(ads2_Voltage_ch2);
    this->pres_int = getPressureASDX005NDAA5(ads2_Voltage_ch3);
}

float Sensors::getFlowAWM720P(float v)
{
  // Readings from AWM720P varies from 1V to 5V
  v = v < 1.0 ? 1.0 : v;
  v = v > 5.0 ? 5.0 : v;

  float flow = 0;

  // Flow is determined by a piece wise function of Tension described below
  // Polynomial coefficients in our interpolation method are local coefficients for each interval
  if (v >= 1 && v < 2.99) {
    float rel = (v - 1);

    flow = 1.9385 * pow(rel, 3) - 3.0982 * pow(rel, 2) + 11.0514 * rel;
  }
  else if (v >= 2.99 && v < 3.82) {
    float rel = (v - 2.99);

    flow = 1.9385 * pow(rel, 3) + 8.4748 * pow(rel, 2) + 21.7509 * rel + 25;
  }
  else if (v >= 3.82 && v < 4.3) {
    float rel = (v - 3.82);

    flow = 25.4906 * pow(rel, 3) + 13.3017 * pow(rel, 2) + 39.8255 * rel + 50;
  }
  else if (v >= 4.3 && v < 4.58) {
    float rel = (v - 4.3);

    flow = 64.6584 * pow(rel, 3) + 50.0081 * pow(rel, 2) + 70.2142 * rel + 75;
  }
  else if (v >= 4.58 && v < 4.86) {
    float rel = (v - 4.58);

    flow = 458.3556 * pow(rel, 3) + 104.3212 * pow(rel, 2) + 113.4264 * rel + 100;
  }
  else {
    float rel = (v - 4.86);

    flow = 458.3556 * pow(rel, 3) + 489.3389 * pow(rel, 2) + 279.6515 * rel + 150;
  }

  return flow;
}

float Sensors::getPressureASDX(float v, float p_min, float p_max)
{  
  v = v < 0.5 ? 0.5 : v;
  v = v > 4.5 ? 4.5 : v;

  return ((v - 0.5)*(p_max - p_min)/4) + p_min;
}

float Sensors::getPressureASDX001PDAA5(float v)
{
  return this->getPressureASDX(v, -1, 1);
}

float Sensors::getPressureASDX005NDAA5(float v)
{
  return this->getPressureASDX(v, -5, 5);
}

float Sensors::getFL_PAC_INS_PSI()
{
  return this->fl_pac_ins;
}

float Sensors::getFL_PAC_EXP_PSI()
{
  return this->fl_pac_exp;
}

float Sensors::getFL_INT()
{
  return this->fl_int;
}

float Sensors::getPRES_PAC_PSI()
{
  return this->pres_pac;
}

float Sensors::getPRES_INT_PSI()
{
  return this->pres_int;
}


float Sensors::getFL_PAC_INS_cm3H2O()
{
  return this->fl_pac_ins * 70.307;
}

float Sensors::getFL_PAC_EXP_cm3H2O()
{
  return this->fl_pac_exp * 70.307;
}

float Sensors::getPRES_PAC_cm3H2O()
{
  return this->pres_pac * 70.307;
}

float Sensors::getPRES_INT_cm3H2O()
{
  return this->pres_int * 70.307;
}