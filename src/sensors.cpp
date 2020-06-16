#include "sensors.h"

Sensors::Sensors(float filter_weight)
{
  // I2C addresses for Analog-Digital Convertes
  this->ads1 = Adafruit_ADS1115(0x48);
  this->ads2 = Adafruit_ADS1115(0x49);
  this->measflow = SFM3000wedo(64);

  this->measflow.init();
  
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
  this->fl_int = 0;
  this->fl_pac = 0;
  this->pres_pac = 0;
  this->pres_int = 0;
  this->pres_ext = 0;
  this->diff_pres_pac = 0;
  this->venturi_pac =0;

  this->const_flux = 81.7140;

  this->filter_weight = filter_weight;
  this->bias = 0;
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
 
    this->fl_int = this->filter_weight * this->fl_int + (1 - this->filter_weight) * getFlowAWM720P(ads1_Voltage_ch0);
    this->pres_pac = this->filter_weight * this->pres_pac + (1 - this->filter_weight) * getPressureNewASDX001PDAA5(ads2_Voltage_ch0);
    this->pres_int = this->filter_weight * this->pres_int + (1 - this->filter_weight) * getPressureNewASDX001PDAA5(ads2_Voltage_ch1);    
    this->pres_ext = this->filter_weight * this->pres_ext + (1 - this->filter_weight) * getPressureNewASDX001PDAA5(ads2_Voltage_ch2);
    this->diff_pres_pac = getPressureASDX005NDAA5(ads2_Voltage_ch3);

    // Calculating flow_pac based on diff_press
    float signal = this->diff_pres_pac >= 0 ? 1 : -1;
    this->venturi_pac = this->filter_weight * this->venturi_pac + (1 - this->filter_weight) * signal * this->const_flux * sqrt(abs(this->diff_pres_pac)); // Flux in m3/s
    this->fl_pac = this->filter_weight * this->fl_pac + (1 - this->filter_weight) * getFlowSFM3300(); // Flux in m3/s
    
}

void Sensors::setFilterWeight(float weight) {
  weight = weight > 1 ? 1 : weight;
  weight = weight < 0 ? 0 : weight;

  filter_weight = weight;
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

// PD series return pressure in psi
float Sensors::getPressureASDX001PDAA5(float v)
{
  return this->getPressureASDX(v, -1, 1);
}

// ND series return pressure in inches of h2o
float Sensors::getPressureASDX005NDAA5(float v)
{  
  return this->getPressureASDX(v, -5, 5);
}

float Sensors::getPressureNewASDX001PDAA5(float v)
{
  v = v < 0.5 ? 0.5 : v;
  v = v > 4.5 ? 4.5 : v;

  return (v - 2.5) / 2;
}

float Sensors::getFL_INT()
{
  return this->fl_int;
}

float Sensors::getFL_PAC()
{
  // fl_pac sensor have a bias
  return this->fl_pac;
}

float Sensors::getVenturi_PAC()
{
  // venturi_pac sensor have a bias
  return this->venturi_pac + this->bias;
}


float Sensors::getPRES_PAC_PSI()
{
  return this->pres_pac;
}

float Sensors::getPRES_INT_PSI()
{
  return this->pres_int;
}

float Sensors::getPRES_EXT_PSI()
{
  return this->pres_ext;
}

float Sensors::getPRES_PAC_cm3H2O()
{
  return this->pres_pac * 70.307;
}

float Sensors::getPRES_INT_cm3H2O()
{
  return this->pres_int * 70.307;
}

float Sensors::getPRES_EXT_cm3H2O()
{
  return this->pres_ext * 70.307;
}

float Sensors::getDIFF_PRES_PAC_cm3H2O()
{
  // Need to convert to psi, then to cmH2O
  return this->diff_pres_pac * 0.0360912 * 70.307;
}

float Sensors::getDIFF_PRES_PAC_PSI()
{
  // Need to convert to psi
  return this->diff_pres_pac * 0.0360912;
}

float Sensors::getFlowSFM3300()
{
  return (measflow.getvalue()-32768)/120.0;
}