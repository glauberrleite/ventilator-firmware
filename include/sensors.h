#include <math.h>
#include <Wire.h>
#include <Adafruit_ADS1015.h>
#include "sfm3000wedo.h"
#include "I2Cbus.h" 


#define   ADC_16BIT_MAX     65536
#define   M_PI              3.14159265358979323846

class Sensors {
    protected:
        //Sensors variables
        // Internal system flow
        float fl_int;
        // Pacient related flow
        float fl_pac;
        // Pressure difference to compute flux
        float diff_pres_pac;
        // Pacient pressure
        float pres_pac;
        // Internal system pressure
        float pres_int;
        // External system pressure
        float pres_ext;

        float filter_weight;

        float venturi_pac;

        // Poiseulle constant calculated from sensor parameters
        float const_flux;

        Adafruit_ADS1115 ads1;
        Adafruit_ADS1115 ads2;
        Adafruit_ADS1115 ads3;
        SFM3000wedo measflow;
        float ads_bit_Voltage;
        float ads3_bit_Voltage;
        float fio2;

    public:
        float bias;

        Sensors(float filter_weight = 0.5);

        void update();//Updates all sensors variables to all current sensor readings

        void setFilterWeight(float weight);

        float getFL_INT();//Returns last updated value from FLow sensor in SLPM 
        float getFL_PAC();
        float getVenturi_PAC();
               
        // Functions below returns latest updated values from Pressure sensors in PSI
        float getPRES_PAC_PSI();
        float getPRES_INT_PSI();
        float getPRES_EXT_PSI();
        
        // Functions below returns latest updated values from Pressure sensors in cm3H2O
        float getPRES_PAC_cm3H2O();
        float getPRES_INT_cm3H2O();
        float getPRES_EXT_cm3H2O();
        float getDIFF_PRES_PAC_cm3H2O();
        float getDIFF_PRES_PAC_PSI();

        float getFi02();

        void resetSFM();

    private:
        //Functions below are called in update() with the readings from corresponding ports set as parameters
        //Equations used in this implementation were educed from datasheet info
        float getFlowAWM720P(float v);
        float getPressureASDX(float v, float p_min, float p_max);//minimum and maximum pressures are determined for each sensor by datasheet
        float getPressureASDX001PDAA5(float v);//datasheet determines [-1,1] PSI reading range
        float getPressureASDX005NDAA5(float v);//datasheet determines [-5,5] inH2O reading range
        float getPressureNewASDX001PDAA5(float v);//datasheet determines [-1,1] PSI reading range
        float getFlowSFM3300(); //Sensirion sensor

};
