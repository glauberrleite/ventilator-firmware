#include <math.h>
#include <Wire.h>
#include <Adafruit_ADS1015.h>

#define   ADC_16BIT_MAX     65536

class Sensors {
    protected:
        //Sensors variables
        //Pressure readings from transducers used in flow estimation
        float fl_ex_pres;//external transducer
        float fl_in_pres;//internal transducer
        //Internal system flow
        float fl_int;
        //Pacient related flow
        float fl_pac;
        //Pacient pressure
        float pres_pac;
        //Internal system pressure
        float pres_int;

        Adafruit_ADS1115 ads1;
        Adafruit_ADS1115 ads2;
        float ads_bit_Voltage;

    public:
        Sensors();

        void update();//Updates all sensors variables to all current sensor readings

        float getFL_INT();//Returns last updated value from FLow sensor in SLPM 
        float getFL_PAC();
                
        //Functions below returns latest updated values from Pressure sensors in PSI
        float getPRES_PAC_PSI();
        float getPRES_INT_PSI();
        
        //Functions below returns latest updated values from Pressure sensors in cm3H2O
        float getFL_PAC_INS_cm3H2O();
        float getFL_PAC_EXP_cm3H2O();
        float getPRES_PAC_cm3H2O();
        float getPRES_INT_cm3H2O();

    private:
        //Functions below are called in update() with the readings from corresponding ports set as parameters
        //Equations used in this implementation were educed from datasheet info
        float getFlowAWM720P(float v);
        float getPressureASDX(float v, float p_min, float p_max);//minimum and maximum pressures are determined for each sensor by datasheet
        float getPressureASDX001PDAA5(float v);//datasheet determines [-1,1] PSI reading range
        float getPressureASDX005NDAA5(float v);//datasheet determines [-5,5] PSI reading range

        //Functions below returns latest updated values from Pressure sensors in PSI
        //These pressures refers to a pair of transducers used to determine flow through Bernoulli equations
        float getFL_PAC_INS_PSI();
        float getFL_PAC_EXP_PSI();
};
