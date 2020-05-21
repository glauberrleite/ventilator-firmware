#include <math.h>
#include <Wire.h>
#include <Adafruit_ADS1015.h>

#define   ADC_16BIT_MAX     65536

class Sensors {
    protected:
        //Sensors variables
        float fl_pac_ins;
        float fl_pac_exp;
        float fl_int;
        float pres_pac;
        float pres_int;

        Adafruit_ADS1115 ads1;
        Adafruit_ADS1115 ads2;
        float ads_bit_Voltage;

    public:
        Sensors();

        void update();//Updates all sensors variables to all current sensor readings

        float getFL_INT();//Returns last updated value from FLow sensor in ________(measure)
        
        //Functions below returns latest updated values from Pressure sensors in PSI
        float getFL_PAC_INS_PSI();
        float getFL_PAC_EXP_PSI();
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
};
