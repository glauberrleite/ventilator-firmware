#include <math.h>
#include <Wire.h>
#include <Adafruit_ADS1015.h>

#define   ADC_16BIT_MAX     65536

class Sensors {
    protected:
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

        void update();

        float getFL_PAC_INS();
        float getFL_PAC_EXP();
        float getFL_INT();
        float getPRES_PAC();
        float getPRES_INT();

    private:
        float getFlowAWM720P(float v);
        float getPressureASDX(float v, float p_min, float p_max);
        float getPressureASDX001PDAA5(float v);
        float getPressureASDX005NDAA5(float v);
};
