#define   PWM_FREQ              25000
#define   PWM_RES               10
#define   INS_PROP_VALVE_PIN    5
#define   EXP_PROP_VALVE_PIN    18
#define   INS_PROP_VALVE_CH     0
#define   EXP_PROP_VALVE_CH     1
#define   MANUAL_SEC_VALVE_PIN  33
#define   AUTO_SEC_VALVE_PIN    25

class Valves {
    public:
        Valves();

        /* Valve 'level' can vary in [0,100], of whitch:
                - 0 will close it completely; 
                - 100 will open it completely.
            In other words, 'level' represents valve's openness*/
        void setINS_VALVE(float level); // Inspiratory valve
        void setINS_VALVE_PWM(int value); // Inspiratory valve in PWM (for tests)
        void setEXP_VALVE(float level); // Expiratory valve
        //Security valves can be either open(true) or closed(false)
        void setMANUAL_SEC_VALVE(bool on);
        void setAUTO_SEC_VALVE(bool on);
};
