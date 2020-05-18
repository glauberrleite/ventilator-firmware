#define   PWM_FREQ              25000
#define   PWM_RES               8
#define   INS_PROP_VALVE_PIN    5
#define   EXP_PROP_VALVE_PIN    18
#define   INS_PROP_VALVE_CH     0
#define   EXP_PROP_VALVE_CH     1
#define   MANUAL_SEC_VALVE_PIN  33
#define   AUTO_SEC_VALVE_PIN    25

class Valves {
    public:
        Valves();

        void setINS_VALVE(float level);
        void setEXP_VALVE(float level);
        void setMANUAL_SEC_VALVE(bool on);
        void setAUTO_SEC_VALVE(bool on);
};
