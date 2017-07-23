typedef struct {
    double kp;
    double ki;
    double kd;
    double last_input;
    double i_term;
} pidstate_t;
