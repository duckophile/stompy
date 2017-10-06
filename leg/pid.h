#ifndef __PID_H__
#define __PID_H__

typedef struct {
    double kp;
    double ki;
    double kd;
    double last_input;
    double i_term;
} pidstate_t;

#endif /* __PID_H__ */
