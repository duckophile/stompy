#ifndef __PWM_H__
#define __PWM_H__

void set_scaled_pwm_goal(int pwm_id, int percentage);
void pwms_off(void);
int set_pwm_scale(int scale);
int get_pwm_scale(void);
void set_pwm_freq(int freq);
void set_pwm(int valve, int percent);
void adjust_pwms(void);
void set_pwm_goal(int pwm_id, int value);

extern int current_pwms[NR_VALVES];

#endif /* __PWM_H__ */
