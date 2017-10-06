#ifndef __MISC_H__
#define __MISC_H__

void enable_leg(void);
void disable_leg(void);
int set_leg_state(int state);

extern volatile int debug_flag;

#define DEBUG   if(debug_flag)Serial.print
#define DEBUGLN if(debug_flag)Serial.println

#endif /* __MISC_H__ */
