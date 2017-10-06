#ifndef __VELOCITY_H__
#define __VELOCITY_H__

void toggle_velocity_debug(void);
int velocity_init(void);
void velocity_loop(void);

extern volatile int velocity_debug;

#endif /* __VELOCITY_H__ */
