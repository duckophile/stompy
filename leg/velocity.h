#ifndef __VELOCITY_H__
#define __VELOCITY_H__

int set_xyz_goal(double xyz[3], int speed);
void toggle_velocity_debug(void);
int velocity_init(void);
void velocity_loop(void);

extern volatile int velocity_debug;

extern uint8_t point_list_buf[1024];

#endif /* __VELOCITY_H__ */
