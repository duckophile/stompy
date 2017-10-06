#ifndef __JOYSTOCK_H__
#define __JOYSTICK_H__

extern int joystick_values[3];

extern int joystick_pins[3];

#define JOYSTICK_BITS	10
#define JOYSTICK_MID	((1 << JOYSTICK_BITS) / 2)

#define JOYSTICK_OFF		0
#define JOYSTICK_JOINT		1
#define JOYSTICK_POSITION	2

extern int joystick_mode;

int toggle_joystick_mode(void);
int func_jtest(void);

#endif /* __JOYSTICK_H__ */
