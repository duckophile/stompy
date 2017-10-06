#ifndef __JOYSTOCK_H__
#define __JOYSTICK_H__

extern int joystick_values[3];

extern int joystick_pins[3];

#define JOYSTICK_BITS	10
#define JOYSTICK_MID	((1 << JOYSTICK_BITS) / 2)

#endif /* __JOYSTICK_H__ */
