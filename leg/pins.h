#ifndef __PINS_H__
#define __PINS_H__

/* Joystick */

#ifdef OLD_BOARD
#define JOYSTICK_X_PIN		14
#define JOYSTICK_Y_PIN		15
#define JOYSTICK_Z_PIN		16
#define DEADMAN_PIN		0
#define ENABLE_PIN		1	/* Knee and thigh */
#define ENABLE_PIN_HIP		2
#else
#define JOYSTICK_X_PIN		31
#define JOYSTICK_Y_PIN		30
#define JOYSTICK_Z_PIN		29
#define DEADMAN_PIN		25
#define ENABLE_PIN		8	/* Knee and thigh */
#define ENABLE_PIN_HIP		2

#define PRESSURE_SENSOR_1	28
#define PRESSURE_SENSOR_2	27
#define PRESSURE_SENSOR_3	26
#define PRESSURE_SENSOR_4	A12
#define PRESSURE_SENSOR_MANIFOLD A13
#endif

#define KNEE_SENSOR_PIN		15
#define THIGH_SENSOR_PIN	17
#define HIP_SENSOR_PIN		20
#define CALF_SENSOR_PIN		16

#define THIGHPWM_OUT_PIN	3
#define THIGHPWM_IN_PIN		4
#define KNEEPWM_OUT_PIN		5
#define KNEEPWM_IN_PIN		6
#define HIPPWM_IN_PIN		10
#define HIPPWM_OUT_PIN		9

#define M1FB_PIN		21
#define M2FB_PIN		22
#define M1FB_HIP_PIN		23

#endif /* __PINS_H__ */
