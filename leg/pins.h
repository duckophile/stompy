/* Joystick */

#ifdef OLD_BOARD
#define JOYSTICK_X_PIN		14
#define JOYSTICK_Y_PIN		15
#define JOYSTICK_Z_PIN		16
#define DEADMAN_PIN		0
#else
#define JOYSTICK_X_PIN		31
#define JOYSTICK_Y_PIN		30
#define JOYSTICK_Z_PIN		29
#define DEADMAN_PIN		25
#endif

#define KNEE_SENSOR_PIN		15
#define THIGH_SENSOR_PIN	17
#define HIP_SENSOR_PIN		20
#define COMPLIANT_SENSOR_PIN	XX

//These are mapped to the right front leg
#define THIGHPWM_DOWN_PIN	3
#define THIGHPWM_UP_PIN		4
#define KNEEPWM_RETRACT_PIN	5
#define KNEEPWM_EXTEND_PIN	6
#define HIPPWM_FORWARD_PIN	9
#define HIPPWM_REVERSE_PIN	10

//enable pins for motor drivers
#define ENABLE_PIN		1
#define ENABLE_PIN_HIP		2

#define M1FB_PIN		21
#define M2FB_PIN		22
#define M1FB_HIP_PIN		23
