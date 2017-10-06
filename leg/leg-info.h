#ifndef _LEG_INFO_
#define _LEG_INFO_

#include <stdint.h>
#include "pins.h"

/*
 * I may also want to store something about temperature compensation.
 * Maybe sensor jitter too?
 *
 * X,Y,Z limits too.
 *
 * And angle min and max for each joint.  This will vary due to
 * manufacturing tolerances, so this may be something we want to
 * measure and set manually.
 */

#define NR_PRESSURES	5		/* 500, 1000, 1500, 2000, 2500 PSI */
#define NR_JOINTS	3		/* Hip, Thigh, Knee. */
#define NR_SENSORS	4		/* Hip, Thigh, Knee, Compliant link (Calf). */
#define NR_VALVES	6		/* One in and one out for each joint. */

/* One of these for each of 4 sensors. */
typedef struct __attribute__((packed)) {
    uint16_t sensor_high;
    uint16_t sensor_low;
} sensor_limits_t;

typedef struct __attribute__((packed)) {
    float angle_high;
    float angle_low;
    float units_per_deg;
} joint_angles_t;

/* One of these for each of 6 valves. */
typedef struct __attribute__((packed)) {
    uint16_t joint_speed[11]; /* Speed, 0-100% pwm, in 10% increments.  0% is not meaningful. */
    uint8_t low_joint_movement; /* Lowest PWM that moves joint. */
    uint8_t padding[3];
} valve_param_t;

/* One of these for each of five pressures. */
typedef struct __attribute__((packed)) {
    valve_param_t valves[NR_VALVES];
} per_pressure_t;

/* One of these for a leg. */
typedef struct __attribute__((packed)) {
    sensor_limits_t sensor_limits[NR_SENSORS];
    joint_angles_t joint_angles[NR_SENSORS];
/*    valve_param_t valves[NR_PRESSURES];*/
    valve_param_t valves[NR_VALVES];
    char name[16];
    uint8_t leg_number;
    uint8_t padding[3];
} leg_info_t;

/* Joint/sensor numbers. */
#define HIP			0
#define THIGH			1
#define KNEE			2
#define CALF			3

/* Valve numbers. */
#define HIPPWM_OUT		0
#define THIGHPWM_OUT		1
#define KNEEPWM_OUT		2
#define HIPPWM_IN		3
#define THIGHPWM_IN		4
#define KNEEPWM_IN		5

const int pwm_pins[6]  = {HIPPWM_OUT_PIN,  THIGHPWM_OUT_PIN,  KNEEPWM_OUT_PIN,
                          HIPPWM_IN_PIN,   THIGHPWM_IN_PIN,   KNEEPWM_IN_PIN};

/*
  * These are used to convert an 0-2 joint number to a valve number in
  * the pwm_pins[] array.  pwm_pins[joint + OUT] is the valve pin to
  * move a joint out, pwm_pins[jount + IN] is the valve pin to move
  * the joint in.
  */
#define OUT	0      /* Sensor value decreasing. */
#define IN	3      /* Sensor value increasing. */

extern leg_info_t leg_info;

#define ANGLE_LOW(__joint)        leg_info.joint_angles[__joint].angle_low
#define ANGLE_HIGH(__joint)       leg_info.joint_angles[__joint].angle_high

#define UNITS_PER_DEG(__joint)    leg_info.joint_angles[__joint].units_per_deg

#define SENSOR_LOW(__joint)       leg_info.sensor_limits[__joint].sensor_low
#define SENSOR_HIGH(__joint)      leg_info.sensor_limits[__joint].sensor_high

#define LEG_NUMBER()              leg_info.leg_number
#define LEG_NAME()                leg_info.name

/*
 * There are six valves and three joints.  The valves are arranged (in
 * software) as three valves for moving out and three valves for
 * moving in.  OUT = 0, IN = 3, so (joint + direction) = valve.
 */

#define LOW_PWM_MOVEMENT(__valve)   leg_info.valves[__valve].low_joint_movement
#define JOINT_SPEED(__valve, __pwm) leg_info.valves[__valve].joint_speed[__pwm]
#define JOINT_MAX_SPEED(__valve)    JOINT_SPEED(__valve, 10)

extern const char *direction_names[];
extern const char *joint_names[];
extern const char *joint_up_actions[];
extern const char *joint_down_actions[];

/* This doesn't really belong here. */
#define ANALOG_BITS	16
#define PWM_BITS	ANALOG_BITS
#define PWM_MAX		((1 << PWM_BITS) - 1)
#define ANALOG_MAX	((1 << ANALOG_BITS) - 1)


#endif /* _LEG_INFO_ */
