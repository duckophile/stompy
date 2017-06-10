#ifndef _LEG_INFO_
#define _LEG_INFO_

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
#define NR_JOINTS	4		/* Hip, Thigh, Knee, Compliant link. */
#define NR_SENSORS	NR_JOINTS       /* Hip, Thigh, Knee, Compliant link. */
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
} leg_info_t;

#define ANGLE_LOW(__joint)        leg_info.joint_angles[__joint].angle_low
#define ANGLE_HIGH(__joint)       leg_info.joint_angles[__joint].angle_high

#define UNITS_PER_DEG(__joint)    leg_info.joint_angles[__joint].units_per_deg

#define SENSOR_LOW(__joint)       leg_info.sensor_limits[__joint].sensor_low
#define SENSOR_HIGH(__joint)      leg_info.sensor_limits[__joint].sensor_high

/*
 * There are six valves and three joints.  The valves are arranged (in
 * software) as three valves for moving out and three valves for
 * moving in.  OUT = 0, IN = 3, so (joint + direction) = valve.
 */

#define LOW_PWM_MOVEMENT(__valve) leg_info.valves[__valve].low_joint_movement

#define JOINT_SPEED(__valve, __pwm) leg_info.valves[__valve].joint_speed[__pwm]

#endif /* _LEG_INFO_ */
