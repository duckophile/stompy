#ifndef __GLOBALS_H__
#define __GLOBALS_H__

/* Most of these should go away. */
extern int sensor_goal[NR_SENSORS];
extern int current_sensor[NR_SENSORS];

extern double xyz_goal[3];
extern double angle_goals[3];

extern int deadMan;
extern int deadman_forced;

extern double current_deg[3];
extern double current_rad[3];
extern double current_xyz[3];

extern volatile int debug_flag;

#define DEBUG   if(debug_flag)Serial.print
#define DEBUGLN if(debug_flag)Serial.println

#endif /* __GLOBALS_H__ */
