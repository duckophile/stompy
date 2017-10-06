#ifndef __LEG_H__
#define __LEG_H__

int func_info(void);
int check_deadman(void);
void read_sensors(int *sensors);
void print_ftuple(double xyz[3]);
int read_sensor(int joint);
int park_leg(void);

void reset_current_location(void);

extern int sensorPin[NR_SENSORS];

/* leg link lengths hip, thigh, and knee */
#define L1		11
#define L2		54
#define L3		72

#define HIP_LEN		11	/* Hip pivot to thigh pivot, in inches. */
#define THIGH_LEN	54	/* Thigh pivot to knee pivot, in inches. */
#define KNEE_LEN	72	/* Knee pivot to ankle pivot, in inches. */

#define X		0
#define Y		1
#define Z		2

#endif /* __LEG_H__ */
