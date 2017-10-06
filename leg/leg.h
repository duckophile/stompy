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

#endif /* __LEG_H__ */
