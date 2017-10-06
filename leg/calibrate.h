#ifndef __CALIBRATE_H__
#define __CALIBRATE_H__

int set_joint_limits(int joint);
int calibrate_joint(int joint, int rep_count);
int move_joint_all_the_way(int valve, int pwm_percent);

#endif /* __CALIBRATE_H__ */
