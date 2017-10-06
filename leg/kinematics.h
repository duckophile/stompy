#ifndef __KINEMATICS_H__
#define __KINEMATICS_H__

void calculate_angles(int sensors[], double degrees[], double rad[]);
void calculate_xyz(double xyz[], double rad[]);
int inverse_kin(double *xyz, int *sense_goals, double *deg_goals);
int test_kinematics(int sensor1, int sensor2, int sensor3);

#endif /* __KINEMATICS_H__ */
