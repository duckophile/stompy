/* ; -*- mode: C ;-*- */

#include <Arduino.h>
#include <math.h>
#include "leg-info.h"
#include "leg.h"
#include "globals.h"
#include "misc.h"

/*
 * Takes an X,Y,Z location and calculates the joint angles in degrees
 * and radians that will give that location.
 *
 * Also Sets the sensor goals that correspond to those angles.
 *
 * Input is (x,y,z) coordinate goals, in inches.
 *
 * Output is desired sensor readings in sensor_goals[HIP, THIGH, KNEE]
 * and degree goals in deg_goals[HIP, THIGH, KNEE].
 *
 * XXX fixme: This should be two functions, one to convert (x,y,z) to
 * three angles, and another to convert that to sensor values.
 */
int inverse_kin(double *xyz, int *sense_goals, double *deg_goals)
{
    int rc = 0;
    double theta1R, theta2R, theta3R; /* Goal angles in degrees. */
    double theta1, theta2, theta3; /* Goal angles in radians. */
    double hipGoal, thighGoal, kneeGoal; /* Goal sensor values. */

#if 0
    Serial.print("New goal (x,y,z): ");
    Serial.print("(");
    Serial.print(xyz[X]);
    Serial.print(", ");
    Serial.print(xyz[Y]);
    Serial.print(", ");
    Serial.print(xyz[Z]);
    Serial.print(")\n");
#endif

    /* HIP - theta1 */
    theta1R = atan(xyz[Y]/xyz[X]);
    //convert to degrees
    theta1 = (theta1R * 4068) / 71;
    //angle goal to pot reading
    hipGoal = ((theta1 - ANGLE_LOW(HIP)) * UNITS_PER_DEG(HIP)) + SENSOR_LOW(HIP);

    if ((hipGoal < SENSOR_LOW(HIP)) || (hipGoal > SENSOR_HIGH(HIP))) {
        Serial.print("HIP GOAL ");
        Serial.print(hipGoal);
        Serial.print(" OUT OF RANGE (");
        Serial.print(SENSOR_LOW(HIP));
        Serial.print(", ");
        Serial.print(SENSOR_HIGH(HIP));
        Serial.print(")\n");
        hipGoal = constrain(hipGoal, SENSOR_LOW(HIP), SENSOR_HIGH(HIP));
        Serial.print("constrained goal: ");
        Serial.print(hipGoal);
        Serial.print('\n');
        rc = -1;
    }

    /* Thigh - theta2 */
    double r;
    double x1;
    if (theta1R == 0) {
        x1 = (xyz[X] - L1);
    }
    else {
        x1 = (xyz[Y]/sin(theta1R)) - L1;
    }
    x1 = abs(x1);
    double beta = atan(xyz[Z]/x1);
    if (xyz[X] == L1) {
        beta = -(PI/2);
    }
    else if (xyz[X] < L1) {
        if (xyz[Z] == 0) {
            r = x1;
        }
        else {
            r = xyz[Z]/sin(beta);
        }
        r = abs(r);
        double gama = asin(x1/r);
        beta = -(gama + PI/2);
    }
    else {
        beta = atan(xyz[Z]/x1);
    }
    if (xyz[Z] == 0) {
        r = x1;
    }
    else {
        r = xyz[Z]/sin(beta);
    }
    r = abs(r);
    theta2R = beta + acos((sq(L2) + sq(r) - sq(L3))/(2*L2*r));
    theta2 = (theta2R * 4068) / 71;
    //thighGoal is sensor reading at goal angle
    thighGoal = SENSOR_LOW(THIGH) + ((ANGLE_HIGH(THIGH) - theta2) * UNITS_PER_DEG(THIGH));
    if (thighGoal < SENSOR_LOW(THIGH) || thighGoal > SENSOR_HIGH(THIGH)) {
        Serial.print("THIGH GOAL ");
        Serial.print(thighGoal);
        Serial.print(" OUT OF RANGE!!\n");
        thighGoal = constrain(thighGoal, SENSOR_LOW(THIGH), SENSOR_HIGH(THIGH));
        Serial.print("constrained goal: ");
        Serial.print(thighGoal);
        Serial.print('\n');
        rc = -1;
    }

    /* Knee - theta3 */
    theta3R = acos( (sq(L3) + sq(L2) - sq(r)) / (double)(2*L3*L2));
    theta3 = (theta3R * 4068) / 71;
#if 0
    Serial.print(acos( (sq(L3) + sq(L2) - sq(r)) / (double)(2*L3*L2)));
    Serial.print(' ');
    Serial.print(theta3R);
    Serial.print('\n');
    Serial.print("knee rad = ");
    Serial.print(theta3R);
    Serial.print(" which is from acos( sq(");
    Serial.print(L3);
    Serial.print(") + sq(");
    Serial.print(L2);
    Serial.print(") - sq(");
    Serial.print(r);
    Serial.print(") = ");
    Serial.print(sq(L3) + sq(L2) - sq(r));
    Serial.print(" / ");
    Serial.print((2*L3*L2));
    Serial.print(" ), or acos( ");
    Serial.print((sq(L3) + sq(L2) - sq(r)) / (double)(2*L3*L2));

    Serial.print(" / ");
    Serial.print((2*L3*L2));
    Serial.print(" )  = ");
    Serial.print(theta3R);
    Serial.print(". theta3 = ");
    Serial.print(theta3);
    Serial.print('\n');
#endif
    kneeGoal= ((ANGLE_HIGH(KNEE) - theta3) * UNITS_PER_DEG(KNEE)) + SENSOR_LOW(KNEE);
    //kneeGoal = ((theta3 - ANGLE_LOW(KNEE)) * UNITS_PER_DEG(KNEE)) + SENSOR_LOW(KNEE);
    if (kneeGoal < SENSOR_LOW(KNEE) || kneeGoal > SENSOR_HIGH(KNEE)) {
        Serial.print("KNEE GOAL ");
        Serial.print(kneeGoal);
        Serial.print(" OUT OF RANGE!!\n");
        rc = -1;
        // kneeGoal = constrain(kneeGoal, SENSOR_LOW(KNEE), SENSOR_HIGH(KNEE));
        // Serial.print("constrained goal: ");
    }

    if (isnan(theta1) || isnan(theta2) || isnan(theta3)) {
        Serial.print("**** Got NaN for angles!\n");
        Serial.print(theta1);
        Serial.print('\n');
        Serial.print(theta2);
        Serial.print('\n');
        Serial.print(theta3);
        Serial.print('\n');

        rc = -1;
    }

    if (rc == 0) {
        sense_goals[HIP]   = hipGoal;
        sense_goals[THIGH] = thighGoal;
        sense_goals[KNEE]  = kneeGoal;

        deg_goals[HIP]   = theta1;
        deg_goals[THIGH] = theta2;
        deg_goals[KNEE]  = theta3;
    }

#if 0
    Serial.print("\nNew Goals: (x,y,z):\t");
    print_ftuple(xyz);
    Serial.print('\n');
    Serial.print("Goal angles (deg):\t");
    Serial.print(theta1);
    Serial.print(" ");
    Serial.print(theta2);
    Serial.print(" ");
    Serial.print(theta3);

    Serial.print('\t');
    Serial.print("Sensor goals:\thip\t");
    Serial.print(sense_goals[HIP]);
    Serial.print("\tthigh\t");
    Serial.print(sense_goals[THIGH]);
    Serial.print("\tknee\t");
    Serial.print(sense_goals[KNEE]);
    Serial.print('\t');
#endif

    return rc;
}

/*
 * Forward kinematics.
 *
 * Calculate joint angles in both degrees and radians, based on the
 * last sensor readings.
 *
 * Takes sensor readings from sensors[][] and calculates the
 * angles in degrees into degrees[] and radians into rad[].
 *
 * XXX fixme:  The units per degree isn't constant over the movement of the joint!
 */
void calculate_angles(int sensors[], double degrees[], double rad[])
{
#warning units_per_deg isnt constant over the travel of the sensor!
    degrees[HIP]   = ((sensors[HIP] - SENSOR_LOW(HIP)) / UNITS_PER_DEG(HIP)) + ANGLE_LOW(HIP);
    degrees[THIGH] = ANGLE_HIGH(THIGH) - ((sensors[THIGH] - SENSOR_LOW(THIGH)) / UNITS_PER_DEG(THIGH));
    degrees[KNEE]  = ANGLE_HIGH(KNEE)  - ((sensors[KNEE]  - SENSOR_LOW(KNEE))  / UNITS_PER_DEG(KNEE));

    for (int i = 0;i < 3;i++)
        rad[i] = (degrees[i] * 71) / 4068;

    DEBUG("Angles:\t");
    for (int i = 0; i < 3; i++) {
        /* Read sensors even if we're not moving. */
        DEBUG("\t");
        DEBUG(joint_names[i]);
        DEBUG("\t");
        DEBUG(degrees[i]);
    }
    DEBUG('\n');

    return;
}

/*
 * Forward kinematics.
 *
 * Takes the current angles from rad[] and converts to the current
 * (x,y,z).
 *
 * XXX swap the order of xyz[] and rad[].
 */
void calculate_xyz(double xyz[], double rad[])
{
    xyz[X] = cos(rad[HIP]) * (L1 + L2*cos(rad[THIGH]) + L3*cos(rad[THIGH] + rad[KNEE] - PI));
    xyz[Y] = xyz[X] * tan(rad[HIP]);
    xyz[Z] = (L2 * sin(rad[THIGH])) + (L3 * sin(rad[THIGH] + rad[KNEE] - PI));

    DEBUG("Current\t\tX:\t");
    DEBUG(xyz[X]);
    DEBUG("\tY:\t");
    DEBUG(xyz[Y]);
    DEBUG("\tZ:\t");
    DEBUG(xyz[Z]);
    DEBUG('\n');

    return;
}

int test_kinematics(int sensor1, int sensor2, int sensor3)
{
    int i;
    int sensors[3];
    double radians[3];
    double degrees[3];
    double xyz[3];

    Serial.print("\n################################################################\n");
    Serial.print("# Testing kinematics.\n");

    sensors[0] = sensor1;
    sensors[1] = sensor2;
    sensors[2] = sensor3;

    calculate_angles(sensors, degrees, radians);
    calculate_xyz(xyz, radians);

    Serial.print("#\n# ---------------- Forward kinematics ----------------\n");

    Serial.print("# Sensors: ");
    for (i = 0;i < 3;i++) {
        Serial.print('\t');
        Serial.print(sensors[i]);
    }
    Serial.print("\n");

    Serial.print("# Degrees: ");
    for (i = 0;i < 3;i++) {
        Serial.print('\t');
        Serial.print(degrees[i]);
    }
    Serial.print("\n");

    Serial.print("# XYZ:     ");
    for (i = 0;i < 3;i++) {
        Serial.print('\t');
        Serial.print(xyz[i]);
    }
    Serial.print("#\n");

    inverse_kin(xyz, sensors, degrees);

    Serial.print("#\n# ---------------- Inverse kinematics ----------------\n");

    Serial.print("# XYZ:     ");
    for (i = 0;i < 3;i++) {
        Serial.print('\t');
        Serial.print(xyz[i]);
    }
    Serial.print("\n");

    Serial.print("# Degrees: ");
    for (i = 0;i < 3;i++) {
        Serial.print('\t');
        Serial.print(degrees[i]);
    }
    Serial.print("\n");

    Serial.print("# Sensors: ");
    for (i = 0;i < 3;i++) {
        Serial.print('\t');
        Serial.print(sensors[i]);
    }
    Serial.print("#\n");

    Serial.print("#\n# Done with kinematics test\n################################################################\n\n");

    return 0;
}
