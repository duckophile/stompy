/* ; -*- mode: C ;-*- */

/*
 * Takes an X,Y,Z location and calculates the joint angles in degrees
 * and radians that will give that location.
 *
 * Also Sets the sensor goals that correspond to those angles.
 *
 * Input is (x,y,z)
 * Output is desired  sensor readings in sensor_goals[HIP, THIGH, KNEE].
 *
 * XXX fixme: This should be two functions, one to convert (x,y,z) to
 * three angles, and another to convert that to sensor values.
 */
void inverse_kin(double *xyz, int *sense_goals, double *deg_goals)
{
    double theta1R, theta2R, theta3R; /* Goal angles in degrees. */
    double theta1, theta2, theta3; /* Goal angles in radians. */
    double hipGoal, thighGoal, kneeGoal; /* Goal sensor values. */

    Serial.print("New goal (x,y,z): ");
    Serial.print("(");
    Serial.print(xyz[X]);
    Serial.print(", ");
    Serial.print(xyz[Y]);
    Serial.print(", ");
    Serial.print(xyz[Z]);
    Serial.println(")");

    /* HIP - theta1 */
    theta1R = atan(xyz[Y]/xyz[X]);
    //convert to degrees
    theta1 = (theta1R * 4068) / 71;
    //angle goal to pot reading
    hipGoal = ((theta1 - ANGLE_LOW(HIP)) * UNITS_PER_DEG(HIP)) + SENSOR_LOW(HIP);
    if (hipGoal < SENSOR_LOW(HIP) || hipGoal > SENSOR_HIGH(HIP)) {
        Serial.println("HIP GOAL OUT OF RANGE!!");
        hipGoal = constrain(hipGoal, SENSOR_LOW(HIP), SENSOR_HIGH(HIP));
        Serial.print("constrained goal: ");
        Serial.println(hipGoal);
    }
    goingHot[HIP] = 1;
    Serial.println("Hip going hot.");

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
        Serial.println("THIGH GOAL OUT OF RANGE!!");
        thighGoal = constrain(thighGoal, SENSOR_LOW(THIGH), SENSOR_HIGH(THIGH));
        Serial.print("constrained goal: ");
        Serial.println(thighGoal);
    }
    goingHot[THIGH] = 1;
    Serial.println("Thigh going hot.");

    /* Knee - theta3 */
    theta3R = acos( (sq(L3) + sq(L2) - sq(r)) / (double)(2*L3*L2));
    theta3 = (theta3R * 4068) / 71;
#if 0
    Serial.print("knee rad = ");
    Serial.print(theta3R);
    Serial.print(" which is from acos( ");
    Serial.print(sq(L3) + sq(L2) - sq(r));
    Serial.print(" / ");
    Serial.print((2*L3*L2));
    Serial.print(" ), or acos( ");
    Serial.print((sq(L3) + sq(L2) - sq(r)) / (double)(2*L3*L2));
    Serial.print(" ) ");

    Serial.print(" and ");
    Serial.print((2*L3*L2));
    Serial.print(" theta3 = ");
    Serial.print(theta3);
#endif
    kneeGoal= ((ANGLE_HIGH(KNEE) - theta3) * UNITS_PER_DEG(KNEE)) + SENSOR_LOW(KNEE);
    //kneeGoal = ((theta3 - ANGLE_LOW(KNEE)) * UNITS_PER_DEG(KNEE)) + SENSOR_LOW(KNEE);
    if (kneeGoal < SENSOR_LOW(KNEE) || kneeGoal > SENSOR_HIGH(KNEE)) {
        Serial.print("(KNEE GOAL OUT OF RANGE) ");
        // kneeGoal = constrain(kneeGoal, SENSOR_LOW(KNEE), SENSOR_HIGH(KNEE));
        // Serial.print("constrained goal: ");
    }
    goingHot[KNEE] = 1;
    Serial.println("Knee going hot.");

    sense_goals[HIP]   = hipGoal;
    sense_goals[THIGH] = thighGoal;
    sense_goals[KNEE]  = kneeGoal;

    deg_goals[HIP]   = theta1;
    deg_goals[THIGH] = theta2;
    deg_goals[KNEE]  = theta3;

    Serial.print("\nNew Goals: (x,y,z):\t");
    print_xyz(xyz[X], xyz[Y], xyz[Z]);
    Serial.println("");
    Serial.print("Goal angles (deg):\t");
    print_xyz(theta1, theta2, theta3);
    Serial.println("");
    Serial.print("Sensor goals:\thip\t");
    Serial.print(sense_goals[HIP]);
    Serial.print("\tthigh\t");
    Serial.print(sense_goals[THIGH]);
    Serial.print("\tknee\t");
    Serial.print(sense_goals[KNEE]);
    Serial.println("");

    return;
}

/*
 * Forward kinematics.
 *
 * Calculate joint angles in both degrees and radians, based on the
 * last sensor readings.
 *
 * Takes sensor readings from sensor_readingss[] and calculates the
 * angles in degrees into current_deg[] and radians into
 * current_rad[].
 *
 * XXX fixme:  This should have the sensor readings passed in and return the angles.
 */
void calculate_angles(int *sensors, double *degrees)
{
    degrees[HIP]   = ((sensors[HIP] - SENSOR_LOW(HIP)) / UNITS_PER_DEG(HIP)) + ANGLE_LOW(HIP);
    degrees[THIGH] = ANGLE_HIGH(THIGH) - ((sensors[THIGH] - SENSOR_LOW(THIGH)) / UNITS_PER_DEG(THIGH));
    degrees[KNEE]  = ANGLE_HIGH(KNEE)  - ((sensors[KNEE]  - SENSOR_LOW(KNEE))  / UNITS_PER_DEG(KNEE));

    for (int i = 0;i < 3;i++)
        current_rad[i] = (degrees[i] * 71) / 4068;

    DEBUG("Angles:\t");
    for (int i = 0; i < 3; i++) {
        /* Read sensors even if we're not moving. */
        DEBUG("\t");
        DEBUG(joint_names[i]);
        DEBUG("\t");
        DEBUG(degrees[i]);
    }
    DEBUGLN("");

    return;
}

/*
 * Forward kinematics.
 *
 * Takes the current angles from rad[] and converts to the current
 * (x,y,z).
 *
 * XXX fixme:  This should take all values as arguments.
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
    DEBUGLN("");

    return;
}
