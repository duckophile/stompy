/* ; -*- mode: C ;-*- */



void set_joint_speed(uint32_t joint_num, uint32_t joint_speed)
{

    set_pwm_goal(joint_num, joint_speed);
}

/*
 * Set the PWMs based on the current position and the position goal.
 *
 * Scale x,y,z speed so they all arrive at their destination at the
 * same time.
 */
void set_pwms(int foot_speed, int xyz_scale[])
{
    int i;
    int joint_speed;
    int joint_num;

    if ((deadMan == 0) && (!deadman_forced))
        return;

    for (i = 0; i < 3; i++) {
        joint_speed = (foot_speed * 100) / xyz_scale[i];

        /* Select PWM based on joint number and direction. */
        joint_num = i;
        if (sensor_readings[i] < sensor_goals[i])
            joint_num += 3;
        set_joint_speed(joint_num, joint_speed);
    }

    return;
}

/*
 * Calculate the delta from the current position to the goal position,
 * and the scaling factor to scale the x,y,z speeds so they all arrive
 * at the goal at the same time.
 *
 * xuz_goal[] and current_xyz[] should be passed in.
 */
void calculate_deltas(float xyz_delta[], int xyz_scale[])
{
    int i;
    float largest_delta = 0;
    static int loop_count = 0;

    /* Calculate the deltas from where we are to where we want to be. */
    for (i = 0;i < 3; i++) {
        xyz_delta[i] = xyz_goal[i] - current_xyz[i];
        if (abs(xyz_delta[i]) > largest_delta)
            largest_delta = abs(xyz_delta[i]);
    }

    /* Calculate scaling factors to normalize x,y,z to the longest axis. */
    for (i = 0;i < 3;i++)
        xyz_scale[i] = (int)((abs(xyz_delta[i]) * 100) / largest_delta);

    if ((loop_count++ % 100) == 0) {
        Serial.print("XYZ deltas from current to goal: ");
        for (i = 0;i < 3; i++) {
            Serial.print(xyz_delta[i]);
            Serial.print("\t");
        }
        Serial.print("\n");

        Serial.print("XYZ scaling: ");
        for (i = 0;i < 3;i++) {
            Serial.print(xyz_scale[i]);
            Serial.print("\t");
        }
        Serial.print("\n");
    }

    return;
}

/*
 * The sensor readings are stored in sensor_readings[].
 *
 * The position we're trying to hit is xyz_goal[].
 *
 * The current xyz is in current_xyz[].
 */

/* This loop runs from the ISR, which is currently 100hz. */
void velocity_loop(void)
{
    float xyz_delta[3];
    int xyz_scale[3];
    int leg_speed = 100;

    return;

    check_deadman();

    /* Read the sensors. */
    read_sensors(sensor_readings);

    /* Turn sensor readings into joint angles. */
    calculate_angles(sensor_readings, current_deg);

    /* Turn joint angles into (x,y,z). */
    calculate_xyz();

    /*
     * Calculate the deltas from where we are to where we want to be,
     * as well as a scaling factor to normalize the speed across all
     * three joints.
     */
    calculate_deltas(xyz_delta, xyz_scale);

    /* Write the PWMs to make the leg move. */
    set_pwms(leg_speed, xyz_scale);

    return;
}
