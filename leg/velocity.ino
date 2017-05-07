/* ; -*- mode: C ;-*- */


/*
 * Calculate the delta from the current position to the goal position,
 * and the scaling factor to scale the x,y,z speeds so they all arrive
 * at the goal at the same time.
 */
void calculate_deltas(float xyz_delta[], int xyz_scale[])
{
    int i;
    float largest_delta = 0;

    /* Calculate the deltas from where we are to where we want to be. */
    for (i = 0;i < 3; i++) {
        xyz_delta[i] = xyz_goal[i] - current_xyz[i];
        if (abs(xyz_delta[i]) > largest_delta)
            largest_delta = abs(xyz_delta[i]);
    }

    /* Calculate scaling factors to normalize x,y,z to the longest axis. */
    for (i = 0;i < 3;i++)
        xyz_scale[i] = (int)((abs(xyz_delta[i]) / largest_delta) * 100);

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

    check_deadman();

    /* Read the sensors. */
    read_sensors(sensor_readings);

    /* Turn sensor readings into joint angles. */
    calculate_angles(sensor_readings, current_deg);

    /* Turn joint angles into (x,y,z). */
    calculate_xyz();

    /*
     * Calculate the deltas from where we are to where we want to be.
     */
    calculate_deltas(xyz_delta, xyz_scale);


    /* Write something to the PWMs to make the leg move. */
    write_pwms();

    return;
}
