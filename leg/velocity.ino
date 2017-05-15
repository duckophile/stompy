/* ; -*- mode: C ;-*- */

#define VELOCITY_HZ	100

/*
 * TODO:
 *
 * I should have the joint speed normalized to some other scale.
 *
 * The joint speed for hip out in sensor_units/sec is calculated with
 * speed = (8.3 * pwm) - 328
 *
 * pwm = (speed + 328) / 8.3
 */

/*
 * Takes a joint and a speed in sensor_units/sec and sets a PWM value
 * that should give that speed.
 */
void set_joint_speed(uint32_t joint_num, uint32_t joint_speed)
{
    int pwm_val;

    /*
     * XXX FIXME: The speed calculation should be different for each
     * joint and direction.
     */
    pwm_val = ((double)joint_speed + 328.0) / 8.3;

    set_pwm_goal(joint_num, pwm_val);
}

/*
 * Set the PWMs based on the current position and the position goal.
 *
 * Scale x,y,z speed so they all arrive at their destination at the
 * same time.
 *
 * foot_speed gives the desired foot speed, 0-100.
 *
 * xyz_scale[] holds scaling factors to scale the foot_speed to the
 * desired speed for each axis.
 *
 * directions[] indicates if each joint is moving IN or OUT.
 */
void set_pwms(int foot_speed, int xyz_scale[], int directions[])
{
    int i;
    int joint_speed;

    if ((deadMan == 0) && (!deadman_forced))
        return;

    for (i = 0; i < 3; i++) {
        joint_speed = (foot_speed * 100) / xyz_scale[i];

        /* Select PWM based on joint number and direction. */
        set_joint_speed(i + directions[i], joint_speed);
    }

    return;
}

/*
 * Calculate the delta from the current position (current_xyz[]) to
 * the goal position (xyz_goal[]), and the scaling factor to scale the
 * x,y,z speeds so they all arrive at the goal at the same time.
 *
 * xuz_goal[] and current_xyz[] should be passed in.
 *
 * xyz_delta[] is populated with the delta between the current
 * position and the current goal.
 *
 * xyz_scale[] is populated with a percentage to scale the speed in
 * three axes to allow them to arrive at the goal at the same time.
 *
 * xyz_direction[] is populated with a direction indicator describing
 * the direction in which the leg is moving.
 *
 * XXX fixme: The direction might be backwards.
 */
void calculate_deltas(double xyz_delta[], int xyz_scale[], int xyz_direction[])
{
    int i;
    double largest_delta = 0;
    static int loop_count = 0;

    /* Calculate the deltas from where we are to where we want to be. */
    for (i = 0;i < 3; i++) {
        if (xyz_goal[i] > current_xyz[i])
            xyz_direction[i] = IN; /* XXX fixme:  Is the direction correct? */
        else
            xyz_direction[i] = OUT;

        xyz_delta[i] = abs(xyz_goal[i] - current_xyz[i]);
        /* XXX Does this make sense when the deltas describe both directions? */
        if (xyz_delta[i] > largest_delta)
            largest_delta = xyz_delta[i];
    }

    /*
     * Calculate scaling factors to normalize the x,y,z speeds to the
     * longest axis.
     *
     * This gives percentage scaling factors to scale a speed
     * appropriate for the longest axis to the two shorter axes.  For
     * example, if the (x,y,z) travel is (9,6,3) inches then the
     * scaling factors will be (100, 66, 33).
     */
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

/* The joystick will move the leg at most 4 feet/sec. */
#define JOYSTICK_SPEED_FPS	4

/*
 * Move the XYZ goal based on the joystick position.
 */

void do_joystick_xyz(void)
{
    int i;
    int val;
    int direction;
    float movement;

    /* Read the three joystick sensors. */
    for (i = 0;i < 3;i++) {
        joystick_values[i] = analogRead(joystick_pins[i]);
        /* Avoid drift due to slight joystick miscalibration. */
        if (abs(joystick_values[i] - JOYSTICK_MID) < 10)
            joystick_values[i] = JOYSTICK_MID;
    }

    /* Move the (X,Y,Z) goal based on the joystick position. */
    for (i = 0;i < 3;i++) {

        if (joystick_values[i] == JOYSTICK_MID)
            continue;	/* Nothing to do here. */

        if (joystick_values[i] > JOYSTICK_MID) {
            direction = 1;
            val = joystick_values[i] - JOYSTICK_MID;
        } else {
            direction = -1;
            val = JOYSTICK_MID - joystick_values[i];
        }

        /* Turn joystick value into FPS. */
        movement = ((float)val * JOYSTICK_SPEED_FPS) / JOYSTICK_MID;
        /* Turns FPS into feet per HZ */
        movement /= VELOCITY_HZ;
        /* Turn speed into speed + direction. */
        movement *= direction;

        xyz_goal[i] += movement;
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
    double xyz_delta[3];
    int xyz_scale[3];
    int xyz_direction[3];
    int leg_speed = 100;

    check_deadman();

    /* Read the sensors. */
    read_sensors(sensor_readings);

    /* Turn sensor readings into joint angles. */
    calculate_angles(sensor_readings, current_deg);

    /* Turn joint angles into (x,y,z). */
    calculate_xyz(current_xyz, current_rad);

    /*
     * If the joystick is enabled then read it and modify the goal
     * position based on the joystick position.
     */
    if (joystick_mode == JOYSTICK_POSITION)
        do_joystick_xyz();

    /*
     * Calculate the deltas from where we are to where we want to be,
     * as well as a scaling factor to normalize the speed across all
     * three joints.
     */
    calculate_deltas(xyz_delta, xyz_scale, xyz_direction);

    /* XXX fixme:  I need to use the xyz_direction. */

    /* Write the PWMs to make the leg move. */
    set_pwms(leg_speed, xyz_scale, xyz_direction);

    return;
}
