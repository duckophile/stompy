/* ; -*- mode: C ;-*- */

#define VELOCITY_HZ	100

#define VELOCITY_DEBUG if(velocity_debug)Serial.print

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
 * Cylinders:
 *
 * Knee: Dalton DBH-2512-WT 2.5" bore, 1.5" rod, 12" travel, 20" retracted length
 *
 * Thigh: Dalton DBH-3514-WT 3.5" bore, 1.75" rod, 14" travel, 24" retracted length
 *
 * Hip: Dalton DBH-2008-WT 2" bore, 1.25" rod, 8" travel, 16" retracted length
 */

#define HIP_CYL_STROKE		8
#define THIGH_CYL_STROKE	14
#define KNEE_CYL_STROKE		12

uint32_t cylinder_stroke[3] = {8, 14, 12}; /* Travel of each cylinder, in inches. */
/* Maybe these speeds should be stored in flash? */
uint32_t cylinder_speeds[6] = {0, 0, 0, 0, 0, 0}; /* Speed of each side of each cylinder, in inches/sec. */
uint32_t cylinder_inch_conv[3] = {0, 0, 0}; /* Multiplier to convert inches to sensor units. */

int do_velocity_debug = 0;

void toggle_velocity_debug(void)
{
    do_velocity_debug = !do_velocity_debug;

    return;
}

/*
 * Boot-time setup for the velocity subsystem.
 */
int velocity_init(void)
{
    int rc = 0;
    int joint;
    int dir;
    int sensor_travel;

    for (joint = 0;joint < 3;joint++) {
        if ((SENSOR_LOW(joint) == 0xFFFF) || (SENSOR_HIGH(joint) == 0xFFFF) ||
            (JOINT_SPEED(joint + IN, 10) == 0xFFFF) || (JOINT_SPEED(joint + OUT, 10) == 0xFFFF)) {
            Serial.print("\nERROR: Calibration parameters for ");
            Serial.print(joint_names[joint]);
            Serial.print(" are not set properly!\n\n");
            rc = -1;
            continue;
        }

        sensor_travel = SENSOR_HIGH(joint) - SENSOR_LOW(joint);
        cylinder_inch_conv[joint] = sensor_travel / cylinder_stroke[joint];
        Serial.print("Sensor travel for ");
        Serial.print(joint_names[joint]);
        Serial.print(" = ");
        Serial.print(sensor_travel);
        Serial.print("\n");

        for (dir = 0;dir < 6;dir += 3) {
            /*
             * Speed (sensor_units/sec) at 100% PWM divided by stroke
             * gives speed in inches/sec.
             */
            cylinder_speeds[joint + dir] = JOINT_SPEED(joint + dir, 10) / cylinder_stroke[joint];
            Serial.print("Speed in inches/sec for ");
            Serial.print(joint_names[joint]);
            Serial.print(" ");
            Serial.print(direction_names[dir]);
            Serial.print(" = ");
            Serial.print(cylinder_speeds[joint + dir]);
            Serial.print("\n");

        }
    }

    return rc;
}

/*
 * Takes a joint and a speed in sensor_units/sec and sets a PWM value
 * that should give that speed.
 */
void set_joint_speed(uint32_t joint_num, uint32_t joint_speed)
{
    int pwm_percent;

    /*
     * XXX FIXME: The speed calculation should be different for each
     * joint and direction.
     */
#warning Need different speeds for each joint!

    pwm_percent = ((double)joint_speed + 328.0) / 8.3;

    if (velocity_debug) {
        Serial.print(" input speed (sensor/sec): ");
        Serial.print(joint_speed);
        Serial.print(", pwm_percent: ");
        Serial.print(pwm_percent);
        Serial.print('\n');
    }

#warning Hardwired 50% PWM!
    pwm_percent = 50;

    set_pwm_goal(joint_num, pwm_percent);

    return;
}

/*
 * Set the PWMs based on the current position and the position goal.
 *
 * Scale x,y,z speed so they all arrive at their destination at the
 * same time.
 *
 * foot_speed is the desired foot speed percentage, 0-100.
 *
 * speed_scale[] holds scaling factors (0-1) to scale the foot_speed to
 * the desired speed for each axis.  This allows the speeds for thre
 * three joints to be adjusted so all three joints arrive at the goal
 * at the same time.
 *
 * directions[] indicates if each joint is moving IN or OUT.
 *
 */
void set_velocity_pwms(int foot_speed, double speed_scale[], int directions[])
{
    int i;
    int sensor_speed;

    if ((deadMan == 0) && (!deadman_forced))
        return;

    if (velocity_debug) {
        Serial.print("Setting PWMs based on desired foot speed of ");
        Serial.print(foot_speed);
        Serial.print('\n');
    }

    /*
     * Iterate through the three joints.
     */
    for (i = 0; i < 3; i++) {
        int joint_speed;
        float pwm_range_scale;
        int valve;

        valve = i + directions[i];

        /*
         * Scale the requested foot speed for each of the three joints.
         */
        joint_speed = (int)((float)foot_speed * speed_scale[i]);

        if (velocity_debug) {
            Serial.print("# Joint ");
            Serial.print(i);
            Serial.print(" valve ");
            Serial.print(valve);
            Serial.print(" low_pwm ");
            Serial.print(LOW_PWM_MOVEMENT(valve));
            Serial.print(" speed_scale ");
            Serial.print(speed_scale[i]);

            Serial.print(" unscaled joint speed: ");
            Serial.print(joint_speed);
            Serial.print("%");
        }
        /* joint_speed is now the foot_speed scaled down by xyz_scale[joint]. */

        /*
         * pwm_range_scale is a scaling factor to scale to the useful
         * PWM range of this valve - only the range between
         * LOW_PWM_MOVEMENT and 100% is useful.
         *
         * This gives a scaling factor, 0-1.
         */
        pwm_range_scale = (100 - LOW_PWM_MOVEMENT(valve)) / 100.0;

        /* Scale the desired speed into the range LOW_PWM_MOVEMENT...100. */
        joint_speed = (int)((float)joint_speed * pwm_range_scale);
        joint_speed += LOW_PWM_MOVEMENT(valve);

        /* joint_speed should always be greater than LOW_PWM_MOVEMENT. */

        if (velocity_debug) {
            Serial.print(" pwm_range_scale ");
            Serial.print(pwm_range_scale);
            Serial.print(" direction ");
            Serial.print(directions[i]);
            Serial.print(" joint speed ");
            Serial.print(joint_speed);

/*            Serial.print("set_pwm(): Joint %d, xyz_scale %d, direction %d, joint_speed %d\n",
                   i, xyz_scale[i], directions[i], joint_speed); */
            Serial.print('\n');
        }

        if (joint_speed > 100) {
            Serial.print("\nERROR: Valve ");
            Serial.print(valve);
            Serial.print(" joint_speed (");
            Serial.print(joint_speed);
            Serial.print(") > 100!\n");
            joint_speed = 100;
        }
        if (joint_speed < LOW_PWM_MOVEMENT(valve)) {
            Serial.print("\nERROR: Valve ");
            Serial.print(valve);
            Serial.print(" joint_speed (");
            Serial.print(joint_speed);
            Serial.print(") is lower than lowest movement value (");
            Serial.print(LOW_PWM_MOVEMENT(valve));
            Serial.print(")\n");
            joint_speed = LOW_PWM_MOVEMENT(valve);
        }

        /* joint_speed is a percentage, 0-100. */
        sensor_speed = (JOINT_MAX_SPEED(valve) * joint_speed) / 100;

        if (velocity_debug) {
            Serial.print("# Joint ");
            Serial.print(i);
            Serial.print(" sensor units/sec = ");
            Serial.print(sensor_speed);
            Serial.print("\n");
        }

        /* Select PWM based on joint number and direction. */
        set_joint_speed(valve, sensor_speed);
    }

    return;
}

#if 0
void XXX_set_pwms(int foot_speed, double xyz_scale[], int directions[])
{
    int i;
    int joint_speed;

    if ((deadMan == 0) && (!deadman_forced))
        return;

    if (velocity_debug) {
        Serial.print("Setting PWMs based on desired foot speed of ");
        Serial.print(foot_speed);
        Serial.print('\n');
    }

    for (i = 0; i < 3; i++) {
        float pwm_range_scale;
        int valve;

        valve = i + directions[i];

        joint_speed = (int)((float)foot_speed * xyz_scale[i]);

        if (velocity_debug) {
            Serial.print("Unscaled joint speed: ");
            Serial.print(joint_speed);
        }
        /* joint_speed is now the foot_speed scaled down by xyz_scale[joint]. */

        /*
         * pwm_range_scale is a scaling factor to scale to the useful
         * PWM range of this valve - only the range between
         * LOW_PWM_MOVEMENT and 100% is useful.
         */
        pwm_range_scale = (100 - LOW_PWM_MOVEMENT(valve)) / 100.0;

        /* Scale the desired speed into the range LOW_PWM_MOVEMENT...100. */
        joint_speed = (int)((float)joint_speed * pwm_range_scale);
        joint_speed += LOW_PWM_MOVEMENT(valve);

        /* joint_speed should always be greater than LOW_PWM_MOVEMENT. */

        if (velocity_debug) {
            Serial.print(" joint ");
            Serial.print(i);
            Serial.print(" valve ");
            Serial.print(valve);
            Serial.print(" low_pwm ");
            Serial.print(LOW_PWM_MOVEMENT(valve));
            Serial.print(" xyz_scale ");
            Serial.print(xyz_scale[i]);
            Serial.print(" pwm_range_scale ");
            Serial.print(pwm_range_scale);
            Serial.print(" direction ");
            Serial.print(directions[i]);
            Serial.print(" joint speed ");
            Serial.print(joint_speed);

/*            Serial.print("set_pwm(): Joint %d, xyz_scale %d, direction %d, joint_speed %d\n",
                   i, xyz_scale[i], directions[i], joint_speed); */
        }

        if (joint_speed > 100) {
            Serial.print("ERROR: Valve ");
            Serial.print(valve);
            Serial.print(" joint_speed (");
            Serial.print(joint_speed);
            Serial.print(") > 100!\n");
            joint_speed = 100;
        }
        if (joint_speed < LOW_PWM_MOVEMENT(valve)) {
            Serial.print("ERROR: Valve ");
            Serial.print(valve);
            Serial.print(" joint_speed (");
            Serial.print(joint_speed);
            Serial.print(") is lower than lowest movement value (");
            Serial.print(LOW_PWM_MOVEMENT(valve));
            Serial.print(")\n");
            joint_speed = LOW_PWM_MOVEMENT(valve);
        }

        /* Select PWM based on joint number and direction. */
        set_joint_speed(valve, joint_speed); /* XXX NO!  This function takes sensor_units/sec! */
    }

    return;
}
#endif

double calculate_distance(double current[], double goal[])
{
    int i;
    double tally = 0;

    /* sqrt( (x2 - x1)^2 + (y2 - y1)^2 + (z2 - z1)^2 ) */

    for (i = 0;i < 3;i++)
        tally += pow(goal[i] - current[i], 2);

    return sqrt(tally);
}

/*
 * Calculate the scaling factor to scale the three joint speeds so
 * all three joints arrive at the goal at the same time.
 *
 * xuz_goal[] and current_xyz[] should be passed in.
 *
 * speed_scale[] is populated with a percentage to scale the speed for
 * each of the three joints to allow them to arrive at the goal at the
 * same time.
 *
 * joint_direction[] is populated with a direction indicator describing
 * the direction in which the leg is moving.
 *
 * XXX fixme: The direction might be backwards.
 */

/*
 * I know the number of sensor units each joint needs to move.
 *
 * - For each of the three joints, subtract the goal sensor units from
 *   the current to get the distance the joint needs to move.
 *
 * - Convert the three joint sensor_unit distances to inches of sensor
 *   travel.
 *
 * - Determine the direction of travel.
 *
 * - Normalize the three speeds to match the slowest joint.
 *
 * - I need to get to PWM values.
 *
 * - The desired speeds can be turned directly into PWM values.
 *
 */

void calculate_speeds(double speed_scale[], int joint_direction[])
{
    int i;
    double largest_delta = 0;
    float movement_inches[3];
    uint32_t sensor_delta[3];

    /*
     * Calculate the sensor deltas from where we are to where we want
     * to be, and the largest delta.
     */
    for (i = 0;i < 3; i++) {
        /*
         * Sensor value gets lower when string gets shorter, which is
         * when the cylinder comes in.
         */
        if (sensor_goal[i] > current_sensor[i])
            joint_direction[i] = OUT;
        else
            joint_direction[i] = IN; /* XXX I think the direction is correct. */

        if (velocity_debug) {
            Serial.print("Sensor goal ");
            Serial.print(sensor_goal[i]);
            Serial.print("\tcompared to current\t");
            Serial.print(current_sensor[i]);
            Serial.print("\tso joint\t");
            Serial.print(i);
            Serial.print("\tgoes\t");
            Serial.print(joint_direction[i] == IN ? "IN" : "OUT");
            Serial.print('\n');
        }

        /* The distance the sensor needs to move. */
        sensor_delta[i] = abs(sensor_goal[i] - current_sensor[i]);
        /* XXX Does this make sense when the deltas describe both directions? */
        if (abs(sensor_delta[i]) > largest_delta)
            largest_delta = abs(sensor_delta[i]);

        /* Convert each sensor reading into inches. */
        movement_inches[i] = fabs((float)sensor_delta[i] / (float)cylinder_inch_conv[i]);

        /* Make a note of the longest movement. */
        if (movement_inches[i] > largest_delta)
            largest_delta = movement_inches[i];
    }

    /*
     * Calculate a scaling factor to normalize the three distances to
     * the longest movement.  This is used to generate a very
     * imperfect speed so all three joints will arrive at the goal at
     * the same time.
     *
     * XXX fixme: This is just normalizing for distance.  It also
     * needs to take the max speed in each direction into account so
     * that no joint moves so fast that the other joints can't keep
     * up.
     */
    for (i = 0;i < 3;i++)
        speed_scale[i] = movement_inches[i] / largest_delta;

    if (velocity_debug) {
        Serial.print("Sensor deltas from current to goal:\t");
        for (i = 0;i < 3; i++) {
            Serial.print(sensor_delta[i]);
            Serial.print("\t");
        }
        Serial.print("\n");
        Serial.print("Scaling factor for each joint:      \t");
        for (i = 0;i < 3; i++) {
            Serial.print(speed_scale[i]);
            Serial.print('\t');
        }
        Serial.print('\n');
    }

    return;
}

#if 0
void XXX_calculate_deltas(double xyz_delta[], double xyz_scale[], int xyz_direction[])
{
    int i;
    double largest_delta = 0;
/*    static int loop_count = 0;*/

    /* Calculate the deltas from where we are to where we want to be. */
    for (i = 0;i < 3; i++) {
        /* XXX FIXME:  I shouldn't move the joint if it's really close. */
        if (xyz_goal[i] > current_xyz[i])
            xyz_direction[i] = OUT;
        else
            xyz_direction[i] = IN; /* XXX fixme:  Is the direction correct? */

        if (velocity_debug) {
            Serial.print("Goal ");
            Serial.print(xyz_goal[i]);
            Serial.print(" compared to current ");
            Serial.print(current_xyz[i]);
            Serial.print(" so joint ");
            Serial.print(i);
            Serial.print(" goes ");
            Serial.print(xyz_direction[i] == IN ? "IN" : "OUT");
            Serial.print('\n');
        }

        xyz_delta[i] = abs(xyz_goal[i] - current_xyz[i]);
        /* XXX Does this make sense when the deltas describe both directions? */
        if (fabs(xyz_delta[i]) > largest_delta)
            largest_delta = fabs(xyz_delta[i]);
    }

    /*
     * Calculate scaling factors to normalize the x,y,z speeds to the
     * longest axis.
     *
     * This gives percentage scaling factors to scale a speed
     * appropriate for the longest axis to the two shorter axes.  For
     * example, if the (x,y,z) travel is (9,6,3) inches then the
     * scaling factors will be (1.0, 0.66, 0.33).
     *
     */
    for (i = 0;i < 3;i++) {
        if (largest_delta)
            xyz_scale[i] = fabs(xyz_delta[i]) / largest_delta;
        else
            xyz_scale[i] = 0;
    }

    if (velocity_debug) {
        Serial.print("XYZ deltas from current to goal: ");
        for (i = 0;i < 3; i++) {
            Serial.print(xyz_delta[i]);
            Serial.print('\t');
        }
        Serial.print('\n');

        Serial.print("XYZ scaling: ");
        for (i = 0;i < 3;i++) {
            Serial.print(xyz_scale[i]);
            Serial.print('\t');
        }
        Serial.print('\n');
    }

    return;
}
#endif

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
        if (abs(joystick_values[i] - JOYSTICK_MID) < 20)
            joystick_values[i] = JOYSTICK_MID;
        if (velocity_debug) {
            Serial.print("Joystick ");
            Serial.print(i);
            Serial.print(' ');
            Serial.print(joystick_values[i]);
        }
    }
    if (velocity_debug)
        Serial.print('\n');

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

        if (velocity_debug)
            Serial.print("axis ");
        Serial.print(i);
        Serial.print(' ');
        Serial.print(joystick_values[i]);

        /* Turn joystick value into FPS. */
        movement = ((float)val * JOYSTICK_SPEED_FPS) / JOYSTICK_MID;
        if (velocity_debug) {
            Serial.print("FPS = ");
            Serial.print( movement);
        }

        /* Turns FPS into feet per HZ */
        movement /= VELOCITY_HZ;

        if (velocity_debug) {
            Serial.print("feet/hz = ");
            Serial.print(movement);
            Serial.print('\n');
        }

        /* Turn speed into speed + direction. */
        movement *= direction;

        if (velocity_debug) {
            Serial.print("movement + direction = ");
            Serial.print(movement);
            Serial.print(". old xyz_goal = ");
            Serial.print(xyz_goal[i]);
            Serial.print('\n');
        }

        xyz_goal[i] += movement;
        Serial.print("new xyz_goal = ");
        Serial.print(xyz_goal[i]);
        Serial.print('\n');
    }

    return;
}

/*
 * The general idea for positon is:

- I start out with the target position in (x,y,z) inches, sensor
  units, and joint angles.  These are all updated every time the goal
  is cnanged.

- Read sensors to get current position in sensor units.

- Turn the sensor readings into current angles.

- Turn the angles into the current (x,y,z) in inches.

- Calculate distance from current (x,y,z) to goal (x,y,z), in inches.
  This is used to determine when the goal has been reached, as well as
  deciding when to decelerate.

- Have max speed in inches/sec for each joint.

<todo>

- Calculate three joint speeds that will make all three joints arrive
  at the goal at the same time.  Sensor_units/sec aren't useful to
  here since they don't describe a meaningful speed, they're just
  for translation to PWM values.

- Convert the three distances to inches/sec?  If I know current and
  goal sensor units, and current and goal (x,y,z), then I can convert
  between sensor units and inches, which is as good as speed.

- I need the slowest of the three joints, in inches/sec, to match the
  speed of the other two axes to the slowest joint.

- I need the speed in sensor_units/sec in order to determine a PWM
  value, but I need to match the speed of two joints to the slowst of
  the three so they can keep up with each other.  That speed
  calculation has to be done in inches/sec, since sensor_units/sec
  isn't really a speed.

  This isn't really the correct behavior - I think I actually want the
  speed of the slowest joint in the context of the movement I need,
  but that's too complicated right now.

- Maybe max speed in inches/sec should be stored in the leg info?
  The travel of each cylinder is known very closely,

- Calculate three distances in sensor units - one for each axis?





</todo>

- Turn joint speeds into PWM values.

- Write values to PWMs.

 */


/*
 * The sensor readings are stored in current_sensor[].
 *
 * The position we're trying to hit is xyz_goal[].
 *
 * The current xyz is in current_xyz[].
 */

/* This loop runs from the ISR, which is currently 100hz. */
void velocity_loop(void)
{
    double distance;
    double last_distance = 0.0;
    double speed_scale[3];
    int joint_direction[3];
    int leg_speed = 70; /* XXX fixme:  Hardwired value. */
    static int debug_count = 0;
    static int pwms_turned_off = 0;

    if (do_velocity_debug) {
        if ((debug_count % 100) == 0) {
            velocity_debug++;
            func_info();
        }
    }

    check_deadman();

    /* Read the sensors. */
    read_sensors(current_sensor);

#define DEBUG_VELOCITY	0

    if (velocity_debug) {
        Serial.print('\n');
        Serial.print("\nSensors:\t");
        Serial.print(current_sensor[0]);
        Serial.print("\t");
        Serial.print(current_sensor[1]);
        Serial.print("\t");
        Serial.print(current_sensor[2]);
        Serial.print("\n");
    }

    /* Turn sensor readings into current joint angles. */
    calculate_angles(current_sensor, current_deg, current_rad);

    if (velocity_debug) {
        Serial.print("Current degrees: ");
        print_ftuple(current_deg);
        Serial.print("\n");
    }

    /* Turn joint angles into current (x,y,z) in inches. */
    calculate_xyz(current_xyz, current_rad);

    if (velocity_debug) {
        Serial.print("XYZ: \t");
        print_ftuple(current_xyz);
        Serial.print("\n");
    }

    /*
     * If the joystick is enabled then read it and modify the goal
     * position based on the joystick position.
     */
#if 0
    if (joystick_mode == JOYSTICK_POSITION)
        do_joystick_xyz();
#endif
    /*
     * The distance between the current XYZ and the goal XYZ will tell
     * if the foot is close enough, and can be used to decelerate as
     * the foot approaches the goal.
     */

    distance = calculate_distance(current_xyz, xyz_goal);

    /* Enable debugging output every time the foot moves an inch. */
    if ((last_distance - distance) > 1.0) {
        velocity_debug++;
        last_distance = distance;
    }

    if (velocity_debug) {
        Serial.print("3D distance to goal: ");
        Serial.print(distance);
        Serial.print('\n');
    }

    /*
     * Calculate the three joint deltas from where we are to where we
     * want to be, as well as a direction and a scaling factor to
     * normalize the speed across all three joints.
     *
     * The deltas should be used to calculate speed and to determine
     * whether the foot is close enough to the goal.
     *
     * Where should acceleration be done?  I suspect it has to be done
     * on a per-joint basis instead of accelerating all three joints
     * together, to allow acceleration when changing course when the
     * leg's already in motion.  Maybe that's the PIDs job.
     */
    calculate_speeds(speed_scale, joint_direction);

    /*
     * If the foot is within 4 inches of its goal then scale the speed
     * down in proportion to the distance to the goal.  This is the
     * wrong place to be doing this, but hopefully it won't hurt.
     *
     * It should be possible to selectively enable deceleration, so
     * the leg doesn't decelerate at waypoints when it's following a
     * list of waypoints.
     */
#if 0 /* Don't confuse things for now. */
    if (distance < 4) {
        double speed_scale;

        /* distance = 4-0 */
        _speed_scale = (distance * 25.0) / 100.0;
        leg_speed = (int)((double)leg_speed * _speed_scale);
        if (velocity_debug) {
            Serial.print("Close to goal, so decelerating - scaling speed by ");
            Serial.print(_speed_scale);
            Serial.print("%, leg_speed is now ");
            Serial.print(leg_speed);
            Serial.print("\n");
        }
    }
#endif

    /* Write the PWMs to make the leg move. */
    if (distance < 1.0) /* We're there! */ {
        if (!pwms_turned_off) {
            Serial.print("# Leg is close enough to goal, so turning PWMs off.\n");
            pwms_off();
            pwms_turned_off = 1;
        }
    } else {
        set_velocity_pwms(leg_speed, speed_scale, joint_direction);
        pwms_turned_off = 0;
    }

    debug_count++;
    if (velocity_debug > 0)
        velocity_debug--;

    return;
}
