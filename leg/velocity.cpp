/* ; -*- mode: C ;-*- */

#include "Arduino.h"
#include <stdint.h>
#include "pins.h"
#include "leg-info.h"
#include "pwm.h"
#include "globals.h"
#include "joystick.h"
#include "leg.h"
#include "kinematics.h"
#include "interrupt.h"
#include "velocity.h"
#include "comm.h"
#include "misc.h"

#define VELOCITY_HZ	100

#define VELOCITY_DEBUG if(velocity_debug)Serial.print

uint8_t point_list_buf[1024];

/*
 * TODO:
 *
 * I should have the joint speed normalized to some other scale.
 */

/*
 * Cylinders:
 *
 * Knee:  Dalton DBH-2512-WT 2.5" bore, 1.50" rod, 12" travel, 20" retracted length
 * Thigh: Dalton DBH-3514-WT 3.5" bore, 1.75" rod, 14" travel, 24" retracted length
 * Hip:   Dalton DBH-2008-WT 2.0" bore, 1.25" rod,  8" travel, 16" retracted length
 *
 * String pots:
 * Thigh, knee:  JX-PA-15-N12-14S-132
 * Hip:          JX-PA-10-N12-14S-132
 *
 * Right front real travel:
 * HIP:   7.081"
 * KNEE: 11.970"
 *
 # Sensor        Low     High    Travel
 # hip           5580    46517   40937 = 5781/inch
 # thigh (meas)  2006    38193   36187 = 2584/inch (wrong!)
 # thigh (theor) 2006    60750   58744 = 4196/inch
 # knee          3056    53285   50229 = 4196/inch
 *
 *
 * case HIP:
 * pwm_percent = (joint_speed + 290) / 7.2;	HIP OUT
 * case KNEE:
 * pwm_percent = (joint_speed + 210) / 7.1;	KNEE OUT.
 * case THIGH:
 * pwm_percent = (joint_speed + 105) / 4.0;	THIGH OUT.
 */

uint32_t cylinder_stroke[3] = {7, 14, 12}; /* Travel of each cylinder, in inches. */
/* Maybe these speeds should be stored in flash? */
uint32_t cylinder_speeds[6] = {0, 0, 0, 0, 0, 0}; /* Speed of each side of each cylinder, in inches/sec. */
uint32_t cylinder_inch_conv[3] = {0, 0, 0}; /* Multiplier to convert inches to sensor units. */

volatile int velocity_debug = 0;
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
 * XXX fixme:  The speed needs to be hooked up.
 */
int set_xyz_goal(double xyz[3], int speed)
{
    int i;
    double deg_goals[3];
    int sense_goals[NR_SENSORS];
    int old_int_state = -1;

    (void)speed;

    velocity_debug++;

    Serial.print("\n# ----------------------------------------------------------------\n");

    /*
     * inverse_kin() verifies the goals are in range and constrains
     * them if they're not.
     */
    if (inverse_kin(xyz, sense_goals, deg_goals))
        return -1;

    Serial.print("# New Goals: (x,y,z):\t");
    print_ftuple(xyz);
    Serial.print(" speed ");
    Serial.print(speed);
    Serial.print('\n');

    Serial.print("# Goal angles (deg):    ");
    for (i = 0;i < 3;i++) {
        Serial.print('\t');
        Serial.print(deg_goals[i]);
    }
    Serial.print('\n');

    Serial.print("# Sensor goals:          ");
    for (i = 0;i < 3;i++) {
        Serial.print('\t');
        Serial.print(sensor_goal[i]);
    }
    Serial.print('\n');

    /* Disable interrupts so the velocity loop won't run until this is done. */
    old_int_state = set_interrupt_state(0);
    for (i = 0;i < 3;i++) {
        xyz_goal[i] = xyz[i];
        angle_goals[i] = deg_goals[i];
        sensor_goal[i] = sense_goals[i];
    }
    set_interrupt_state(old_int_state);

    Serial.print("# ----------------------------------------------------------------\n\n");

    return 0;
}

/*
 * Takes a joint and a speed in sensor_units/sec and sets a PWM value
 * that should give that speed.
 *
 * XXX fixme: I think the table should contain inches/sec of foot
 * movement.
 */
static int set_joint_speed(uint32_t valve, uint32_t joint_speed)
{
    uint32_t n;
    uint32_t pwm_offset;
    uint32_t pwm_percent;
    float scale;
    float diff;
    float offset;

    for (n = 0;n <= 10;n++) {
        if ((leg_info.valves[valve].joint_speed[n    ] <= joint_speed) &&
            (leg_info.valves[valve].joint_speed[n + 1] >= joint_speed)) {
            break;
        }
    }

    if (n == 11) {
        Serial.print("ERROR: Valve ");
        Serial.print(valve);
        Serial.print(" cannot give speed ");
        Serial.print(joint_speed);
        Serial.print('\n');
        disable_leg();
        return -1;
    }

    /* The speed difference in this 10% window of PWM. */
    diff = leg_info.valves[valve].joint_speed[n + 1] -
           leg_info.valves[valve].joint_speed[n];
    /* Where our desired speed falls in that window. */
    offset = joint_speed - leg_info.valves[valve].joint_speed[n];
    scale = offset / diff;
    pwm_offset = (uint32_t)(10.0 * scale);
    pwm_percent = (n * 10) + pwm_offset;

#if 0
    Serial.print("# Setting PWM for Valve ");
    Serial.print(valve);
    Serial.print(" speed ");
    Serial.print(joint_speed);
    Serial.print(", which falls in window ");
    Serial.print(leg_info.valves[valve].joint_speed[n]);
    Serial.print("-");
    Serial.print(leg_info.valves[valve].joint_speed[n + 1]);
    Serial.print(", scale = ");
    Serial.print(scale);
    Serial.print(", final percent = ");
    Serial.print(pwm_percent);
    Serial.print(", diff = ");
    Serial.print(diff);
    Serial.print(", offset = ");
    Serial.print(offset);
    Serial.print("\n");
#endif

/*    set_scaled_pwm_goal(valve, pwm_percent); */
    set_pwm_goal(valve, pwm_percent);

    return 0;
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
static void set_velocity_pwms(int foot_speed, double speed_scale[], int directions[])
{
    int i;
    int sensor_speed;

    if ((deadMan == 0) && (!deadman_forced))
        return;

    if (velocity_debug) {
        Serial.print("# Setting PWMs based on desired foot speed of ");
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
        if (joint_speed != 0)
            joint_speed += LOW_PWM_MOVEMENT(valve);

        /*
         * joint_speed should always be greater than LOW_PWM_MOVEMENT,
         * UNLESS the desired joint speed is 0.
         */

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
        if ((joint_speed != 0) && (joint_speed < LOW_PWM_MOVEMENT(valve))) {
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

static double calculate_distance(double current[], double goal[])
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
 * XXX fixme:  I think this should take an input speed!
 */

static void calculate_speeds(double speed_scale[], int joint_direction[])
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
            Serial.print("# Sensor goal ");
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
#if 0
        if (abs(sensor_delta[i]) > largest_delta)
            largest_delta = abs(sensor_delta[i]);
#endif

        /* Convert each sensor reading into inches. */
        movement_inches[i] = fabs((float)sensor_delta[i] / (float)cylinder_inch_conv[i]);
#if 0
        Serial.print(i);
        Serial.print(' ');
        Serial.print(sensor_delta[i]);
        Serial.print(' ');
        Serial.print(cylinder_inch_conv[i]);
        Serial.print(" = ");
        Serial.print(movement_inches[i]);
        Serial.print('\n');
#endif
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
    for (i = 0;i < 3;i++) {
        speed_scale[i] = movement_inches[i] / largest_delta;
#if 0
        Serial.print(i);
        Serial.print(' ');
        Serial.print(movement_inches[i]);
        Serial.print(" / ");
        Serial.print(largest_delta);
        Serial.print(" = ");
        Serial.print(speed_scale[i]);
        Serial.print('\n');
#endif
    }

    if (velocity_debug) {
        Serial.print("# Largest delta = ");
        Serial.print(largest_delta);
        Serial.print('\n');
        Serial.print("# Sensor deltas from current to goal:\t");
        for (i = 0;i < 3; i++) {
            Serial.print(sensor_delta[i]);
            Serial.print("\t");
        }
        Serial.print("\n");
        Serial.print("# Scaling factor for each joint:      \t");
        for (i = 0;i < 3; i++) {
            Serial.print(speed_scale[i]);
            Serial.print('\t');
        }
        Serial.print('\n');
    }

    return;
}

#if 0
/* The joystick will move the leg at most 4 feet/sec. */
#define JOYSTICK_SPEED_FPS	4

/*
 * Move the XYZ goal based on the joystick position.
 */

static void do_joystick_xyz(void)
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
            Serial.print(movement);
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
#endif

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
    int leg_speed = 30; /* XXX fixme:  Hardwired value. */
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

    if (velocity_debug) {
        Serial.print('\n');
        Serial.print("\n# Sensors:\t");
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
        Serial.print("# Current degrees: ");
        print_ftuple(current_deg);
        Serial.print("\n");
    }

    /* Turn joint angles into current (x,y,z) in inches. */
    calculate_xyz(current_xyz, current_rad);

    if (velocity_debug) {
        Serial.print("# XYZ: \t");
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

redo:

    /*
     * The distance between the current XYZ and the goal XYZ will tell
     * if the foot is close enough, and can be used to decelerate as
     * the foot approaches the goal.
     */

    distance = calculate_distance(current_xyz, xyz_goal);

    /* Enable debugging output every time the foot moves an inch. */
    if ((last_distance - distance) > 1.0) {
#if 0
        velocity_debug++;
#endif
        last_distance = distance;
    }

    if (velocity_debug) {
        Serial.print("# 3D distance to goal: ");
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
    /* XXX fixme:  I should't calculate the speeds until I know I'll need them. */
    /* XXX fixme:  I need to take a desired speed input here! */
    calculate_speeds(speed_scale, joint_direction);

#if 0
    /*
     * If the foot is within 4 inches of its goal then scale the speed
     * down in proportion to the distance to the goal.  This is the
     * wrong place to be doing this, but hopefully it won't hurt.
     *
     * It should be possible to selectively enable deceleration, so
     * the leg doesn't decelerate at waypoints when it's following a
     * list of waypoints.
     */
    if (distance < 4.0) {
        double _speed_scale;

        /*
         * Decelerate by 25% for every inch under 4 as the goal nears.
         * The main purpose of this is to avoid oscillation when the
         * leg tries to stop.
         */
        _speed_scale = (distance * 25.0) / 100.0;
        leg_speed = (int)((double)leg_speed * _speed_scale);
        if (velocity_debug) {
            Serial.print("# Close to goal, so decelerating - scaling speed by ");
            Serial.print(_speed_scale);
            Serial.print("%, leg_speed is now ");
            Serial.print(leg_speed);
            Serial.print("\n");
        }
    }
#endif

    /* Write the PWMs to make the leg move. */

    if (distance < 0.75) {
        /*
         * If there is a list of waypoints queued then go to the next
         * one and goto redo:.
         */
        if (queued_point_count()) {
            double next_xyz[3];
            int speed;

            Serial.print("\n# Leg is close enough to goal and points are queued: setting next goal.\n");
            get_next_point(next_xyz, &speed, 0);
            if (set_xyz_goal(next_xyz, speed)) {
                cancel_point_list();
                reset_current_location();
            } else
                goto redo;
        }

        if (!pwms_turned_off) {
            Serial.print("\n# Leg is close enough to goal, so turning PWMs off.\n");
            pwms_off();
            pwms_turned_off = 1;
            Serial.print("OK: Leg reached goal.\n");
        }
    } else {
        /*
         * If the PWMs are turned off then don't turn them back on
         * unless the foot has strayed 1.0 inches.
         */
        if ((pwms_turned_off && distance > 1.0) ||
            (!pwms_turned_off)) {
            if (pwms_turned_off) {
                Serial.print("\n# Distance = ");
                Serial.print(distance);
                Serial.print(", so turning PWMs back on.\n");
                pwms_turned_off = 0;
            }
            set_velocity_pwms(leg_speed, speed_scale, joint_direction);
        }
    }

    debug_count++;
    if (velocity_debug > 0)
        velocity_debug--;

    return;
}
