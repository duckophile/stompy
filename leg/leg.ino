/* ; -*- mode: C ;-*- */

#include <stdint.h>
#include <Arduino.h>
#include <EEPROM.h>
#include "pins.h"
#include "leg-info.h"
#include "globals.h"
#include "misc.h"
#include "pid.h"
#include "velocity.h"
#include "pwm.h"
#include "calibrate.h"
#include "leg.h"
#include "kinematics.h"
#include "interrupt.h"
#include "joystick.h"
#include "cmd.h"

/*
 * Things I want to store in flash:
 *
 * Sensor min and max for three sensors.
 *
 * PWM scaling factor to scale speed to PWM.
 *
 * Angle min and max (could be slightly different per leg?).
 *
 * Foot x,y,z min and max.
 *
 * Which joint I am.
 *
 * Sensor jitter?
 *
 * Firmware rev.
 */

/*
 * TODO:
 *
 * I should verify the forward and inverse kinematics every boot.
 */

/*
 * XXX fixme:  info command should print out more stuff - goals, sensor goals, etc.
 *
 * Joystick mode should move the goal, not actuate the PWMs.
 *
 * The command interpreter should be able to use a regular serial terminal.
 */

/*  Inverse kinematics for stompy
 *   theta1 = arctan(y/x)
 *   theta2 = arctan(z/x1) + arccos([L2^2 + r^2 - L3^2] / [2*L2*r])
 *        where: x1 = [y/sin(theta1)] - L1
 *               r = z/[sin(beta)]
 *               beta = arctan(z/x1)
 *               alpha = arccos[(L2^2 + r^2 - L3^2) / (2*L2*r)]
 *   theta3 = arccos[(L3^2 + L2^2  - r^2) / [2*L3*L2]
 *
 *   There are three cases that must be caught for this math to work.
 *   1) If theta1 (hip angle) equals zero then x1 = (x - L1).
 *   2) If z equals zero then r = x1.
 *   3) If x = 0 then the hip angle is degenerate so I add .0001" to x when x = 0 to avoid these degenerate cases.
 */

/*
 * to convert deg to rad and rad to deg
 *  radians = (degrees * 71) / 4068
 *  degrees = (radians * 4068) / 71
 */

volatile int leg_enabled = 0;

volatile int debug_flag = 0;  /* Enable verbose output. */
volatile int old_debug_flag = 0;
volatile int periodic_debug_flag = 0;

#define ENABLED  1
#define DISABLED 0

const char *direction_names[]    = {"OUT",     "ERROR1", "ERROR2",   "IN",     "ERROR3"};
const char *joint_names[]        = {"hip",     "thigh",  "knee",     "calf"};
const char *joint_up_actions[]   = {"back",    "up",    "out"};
const char *joint_down_actions[] = {"forward", "down",  "in"};

/*
 * XXX fixme: Put all the kinematics-related values in one
 * container?
 * And standardize sensor_goal vs. current_sensor etc.
 * and *_goal vs. *_goals
 */

int sensor_goal[NR_SENSORS];
int current_sensor[NR_SENSORS];

double xyz_goal[3];
double angle_goals[3];

/*
 * Angles in degrees -
 * Hip is zero when straight out from the robot body.
 * Hip angle is zero when the thigh is parallel to the ground
 * Knee angle is between thigh and calf (13 degrees fully retracted)
 */
double current_deg[3];
double current_rad[3];

int sensorPin[NR_SENSORS] = {HIP_SENSOR_PIN, THIGH_SENSOR_PIN, KNEE_SENSOR_PIN, CALF_SENSOR_PIN};
// this is the distance in sensor reading that is close enough for directed movement
// I am putting this here so we can avoid chasing our tails early in positional control
int closeEnough = ((1 << PWM_BITS) / 512);

//Deadman button -- with the joystick I'll be using a momentary switch
//on the panel.  It will need to be held down or jumpered to set the
//joystick as "hot".

int deadMan = 0; //JOYSTICK is OFF if pin is low (pull high to enable joystick).
int deadman_forced = 0; /* Ignore the deadman if this is set. */

// kinematics block
/*  Forward kinematics for stompy
    x = cos(theta1) * [L1 + L2*cos(theta2) + L3*cos(theta2 + theta3 -180deg)]
    y = x * tan(theta1)
    z = [L2 * sin(theta2)] + [L3 * sin(theta2 + theta3 - 180deg)]
*/

double current_xyz[3];

leg_info_t leg_info;

/* Track the highest and lowest sensor values seen. */
int max_sensor_seen[NR_SENSORS];
int min_sensor_seen[NR_SENSORS] = {1 << 30, 1 << 30, 1 << 30, 1 << 30};

/*
 * Take three threadings and return the middle one.  This should
 * remove some sensor jitter.
 *
 * joint argument is 0-3.
 */
int read_sensor(int joint)
{
    int r1, r2, r3;
    int pin;
    int tmp;

    if ((joint > NR_SENSORS) || (joint < 0)) {
        Serial.print("**************** JOINT OUT OF RANGE!\n");
        return -1;
    }

    pin = sensorPin[joint];

    r1 = analogRead(pin);
    r2 = analogRead(pin);
    r3 = analogRead(pin);

#define SWAP(x,y) {tmp = (x); (x) = (y); (y) = tmp; }

    if (r3 < r2)
        SWAP(r2, r3);
    if (r2 < r1)
        SWAP(r1, r2);
    if (r3 < r2)
        SWAP(r2, r3);

    if (r2 < min_sensor_seen[joint])
        min_sensor_seen[joint] = r2;
    if (r2 > max_sensor_seen[joint])
        max_sensor_seen[joint] = r2;

    return r2;
}

void dbg_n(int n)
{
    old_debug_flag = debug_flag;
    debug_flag = n;

    return;
}

void print_ftuple(double xyz[3])
{
    Serial.print("(");
    Serial.print(xyz[X]);
    Serial.print("\t");
    Serial.print(xyz[Y]);
    Serial.print("\t");
    Serial.print(xyz[Z]);
    Serial.print(")");

    return;
}

int park_leg(void)
{
    int old_int_state;
    int old_leg_state;
    int speed;

    Serial.print("\nParking leg.\n");

    pwms_off();
    old_int_state = set_interrupt_state(0);
    old_leg_state = set_leg_state(ENABLED);

    /* Get thigh and knee into place. */

    /* I should use a low speed PWM value if I have one. */
    Serial.print("# Retracting thigh.\n");
    speed = LOW_PWM_MOVEMENT(THIGHPWM_IN) + 5;
    if (speed == 0xFFFF)
        speed = 50;
    if (move_joint_all_the_way(THIGHPWM_IN, speed) == -1)
        Serial.print("ERROR - couldn't retract thigh!\n");
    pwms_off();

    delay(100);

    Serial.print("# Retracting knee.\n");
    /* Move knee up. */
    speed = LOW_PWM_MOVEMENT(KNEEPWM_OUT) + 5;
    if (speed == 0xFFFF)
        speed = 50;
    if (move_joint_all_the_way(KNEEPWM_OUT, speed) == -1)
        Serial.print("ERROR - couldn't retract knee!\n");

    /* Centering the hip would be better. */
    Serial.print("# Retracting hip.\n");
    /* Move hip back. */
    speed = LOW_PWM_MOVEMENT(HIPPWM_IN) + 5;
    if (speed == 0xFFFF)
        speed = 50;
    if (move_joint_all_the_way(HIPPWM_IN, speed) == -1)
        Serial.print("ERROR - couldn't retract HIP!\n");

    pwms_off();

    /* Set the goal to wherever we are now. */
    reset_current_location();

    set_leg_state(old_leg_state);
    set_interrupt_state(old_int_state);

    return 0;
}

/*
 * Reads the sensors and sets the x,y,z goal to the current location.
 *
 * XXX fixme:  This should use local variables.
 *
 * XXX fixme:  This should move to kinematics.
 */
void reset_current_location(void)
{
    int i;

    /* Set our goal to the current position. */
    read_sensors(current_sensor);

    Serial.print("# Sensors:");
    for (i = 0; i < 3;i++) {
        Serial.print('\t');
        Serial.print(current_sensor[i]);
    }
    Serial.print('\n');

    calculate_angles(current_sensor, current_deg, current_rad);

    Serial.print("# Angles: ");
    for (i = 0; i < 3;i++) {
        Serial.print('\t');
        Serial.print(current_deg[i]);
    }
    Serial.print('\n');

    calculate_xyz(current_xyz, current_rad);

    Serial.print("# Location:");
    for (i = 0; i < 3;i++) {
        Serial.print('\t');
        Serial.print(current_xyz[i]);
    }
    Serial.print('\n');

    xyz_goal[X] = current_xyz[X];
    xyz_goal[Y] = current_xyz[Y];
    xyz_goal[Z] = current_xyz[Z];

    for (i = 0;i < 3;i++) {
        angle_goals[i] = current_deg[i];
        sensor_goal[i] = current_sensor[i];
    }

    Serial.print("# Current goal set to ");
    print_ftuple(current_xyz);
    Serial.print("\n");

    return;
}

/*
 * This replaces some empty values in the leg_info struct with fake
 * values, in hopes of keeping some arithmetic from going haywire.
 * Ideally there'd be something to prevent the use of empty flash, but
 * that'll have to come later.
 */
static void fixup_blank_flash_values(void)
{
    int i;

    /*
     * XXX fixme: These numbers should be determined experimentally
     * (manually) and should be stored in flash manually.
     */
    ANGLE_LOW(HIP)    = -40.46;
    ANGLE_HIGH(HIP)   = 40.46;
    ANGLE_LOW(THIGH)  = -6;
    ANGLE_HIGH(THIGH) = 84;
    ANGLE_LOW(KNEE)   = 13;
    ANGLE_HIGH(KNEE)  = 123;

    /*
     * Actual values seen:
     * HIP:  101-723
     * THIGH: 62-850
     * KNEE: -933
     */

#if 0
    /* These are the original values used. */
    SENSOR_LOW(HIP)    = 93;
    SENSOR_HIGH(HIP)   = 722;
    SENSOR_LOW(THIGH)  = 34;
    SENSOR_HIGH(THIGH) = 917;
    SENSOR_LOW(KNEE)   = 148;
    SENSOR_HIGH(KNEE)  = 934;
#endif

    /*
     * If any of the sensor parameters aren't set then fudge them with
     * wildly conservative values.  This will keep any of the
     * kinematics that get run from coughing up a lung.
     *
     * These shouldn't be as high or low as the real values, to make
     * sure that we actually set them.
     */
    if (SENSOR_LOW(HIP)    == 0xFFFF)
        SENSOR_LOW(HIP)    = 10880;
    if (SENSOR_HIGH(HIP)   == 0xFFFF)
        SENSOR_HIGH(HIP)   = 41600;
    if (SENSOR_LOW(THIGH)  == 0xFFFF)
        SENSOR_LOW(THIGH)  = 10880;
    if (SENSOR_HIGH(THIGH) == 0xFFFF)
        SENSOR_HIGH(THIGH) = 41600;
    if (SENSOR_LOW(KNEE)   == 0xFFFF)
        SENSOR_LOW(KNEE)   = 10880;
    if (SENSOR_HIGH(KNEE)  == 0xFFFF)
        SENSOR_HIGH(KNEE)  = 41600;

    UNITS_PER_DEG(HIP)   = (SENSOR_HIGH(HIP)   - SENSOR_LOW(HIP))   / (ANGLE_HIGH(HIP)   - ANGLE_LOW(HIP));
    UNITS_PER_DEG(THIGH) = (SENSOR_HIGH(THIGH) - SENSOR_LOW(THIGH)) / (ANGLE_HIGH(THIGH) - ANGLE_LOW(THIGH));
    UNITS_PER_DEG(KNEE)  = (SENSOR_HIGH(KNEE)  - SENSOR_LOW(KNEE))  / (ANGLE_HIGH(KNEE)  - ANGLE_LOW(KNEE));

    for (i = 0;i < 6;i++)
        if (LOW_PWM_MOVEMENT(i) == 0xFF)
            LOW_PWM_MOVEMENT(i) = 50;

    /*
     * Fudge the thigh high sensor reading, since we can't determine it
     * experimentally due to ground interference.
     *
     * The thigh cylinder is 14 inches long, and the string pot should
     * give 4196 units/inch.
     */
    if (SENSOR_HIGH(THIGH) < 40000) {
        Serial.print("# Fixing too-low thigh high sensor reading.\n");
        SENSOR_HIGH(THIGH) = SENSOR_LOW(THIGH) + (4196 * 14);
    }

    return;
}

#define EEPROM_BASE 0x14000000

void read_leg_info(leg_info_t *li)
{
    memcpy(li, (void *)EEPROM_BASE, sizeof(leg_info_t));

    Serial.print("# Read ");
    Serial.print(sizeof(leg_info_t));
    Serial.print(" bytes of saved leg data.\n");

    if (li->name[0] == 0xFF)
        memcpy(li->name, "leg", 4);

    if (SENSOR_LOW(HIP) == 0xFFFF)
        Serial.print("\n\nLeg calibration parameters in flash appears to unpopulated!\n\n");

    return;
}

void write_leg_info(leg_info_t *li)
{
    uint n;

    for (n = 0;n < sizeof(leg_info_t);n++)
        EEPROM.write(n, *((uint8_t *)li + n));

    return;
}

void erase_leg_info(void)
{
    uint n;

    for (n = 0;n < sizeof(leg_info_t);n++)
        EEPROM.write(n, 0xFF);

    return;
}

void print_leg_info(leg_info_t *li)
{
    int i;
    int n;

    Serial.print("#\n# Sensor\tLow\tHigh\tTravel\n");
    for (i = 0;i < NR_JOINTS;i++) {
        Serial.print("# ");
        Serial.print(joint_names[i]);
        Serial.print("     \t");
        if (li->sensor_limits[i].sensor_low == 0xFFFF)
            Serial.print("N/A");
        else
            Serial.print(li->sensor_limits[i].sensor_low);
        Serial.print('\t');
        if (li->sensor_limits[i].sensor_high == 0xFFFF)
            Serial.print("N/A");
        else
            Serial.print(li->sensor_limits[i].sensor_high);
        Serial.print('\t');
        if ((li->sensor_limits[i].sensor_low == 0xFFFF) || (li->sensor_limits[i].sensor_high == 0xFFFF))
            Serial.print("N/A");
        else
            Serial.print(li->sensor_limits[i].sensor_high - li->sensor_limits[i].sensor_low);

        Serial.print('\n');
    }

    Serial.print("#\n# Angle data:\n#\n");
    Serial.print("# Joint\t\tunits_per_deg\tAngle low\tAngle high\tDegrees travel\n");
    for (n = 0;n < NR_JOINTS;n++) {
        Serial.print("# ");
        Serial.print(joint_names[n]);
        Serial.print("\t\t");
        Serial.print(li->joint_angles[n].units_per_deg);
        Serial.print("\t\t");
        Serial.print(li->joint_angles[n].angle_low);
        Serial.print("\t\t");
        Serial.print(li->joint_angles[n].angle_high);
        Serial.print("\t\t");
        Serial.print(li->joint_angles[n].angle_high - li->joint_angles[n].angle_low);
        Serial.print('\n');
    }

    Serial.print("#\n# PWM to speed mappings:\n#\n");
    for (n = 0;n < 3;n++) {
        Serial.print("# ");
        Serial.print(joint_names[n]);
        Serial.print(" out:\tlowest PWM:  ");
        Serial.print(li->valves[n].low_joint_movement);
        Serial.print("%\t");

        for (i = 1;i <= 10;i++) {
            Serial.print(i * 10);
            Serial.print("%: ");
            if (li->valves[n].joint_speed[i] == 0xFFFF)
                Serial.print("N/A");
            else
                Serial.print(li->valves[n].joint_speed[i]);
            Serial.print("  \t");
        }
        Serial.print("\n");

        Serial.print("# ");
        Serial.print(joint_names[n]);
        Serial.print(" in: \tlowest PWM:  ");
        Serial.print(li->valves[n + 3].low_joint_movement);
        Serial.print("%\t");

        for (i = 1;i <= 10;i++) {
            Serial.print(i * 10);
            Serial.print("%: ");
            if (li->valves[n].joint_speed[i] == 0xFFFF)
                Serial.print("N/A");
            else
                Serial.print(li->valves[n + 3].joint_speed[i]);
            Serial.print("   \t");
        }
        Serial.print("\n");
    }

    Serial.print("\n");

    return;
}

void setup(void)
{
    Serial.begin(115200);
    Serial.setTimeout(10);

    /* Analog write resolution and PWM frequency. */
    analogWriteResolution(ANALOG_BITS);
    analogReadResolution(ANALOG_BITS);
    analogReadAveraging(16);

    /* max PWM freqency of the motor driver board is 20kHz */
    set_pwm_freq(20000);

    //setup deadman, enable, and error digital I/O pins
    pinMode(DEADMAN_PIN, INPUT);
    digitalWrite(ENABLE_PIN, LOW);
    pinMode(ENABLE_PIN, OUTPUT);
    digitalWrite(ENABLE_PIN_HIP, LOW);
    pinMode(ENABLE_PIN_HIP, OUTPUT);

    disable_leg();
    pwms_off();

    set_pwm_scale(30);	/* Default to 30% of PWM max speed. */

    delay(2000); /* Give the serial console time to come up. */

    read_leg_info(&leg_info);

    print_leg_info(&leg_info);

    velocity_init();

    fixup_blank_flash_values();

    test_kinematics(      ANALOG_MAX / 2,         ANALOG_MAX / 2,        ANALOG_MAX / 2);
    test_kinematics( SENSOR_LOW(HIP) + 1,  SENSOR_LOW(THIGH) + 1,  SENSOR_LOW(KNEE) + 1);
    test_kinematics(SENSOR_HIGH(HIP) - 1, SENSOR_HIGH(THIGH) - 1, SENSOR_HIGH(KNEE) - 1);

#if 0
    /* sensor units per degree */
    UNITS_PER_DEG(HIP) = (SENSOR_HIGH(HIP) - SENSOR_LOW(HIP)) / ((ANGLE_HIGH(HIP) - ANGLE_LOW(HIP)));
    Serial.print("units_per_deg = ");
    Serial.print(UNITS_PER_DEG(HIP));

    Serial.print(' ');
    Serial.print(SENSOR_HIGH(HIP));
    Serial.print(' ');
    Serial.print(SENSOR_LOW(HIP));
    Serial.print(' ');
    Serial.print(UNITS_PER_DEG(HIP));
    Serial.print(' ');
    Serial.print(ANGLE_LOW(HIP));
    Serial.print(' ');
    Serial.print(ANGLE_HIGH(HIP));
    Serial.print(' ');
    Serial.print('\n');

    UNITS_PER_DEG(THIGH) = (SENSOR_HIGH(THIGH) - SENSOR_LOW(THIGH)) / ((ANGLE_HIGH(THIGH) - ANGLE_LOW(THIGH)));
    UNITS_PER_DEG(KNEE) = (SENSOR_HIGH(KNEE) - SENSOR_LOW(KNEE)) / ((ANGLE_HIGH(KNEE) - ANGLE_LOW(KNEE)));
#endif

    reset_current_location();

    interrupt_setup();

    Serial.print("\n# Ready to rock and roll!\n\n");
    Serial.print("# (Leg disabled, use 'enable' command to enable it)\n\n");

    return;
}

#if 0
static void print_reading(int i, int sensor, int goal, const char *joint, const char *action)
{
    Serial.print(i);
    Serial.print('\t');
    Serial.print(sensor);
    Serial.print('\t');
    Serial.print(goal);
    Serial.print('\t');
    Serial.print(joint);
    Serial.print(' ');
    Serial.print(action);
    Serial.print('\n');

    return;
}
#endif

/* Reads the current leg position sensors. */
void read_sensors(int *sensors)
{
    int n;

    DEBUG("Sensors:");
    for (int i = 0; i < NR_SENSORS; i++) {
        /* Read sensors even if we're not moving. */
        n = read_sensor(i);
        sensors[i] = n;
        DEBUG("\t");
        DEBUG(joint_names[i]);
        DEBUG("\t");
        DEBUG(current_sensor[i]);
    }
    DEBUG('\n');

    return;
}

#if 0
/*
 * Write the PWMs based on the last read sensor values and the
 * position goal.
 */
static void write_pwms(void)
{
    static int dbg_msg = 0;

    if ((deadMan == 0) && (!deadman_forced)) {
        if (!dbg_msg) {
            DEBUG("Deadman has leg disabled.\n");
            dbg_msg = 1;
        }
        return;
    }
    dbg_msg = 0;

    for (int i = 0; i < 3; i++) {
        DEBUG(joint_names[i]);

        DEBUG(" sensor ");
        DEBUG(i);
        DEBUG(":\t");
        DEBUG(current_sensor[i]);
        DEBUG("\tgoal:\t");
        DEBUG(sensor_goal[i]);
        //compare sensor reading to goal and only move if not close enough
        if (abs(current_sensor[i] - sensor_goal[i]) >= closeEnough) {
            DEBUG("\tJoint is not close enough.\n");
            if (current_sensor[i] > sensor_goal[i]) {
                set_pwm_goal(i, 100); /* 100% is ambitious. */
/*                print_reading(i, current_sensor[i], sensor_goal[i], joint_names[i], joint_up_actions[i]);*/
            } else if (current_sensor[i] < sensor_goal[i]) {
                set_pwm_goal(i + 3, 100);
/*                print_reading(i, current_sensor[i], sensor_goal[i], joint_names[i], joint_down_actions[i]);*/
            }
        } else {
            Serial.print(joint_names[i]);
            Serial.print(" joint is close enough: ");
            Serial.print(" sensor ");
            Serial.print(i);
            Serial.print(":\t");
            Serial.print(current_sensor[i]);
            Serial.print("\tgoal:\t");
            Serial.print(sensor_goal[i]);

            /*
             * if joint was in motion and the goal was reached - turn
             * motion flag and solenoids off.
             *
             * I don't think the leg should go cold when it reaches
             * its destination.  It should set the PWMs based on the
             * desired speed, and the desired speed is 0.
             */
            set_pwm_goal(i, 0);
            set_pwm_goal(i + 3, 0);

            Serial.print("\treached goal - going cold.\n");
        }
    }

    return;
}
#endif

/*
 * Returns 0 if the deadman is released (leg disabled).
 * Returns 1 if the deadman is held (leg enabled).
 */
int check_deadman(void)
{
    int i;

    i = digitalRead(DEADMAN_PIN) || deadman_forced;

    if (i != deadMan) {
        /* Disable deadman override whenever deadman button changes. */
/*        deadman_forced = 0;*/
        if (i == 0) {
            /*
             * if deadman is off -- joystick is deactivated, then disable
             * driver boards and write zero to PWM lines
             */
            disable_leg();
            Serial.print("OK - Deadman on - leg disabled.\n");
        } else {
            enable_leg();
            Serial.print("OK - Deadman disabled - leg enabled.\n");
        }
        deadMan = i;
    }

    return deadMan;
}

#if 1
/*
 * Loop for velocity control.
 *
 * All the actual work is done in the interrupt thread.
 */
void loop(void)
{
    static int last_seconds = 0;
    int seconds;
    static double last_x_inches, last_y_inches, last_z_inches;

    /*
     * If debug_flag > 0 then it specifies the number of times through
     * this loop to print debugging output.
     */
    if (debug_flag > 0) {
        debug_flag--;
        if (debug_flag == 0) {
            debug_flag = old_debug_flag;
            old_debug_flag = 0;
        }
    }

    if (periodic_debug_flag) {
        seconds = millis() / 1000;
        if (last_seconds != seconds) {
            dbg_n(1);
            last_seconds = seconds;
        }
    }

    /* Check for any input on the console. */
    read_cmd();

    if (!check_deadman())
        return;

    /*
     * If the foot has moved an inch in any direction then print the
     * current locatioin.
     */
    if ((abs(current_xyz[X] - last_x_inches) > 1) ||
        (abs(current_xyz[Y] - last_y_inches) > 1) ||
        (abs(current_xyz[Z] - last_z_inches) > 1))
    {
#if 0
        Serial.print("Current (x,y,z): ");
        print_ftuple(current_xyz);
        Serial.print(" Goal: ");
        print_ftuple(xyz_goal);
        Serial.print('\n');
#endif
/*
        Serial.print(" current angles (hip, thigh, knee): ");
        print_ftuple(current_deg);
        Serial.print('\n');
*/

        last_x_inches = current_xyz[X];
        last_y_inches = current_xyz[Y];
        last_z_inches = current_xyz[Z];
    }

    return;
}

#else

/* XXX old loop */
void loop(void)
{
    static int last_seconds = 0;
    int seconds;
    static double last_x_inches, last_y_inches, last_z_inches;

    /*
     * If debug_flag > 0 then it specifies the number of times through
     * this loop to print debugging output.
     */
    if (debug_flag > 0) {
        debug_flag--;
        if (debug_flag == 0) {
            debug_flag = old_debug_flag;
            old_debug_flag = 0;
        }
    }

    if (periodic_debug_flag) {
        seconds = millis() / 1000;
        if (last_seconds != seconds) {
            dbg_n(1);
            last_seconds = seconds;
        }
    }

    /* Check for any input on the console. */
    read_cmd();

    check_deadman();

    /* Read the sensors. */
    read_sensors(current_sensor);
    /* Turn sensor readings into joint angles. */
    calculate_angles(current_sensor, current_deg, current_rad);
    /* Turn joint angles into (x,y,z). */
    calculate_xyz(current_xyz, current_rad);

    /* Joystic mode should move the desired x,y,z, but only after I fix positional. */
    if (joystick_mode == JOYSTICK_JOINT) {
        /* In joystick mode the joystick controls the PWMs. */
        do_joystick_joints();
    } else {
        /* Write something to the PWMs to make the leg move. */
        write_pwms();
    }

    /*
     * If the foot has moved an inch in any direction then print the
     * current locatioin.
     */
    if ((abs(current_xyz[X] - last_x_inches) > 1) ||
        (abs(current_xyz[Y] - last_y_inches) > 1) ||
        (abs(current_xyz[Z] - last_z_inches) > 1))
    {
#if 0
        Serial.print("Current (x,y,z): ");
        print_ftuple(current_xyz);
        Serial.print(" Goal: ");
        print_ftuple(xyz_goal);
        Serial.print('\n');
#endif
/*
        Serial.print(" current angles (hip, thigh, knee): ");
        print_ftuple(current_deg);
        Serial.print('\n');
*/

        last_x_inches = current_xyz[X];
        last_y_inches = current_xyz[Y];
        last_z_inches = current_xyz[Z];
    }

    // A short delay so I can read the serial while programing.
/*    delay(10);*/

    return;
}
#endif

/*
 * This makes sure the driver board is not enabled if the joystick is
 * turned off (deadMan is low).
 */
void disable_leg(void)
{
    leg_enabled = 0;

    digitalWrite(ENABLE_PIN, LOW);
    digitalWrite(ENABLE_PIN_HIP, LOW);
    pwms_off();

    Serial.print("\n# **************** Disabling leg. ****************\n\n");

    return;
}

void enable_leg(void)
{
    pwms_off();

    digitalWrite(ENABLE_PIN, HIGH);
    digitalWrite(ENABLE_PIN_HIP, HIGH);

    leg_enabled = 1;

    return;
}

int set_leg_state(int state)
{
    int old_leg_state = leg_enabled;

    if (state)
        enable_leg();
    else
        disable_leg();

    return old_leg_state;
}
