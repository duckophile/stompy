/* ; -*- mode: C ;-*- */

#include <EEPROM.h>

#include "pins.h"

#include "leg-info.h"

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

#define ANALOG_BITS	10
#define PWM_BITS	ANALOG_BITS
#define PWM_MAX		((1 << PWM_BITS) - 1)
#define ANALOG_MAX	((1 << ANALOG_BITS) - 1)

volatile int velocity_debug = 0;

volatile int leg_enabled = 0;
volatile int interrupts_enabled = 0;

volatile static int debug_flag = 0;  /* Enable verbose output. */
volatile static int old_debug_flag = 0;
volatile static int periodic_debug_flag = 0;
#define DEBUG   if(debug_flag)Serial.print
#define DEBUGLN if(debug_flag)Serial.println

/* Timing info for the ISR loop. */
uint32_t isr_max = 0;
uint32_t isr_min = 1 << 31;
uint32_t isr_count = 0;

/* Joint/sensor numbers. */
#define HIP			0
#define THIGH			1
#define KNEE			2
#define COMPLIANT		3

/* Valve numbers. */
#define HIPPWM_OUT		0
#define THIGHPWM_OUT		1
#define KNEEPWM_OUT		2
#define HIPPWM_IN		3
#define THIGHPWM_IN		4
#define KNEEPWM_IN		5

const int pwm_pins[6]  = {HIPPWM_OUT_PIN,  THIGHPWM_OUT_PIN,  KNEEPWM_OUT_PIN,
                          HIPPWM_IN_PIN,   THIGHPWM_IN_PIN,   KNEEPWM_IN_PIN};

/*
  * These are used to convert an 0-2 joint number to a valve number in
  * the pwm_pins[] array.  pwm_pins[joint + OUT] is the valve pin to
  * move a joint out, pwm_pins[jount + IN] is the valve pin to move
  * the joint in.
  */
#define OUT	0      /* Sensor value decreasing. */
#define IN	3      /* Sensor value increasing. */

const char *direction_names[]    = {"OUT",     "ERROR1", "ERROR2",   "IN",     "ERROR3"};
const char *joint_names[]        = {"hip",     "thigh",  "knee",     "compliant"};
const char *joint_up_actions[]   = {"back",    "up",    "out"};
const char *joint_down_actions[] = {"forward", "down",  "in"};

/*
 * XXX fixme: Put all the kinematics-related values in one
 * container?
 * And standardize sensor_goal vs. current_sensor etc.
 * and *_goal vs. *_goals
 */

int sensor_goal[3]     = {0,0,0};
double xyz_goal[3]     = {0,0,0};
double angle_goals[3]  = {0,0,0};

int current_sensor[3]  = {0,0,0};

/* The minimum PWM speed at which a valve can move a joint. */
/* XXX fixme:  These should be stored in flash. */
int valve_min_pwm_speed[6] = {-1, -1, -1, -1, -1, -1};

/*
 * Angles in degrees -
 * Hip is zero when straight out from the robot body.
 * Hip angle is zero when the thigh is parallel to the ground
 * Knee angle is between thigh and calf (13 degrees fully retracted)
 */
double current_deg[3];
double current_rad[3];

int sensorPin[3] = {HIP_SENSOR_PIN, THIGH_SENSOR_PIN, KNEE_SENSOR_PIN};
// this is the distance in sensor reading that is close enough for directed movement
// I am putting this here so we can avoid chasing our tails early in positional control
int closeEnough = 2;

#define JOYSTICK_OFF		0
#define JOYSTICK_JOINT		1
#define JOYSTICK_POSITION	2
int joystick_mode = JOYSTICK_OFF;

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

/* leg link lengths hip, thigh, and knee */
#define L1		11
#define L2		54
#define L3		72

#define HIP_LEN		11	/* Hip pivot to thigh pivot. */
#define THIGH_LEN	54	/* Thigh pivot to knee pivot. */
#define KNEE_LEN	72	/* Knee pivot to ankle pivot. */

#define X		0
#define Y		1
#define Z		2
double current_xyz[3];

leg_info_t leg_info;

char cmd_buf[128];
int cmd_len = 0;
char *cmd_ptr;

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

    if ((joint > NR_JOINTS) || (joint < 0)) {
        Serial.println("**************** JOINT OUT OF RANGE!");
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

    return r2;
}

void print_xyz(double xyz[3])
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

int func_none(void)
{
    Serial.println("ERROR - Not implemented.");

    return 0;
}

/* Set PWM scaling factor. */
int func_scale(void)
{
    int scale;

    scale = read_int();

    set_pwm_scale(scale);

    return 0;
}

/* Set PWM. */
int func_pwm(void)
{
    int pin;
    int regval;

    pin = read_int();
    regval = read_int();

    analogWrite(pin, regval);

    return 0;
}


/* Set PWM frequency. */
int func_freq(void)
{
    int freq;

    freq = read_int();

    set_pwm_freq(freq);

    return 0;
}

int func_debug(void)
{
    if (debug_flag)
        debug_flag = 0;
    else
        debug_flag = -1; /* -1 = Permanently on. */
    Serial.print("OK - Debug turned ");
    Serial.println(debug_flag ? "on" : "off");

    return 0;
}

void dbg_n(int n)
{
    old_debug_flag = debug_flag;
    debug_flag = n;

    return;
}

int func_dbg(void)
{
    periodic_debug_flag = !periodic_debug_flag;

    return 0;
}

extern int current_pwms[];

/* XXX fixme:  Unused. */
void print_current(void)
{
    int i;

    Serial.print("Current PWMs:    ");
    for (i = 0;i < 3;i++) {
        Serial.print("\t");
        Serial.print(joint_names[i]);
        Serial.print("\t");
        Serial.print(current_pwms[i]);
    }
    Serial.println("");
    Serial.print("Current PWMs:    ");
    for (i = 3;i < 6;i++) {
        Serial.print("\t");
        Serial.print(joint_names[i - 3]);
        Serial.print("\t");
        Serial.print(current_pwms[i]);
    }
    Serial.println("");

    Serial.print("Current angles:  ");
    for (i = 0; i < 3; i++) {
        Serial.print("\t");
        Serial.print(joint_names[i]);
        Serial.print("\t");
        Serial.print(current_deg[i]);
    }
    Serial.println("");

    Serial.print("Current sensors: ");
    for (i = 0; i < 3; i++) {
        Serial.print("\t");
        Serial.print(joint_names[i]);
        Serial.print("\t");
        Serial.print(current_sensor[i]);
    }
    Serial.println("");

    Serial.print("Current XYZ:    \t");
    for (i = 0; i < 3; i++) {
        Serial.print("\t");
        Serial.print(current_xyz[i]);
    }
    Serial.println("");

    return;
}

/* XXX fixme:  Unused. */
void print_goals(void)
{
    int i;

    Serial.print("Sensor Goals:    ");
    for (i = 0;i < 3;i++) {
        Serial.print("\t");
        Serial.print(joint_names[i]);
        Serial.print("\t");
        Serial.print(sensor_goal[i]);
    }
    Serial.println("");

    Serial.print("XYZ goal:        ");
    for (i = 0;i < 3;i++) {
        Serial.print("\t");
        Serial.print(xyz_goal[i]);
    }
    Serial.println("");

    Serial.print("Angle goals:     ");
    for (i = 0;i < 3;i++) {
        Serial.print("\t");
        Serial.print(angle_goals[i]);
    }
    Serial.print("\n");

    return;
}

int func_info(void)
{
    int i;

    Serial.print("\n# ================================================================\n");
    Serial.print("# Current info:\n#\n");
    Serial.print("# Leg number ");
    Serial.print(leg_info.leg_number);
    Serial.print(" named \"");
    Serial.print(leg_info.name);
    Serial.print("\" Deadman: ");
    Serial.print(deadMan);
    Serial.print(" Deadman forced: ");
    Serial.print(deadman_forced);
    Serial.print(" Leg enabled: ");
    Serial.print(leg_enabled);
    Serial.print(" Interrupts enabled: ");
    Serial.print(interrupts_enabled);
    Serial.print("\n");

    Serial.print("# Current XYZ:     ");
    for (i = 0; i < 3; i++) {
        Serial.print("\t");
        Serial.print(current_xyz[i]);
    }
    Serial.print('\n');

    Serial.print("# XYZ goal:        ");
    for (i = 0;i < 3;i++) {
        Serial.print("\t");
        Serial.print(xyz_goal[i]);
    }
    Serial.print('\n');

    Serial.print("# Current angles:  ");
    for (i = 0; i < 3; i++) {
        Serial.print("\t");
        Serial.print(joint_names[i]);
        Serial.print("\t");
        Serial.print(current_deg[i]);
    }
    Serial.print('\n');

    Serial.print("# Angle goals:     ");
    for (i = 0;i < 3;i++) {
        Serial.print("\t");
        Serial.print(joint_names[i]);
        Serial.print("\t");
        Serial.print(angle_goals[i]);
    }
    Serial.print('\n');

    Serial.print("# Current sensors: ");
    for (i = 0; i < 3; i++) {
        Serial.print("\t");
        Serial.print(joint_names[i]);
        Serial.print("\t");
        Serial.print(current_sensor[i]);
    }
    Serial.print('\n');

    Serial.print("# Sensor Goals:    ");
    for (i = 0;i < 3;i++) {
        Serial.print("\t");
        Serial.print(joint_names[i]);
        Serial.print("\t");
        Serial.print(sensor_goal[i]);
    }
    Serial.print('\n');

    Serial.print("# PWMs:            ");
    for (i = 0;i < 3;i++) {
        Serial.print("\t");
        Serial.print(joint_names[i]);
        Serial.print("\t");
        Serial.print(current_pwms[i]);
    }
    Serial.print('\n');

    Serial.print("# Sensor lows/highs:\n");
    for (i = 0;i < 3;i++) {
        Serial.print("# ");
        Serial.print(joint_names[i]);
        Serial.print("\tLow: ");
        Serial.print(SENSOR_LOW(i));
        Serial.print("\tHigh: ");
        Serial.print(SENSOR_HIGH(i));
        Serial.print("\n");
    }
    Serial.print("#\n");
    Serial.print("# ISR min: ");
    Serial.print(isr_min);
    Serial.print("us, ISR max: ");
    Serial.print(isr_max);
    Serial.print("us.  ISR count: ");
    Serial.println(isr_count);
    Serial.print("# ================================================================\n\n");

    return 0;
}

int func_deadman(void)
{
    pwms_off();

    deadman_forced = !deadman_forced;

    Serial.print("Deadman ");
    Serial.println(deadman_forced ? "disabled" : "enabled");

    if (deadman_forced) {
        velocity_debug++;
        velocity_debug++;
    }

    return 0;
}

int func_joystick(void)
{
    return toggle_joystick_mode();
}

int func_joyxyz(void)
{
    if (joystick_mode == JOYSTICK_POSITION) {
        Serial.print("OK - Joystick mode disabled.\n\n");
        joystick_mode = JOYSTICK_OFF;
    } else {
        joystick_mode = JOYSTICK_POSITION;
        Serial.print("OK - Joystick positional mode enabled.\n\n");
        enable_leg();
    }

    return 0;
}

void print_leg_info(leg_info_t *li)
{
    int i;
    int n;

    Serial.print("#\n# Sensor highs\n");
    for (i = 0;i < NR_JOINTS;i++) {
        Serial.print("#\t");
        Serial.print(joint_names[i]);
        Serial.print("     \t");
        if (li->sensor_limits[i].sensor_high == 0xFFFF)
            Serial.println("N/A");
        else
            Serial.println(li->sensor_limits[i].sensor_high);
    }

    Serial.print("#\n# Sensor lows\n");
    for (i = 0;i < NR_JOINTS;i++) {
        Serial.print("#\t");
        Serial.print(joint_names[i]);
        Serial.print("     \t");
        if (li->sensor_limits[i].sensor_low == 0xFFFF)
            Serial.println("N/A");
        else
            Serial.println(li->sensor_limits[i].sensor_low);
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

    Serial.println("\n");

    return;
}

/*
 * This replaces some empty values in the leg_info struct with fake
 * values, in hopes of keeping some arithmetic from going haywire.
 * Ideally there'd be something to prevent the use of empty flash, but
 * that'll have to come later.
 */
void fixup_blank_flash_values(void)
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
        SENSOR_LOW(HIP)    = 170;
    if (SENSOR_HIGH(HIP)   == 0xFFFF)
        SENSOR_HIGH(HIP)   = 650;
    if (SENSOR_LOW(THIGH)  == 0xFFFF)
        SENSOR_LOW(THIGH)  = 170;
    if (SENSOR_HIGH(THIGH) == 0xFFFF)
        SENSOR_HIGH(THIGH) = 650;
    if (SENSOR_LOW(KNEE)   == 0xFFFF)
        SENSOR_LOW(KNEE)   = 170;
    if (SENSOR_HIGH(KNEE)  == 0xFFFF)
        SENSOR_HIGH(KNEE)  = 650;

    UNITS_PER_DEG(HIP)   = (SENSOR_HIGH(HIP)   - SENSOR_LOW(HIP))   / (ANGLE_HIGH(HIP)   - ANGLE_LOW(HIP));
    UNITS_PER_DEG(THIGH) = (SENSOR_HIGH(THIGH) - SENSOR_LOW(THIGH)) / (ANGLE_HIGH(THIGH) - ANGLE_LOW(THIGH));
    UNITS_PER_DEG(KNEE)  = (SENSOR_HIGH(KNEE)  - SENSOR_LOW(KNEE))  / (ANGLE_HIGH(KNEE)  - ANGLE_LOW(KNEE));

    for (i = 0;i < 6;i++)
        if (LOW_PWM_MOVEMENT(i) == 0xFF)
            LOW_PWM_MOVEMENT(i) = 50;

    return;
}

#define EEPROM_BASE 0x14000000

void read_leg_info(leg_info_t *li)
{
    memcpy(li, (void *)EEPROM_BASE, sizeof(leg_info_t));

    Serial.print("# Read ");
    Serial.print(sizeof(leg_info_t));
    Serial.println(" bytes of saved leg data.");

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

int func_flashinfo(void)
{
    leg_info_t li;

    read_leg_info(&li);

    Serial.println("# Leg parameters stored in flash:");
    print_leg_info(&li);

    return 0;
}

int func_leginfo(void)
{
    Serial.println("# Leg parameters in memory and maybe not yet in flash:");
    print_leg_info(&leg_info);

    return 0;
}

int func_saveflash(void)
{
    write_leg_info(&leg_info);

    Serial.println("OK - Leg parameters written to flash.\n");

    return 0;
}

int func_eraseflash(void)
{
    erase_leg_info();

    Serial.println("OK - Flash parameters erased.\n");

    return 0;
}

int func_name(void)
{
    strncpy(leg_info.name, cmd_ptr, 14);

    return 0;
}

/* Set the leg number. */
int func_legnum(void)
{
    leg_info.leg_number = read_int();

    return 0;
}

#define CPU_RESTART_ADDR (uint32_t *)0xE000ED0C
#define CPU_RESTART_VAL 0x5FA0004
#define CPU_RESTART (*CPU_RESTART_ADDR = CPU_RESTART_VAL)

int func_reset(void)
{
    CPU_RESTART;

    return 0;
}

/*
 * Show sensor jitter.
 *
 * Read the sensors until a key is pressed, count the number of times
 * each value is read, and print a crude histogram(ish) of the values
 * seen.
 */
int func_sensors(void)
{
    int n;
    int i;
    int sample_count = 0;
    int sense_highs[3]    = {    0,     0,     0};
    int sense_lows[3]     = {65536, 65536, 65536};
    int readings[8192];

    memset(readings, 0, sizeof(readings));

    Serial.println("");
    Serial.print("Sensors: ");
    while (1) {
        sample_count++;
        for (i = 0; i < 3; i++) {
            n = analogRead(sensorPin[i]);
/*            n = read_sensor(i);*/
            if (n < sense_lows[i]) {
#if 0
                Serial.print("New low for ");
                Serial.print(joint_names[i]);
                Serial.print(" old = ");
                Serial.print(sense_lows[i]);
                Serial.print(" new = ");
                Serial.println(n);
#endif
                sense_lows[i] = n;
            }
            if (n > sense_highs[i]) {
#if 0
                Serial.print("New high for ");
                Serial.print(joint_names[i]);
                Serial.print(" old = ");
                Serial.print(sense_highs[i]);
                Serial.print(" new = ");
                Serial.println(n);
#endif

                sense_highs[i] = n;
            }

/*            n = n / 8; */	/* Fit 64K samples into 8192 buckets. */
            readings[n]++;

            Serial.print("\t");
            Serial.print(n);
        }
        Serial.print("\n");

        if (Serial.available() > 0) {
            Serial.read();
            break;
        }
    }

    Serial.print(" ");
    Serial.print(sample_count);
    Serial.println(" samples.");
    for (i = 0;i < 8192;i++) {
        if (readings[i] != 0) {
/*            Serial.print(i * 8);*/
            Serial.print(i);
            Serial.print("\t = ");
            Serial.println(readings[i]);
        }
    }

    Serial.print("Low sensor readings:  ");
    for (i = 0;i < 3;i++) {
        Serial.print("\t");
        Serial.print(joint_names[i]);
        Serial.print("\t");
/*        Serial.print(leg_info.sensor_limits[i].sensor_low);*/
        Serial.print(sense_lows[i]);
    }
    Serial.println("");
    Serial.print("High sensor readings: ");
    for (i = 0;i < 3;i++) {
        Serial.print("\t");
        Serial.print(joint_names[i]);
        Serial.print("\t");
/*        Serial.print(leg_info.sensor_limits[i].sensor_high);*/
        Serial.print(sense_highs[i]);
    }
    Serial.println("");
    Serial.println("");

    return 0;
}

/*
 * Stick the leg straight out and wiggle it up and down in hopes of
 * making it easier for air to get out.
 */
int func_bleed(void)
{
    return 0;
}

int func_stop(void)
{
    disable_leg();

    return 0;
}

int func_findlimits(void)
{
    int joint = -1;

    if (!strcmp(cmd_ptr, "hip"))
        joint = HIP;
    if (!strcmp(cmd_ptr, "thigh"))
        joint = THIGH;
    if (!strcmp(cmd_ptr, "knee"))
        joint = KNEE;

    if (joint == -1) {
        Serial.print("ERROR - Please specify hip, thigh, or knee.\n");
        return -1;
    }

    return set_joint_limits(joint);
}

int func_hipcal(void)
{
    int count;

    count = read_int();
    return calibrate_joint(HIP, count);
}

int func_kneecal(void)
{
    int count;

    count = read_int();
    return calibrate_joint(KNEE, count);
}

int func_thighcal(void)
{
    int count;

    count = read_int();
    return calibrate_joint(THIGH, count);
}

int func_enable(void)
{
    enable_leg();

    Serial.println("OK - Leg enabled.");

    return 0;
}

int func_intoff(void)
{
    disable_interrupts();

    Serial.print("OK - Interrupts disabled.\n");

    return 0;
}

int do_timing(int, int);

int func_timing(void)
{
    int direction, pwm;

    direction = read_int();
    pwm = read_int();

    return do_timing(direction, pwm);
}

int func_foo(void)
{
    Serial.print("Extending knee (IN).\n");
    if (move_joint_all_the_way(KNEEPWM_IN, 50) == -1)
        Serial.print("ERROR - couldn't extend knee!\n");
    Serial.print("\n\n");
    Serial.print("Retracting knee (OUT).\n");
    if (move_joint_all_the_way(KNEEPWM_OUT, 50) == -1)
        Serial.print("ERROR - couldn't retract knee!\n");

    return 0;
}


/* XXX fixme:  This should move the hip to the center of travel. */
int func_park(void)
{
    Serial.print("\nParking leg.\n");

    pwms_off();
    disable_interrupts();
    set_pwm_scale(100);
    enable_leg();

    /* Get thigh and knee into place. */

    /* I should use a low speed PWM value if I have one. */
    Serial.println("# Retracting thigh.");
    if (move_joint_all_the_way(THIGHPWM_IN, 50) == -1)
        Serial.print("ERROR - couldn't retract thigh!\n");
    pwms_off();

    Serial.println("# Done, waiting...");
    delay(1000);

    Serial.println("# Retracting knee.");
    /* Move knee up. */
    if (move_joint_all_the_way(KNEEPWM_OUT, 50) == -1)
        Serial.print("ERROR - couldn't retract knee!\n");

    pwms_off();
    disable_leg();

    /* Set the goal to wherever we are now. */
    reset_current_location();

    /* Center hip? */

    Serial.print("Done.\n\n");

    return 0;
}

int measure_speed(int joint, int direction, int pwm_goal, int verbose);

int func_setloc(void)
{
    reset_current_location();

    return 0;
}

/*
 * A speed test.  Tries random PWM values within 32 of the lowest
 * joint movement PWM value and reports joint speed.
 */
int func_speed(void)
{
    int pwm;
    int joint = HIP;
    int rnd;
    int direction;

    srand(millis());

    while (1) {
        direction = IN;
        pwm = LOW_PWM_MOVEMENT(joint + direction);
        rnd = (rand() >> 1) & 0x1F;	/* 0-32 */
        if (measure_speed(HIP, direction, pwm + rnd, 1) == -1)
            break;

        delay(400);

        direction = OUT;
        pwm = LOW_PWM_MOVEMENT(joint + direction);
        rnd = (rand() >> 1) & 0x1F;	/* 0-32 */
        if (measure_speed(HIP, direction, pwm + rnd, 1) == -1)
            break;

        delay(400);
    }

    return 0;
}

int func_go(void);

struct {
    const char *name;
    int (*func)();
} cmd_table[] = {
    { "bleed",      func_bleed     },
    { "deadman",    func_deadman   }, /* Ignore the deadman. */
    { "dbg",        func_dbg       }, /* Enable debug once. */
    { "debug",      func_debug     }, /* Enable debugging output. */
    { "dither",     func_none      }, /* Set the dither amount. */
    { "enable",     func_enable    }, /* Enable the leg, allowing it to move. */
    { "eraseflash", func_eraseflash}, /* Erase the parameters saved in flash. */
    { "findlimits", func_findlimits}, /* Find the sensor limits for a joint. */
    { "flashinfo",  func_flashinfo }, /* Print leg parameters stored in flash. */
    { "foo",        func_foo       }, /* Whatever I want it to do. */
    { "freq",       func_none      }, /* Set PWM frequency. */
    { "go",         func_go        }, /* goto given x, y, z. */
    { "help",       func_help      },
    { "hipcal",     func_hipcal    }, /* Run hip calibration. */
    { "home",       func_none      }, /* Some neutral position?  Standing positioon maybe? */
    { "info",       func_info      },
    { "intoff",     func_intoff    }, /* Disable interrupts. */
    { "joystick",   func_joystick  }, /* Enable jpoystick joint control mode. */
    { "joyxyz",     func_joyxyz    }, /* Enable jpoystick positional mode. */
    { "jtest",      func_jtest     }, /* Print joystick values for calibration. */
    { "kneecal",    func_kneecal   }, /* Run knee calibration. */
    { "leginfo",    func_leginfo   }, /* Print leg paramters stored in memory. */
    { "legnum",     func_legnum    }, /* Set the leg number. */
    { "name",       func_name      }, /* Nmme the leg. */
    { "park",       func_park      }, /* Move leg to parked position. */
    { "pwm",        func_pwm       }, /* Set a PWM. */
    { "reset",      func_reset     }, /* Reset and reboot the teensy. */
    { "saveflash",  func_saveflash }, /* Write in-memory leg parameters to flash. */
    { "scale",      func_scale     }, /* Set max PWM value. */
    { "sensors",    func_sensors   }, /* Continuously read and print sensor readings. */
    { "setloc",     func_setloc    }, /* Set the current location as the goal. */
    { "speed",      func_speed     }, /* Do speed test. */
    { "stop",       func_stop      }, /* Stop moving. */
    { "thighcal",   func_thighcal  }, /* Run thigh calibration. */
    { "timing",     func_timing    }, /* Collect timing info. */
    { "where",      func_none      }, /* Print current x, y, z, and degrees. */
    { NULL,         NULL           }
};

int read_int(void)
{
    int n;
    char *p;

    n = strtol(cmd_ptr, &p, 0);
    cmd_ptr = p;

    return n;
}

double read_double(void)
{
    double f;
    char *p;

    f = strtof(cmd_ptr, &p);
    cmd_ptr = p;

    return f;
}

void read_cmd(void)
{
    int n;
    int caught = 0;
    int ch = 0;
    static int prompted = 0;

    if (!prompted) {
        if (leg_info.name[0] == 0xFF)
            Serial.print("bash$ ");
        else {
            Serial.print(leg_info.name);
            Serial.print("> ");
        }
        prompted = 1;
    }

    if (Serial.available() <= 0)
        return;

    while (Serial.available() > 0) {
        ch = Serial.read();

        if (ch == 0x3) { /* ^C */
            disable_leg();
            continue;
        }
        if (ch == 22) { /* ^V */
            Serial.print("Toggling velocity debugging output.\n");
            toggle_velocity_debug();
            continue;
        }
        if (ch == 24) { /* ^X */
            func_dbg();
            velocity_debug++;
            velocity_debug++;
            continue;
        }

        if (ch == 127) {
            if (cmd_len > 0) {
                cmd_len--;
                cmd_buf[cmd_len] = 0;
                Serial.print("\b \b");
            }
            continue;
        }

        if (cmd_len > 127)
            continue;

        Serial.print((char)ch);
        if (ch == '\r')
            Serial.print('\n');

        if ((ch == '\r') || (ch == '\n'))
            break;

        if (ch < 0x20) /* Ignore other control characters. */
            continue;

        cmd_buf[cmd_len++] = ch;
        cmd_buf[cmd_len] = 0;
    }
    if ((ch != '\r') && (ch != '\n'))
        return;

    /* We have a newline, so read the cmd out of the buffer. */

    prompted = 0;

    if (cmd_len == 0) {
        func_info();
        return;
    }

    /* Null terminate the command. */
    cmd_ptr = cmd_buf;
    while ((*cmd_ptr != 0) && (*cmd_ptr != ' '))
        cmd_ptr++;
    *(cmd_ptr++) = 0;

    for (n = 0; cmd_table[n].name != NULL;n++) {
        if (!strcmp(cmd_buf, cmd_table[n].name)) {
            cmd_table[n].func();
            caught = 1;
        }
    }

    if (!caught) {
        Serial.print("ERROR - Unknown command ");
        Serial.println(cmd_buf);
    }

    cmd_len = 0;

    return;
}

int func_help(void)
{
    for (int i = 0;cmd_table[i].name != NULL;i++) {
        Serial.print(cmd_table[i].name);
        Serial.print(" ");
    }
    Serial.print("\n"
                 "^C to stop and disable leg.  ^X to enable/disable general debugging messages.\n"
                 "^V to enable velocity debugging messages.\n"
                 "\n");

    return 0;
}

/*
 * Reads the sensors and sets the x,y,z goal to the current location.
 *
 * XXX fixme:  This should use local variables.
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
    print_xyz(current_xyz);
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

    delay(2000); /* Give the serial console time to come up. */

    read_leg_info(&leg_info);

    print_leg_info(&leg_info);

    velocity_init();

    fixup_blank_flash_values();

    test_kinematics(                 512,                    512,                   512);
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

void print_reading(int i, int sensor, int goal, const char *joint, const char *action)
{
    Serial.print(i);
    Serial.print('\t');
    Serial.print(sensor);
    Serial.print('\t');
    Serial.print(goal);
    Serial.print('\t');
    Serial.print(joint);
    Serial.print(' ');
    Serial.println(action);

    return;
}

/* Reads the current leg position sensors. */
void read_sensors(int *sensors)
{
    int n;

    DEBUG("Sensors:");
    for (int i = 0; i < 3; i++) {
        /* Read sensors even if we're not moving. */
        n = read_sensor(i);
        sensors[i] = n;
        DEBUG("\t");
        DEBUG(joint_names[i]);
        DEBUG("\t");
        DEBUG(current_sensor[i]);
    }
    DEBUGLN("");

    return;
}

/*
 * Write the PWMs based on the last read sensor values and the
 * position goal.
 */
void write_pwms(void)
{
    static int dbg_msg = 0;

    if ((deadMan == 0) && (!deadman_forced)) {
        if (!dbg_msg) {
            DEBUGLN("Deadman has leg disabled.");
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
            DEBUGLN("\tJoint is not close enough.");
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

            Serial.println("\treached goal - going cold.");
        }
    }

    return;
}

int read_xyz(double *xyz)
{
    // look for first valid integar to be x
    xyz[X] = read_double();
    xyz[Y] = read_double();
    xyz[Z] = read_double();

    /* XXX fixme:  Kludge to avoid divide by zero. */
    if (xyz[X] == 0)
        xyz[X] = 0.0001;

#if 0
    Serial.println("");
    Serial.print("I was given a goal of (x,y,z): ");
    print_xyz(xyz);
    Serial.println("");
#endif

    return 0;
}

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
            Serial.println("OK - Deadman on - leg disabled.");
        } else {
            enable_leg();
            Serial.println("OK - Deadman disabled - leg enabled.");
        }
        deadMan = i;
    }

    return deadMan;
}

void thing(int n)
{
    static int done = 0;

    if (joystick_mode == 1)
        done = 0;

    if (done)
        return;
    Serial.print(n);
    Serial.print(" ");
    Serial.print(joystick_mode);
    Serial.println("");
    if (joystick_mode == 0)
        done = 1;

    return;
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
        print_xyz(current_xyz);
        Serial.print(" Goal: ");
        print_xyz(xyz_goal);
        Serial.println("");
#endif
/*
        Serial.print(" current angles (hip, thigh, knee): ");
        print_xyz(current_deg);
        Serial.println("");
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
        print_xyz(current_xyz);
        Serial.print(" Goal: ");
        print_xyz(xyz_goal);
        Serial.println("");
#endif
/*
        Serial.print(" current angles (hip, thigh, knee): ");
        print_xyz(current_deg);
        Serial.println("");
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
 * The 'go' command.
 *
 * Takes 3 doubles for (x,y,z) goal.
 */
int func_go(void)
{
    int i;
    double xyz[3];
    double deg_goals[3];

    /* XXX fixme:  I should make sure the interrupt thread doesn't run until this is done. */

    joystick_mode = 0;

    Serial.print("\n# ----------------------------------------------------------------\n");

    old_debug_flag = debug_flag;
    debug_flag = 1;	/* Enable debugging for one loop. */

    /* Read x,y,z from console, and update thetas if there's a new one. */
    read_xyz(xyz);

    /*
     * inverse_kin() verifies the goals are in range and constrains
     * them if they're not.
     */
    /*
     * XXX fixme: This shouldn't fill sensor_goal yet, it should be
     * populated at the same time as the other goals.
     */
    inverse_kin(xyz, sensor_goal, deg_goals);

    Serial.print("\n# New Goals: (x,y,z):\t");
    print_xyz(xyz);
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

    for (i = 0;i < 3;i++) {
        xyz_goal[i] = xyz[i];
        angle_goals[i] = deg_goals[i];
    }

    velocity_debug = 1;

    Serial.print("# ----------------------------------------------------------------\n\n");

    return 0;
}

/*
 * This makes sure the driver board is not enabled if the joystick is
 * turned off (deadMan is low).
 */
void disable_leg()
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

    pwms_off();

    leg_enabled = 1;

    return;
}
