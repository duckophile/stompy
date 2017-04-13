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

static int debug_flag = 0;  /* Enable verbose output. */
static int old_debug_flag = 0;
static int periodic_debug_flag = 0;
#define DEBUG   if(debug_flag)Serial.print
#define DEBUGLN if(debug_flag)Serial.println

#define HIP		0
#define THIGH		1
#define KNEE		2
#define COMPLIANT	3

/*
 * XXX fixme: This should be ordered such that the first three have
 * the sensor reading going down and the last three have it going up,
 * but I'm not sure if that's true or if that can be made to work.
 */
#define HIPPWM_FORWARD		0
#define THIGHPWM_UP		1
#define KNEEPWM_RETRACT		2
#define HIPPWM_REVERSE		3
#define THIGHPWM_DOWN		4
/* XXX fixme:  Knee extend and retract appear to be reversed. */
#define KNEEPWM_EXTEND		5

/*
 * *_up and *_down are poor names.
 */
#if 1
const int up_pwms[3]   = {HIPPWM_REVERSE_PIN,  THIGHPWM_UP_PIN,   KNEEPWM_EXTEND_PIN};
const int down_pwms[3] = {HIPPWM_FORWARD_PIN,  THIGHPWM_DOWN_PIN, KNEEPWM_RETRACT_PIN};
const int pwm_pins[6]  = {HIPPWM_REVERSE_PIN,  THIGHPWM_UP_PIN,   KNEEPWM_EXTEND_PIN,
                          HIPPWM_FORWARD_PIN,  THIGHPWM_DOWN_PIN, KNEEPWM_RETRACT_PIN};
#else
/* This ordering was to match the joystick, but messed up the kinematics, */
const int up_pwms[3]   = {HIPPWM_FORWARD_PIN,  THIGHPWM_UP_PIN,   KNEEPWM_RETRACT_PIN};
const int down_pwms[3] = {HIPPWM_REVERSE_PIN,  THIGHPWM_DOWN_PIN, KNEEPWM_EXTEND_PIN};
const int pwm_pins[6]  = {HIPPWM_FORWARD_PIN,  THIGHPWM_UP_PIN,   KNEEPWM_RETRACT_PIN,
                          HIPPWM_REVERSE_PIN,  THIGHPWM_DOWN_PIN, KNEEPWM_EXTEND_PIN};
#endif

/*
  * These are used to convert an 0-2 joint number to a valve number in
  * the pwm_pins[] array.
  */
#define OUT	0      /* Sensor value decreasing. */
#define IN	3      /* Sensor value increasing. */

const char *joint_names[]        = {"hip",     "thigh", "knee", "compliant"};
const char *joint_up_actions[]   = {"back",    "up",    "out"};
const char *joint_down_actions[] = {"forward", "down",  "in"};

int sensor_goals[3]    = {0,0,0};
float xyz_goal[3]      = {0,0,0};
float angle_goals[3]   = {0,0,0};

int goingHot[3]        = {0,0,0};

int sensor_readings[3] = {0,0,0};

int first_move_in[3]   = {   -1,    -1,    -1};
int first_move_out[3]  = {   -1,    -1,    -1};

/* The minimum PWM speed at which a valve can move a joint. */
/* XXX fixme:  These should be stored in flash. */
int valve_min_pwm_speed[6] = {-1, -1, -1, -1, -1, -1};

/*
 * Angles in degrees -
 * Hip is zero when straight out from the robot body.
 * Hip angle is zero when the thigh is parallel to the ground
 * Knee angle is between thigh and calf (13 degrees fully retracted)
 */
float current_deg[3];
float current_rad[3];

int sensorPin[3] = {HIP_SENSOR_PIN, THIGH_SENSOR_PIN, KNEE_SENSOR_PIN};
// this is the distance in sensor reading that is close enough for directed movement
// I am putting this here so we can avoid chasing our tails early in positional control
int closeEnough = 2;

int joystick_mode = 0;

//Deadman button -- with the joystick I'll be using a momentary switch
//on the panel.  It will need to be held down or jumpered to set the
//joystick as "hot".

int deadMan = 0; //JOYSTICK is OFF if pin is low (pull high to enable joystick).
int deadman_forced = 0; /* Ignore the deadman if this is set. */

//values for home position of leg
//x home is knee fully retracted (cylindar fully extended) i.e. large pot value
//y home is thigh fully up (cylindar fully retracted) i.e. small pot value
//z home is hip in the ~middle i.e. mid pot value
int homePosition[] = {900, 200, 350};

//Sensor reading to angle of joint block
//These sensor values are for the right front leg

int hipPotMax = 722;
int hipPotMin = 93;
float hipAngleMin = -40.46;
float hipAngleMax = 40.46;
float hipSensorUnitsPerDeg;

int thighPotMax = 917;
int thighPotMin = 34;
float thighAngleMin = -6;
float thighAngleMax = 84;
float thighSensorUnitsPerDeg;

int kneePotMax = 934;
int kneePotMin = 148;
float kneeAngleMin = 13;
float kneeAngleMax = 123;
float kneeSensorUnitsPerDeg;

// kinematics block
/*  Forward kinematics for stompy
    x = cos(theta1) * [L1 + L2*cos(theta2) + L3*cos(theta2 + theta3 -180deg)]
    y = x * tan(theta1)
    z = [L2 * sin(theta2)] + [L3 * sin(theta2 + theta3 - 180deg)]
*/

//leg link lengths hip, thigh, and knee

#define L1		11
#define L2		54
/* Knee to ankle */
#define L3		72

#define HIP_LEN		11
#define THIGH_LEN	54
#define KNEE_LEN	72

#define X		0
#define Y		1
#define Z		2
float current_xyz[3];

leg_info_t leg_info;

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

void print_xyz(float x, float y, float z)
{
    Serial.print("(");
    Serial.print(x);
    Serial.print("\t");
    Serial.print(y);
    Serial.print("\t");
    Serial.print(z);
    Serial.print(")");

    return;
}

int func_none(void)
{
    Serial.println("Not implemented.");

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
    int pwm;
    int percent;

    pwm = read_int();
    percent = read_int();

    set_pwm(pwm, percent);

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
    Serial.print("Debug turned ");
    Serial.println(debug_flag ? "on" : "off");

    return 0;
}

void dbg_n(int n)
{
    old_debug_flag = debug_flag;
    debug_flag = n;
}

int func_dbg(void)
{
    periodic_debug_flag = !periodic_debug_flag;

    return 0;
}

extern int current_pwms[];

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
        Serial.print(sensor_readings[i]);
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

void print_goals(void)
{
    int i;

    Serial.print("Sensor Goals:    ");
    for (i = 0;i < 3;i++) {
        Serial.print("\t");
        Serial.print(joint_names[i]);
        Serial.print("\t");
        Serial.print(sensor_goals[i]);
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
    Serial.println("");

    return;
}

int func_info(void)
{
    int i;

    Serial.println("================================================================");
    Serial.println("Current info:\n");
    Serial.print("deadMan: ");
    Serial.print(deadMan);
    Serial.print(" deadman forced: ");
    Serial.print(deadman_forced);
    Serial.print(" goingHot: ");
    for (i = 0;i < 3;i++)
        Serial.print(goingHot[i]);
    Serial.println("");

    print_current();
    Serial.println("");
    print_goals();

    Serial.println("================================================================");
    Serial.println("");

    return 0;
}

int func_deadman(void)
{
    deadman_forced = !deadman_forced;

    Serial.print("Deadman ");
    Serial.println(deadman_forced ? "disabled" : "enabled");

    return 0;
}

int func_joystick(void)
{
    return toggle_joystick_mode();
}

void print_leg_info(leg_info_t *li)
{
    int i;
    int n;

    Serial.println("\nSensor highs");
    for (i = 0;i < NR_JOINTS;i++) {
        Serial.print("\t");
        Serial.print(joint_names[i]);
        Serial.print("     \t");
        if (li->sensor_limits[i].sensor_high == 0xFFFF)
            Serial.println("N/A");
        else
            Serial.println(li->sensor_limits[i].sensor_high);
    }

    Serial.println("\nSensor lows");
    for (i = 0;i < NR_JOINTS;i++) {
        Serial.print("\t");
        Serial.print(joint_names[i]);
        Serial.print("     \t");
        if (li->sensor_limits[i].sensor_low == 0xFFFF)
            Serial.println("N/A");
        else
            Serial.println(li->sensor_limits[i].sensor_low);
    }

    Serial.println("\nPWM to speed mappings:");
    for (n = 0;n < 3;n++) {
        Serial.print(joint_names[n]);
        Serial.print(" out:\tlowest PWM: ");
        Serial.print(li->valves[n].low_joint_movement);
        Serial.print("%\t");

        for (i = 1;i <= 10;i++) {
            Serial.print(i * 10);
            Serial.print("%: ");
            if (li->valves[n].joint_speed[i] == 0xFFFF)
                Serial.print("N/A");
            else
                Serial.print(li->valves[n].joint_speed[i]);
            Serial.print("\t");
        }
        Serial.print("\n");

        Serial.print(joint_names[n]);
        Serial.print(" in: \tlowest PWM: ");
        Serial.print(li->valves[n + 3].low_joint_movement);
        Serial.print("%\t");

        for (i = 1;i <= 10;i++) {
            Serial.print(i * 10);
            Serial.print("%: ");
            if (li->valves[n].joint_speed[i] == 0xFFFF)
                Serial.print("N/A");
            else
                Serial.print(li->valves[n + 3].joint_speed[i]);
            Serial.print("\t");
        }
        Serial.print("\n");
    }

    Serial.println("\n");

    return;
}

#define EEPROM_BASE 0x14000000

void read_leg_info(leg_info_t *li)
{
    memcpy(li, (void *)EEPROM_BASE, sizeof(leg_info_t));

    Serial.print("Read ");
    Serial.print(sizeof(leg_info_t));
    Serial.println(" bytes of saved leg data.");

    /* I need something to hack in defaults for unset but needed parameters. */

    return;
}

void write_leg_info(leg_info_t *li)
{
    uint n;

    for (n = 0;n < sizeof(leg_info_t);n++)
        EEPROM.write(n, *((uint8_t *)li + n));

    return;
}

int func_flashinfo(void)
{
    leg_info_t li;

    read_leg_info(&li);

    Serial.println("Leg parameters stored in flash:");
    print_leg_info(&li);

    return 0;
}

int func_leginfo(void)
{
    Serial.println("Leg parameters in memory and maybe not yet in flash:");
    print_leg_info(&leg_info);

    return 0;
}

int func_saveflash(void)
{
    write_leg_info(&leg_info);

    Serial.println("Leg parameters written to flash.\n");

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

/*
            Serial.print("\t");
            Serial.print(n);
*/
        }
/*        Serial.println("");*/

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

int func_calibrate(void)
{
    return calibrate();
}

int func_go(void);

struct {
    const char *name;
    int (*func)();
} cmd_table[] = {
    { "bleed",     func_bleed     },
    { "calibrate", func_calibrate },
    { "deadman",   func_deadman   }, /* Ignore the deadman. */
    { "dbg",       func_dbg       }, /* Enable debug once. */
    { "debug",     func_debug     },
    { "dither",    func_none      }, /* Set the dither amount. */
    { "flashinfo", func_flashinfo }, /* Print leg parameters stored in flash. */
    { "freq",      func_none      }, /* Set PWM frequency. */
    { "go",        func_go        }, /* goto given x, y, z. */
    { "help",      func_help      },
    { "home",      func_none      }, /* Some neutral position?  Standing positioon maybe? */
    { "info",      func_info      },
    { "jtest",     func_jtest     }, /* Print joystick values for calibration. */
    { "leginfo",   func_leginfo   }, /* Print leg paramters stored in memory. */
    { "scale",     func_scale     }, /* Set max PWM value. */
    { "joystick",  func_joystick  }, /* Enable jpoystick mode. */
    { "park",      func_none      }, /* Move leg to parked position. */
    { "pwm",       func_pwm       }, /* Set a PWM. */
    { "saveflash", func_saveflash }, /* Write in-memory leg parameters to flash. */
    { "sensors",   func_sensors   }, /* Continuously read and print sensor readings. */
    { "stop",      func_stop      }, /* Stop moving. */
    { "where",     func_none      }, /* Print current x, y, z, and degrees. */
    { NULL,        NULL           }
};

char cmd_buf[128];
int cmd_len = 0;
char *cmd_ptr;

int read_int(void)
{
    int n;
    char *p;

    n = strtol(cmd_ptr, &p, 0);
    cmd_ptr = p;

    return n;
}

float read_float(void)
{
    float f;
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
        Serial.print("bash$ ");
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
        if (ch == 24) { /* ^X */
            func_dbg();
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
        Serial.print("Unknown command ");
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
    Serial.println("");

    return 0;
}

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
void inverse_kin(float *xyz, int *sense_goals, float *deg_goals)
{
    float theta1R, theta2R, theta3R; /* Goal angles in degrees. */
    float theta1, theta2, theta3; /* Goal angles in radians. */
    float hipGoal, thighGoal, kneeGoal; /* Goal sensor values. */

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
    hipGoal = ((theta1 - hipAngleMin) * hipSensorUnitsPerDeg) + hipPotMin;
    if (hipGoal < hipPotMin || hipGoal > hipPotMax) {
        Serial.println("HIP GOAL OUT OF RANGE!!");
        hipGoal = constrain(hipGoal, hipPotMin, hipPotMax);
        Serial.print("constrained goal: ");
        Serial.println(hipGoal);
    }
    goingHot[HIP] = 1;
    Serial.println("Hip going hot.");

    /* Thigh - theta2 */
    float r;
    float x1;
    if (theta1R == 0) {
        x1 = (xyz[X] - L1);
    }
    else {
        x1 = (xyz[Y]/sin(theta1R)) - L1;
    }
    x1 = abs(x1);
    float beta = atan(xyz[Z]/x1);
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
        float gama = asin(x1/r);
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
    thighGoal = thighPotMin + ((thighAngleMax - theta2) * thighSensorUnitsPerDeg);
    if (thighGoal < thighPotMin || thighGoal > thighPotMax) {
        Serial.println("THIGH GOAL OUT OF RANGE!!");
        thighGoal = constrain(thighGoal, thighPotMin, thighPotMax);
        Serial.print("constrained goal: ");
        Serial.println(thighGoal);
    }
    goingHot[THIGH] = 1;
    Serial.println("Thigh going hot.");

    /* Knee - theta3 */
    theta3R = acos( (sq(L3) + sq(L2) - sq(r)) / (float)(2*L3*L2));
    theta3 = (theta3R * 4068) / 71;
#if 0
    Serial.print("knee rad = ");
    Serial.print(theta3R);
    Serial.print(" which is from acos( ");
    Serial.print(sq(L3) + sq(L2) - sq(r));
    Serial.print(" / ");
    Serial.print((2*L3*L2));
    Serial.print(" ), or acos( ");
    Serial.print((sq(L3) + sq(L2) - sq(r)) / (float)(2*L3*L2));
    Serial.print(" ) ");

    Serial.print(" and ");
    Serial.print((2*L3*L2));
    Serial.print(" theta3 = ");
    Serial.print(theta3);
#endif
    kneeGoal= ((kneeAngleMax - theta3) * kneeSensorUnitsPerDeg) + kneePotMin;
    //kneeGoal = ((theta3 - kneeAngleMin) * kneeSensorUnitsPerDeg) + kneePotMin;
    if (kneeGoal < kneePotMin || kneeGoal > kneePotMax) {
        Serial.print("(KNEE GOAL OUT OF RANGE) ");
        // kneeGoal = constrain(kneeGoal, kneePotMin, kneePotMax);
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
void calculate_angles(int *sensors, float *degrees)
{
    degrees[HIP]   = ((sensors[HIP] - hipPotMin) / hipSensorUnitsPerDeg) + hipAngleMin;
    degrees[THIGH] = thighAngleMax - ((sensors[THIGH] - thighPotMin) / thighSensorUnitsPerDeg);
    degrees[KNEE]  = kneeAngleMax  - ((sensors[KNEE]  - kneePotMin)  / kneeSensorUnitsPerDeg);

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
 * Takes the current angles from current_rad[] and converts to the
 * current (x,y,z).
 */
void calculate_xyz(void)
{
    current_xyz[X] = cos(current_rad[HIP]) * (L1 + L2*cos(current_rad[THIGH]) + L3*cos(current_rad[THIGH] + current_rad[KNEE] - PI));
    current_xyz[Y] = current_xyz[X] * tan(current_rad[HIP]);
    current_xyz[Z] = (L2 * sin(current_rad[THIGH])) + (L3 * sin(current_rad[THIGH] + current_rad[KNEE] - PI));

    DEBUG("Current\t\tX:\t");
    DEBUG(current_xyz[X]);
    DEBUG("\tY:\t");
    DEBUG(current_xyz[Y]);
    DEBUG("\tZ:\t");
    DEBUG(current_xyz[Z]);
    DEBUGLN("");

    return;
}

/*
 * Reads the sensors and sets the x,y,z goal to the current location.
 */
void reset_current_location(void)
{
    /* Set our goal to the current position. */
    read_sensors(sensor_readings);
    calculate_angles(sensor_readings, current_deg);
    calculate_xyz();

    xyz_goal[X] = current_xyz[X];
    xyz_goal[Y] = current_xyz[Y];
    xyz_goal[Z] = current_xyz[Z];
}

void setup(void)
{
    Serial.begin(115200);
    Serial.setTimeout(10);

    /* Analog write resolution and PWM frequency. */
    analogWriteResolution(ANALOG_BITS);
    analogReadResolution(ANALOG_BITS);

    /* max PWM freqency of the motor driver board is 20kHz */
    set_pwm_freq(20000);

    //setup deadman, enable, and error digital I/O pins
    pinMode(DEADMAN_PIN, INPUT);
    pinMode(ENABLE_PIN, OUTPUT);
    digitalWrite(ENABLE_PIN, LOW);
    pinMode(ENABLE_PIN_HIP, OUTPUT);
    digitalWrite(ENABLE_PIN_HIP, LOW);

    pinMode(HIPPWM_REVERSE_PIN, OUTPUT);

    /* Copy stored leg params from eeprom. */
    memcpy(&leg_info, (void *)0x14000000, sizeof(leg_info));

    print_leg_info(&leg_info);

    //sensor units per deg
    hipSensorUnitsPerDeg = (hipPotMax - hipPotMin) / (hipAngleMax - hipAngleMin);
    thighSensorUnitsPerDeg = (thighPotMax - thighPotMin) / (thighAngleMax - thighAngleMin);
    kneeSensorUnitsPerDeg = (kneePotMax - kneePotMin) / (kneeAngleMax - kneeAngleMin);

    interrupt_setup();

    disable_leg();

    delay(2000); /* Give the serial console time to come up. */

    read_leg_info(&leg_info);

    reset_current_location();

    Serial.println("Ready to rock and roll!\n");

#if 0

    EEPROM.write(0x101, 'o');
    EEPROM.write(0x105, 'r');
    EEPROM.write(0x103, 'b');
    EEPROM.write(0x100, 'f');
    EEPROM.write(0x102, 'o');
    EEPROM.write(0x104, 'a');

    char p;

    p = EEPROM.read(0x100);
    Serial.print(p);
    p = EEPROM.read(0x101);
    Serial.print(p);
    p = EEPROM.read(0x102);
    Serial.print(p);
    p = EEPROM.read(0x103);
    Serial.print(p);
    p = EEPROM.read(0x104);
    Serial.print(p);
    p = EEPROM.read(0x105);
    Serial.print(p);
    Serial.println("");

    uint8_t *q;
    int n;

    q = (uint8_t *)0x14000000;  /* Address of flash - or FlexRam? */
    for (n = 0;n < 1024;n++) {
        if (!strncmp((char *)&q[n], "foobar", 6)) {
            Serial.print("Found it at ");
            Serial.println(n);
            break;
        }
    }

    /* 0xD150 = 53584. */

    for (int x = n - 10;x < n + 256;x++) {
        Serial.print(" ");
        if (isprint(q[x]))
            Serial.print(q[x]);
        else
            Serial.print((int)q[x]);
    }
#endif

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
        DEBUG(sensor_readings[i]);
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
        if (goingHot[i] == 0) {
            DEBUGLN(" goingHot = 0");
            continue;
        }

        DEBUG(" sensor ");
        DEBUG(i);
        DEBUG(":\t");
        DEBUG(sensor_readings[i]);
        DEBUG("\tgoal:\t");
        DEBUG(sensor_goals[i]);
        //compare sensor reading to goal and only move if not close enough
        if (abs(sensor_readings[i] - sensor_goals[i]) >= closeEnough) {
            DEBUGLN("\tJoint is not close enough.");
            if (sensor_readings[i] > sensor_goals[i]) {
                set_pwm_goal(i, 100); /* 100% is ambitious. */
/*                print_reading(i, sensor_readings[i], sensor_goals[i], joint_names[i], joint_up_actions[i]);*/
            } else if (sensor_readings[i] < sensor_goals[i]) {
                set_pwm_goal(i + 3, 100);
/*                print_reading(i, sensor_readings[i], sensor_goals[i], joint_names[i], joint_down_actions[i]);*/
            }
        } else {
            Serial.print(joint_names[i]);
            Serial.print(" joint is close enough: ");
            Serial.print(" sensor ");
            Serial.print(i);
            Serial.print(":\t");
            Serial.print(sensor_readings[i]);
            Serial.print("\tgoal:\t");
            Serial.print(sensor_goals[i]);

            /*
             * if joint was in motion and the goal was reached - turn
             * motion flag and solenoids off.
             *
             * I don't think the leg should go cold when it reaches
             * its destination.  It should set the PWMs based on the
             * desired speed, and the desired speed is 0.
             */
            goingHot[i] = 0;
            set_pwm_goal(i, 0);
            set_pwm_goal(i + 3, 0);

            Serial.println("\treached goal - going cold.");
        }
    }
}

int read_xyz(float *xyz)
{
    // look for first valid integar to be x
    xyz[X] = read_float();
    xyz[Y] = read_float();
    xyz[Z] = read_float();

    if (xyz[X] == 0) {
        xyz[X] = xyz[X] + .0001;
        //Serial.println("x = 0 so .0001 was added to avoid degenerate case");
    }

#if 0
    Serial.println("");
    Serial.print("I was given a goal of (x,y,z): ");
    print_xyz(*x, *y, *z);
    Serial.println("");
#endif

    return 0;
}

void check_deadman(void)
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
            Serial.println("Deadman off - leg disabled.");
        } else {
            enable_leg();
            Serial.println("Deadman on - leg enabled.");
        }
        deadMan = i;
    }

    return;
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
}

void loop()
{
    static int last_seconds = 0;
    int seconds;
    static float last_x_inches, last_y_inches, last_z_inches;

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
        seconds = micros() / 500000;
        if (last_seconds != seconds) {
            dbg_n(1);
            last_seconds = seconds;
        }
    }

    /* Check for any input on the console. */
    read_cmd();

    check_deadman();

    /* Read the sensors. */
    read_sensors(sensor_readings);
    /* Turn sensor readings into joint angles. */
    calculate_angles(sensor_readings, current_deg);
    /* Turn joint angles into (x,y,z). */
    calculate_xyz();

    /* Joystic mode should move the desired x,y,z, but only after I fix positional. */
    if (joystick_mode) {
        /* In joystick mode the joystick controls the PWMs. */
        do_joystick();
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
        print_xyz(current_xyz[X], currentY, currentZ);
        Serial.print(" Goal: ");
        print_xyz(xyz_goal[0], xyz_goal[1], xyz_goal[2]);
        Serial.println("");
#endif
/*
        Serial.print(" current angles (hip, thigh, knee): ");
        print_xyz(current_deg[HIP], current_deg[THIGH], current_deg[KNEE]);
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

/*
 * The 'go' command.
 *
 * Takes 3 floats for (x,y,z) goal.
 */
int func_go(void)
{
    float xyz[3];

    joystick_mode = 0;

    Serial.println("\n----------------------------------------------------------------");

    old_debug_flag = debug_flag;
    debug_flag = 1;	/* Enable debugging for one loop. */

    /* Read x,y,z from console, and update thetas if there's a new one. */
    read_xyz(xyz);
    inverse_kin(xyz, sensor_goals, angle_goals);

    xyz_goal[X] = xyz[X];
    xyz_goal[Y] = xyz[Y];
    xyz_goal[Z] = xyz[Z];

    for (int i = 0; i < 3; i++)
        goingHot[i] = 1;

    Serial.println("----------------------------------------------------------------\n");

    return 0;
}

/*
 * This makes sure the driver board is not enabled if the joystick is
 * turned off (deadMan is low).
 *
 * This sets goingHot to 0, which means the leg will not try to move
 * until it's given another goal, even if it's the same as the current
 * goal.
 */
void disable_leg()
{
    Serial.println("Disabling leg.");

    digitalWrite(ENABLE_PIN, LOW);
    digitalWrite(ENABLE_PIN_HIP, LOW);
    pwms_off();
    for (int i = 0; i < 3; i++)
        goingHot[i] = 0;
    //Serial.println("Joystick is deactivated -- all off!");
    Serial.println("Leg disabled.");
}

void enable_leg(void)
{
    digitalWrite(ENABLE_PIN, HIGH);
    digitalWrite(ENABLE_PIN_HIP, HIGH);

    Serial.println("Leg enabled.");

    return;
}
