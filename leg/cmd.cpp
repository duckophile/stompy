/* ; -*- mode: C ;-*- */

#include <stdint.h>
#include <Arduino.h>
#include "leg-info.h"
#include "velocity.h"
#include "misc.h"
#include "pwm.h"
#include "joystick.h"
#include "globals.h"
#include "calibrate.h"
#include "leg.h"
#include "kinematics.h"
#include "calibrate.h"
#include "interrupt.h"
#include "comm.h"

char cmd_buf[128];
int cmd_len = 0;
char *cmd_ptr;

char *read_string(void)
{
    char *string;

    while (*cmd_ptr == ' ')
        cmd_ptr++;
    string = cmd_ptr;
    if (*string == 0)
        string = NULL;
    while ((*cmd_ptr != ' ') && (*cmd_ptr != 0))
        cmd_ptr++;
    *(cmd_ptr++) = 0;

    return string;
}

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

static int func_none(void)
{
    Serial.print("ERROR:  Not implemented.\n");

    return 0;
}

/* Set PWM scaling factor. */
static int func_scale(void)
{
    int scale;

    scale = read_int();

    set_pwm_scale(scale);

    return 0;
}

/* Set PWM. */
static int func_pwm(void)
{
    int valve;
    int percent;

    valve = read_int();
    percent = read_int();

#if 0
    int pin;
    regval = read_int();
    analogWrite(pin, regval);
#endif
    set_pwm(valve, percent);

    return 0;
}

/* Set PWM frequency. */
static int func_freq(void)
{
    int freq;

    freq = read_int();

    set_pwm_freq(freq);

    return 0;
}

static int func_debug(void)
{
    if (debug_flag)
        debug_flag = 0;
    else
        debug_flag = -1; /* -1 = Permanently on. */
    Serial.print("OK:  Debug turned ");
    Serial.print(debug_flag ? "on\n" : "off\n");

    return 0;
}

static int func_dbg(void)
{
    periodic_debug_flag = !periodic_debug_flag;

    return 0;
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
    Serial.print(" PWM scale: ");
    Serial.print(get_pwm_scale());
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

    Serial.print("# Current degrees: ");
    for (i = 0; i < 3; i++) {
        Serial.print("\t");
        Serial.print(joint_names[i]);
        Serial.print("\t");
        Serial.print(current_deg[i]);
    }
    Serial.print('\n');
    Serial.print("# Current radians: ");
    for (i = 0; i < 3; i++) {
        Serial.print("\t");
        Serial.print(joint_names[i]);
        Serial.print("\t");
        Serial.print(current_rad[i]);
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
    for (i = 0; i < NR_SENSORS; i++) {
        Serial.print("\t");
        Serial.print(joint_names[i]);
        Serial.print("\t");
        Serial.print(current_sensor[i]);
    }
    Serial.print('\n');

    Serial.print("# Sensor Goals:    ");
    for (i = 0;i < NR_SENSORS;i++) {
        Serial.print("\t");
        Serial.print(joint_names[i]);
        Serial.print("\t");
        Serial.print(sensor_goal[i]);
    }
    Serial.print('\n');

    Serial.print("# PWMs:            ");
    for (i = 0;i < NR_JOINTS;i++) {
        Serial.print("\t");
        Serial.print(joint_names[i]);
        Serial.print("\t");
        Serial.print(current_pwms[i]);
        Serial.print("/");
        Serial.print(current_pwms[i + 3]);
    }
    Serial.print('\n');

    Serial.print("# Stored sensor lows/highs:\n");
    for (i = 0;i < NR_SENSORS;i++) {
        Serial.print("# ");
        Serial.print(joint_names[i]);
        Serial.print("\tLow: ");
        Serial.print(SENSOR_LOW(i));
        Serial.print("  \tHigh: ");
        Serial.print(SENSOR_HIGH(i));
        Serial.print("\n");
    }
    Serial.print("# Recent sensor lows/highs:\n");
    for (i = 0;i < NR_SENSORS;i++) {
        Serial.print("# ");
        Serial.print(joint_names[i]);
        Serial.print("\tLow: ");
        Serial.print(min_sensor_seen[i]);
        Serial.print("  \tHigh: ");
        Serial.print(max_sensor_seen[i]);
        Serial.print("\n");
    }
    Serial.print("#\n");
    Serial.print("# ISR min: ");
    Serial.print(isr_min);
    Serial.print("us, ISR max: ");
    Serial.print(isr_max);
    Serial.print("us.  ISR count: ");
    Serial.print(isr_count);
    Serial.print("\n# ================================================================\n\n");

    return 0;
}

static int func_deadman(void)
{
    pwms_off();

    deadman_forced = !deadman_forced;

    Serial.print("OK:  Deadman ");
    Serial.print(deadman_forced ? "disabled\n" : "enabled\n");

    if (deadman_forced) {
        velocity_debug++;
        velocity_debug++;
    }

    return 0;
}

static int func_joystick(void)
{
    return toggle_joystick_mode();
}

static int func_joyxyz(void)
{
    if (joystick_mode == JOYSTICK_POSITION) {
        Serial.print("OK:  Joystick mode disabled.\n\n");
        joystick_mode = JOYSTICK_OFF;
    } else {
        joystick_mode = JOYSTICK_POSITION;
        Serial.print("OK:  Joystick positional mode enabled.\n\n");
        enable_leg();
    }

    return 0;
}

static int func_flashinfo(void)
{
    leg_info_t li;

    read_leg_info(&li);

    Serial.print("# Leg parameters stored in flash:\n");
    print_leg_info(&li);

    return 0;
}

static int func_leginfo(void)
{
    Serial.print("# Leg parameters in memory and maybe not yet in flash:\n");
    print_leg_info(&leg_info);

    return 0;
}

static int func_saveflash(void)
{
    write_leg_info(&leg_info);

    Serial.print("OK:  Leg parameters written to flash.\n\n");

    return 0;
}

static int func_eraseflash(void)
{
    erase_leg_info();

    Serial.print("OK:  Flash parameters erased.\n\n");

    return 0;
}

static int func_name(void)
{
    strncpy(leg_info.name, cmd_ptr, 14);

    return 0;
}

/* Set the leg number. */
static int func_legnum(void)
{
    leg_info.leg_number = read_int();

    return 0;
}

#define CPU_RESTART_ADDR (uint32_t *)0xE000ED0C
#define CPU_RESTART_VAL 0x5FA0004
#define CPU_RESTART (*CPU_RESTART_ADDR = CPU_RESTART_VAL)

static int func_reset(void)
{
    CPU_RESTART;

    return 0;
}

#define STEST_SIZE 16384

static int func_stest(void)
{
    uint16_t readings[STEST_SIZE];
    int start_time, stop_time;
    int sensor_high;
    int sensor_low;
    int total;
    int sensor;
    int averaging;
    int n;

    for (averaging = 0;averaging < 6;averaging++) {
        Serial.print("\nAvering: ");
        Serial.print(1 << averaging);
        Serial.print('\n');

        analogReadAveraging(1 << averaging);

        for (sensor = 0;sensor < NR_SENSORS;sensor++) {
            sensor_high = 0;
            sensor_low = 1 << 30;
            total = 0;

            start_time = micros();
            for (n = 0;n < STEST_SIZE;n++)
                readings[n] = analogRead(sensorPin[sensor]);
            stop_time = micros();

            for (n = 0;n < STEST_SIZE;n++) {
                if (readings[n] > sensor_high)
                    sensor_high = readings[n];
                if (readings[n] < sensor_low)
                    sensor_low = readings[n];
                total += readings[n];
            }
            Serial.print(joint_names[sensor]);
            Serial.print("\tHigh: ");
            Serial.print(sensor_high);
            Serial.print("\tLow: ");
            Serial.print(sensor_low);
            Serial.print("\tDiff: ");
            Serial.print(sensor_high - sensor_low);
            Serial.print("\tAverage: ");
            Serial.print(total / 16384);
            Serial.print("\tRead time: ");
            Serial.print((float)(stop_time - start_time) / (float)STEST_SIZE);
            Serial.print('\n');
        }
    }

    Serial.print('\n');

    return 0;
}

/*
 * Show sensor jitter.
 *
 * Read the sensors until a key is pressed, count the number of times
 * each value is read, and print a crude histogram(ish) of the values
 * seen.
 */
static int func_sensors(void)
{
    int n;
    int i;
    int sample_count = 0;
    int sense_highs[NR_SENSORS]    = {    0,     0,     0,     0};
    int sense_lows[NR_SENSORS]     = {65535, 65535, 65535, 65535};
    int readings[8192]; /* Need to collapse 16 bits into 8192 buckets. */

    for (i = 0;i < 8192;i++)
        readings[i] = 0;

    Serial.print('\n');
    Serial.print("Sensors: ");
    while (1) {
        sample_count++;
        for (i = 0; i < NR_SENSORS; i++) {
/*          n = analogRead(sensorPin[i]);*/
            n = read_sensor(i);
            if (n < sense_lows[i]) {
#if 0
                Serial.print("New low for ");
                Serial.print(joint_names[i]);
                Serial.print(" old = ");
                Serial.print(sense_lows[i]);
                Serial.print(" new = ");
                Serial.print(n);
                Serial.print('\n');
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
                Serial.print(n);
                Serial.print('\n');
#endif

                sense_highs[i] = n;
            }

            if ((sample_count % 10) == 0) {
                Serial.print("\t");
                Serial.print(n);
            }

            n = n / 8;	/* Fit 64K samples into 8192 buckets. */
            readings[n]++;
        }
        if ((sample_count % 10) == 0) {
            Serial.print("\n");
        }

        if (Serial.available() > 0) {
            Serial.read();
            break;
        }
    }

    Serial.print(" ");
    Serial.print(sample_count);
    Serial.print(" samples.\n");
    for (i = 0;i < 8192;i++) {
        if (readings[i] != 0) {
/*            Serial.print(i * 8);*/
            Serial.print(i * 8);
            Serial.print("\t = ");
            Serial.print(readings[i]);
            Serial.print('\n');
        }
    }

    Serial.print("Low sensor readings:  ");
    for (i = 0;i < NR_SENSORS;i++) {
        Serial.print("\t");
        Serial.print(joint_names[i]);
        Serial.print("\t");
/*        Serial.print(leg_info.sensor_limits[i].sensor_low);*/
        Serial.print(sense_lows[i]);
    }
    Serial.print('\n');
    Serial.print("High sensor readings: ");
    for (i = 0;i < NR_SENSORS;i++) {
        Serial.print("\t");
        Serial.print(joint_names[i]);
        Serial.print("\t");
/*        Serial.print(leg_info.sensor_limits[i].sensor_high);*/
        Serial.print(sense_highs[i]);
    }
    Serial.print('\n');
    Serial.print("Delta                : ");
    for (i = 0;i < NR_SENSORS;i++) {
        Serial.print("\t");
        Serial.print(joint_names[i]);
        Serial.print("\t");
/*        Serial.print(leg_info.sensor_limits[i].sensor_high);*/
        Serial.print(sense_highs[i] - sense_lows[i]);
    }
    Serial.print('\n');
    Serial.print('\n');

    return 0;
}

/*
 * Stick the leg straight out and wiggle it up and down in hopes of
 * making it easier for air to get out.
 */
static int func_bleed(void)
{
    return 0;
}

static int func_stop(void)
{
    set_leg_state(0);

    return 0;
}

/*
 * Finds the lowest PWM value that gives joint movement
 * (find_joint_first_movement()) and the sensor low and high limits
 * (find_joint_sensor_limits()).
 *
 */
static int func_findlimits(void)
{
    int joint = -1;

    if (!strcmp(cmd_ptr, "hip"))
        joint = HIP;
    if (!strcmp(cmd_ptr, "thigh"))
        joint = THIGH;
    if (!strcmp(cmd_ptr, "knee"))
        joint = KNEE;

    if (joint == -1) {
        Serial.print("ERROR:  Please specify hip, thigh, or knee.\n");
        return -1;
    }

    return set_joint_limits(joint);
}

/* Find the PWM/Speed mappings for the hip. */
static int func_hipcal(void)
{
    int count;

    count = read_int();
    return calibrate_joint(HIP, count);
}

/* Find the PWM/Speed mappings for the knee. */
static int func_kneecal(void)
{
    int count;

    count = read_int();
    return calibrate_joint(KNEE, count);
}

/* Find the PWM/Speed mappings for the thigh. */
static int func_thighcal(void)
{
    int count;

    count = read_int();

    return calibrate_joint(THIGH, count);
}

static int func_enable(void)
{
    enable_leg();

    Serial.print("OK:  Leg enabled.\n");

    return 0;
}

static int func_intoff(void)
{
    set_interrupt_state(0);

    Serial.print("OK:  Interrupts disabled.\n");

    return 0;
}

/* Put random test stuff here. */
static int func_foo(void)
{
    Serial.print("Extending knee (IN).\n");
    if (move_joint_all_the_way(KNEEPWM_IN, 50) == -1)
        Serial.print("ERROR:  couldn't extend knee!\n");
    Serial.print("\n\n");
    Serial.print("Retracting knee (OUT).\n");
    if (move_joint_all_the_way(KNEEPWM_OUT, 50) == -1)
        Serial.print("ERROR:  couldn't retract knee!\n");

    return 0;
}

static int func_park(void)
{
    return park_leg();
}

static int func_setloc(void)
{
    reset_current_location();

    return 0;
}

/*
 * A speed test.  Tries random PWM values within 32 of the lowest
 * joint movement PWM value and reports joint speed.
 */
static int func_speed(void)
{
    Serial.print("Wrong command.\n");

#if 0
    int pwm;
    int joint = HIP;
    int rnd;
    int direction;
    int old_int_state;

    old_int_state = set_interrupt_state(0);

    srand(millis());

    while (1) {
        direction = IN;
        pwm = LOW_PWM_MOVEMENT(joint + direction);
        rnd = (rand() >> 1) % 50 + 3;
        if (measure_speed(HIP, direction, pwm + rnd, 1) == -1)
            break;

        delay(400);

        direction = OUT;
        pwm = LOW_PWM_MOVEMENT(joint + direction);
        rnd = (rand() >> 1) % 50 + 3;
        if (measure_speed(HIP, direction, pwm + rnd, 1) == -1)
            break;

        delay(400);
    }

    set_interrupt_state(old_int_state);
#endif

    return 0;
}

static int func_move_joint(int joint)
{
    char *direction;
    int pwm_percent;
    int dir;

    direction = read_string();
    pwm_percent = read_int();

    if ((direction == NULL) || (pwm_percent == 0)) {
        Serial.print("ERROR:  Missing arguemnts.\n");
        return -1;
    }

    if (!strcasecmp(direction, "in"))
        dir = IN;
    else if (!strcasecmp(direction, "out"))
        dir = OUT;
    else {
        Serial.print("ERROR: Invalid direction.\n");
        return -1;
    }

    measure_speed(joint, dir, pwm_percent, 0);

    return 0;
}

static int func_knee(void)
{
    return func_move_joint(KNEE);
}

static int func_hip(void)
{
    return func_move_joint(HIP);
}

static int func_thigh(void)
{
    return func_move_joint(THIGH);
}

static int func_pid(void)
{
#if 0
    Serial.print("\nOut...\n");
    pid_test(HIP, OUT, 60, 0);
    Serial.print("\nIn...\n");
    pid_test(HIP, IN, 60, 0);
    Serial.print("\nDone.\n");
#endif
    return 0;
}

static int read_xyz(double *xyz)
{
    // look for first valid integar to be x
    xyz[X] = read_double();
    xyz[Y] = read_double();
    xyz[Z] = read_double();

    /* XXX fixme:  Kludge to avoid divide by zero. */
    if (xyz[X] == 0)
        xyz[X] = 0.0001;

#if 0
    Serial.print('\n');
    Serial.print("I was given a goal of (x,y,z): ");
    print_ftuple(xyz);
    Serial.print('\n');
#endif

    return 0;
}

/*
 * The 'go' command.
 *
 * Takes 3 doubles for (x,y,z) goal.  Runs the inverse kinematics and
 * sets the goals to match the desired location.  The velocity loop
 * then picks up the new goal next time it runs.
 *
 * XXX fixme: This should also be able to take a speed.
 */
static int func_go(void)
{
    double xyz[3];

    velocity_debug++;

    joystick_mode = 0;

    old_debug_flag = debug_flag;
    debug_flag = 1;	/* Enable debugging for one loop. */

    /* Read x,y,z from console, and update thetas if there's a new one. */
    read_xyz(xyz);

    if (set_xyz_goal(xyz, 0)) {
        Serial.print("ERROR:  invalid position.\n");
        return -1;
    }

    return 0;
}

static int func_circle(void)
{
    generate_circle_point_list((point_list_t *)point_list_buf, 30);

    return 0;
}

static int func_triangle(void)
{
    generate_triangle_point_list((point_list_t *)point_list_buf, 30);

    return 0;
}

static int func_help(void);

struct {
    const char *name;
    int (*func)();
} cmd_table[] = {
    { "bleed",      func_bleed     },
    { "circle",     func_circle    }, /* Draw a circle in the air. */
    { "deadman",    func_deadman   }, /* Ignore the deadman. */
    { "dbg",        func_dbg       }, /* Enable debug once. */
    { "debug",      func_debug     }, /* Enable debugging output. */
    { "dither",     func_none      }, /* Set the dither amount. */
    { "enable",     func_enable    }, /* Enable the leg, allowing it to move. */
    { "eraseflash", func_eraseflash}, /* Erase the parameters saved in flash. */
    { "findlimits", func_findlimits}, /* Find the sensor limits for a joint. */
    { "flashinfo",  func_flashinfo }, /* Print leg parameters stored in flash. */
    { "foo",        func_foo       }, /* Whatever I want it to do. */
    { "freq",       func_freq      }, /* Set PWM frequency. */
    { "go",         func_go        }, /* goto given x, y, z. */
    { "help",       func_help      },
    { "hip",        func_hip,      }, /* Move the hip. */
    { "hipcal",     func_hipcal    }, /* Run hip calibration. */
    { "home",       func_none      }, /* Some neutral position?  Standing positioon maybe? */
    { "info",       func_info      },
    { "intoff",     func_intoff    }, /* Disable interrupts. */
    { "joystick",   func_joystick  }, /* Enable jpoystick joint control mode. */
    { "joyxyz",     func_joyxyz    }, /* Enable jpoystick positional mode. */
    { "jtest",      func_jtest     }, /* Print joystick values for calibration. */
    { "knee",       func_knee,     }, /* Move the knee. */
    { "kneecal",    func_kneecal   }, /* Run knee calibration. */
    { "leginfo",    func_leginfo   }, /* Print leg paramters stored in memory. */
    { "legnum",     func_legnum    }, /* Set the leg number. */
    { "name",       func_name      }, /* Nmme the leg. */
    { "park",       func_park      }, /* Move leg to parked position. */
    { "pid",        func_pid       }, /* For debugging PID */
    { "pwm",        func_pwm       }, /* Set a PWM. */
    { "reset",      func_reset     }, /* Reset and reboot the teensy. */
    { "saveflash",  func_saveflash }, /* Write in-memory leg parameters to flash. */
    { "scale",      func_scale     }, /* Set max PWM value. */
    { "sensors",    func_sensors   }, /* Continuously read and print sensor readings. */
    { "setloc",     func_setloc    }, /* Set the current location as the goal. */
    { "speed",      func_speed     }, /* Do speed test. */
    { "stest",      func_stest     }, /* Another sensor test. */
    { "stop",       func_stop      }, /* Stop moving. */
    { "thigh",      func_thigh     }, /* Move the thigh all the way. */
    { "thighcal",   func_thighcal  }, /* Run thigh calibration. */
    { "triangle",   func_triangle  }, /* Draw a triangle in the air. */
    { "where",      func_none      }, /* Print current x, y, z, and degrees. */
    { NULL,         NULL           }
};

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
            Serial.print(">");
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
            Serial.print('\n');
            disable_leg();
            continue;
        }
        if (ch == 22) { /* ^V */
            Serial.print("\n# Toggling velocity debugging output.\n");
            toggle_velocity_debug();
            continue;
        }
        if (ch == 24) { /* ^X */
            Serial.print('\n');
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

        if ((ch == '\r') || (ch == '\n')) {
            Serial.print('\n');
            break;
        }

        if (ch < 0x20) /* Ignore other control characters. */
            continue;

        if (cmd_len > 127)
            continue;

        Serial.print((char)ch);

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
        Serial.print("ERROR:  Unknown command ");
        Serial.print(cmd_buf);
        Serial.print('\n');

    }

    cmd_len = 0;

    return;
}

static int func_help(void)
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
