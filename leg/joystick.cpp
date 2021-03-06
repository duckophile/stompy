/* ; -*- mode: C ;-*- */

#include "Arduino.h"
#include "pins.h"
#include "globals.h"
#include "joystick.h"
#include "pwm.h"
#include "interrupt.h"
#include "misc.h"
#include "leg.h" /* XXX remove when reset_current_location() moves to kinematics.cpp. */

int joystick_mode;
int joystick_values[3] = {0, 0, 0};

#define JOYSTICK_BITS	10
#define JOYSTICK_MID	((1 << JOYSTICK_BITS) / 2)

/*
 * 0 - forward/backward - forward = lower
 * 1 - left/right - left = lower
 * 2 - rotate -  CCW = lower
 *
 * These are out of order to match the PWM solonoid controls.
 */

int joystick_pins[3] = {JOYSTICK_Y_PIN, JOYSTICK_X_PIN, JOYSTICK_Z_PIN};

/*
 * This should also stop any current leg motion.
 */
int toggle_joystick_mode(void)
{
    int old_int_state;
    int old_leg_state;

    joystick_mode = !joystick_mode;

    pwms_off(); /* Stop moving the leg. */
    old_int_state = set_interrupt_state(0);
    old_leg_state = set_leg_state(1);
    reset_current_location(); /* Current XYZ is new goal. */

    Serial.print("Joystick ");
    Serial.print(joystick_mode ? "enabled\n" : "disabled\n");

    set_leg_state(old_leg_state);
    set_interrupt_state(old_int_state);
    return 0;
}

/* Print out the joystick readings for joystick calibration. */
int func_jtest(void)
{
    int i;
    int val;

    while (1) {
        for (i = 0;i < 3;i++) {
            val = analogRead(joystick_pins[i]);
            Serial.print(val);
            Serial.print('\t');
        }
        Serial.print('\n');
        delay(100);
        if (Serial.available() > 0) {
            Serial.read();
            break;
        }
    }

    return 0;
}

/*
 * This makes the joystick control the individiual joints.
 */
void do_joystick_joints(void)
{
    int i;
    int pwm;
    int val;
    static int last_micros;

    /* Read the three joystick sensors. */
    for (i = 0;i < 3;i++)
        joystick_values[i] = analogRead(joystick_pins[i]);

    /* Ignore a joystick value of < 10 to account for inaccorate centering. */
    for (i = 0;i < 3;i++)
        if (abs(JOYSTICK_MID - joystick_values[i]) < 20)
            joystick_values[i] = JOYSTICK_MID;

    /* Adjust the PWMs based on the values read. */
    for (i = 0;i < 3;i++) {
        /*
         * The joystick midpoint is ~512, but we want a PWM percentage
         * from 0-100 and a direction (or a valve number if not a
         * direction).
         */
        if (joystick_values[i] > JOYSTICK_MID) {
            pwm = i + 3;
            val = joystick_values[i] - JOYSTICK_MID;
        } else {
            pwm = i;
            val = JOYSTICK_MID - joystick_values[i];
        }
        /* Normalize from 0-511 joystick value to 0-100 PWM percentage. */
        val = (val * 100) / JOYSTICK_MID;

        set_pwm_goal(pwm, val);

#if 1
        DEBUG("#");
        DEBUG(i);
        DEBUG(" = ");
        DEBUG(joystick_values[i]);
        DEBUG(" =\t");
        DEBUG(val);
        DEBUG("\t");
        DEBUG("");
#endif

    }
#if 1
    DEBUG('\n');
#endif

    if (debug_flag && (micros() - last_micros) > 100000)
    {
        Serial.print("Pressure: ");
        Serial.print("Knee extend: ");
        Serial.print(analogRead(PRESSURE_SENSOR_1)); /* Knee extend */
        Serial.print("\tThigh extend:\t");
        Serial.print(analogRead(PRESSURE_SENSOR_2)); /* Thigh extend */
        Serial.print("\tThigh retract:\t");
        Serial.print(analogRead(PRESSURE_SENSOR_3)); /* Thigh retract */
        Serial.print("\tKnee retract:\t ");
        Serial.print(analogRead(PRESSURE_SENSOR_4)); /* Knee retract */

        Serial.print(" PWMs: ");
        for (i = 0;i < 6;i++) {
            Serial.print(current_pwms[i]);
            Serial.print("\t");
        }

        Serial.print("Joint sensors: : ");
        for (i = 0;i < 3;i++) {
            Serial.print(current_sensor[i]);
            Serial.print("\t");
        }
        Serial.print("\n");

        last_micros = micros();
    }

    return;
}
