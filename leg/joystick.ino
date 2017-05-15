/* ; -*- mode: C ;-*- */

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
    joystick_mode = !joystick_mode;

    pwms_off(); /* Stop moving the leg. */
    reset_current_location(); /* Current XYZ is new goal. */

    Serial.print("Joystick ");
    Serial.println(joystick_mode ? "enabled" : "disabled");

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
        Serial.println("");
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

    /* Adjust the PWMs based on the values read. */
    for (i = 0;i < 3;i++) {
        /*
         * The joystick midpoint is 512, but we want a PWM percentage
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
    DEBUGLN("");
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
            Serial.print(sensor_readings[i]);
            Serial.print("\t");
        }
        Serial.print("\n");

        last_micros = micros();
    }

    return;
}
