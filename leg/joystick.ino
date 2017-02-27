/* ; -*- mode: C ;-*- */

int joystick_values[3] = {0, 0, 0};

#define JOYSTICK_BITS	10
#define JOYSTICK_MID	((1 << JOYSTICK_BITS) / 2)

/* Joystick */
#define JOYSTICK_X_PIN		14
#define JOYSTICK_Y_PIN		15
#define JOYSTICK_Z_PIN		16
#define JS_X			0
#define JS_Y			1
#define JS_Z			2

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

    disable_leg(); /* Stop moving the leg. */
    reset_current_location(); /* Current XYZ is new goal. */

    Serial.print("Joystick ");
    Serial.println(joystick_mode ? "enabled" : "disabled");

    return 0;
}

/*
 * This should move the foot goal x,y,z, and the foot should move to
 * match it.
 */
void do_joystick(void)
{
    int i;
    int pwm;
    int val;

    /* Read the three joystick sensors. */
    for (i = 0;i < 3;i++)
        joystick_values[i] = read_sensor(joystick_pins[i]);

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

    return;
}
