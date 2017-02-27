/* ; -*- mode: C ;-*- */

/*
 * Note that there are potential race conditions if calls are made
 * into this module while the interrupt-driven routine is running.
 */
static int pwm_goals[6] = {0};
int current_pwms[6] = {0};
static int pwm_scale = 40;

void set_pwm_freq(int freq)
{
    //max PWM freqency of the motor driver board is 20kHz
    analogWriteFrequency(3, freq);
    analogWriteFrequency(5, freq);

    return;
}

/*
 * Sets a scaling factor for all values written to the PWMs, from
 * 0-100%.
 */
void set_pwm_scale(int scale)
{
    pwm_scale = scale;

    Serial.print("PWM scaling factor set to ");
    Serial.println(pwm_scale);

    return;
}

/*
 * This writes the value to the PWM, with no slow-start.
 *
 * The value is a percentage - 0-100.
 *
 * The PWM numbering is wacky, since there are six PWMs for three
 * joints.  This takes a valve number (0-5) instead of a joint number
 * (0-2).
 */
void set_pwm(int pwm_id, int value)
{
    int scaled;

    if (value > 100) {
        Serial.print("PWM value ");
        Serial.print(value);
        Serial.println(" is invalid.");
        return;
    }

    if ((pwm_id > 5) || (pwm_id < 0)) {
        Serial.print("pwm_id ");
        Serial.print(pwm_id);
        Serial.println(" is invalid!");
        return;
    }

#if 0
    Serial.print("Value in: ");
    Serial.print(value);
#endif

    scaled = (value * 1024) / 100; /* Normalize to 10 bits. */

#if 0
    Serial.print(" 10 bit value: ");
    Serial.print(scaled);
#endif

    scaled = (scaled * pwm_scale) / 100; /* Scale. */
#if 0
    Serial.print(" scaled value: ");
    Serial.println(scaled);
#endif

    /* Write the normalized, scaled PWM value to the PWM. */
    if ((value == 0) || (deadMan != 0) || (!deadman_forced != 0)) {
#if 0
        Serial.print("PWM ");
        Serial.print(pwm_id);
        Serial.print(" pin ");
        Serial.print(pwm_pins[pwm_id]);
        Serial.print(" set to ");
        Serial.println(scaled);
#endif
        analogWrite(pwm_pins[pwm_id], scaled);
    }

    if ((current_pwms[pwm_id] != value) && (value != 0)) {
        DEBUG("PWM ");
        DEBUG(pwm_id);
        DEBUG(" pin ");
        DEBUG(pwm_pins[pwm_id]);
        DEBUG(" scaled from ");
        DEBUG(value);
        DEBUG(" to ");
        DEBUGLN(scaled);
    }
    /* Save the requested PWM percentage, not scaled or normalized. */
    current_pwms[pwm_id] = value;

    return;
}

/*
 * Set a goal for a particular PWM.  The PWM is accelerated towards
 * that goal.  XXX fixme:  No, it's not currently accelerated.
 *
 * pwm_id is 0-5, value is percentage - 0-100.
 */

/* XXX fixme:  This should take a percentage. */
void set_pwm_goal(int pwm_id, int value)
{
    int other_pwm;

/*
    DEBUG("set_pwm_goal(id = ");
    DEBUG(pwm_id);
    DEBUG(", value = ");
    DEBUG(value);
    DEBUGLN(")");
*/

    if (value > 100) {
        Serial.print("PWM value ");
        Serial.print(value);
        Serial.println(" is invalid.");
        return;
    }

    if ((pwm_id > 5) || (pwm_id < 0)) {
        Serial.print("pwm_id ");
        Serial.print(pwm_id);
        Serial.println(" is invalid!");
        return;
    }

    /*
     * If we're increasing the PWM percentage then set the PWM goal
     * and let it accelerate slowly.
     *
     * If we're decreasing the PWM percentage then set it immediately
     * to avoid overshooting the target.
     */

    pwm_goals[pwm_id] = value;

    if (current_pwms[pwm_id] != value)
        set_pwm(pwm_id, value);

    /* Make sure the opposing PWM is off. */
    if (pwm_id < 3)
        other_pwm = 3;
    else
        other_pwm = -3;

    if (current_pwms[pwm_id + other_pwm] != 0) {
        pwm_goals[pwm_id + other_pwm] = 0;
        set_pwm(pwm_id + other_pwm, 0);
    }

    return;
}

/*
 * Acceleration controls for the PWMs.  This is called from the
 * interval timer and increments the PWM values one step closer to the
 * goal.
 */
void adjust_pwms(void)
{
#if 0
    int i;
    int diff;

    for (i = 0;i < 6;i++) {
        diff = pwm_goals[i] - current_pwms[i];
        if (diff > 0)
            set_pwm(i, current_pwms[i] + 1);
        else if (diff < 0)
            set_pwm(i, pwm_goals[i]);
    }
#endif
    return;
}

void pwms_off(void)
{
    int i;

    Serial.println("Turning all PWMs off.");

    for (i = 0;i < 6;i++)
        set_pwm_goal(i, 0);
    for (i = 0;i < 6;i++)
        set_pwm(i, 0);

    return;
}
