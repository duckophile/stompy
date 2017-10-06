/* ; -*- mode: C ;-*- */

#include "Arduino.h"
#include <stdint.h>
#include "leg-info.h"
#include "globals.h"
#include "pwm.h"

/*
 * Note that there are potential race conditions if calls are made
 * into this module while the interrupt-driven routine is running.
 */
static int pwm_goals[6] = {0};
static int pwm_scale = 60;

int current_pwms[NR_VALVES] = { 0xFFFFFF, 0xFFFFFF, 0xFFFFFF, 0xFFFFFF, 0xFFFFFF, 0xFFFFFF};

void set_pwm_freq(int freq)
{
    //max PWM freqency of the motor driver board is 20kHz
    analogWriteFrequency(3, freq);
    analogWriteFrequency(5, freq);

    return;
}

int get_pwm_scale(void)
{
    return pwm_scale;
}

/*
 * Sets a scaling factor for all values written to the PWMs, from
 * 0-100%.
 */
int set_pwm_scale(int scale)
{
    int old_scale = pwm_scale;

    pwm_scale = scale;

    Serial.print("PWM scaling factor set to ");
    Serial.print(pwm_scale);
    Serial.print('\n');

    return old_scale;
}

/*
 * This writes the value to the PWM, with no slow-start.
 *
 * The value is a percentage (0-100), and is saved in current_pwms[].
 *
 * The PWM numbering is wacky, since there are six PWMs for three
 * joints.  This takes a valve number (0-5) instead of a joint number
 * (0-2).
 */
void set_pwm(int valve, int percent)
{
    int normalized;
    int other_valve;

    if (percent > 100) {
        Serial.print("\nERROR: PWM percent ");
        Serial.print(percent);
        Serial.print(" is invalid.\n\n");
        return;
    }
    if ((valve > 5) || (valve < 0)) {
        Serial.print("\nERROR: pwm_id ");
        Serial.print(valve);
        Serial.print(" is invalid!\n\n");
        return;
    }

    /* Turn the opposing valve off. */
    if (valve >= 3)
        other_valve = valve - 3;
    else
        other_valve = valve + 3;
    analogWrite(pwm_pins[other_valve], 0);
    current_pwms[other_valve] = 0;

    normalized = (percent * PWM_MAX) / 100; /* Normalize to the analog resolution. */
    if (normalized > PWM_MAX)
        normalized = PWM_MAX;

#if 0
    /* Write the normalized PWM percent to the PWM. */
    if (normalized == 0) {
        Serial.print("Writing 0 to PWM ");
        Serial.print(valve);
        Serial.print("\n");
        analogWrite(pwm_pins[valve], 0);
    }
#endif

    if ((normalized == 0) || (deadMan || !deadman_forced != 0)) {
#if 0
        Serial.print("PWM ");
        Serial.print(valve);
        Serial.print(" pin ");
        Serial.print(pwm_pins[valve]);
        Serial.print(" set to ");
        Serial.print(normalized);
        Serial.print('\n');
#endif
        if (normalized != 0) {
#if 0
            Serial.print("Writing ");
            Serial.print(normalized);
            Serial.print(" to PWM ");
            Serial.print(valve);
            Serial.print(" (");
            Serial.print(percent);
            Serial.print("% requested)\n");
#endif
        }
        analogWrite(pwm_pins[valve], normalized);
    }

    if ((current_pwms[valve] != percent) && (percent != 0)) {
        DEBUG("PWM ");
        DEBUG(valve);
        DEBUG(" pin ");
        DEBUG(pwm_pins[valve]);
        DEBUG(" normalized from ");
        DEBUG(percent);
        DEBUG(" to ");
        DEBUG(normalized);
        DEBUG('\n');
    }
    /* Save the requested PWM percentage, not scaled or normalized. */
    current_pwms[valve] = percent;

    return;
}


/*
 * Set a goal for a particular PWM.  The PWM is accelerated towards
 * that goal.  XXX fixme:  No, it's not currently accelerated.
 *
 * pwm_id is 0-5, value is percentage - 0-100.
 */

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
        Serial.print("ERROR PWM percentage ");
        Serial.print(value);
        Serial.print(" is invalid for PWM ");
        Serial.print(pwm_id);
        Serial.print('\n');
        return;
    }

    if ((pwm_id > 5) || (pwm_id < 0)) {
        Serial.print("ERROR: pwm_id ");
        Serial.print(pwm_id);
        Serial.print(" is invalid!\n");
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
 * Percentage is a desired percentage of the PWM usable range - for
 * most valves this is ~40% PWM to 100% PWM.  This is scaled to an
 * absolute PWM value, which is written to the PWM.
 */
void set_scaled_pwm_goal(int pwm_id, int percentage)
{
    int usable_range;
    int ljm;
    int scaled_percentage;

    ljm = leg_info.valves[pwm_id].low_joint_movement;
    usable_range = 100 - ljm; /* probably around 60. */

    /* Scale PWM for global PWM scaling factor. */
    percentage = (percentage * pwm_scale) / 100;

    /* Fit the percentage to the usable range of the PWM. */
    scaled_percentage = (percentage * usable_range) / 100;
    /* And move it to the proper place of thge scale. */
    scaled_percentage += ljm;

#if 0
    Serial.print("Scaling pwm ");
    Serial.print(pwm_id);
    Serial.print(" desired percentage ");
    Serial.print(percentage);
    Serial.print("% with scaling factor of ");
    Serial.print(pwm_scale);
    Serial.print("% to fit usable range of ");
    Serial.print(usable_range);
    Serial.print(" and got absolute PWM ");
    Serial.print(scaled_percentage);
    Serial.print("%.\n");
#endif

    set_pwm_goal(pwm_id, scaled_percentage);

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

    for (i = 0;i < 6;i++)
        set_pwm_goal(i, 0);
    for (i = 0;i < 6;i++)
        set_pwm(i, 0);

    return;
}
