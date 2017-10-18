/* ; -*- mode: C ;-*- */

#include <Arduino.h>
#include "leg-info.h"
#include "pwm.h"
#include "leg.h"
#include "pwm.h"
#include "misc.h"
#include "interrupt.h"
#include "calibrate.h"

/*
 * Objective:
 *
 * - Collect min and max sensor values, angles, (x,y,z).
 * - Collect PWM to speed mapping data for each joint, at high (2000?
 *   2500?) and low (1000?  1500?) pressure.
 * - Save data in flash on the Teensy.
 * - Verify that the results of a calibration match previouws results
 *   stored in flash.
 * - Measure sensor noise.
 *
 * Sensor jitter can be collected in motion by looking for sensor
 * values indicating movement opposite to actual movement.  This
 * should give a best guess for a "close enough" value.
 *
 * Calibration procedure:
 *
 * - Move thigh all the way up, then knee all the way down.  Take
 *   thigh and knee sensor readings.  The thigh cannot be moved all
 *   the way down due to ground interference, so take a "best guess"
 *   reading at the ground interference point.
 * - Move hip all the way left and all the way right.  Take sensor
 *   reading.  Move hip to center.
 * - Move knee all the way up.  Take sensor reading.  Move knee back
 *   down.
 *
 */

#warning I need convert units to inches/sec of foot travel!
/*
 * I THINK I need inches/sec of foot travel, with the foot traveling
 * in an arc.  Maybe I can add up discrete distance measurements as
 * the foot moves through its calibration arc?
 *
 *
 */

int check_keypress(void)
{
    if (Serial.available() > 0) {
        (void)Serial.read();
        set_pwm_scale(60);
        pwms_off();
        disable_leg();
        Serial.print("\n# Aborted by keypress.\n\n");
        return 1;
    }

    return 0;
}

void capped_pwm_write(int valve, int percent)
{
    int pwm_val;

    pwm_val = (percent * PWM_MAX) / 100;
    if (pwm_val > PWM_MAX)
        pwm_val = PWM_MAX;
    if (pwm_val < 0)
        pwm_val = 0;

    set_pwm(valve, percent);

#if 0
    Serial.print("Wrote ");
    Serial.print(pwm_val);
    Serial.print(" to valve ");
    Serial.print(valve);
    Serial.print(" pin ");
    Serial.print(pwm_pins[valve]);
    Serial.print("\n");
#endif

    return;
}

/*
 * Move a joint associated with a specified valve until it stops
 * moving.
 *
 * Returns the last sensor reading from the joint, or -1 on failure.
 *
 * The valve number is 0-5.
 *
 * This should take a joint (0-2) and a direction.
 *
 * This is fairly crude.
 *
 * XXX fixme: Looks like this isn't moving the thigh fully in against
 * the stop?
 */
int move_joint_all_the_way(int valve, int pwm_percent)
{
    int out = 0;
    int no_change_count = 0;
    int sensor_num;
    int sensor_reading;
    int last_sensor_reading = 0;
    int failed = 0;
    int i;

    pwms_off();

    if (!check_deadman())
        return -1;

    sensor_num = valve;
    /* Sensor number is 0-2. */
    if (sensor_num > 2) {
        sensor_num = sensor_num - 3;
        out = 0;
        Serial.print("IN - Sensor value should be decreasing.\n");
    } else {
        out = 1;	/* Sensor reading should be going up?  Hopefully? */
        Serial.print("OUT - Sensor value should be increasing.\n");
    }

    Serial.print("Setting valve ");
    Serial.print(valve);
    Serial.print(" PWM pin ");
    Serial.print(pwm_pins[valve]);
    Serial.print("\tsensor ");
    Serial.print(sensor_num);
    Serial.print(" sensor pin ");
    Serial.print(sensorPin[sensor_num]);
    Serial.print('\n');

    /* Set the PWM to move the joint. */
    capped_pwm_write(valve, pwm_percent);

    /* Look for movement, wait until it stops. */
    sensor_reading = read_sensor(sensor_num);
    last_sensor_reading = sensor_reading;
    /* 600 readings of 100ms each. */
    for (i = 0;i < 600;i++) {
        if (!check_deadman()) {
            failed = 1;
            break;
        }

        if (check_keypress()) {
            failed = 1;
            break;
        }

        no_change_count++;
        sensor_reading = read_sensor(sensor_num);

        /*
         * Save the largest or smallest sensor reading, depending on
         * which way the joint is moving.
         */
        if (out) {
            /* Joint going out, sensor reading going up. */
            if (sensor_reading > last_sensor_reading) {
                last_sensor_reading = sensor_reading;
                no_change_count = 0;
            }
        } else {
            /* Joint going in, sensor reading going down. */
            if (sensor_reading < last_sensor_reading) {
                last_sensor_reading = sensor_reading;
                no_change_count = 0;
            }
        }

        if (no_change_count) {
            Serial.print("Stopped: ");
            Serial.print(no_change_count);
            Serial.print(" ");
        }
        Serial.print(sensor_reading);
        Serial.print(" ");

        /*
         * If we've been through this loop 15 times without movement
         * then we're at the end.
         */
        if (no_change_count > 15)
            break;

        delay(100);
    }

    Serial.print("\n");

    pwms_off();

    if (i == 600) {
        Serial.print("Leg didn't stop moving!\n");
        return -1;
    } else if (failed) {
        Serial.print("Aborted by keypress or deadman?\n");
        return -1;
    } else
        return last_sensor_reading;
}

/* I need a function to test for movement. */

#define ACCELERATING	0
#define FULL_SPEED	1
#define DECELERATING	2

/* If the joint hasn't moved after this many readings then it's not moving. */
/* XXX fixme:  I think this should be a lot lower - 100?  50?  20? */
/* #define NO_CHANGE_LIMIT		200 */
/* #define NO_CHANGE_LIMIT		40 times out too fast for some joints */
#define NO_CHANGE_LIMIT		100

/*
 * Move a joint in a direction at a specific PWM percentage.
 * Accelerate and decelerate smoothly(ish) at the ends of travel,
 * detect whether or not the joint's moving, record high and low
 * sensor values, etc.
 *
 * This is currently a bit scatterbrained.
 *
 * XXX fixme:  The acceleration and decceleration need work!
 *
 * I need to be able to tell this function to quit once it's detected
 * the leg moving.
 *
 * And detect movement based on total sensor movement rather than just
 * some number of iterations without moving?  Or at least report total
 * sensor movement.
 *
 * And, try different PWM frequencies.
 *
 * And, I need some way to overcome stiction.  Maybe every time the
 * PWM is changed it should be changed to a radically different value
 * (0 or 100?) for a 100th of a second then changed to the desired
 * value?  Or at least slam it once when it hits the destination
 * percentage?
 *
 */

/*
 * This is pointlessly contorted - measure_speed() is
 * better and does the same stuff.
 *
 * XXX fixme: This doesn't appear to be decelerating on the IN stroke
 * (for the knee and thigh anyway, the hip might be).
 */

int exercise_joint(int joint, int direction, int pwm_goal, int verbose)
{
    int sensor_reading;
    int i;
    int current_pwm;
    int valve;
    int start_millis = 0;
    int start_sensor = 0;
    int stop_millis = 0;
    int stop_sensor = 0;
    int state = ACCELERATING;
    int no_change_count = 0;
    int last_sensor_reading;
#if 0
    int real_last_sensor_reading;
#endif
    int pwm_inc;
    int low_percent;
    int failed = 0;
    int no_movement = 0;
    int speed;
    int total_millis;
    int total_sensor;
    int high_sensor_decel, low_sensor_decel;
    int sensor_high_end, sensor_low_end;
    int no_change_limit;
    int32_t this_time, last_time;

    Serial.print("----------------------------------------------------------------\n");
    pwms_off();

    if ((LOW_PWM_MOVEMENT(joint + direction) == 0) || (LOW_PWM_MOVEMENT(joint + direction) == 0)) {
        Serial.print("# ****************************************************************\n");
        Serial.print("ERROR:  LOW_PWM_MOVEMENT == ");
        Serial.print(LOW_PWM_MOVEMENT(joint + direction));
        Serial.print('\n');
        Serial.print("# ****************************************************************\n");
        return -1;
    }

    /* Accelerate from the lowest PWM value that gives movement. */
    current_pwm = LOW_PWM_MOVEMENT(joint + direction);
    if (current_pwm > pwm_goal)
        current_pwm = pwm_goal;

    /* Decelerate down to this value when stopping. */
    low_percent = LOW_PWM_MOVEMENT(joint + direction) + 5;
    if (low_percent > pwm_goal)
        low_percent = pwm_goal;

    /* Start accelerating/decelerating this far from the end of travel. */
    /*
     * XXX fixme: The distance from the end of travel should be based
     * on the difference between the desired PWM and LOW_PWM_MOVEMENT.
     */
    high_sensor_decel = SENSOR_HIGH(joint) - 2400; /* When to start decelerating. */
    low_sensor_decel  = SENSOR_LOW(joint)  + 2400; /* When to start decelerating. */
    sensor_high_end   = SENSOR_HIGH(joint) - 1200; /* close enough to the end of travel. */
    sensor_low_end    = SENSOR_LOW(joint)  + 1200;

    /* How much to increment/decrement the PWM at once when accelerating/decelerating. */
    pwm_inc = 5;

    /* XXX fixme:  This should only be printed if verbose. */
    Serial.print("# Exercising ");
    Serial.print(joint_names[joint]);
    Serial.print(direction == IN ? " in " : " out ");
    Serial.print(pwm_goal);
    Serial.print(" percent PWM (which might be scaled by the PWM module.\n");

    valve = joint + direction;

    /* Sensor number is 0-2. */
#if 0
    if (verbose) {
        if (direction == OUT) {
            Serial.print("# Sensor value should be increasing, looking for high value of ");
            Serial.print(sensor_high_end);
        } else {
            Serial.print("# Sensor value should be decreasing, looking for low value of ");
            Serial.print(sensor_low_end);
        }
        Serial.print('\n');

        Serial.print("# exercising valve ");
        Serial.print(valve);
        Serial.print(" PWM pin ");
        Serial.print(pwm_pins[valve]);
        Serial.print(" sensor ");
        Serial.print(joint);
        Serial.print(" sensor pin ");
        Serial.print(sensorPin[joint]);
        Serial.print('\n');
    }
#endif

    sensor_reading = read_sensor(joint);
    last_sensor_reading = sensor_reading;
#if 0
    real_last_sensor_reading = sensor_reading;
#endif
    Serial.print("# Starting at sensor reading ");
    Serial.print(sensor_reading);
    Serial.print('\n');

    this_time = millis();
    last_time = 0;

    Serial.print("# Writing PWM percentage ");
    Serial.print(current_pwm);
    Serial.print('\n');

    if (joint == KNEE)
        no_change_limit = 300;	/* Knee needs longer due to gravity crossing. */
    else
        no_change_limit = 100;

    for (i = 0;i < 3000;i++) {
        if (!check_deadman()) {
            failed = 1;
            break;
        }

        if (check_keypress()) {
            Serial.print("\n\n**** Aborted by keypress\n\n");
            goto fail;
        }

#if 0
        Serial.print("Writing ");
        Serial.print(current_pwm);
        Serial.print("% to pwm pin ");
        Serial.print(pwm_pins[valve]);
        Serial.print("\n");
#endif
        capped_pwm_write(valve, current_pwm);

        /* Only run through this loop every 10ms. */ /* XXX fixme: This might break if millis wraps! */
        /*
         * I delay this way instead of using delay to get accurate
         * 10ms timing regardless of how long the work in this loop
         * takes.
         */
        while ((this_time - last_time) < 10)
            this_time = millis();
        last_time = this_time;

        sensor_reading = read_sensor(joint);

        /* Save min & max sensor values.  0xFFFF means blank flash. */
        if ((direction == IN) && sensor_reading < SENSOR_LOW(joint)) {
#if 0
            Serial.print("**** Set SENSOR_LOW from ");
            Serial.print(SENSOR_LOW(joint));
            Serial.print(" to ");
            Serial.print(sensor_reading);
            Serial.print('\n');
#endif
            SENSOR_LOW(joint) = sensor_reading;
        }
        if ((direction == OUT) &&
            ((sensor_reading > SENSOR_HIGH(joint)) || (SENSOR_HIGH(joint) == 0xFFFF))) {
#if 0
            Serial.print("**** Set SENSOR_HIGH from ");
            Serial.print(SENSOR_HIGH(joint));
            Serial.print(" to ");
            Serial.print(sensor_reading);
            Serial.print('\n');
#endif
            SENSOR_HIGH(joint) = sensor_reading;
        }

        /* Check if the joint's moving. */
        no_change_count++;
        if (((direction == IN)   && (sensor_reading < last_sensor_reading)) ||
            ((direction == OUT)  && (sensor_reading > last_sensor_reading))) {
            last_sensor_reading = sensor_reading;
            no_change_count = 0;
        }
        if (no_change_count > no_change_limit) {
            Serial.print("\n# Joint is not moving at ");
            Serial.print(current_pwm);
            Serial.print("% pwm.\nAborting.\n\n");
            no_movement = 1;
            break;
        }

        /*
         * Check to see if the sensor has passed the high or low
         * limit.  The actuator should not be allowed to hit the stops
         * at the end of travel.
         */

        /* Check to see if we're close enough to the end of travel. */
        if (((direction == IN)  && (sensor_reading < sensor_low_end)) ||
            ((direction == OUT) && (sensor_reading > sensor_high_end))) {
#if 0
            Serial.print("# Reached end of travel at sensor reading ");
            Serial.print(sensor_reading);
            Serial.print('\n');
            Serial.print(direction == IN ? "# IN " : "# OUT ");
            Serial.print("sensor_high_end ");
            Serial.print(sensor_high_end);
            Serial.print(" sensor_low_end ");
            Serial.print(sensor_low_end);
            Serial.print(" sensor = ");
            Serial.print(sensor_reading);
            Serial.print('\n');
#endif

            break;
        }

        /* This handles the transition from FULL_SPEED to DECELERATING. */
        if (state <= FULL_SPEED) {
            if (((direction == IN)  && (sensor_reading < low_sensor_decel)) ||
                ((direction == OUT) && (sensor_reading > high_sensor_decel))) {
                state = DECELERATING;
                stop_millis = millis();
                stop_sensor = sensor_reading;
                Serial.print("# Decelerating from ");
                Serial.print(current_pwm);
                Serial.print("%\n");
            }
        }

        /* This handles acceleration and deceleration. */
        switch(state) {
        case ACCELERATING:
            current_pwm += pwm_inc;
            if (current_pwm >= pwm_goal) {
                Serial.print("current_pwm = ");
                Serial.print(current_pwm);
                Serial.print(", goal = ");
                Serial.print(pwm_goal);
                Serial.print('\n');
                current_pwm = pwm_goal;
                state = FULL_SPEED;
                start_millis = millis();
                start_sensor = sensor_reading;
                Serial.print("# Hit full PWM at sensor reading ");
                Serial.print(start_sensor);
                Serial.print('\n');
            }
            break;
        case FULL_SPEED:
            break;
        case DECELERATING:
            /* Decelerate. */
            current_pwm -= pwm_inc;
            if (current_pwm < low_percent)
                current_pwm = low_percent;
            break;
        }

#if 0
        /* This is just debugging output. */
        if (verbose) {
            if ((i % 10) == 0) {
                Serial.print(i);
                Serial.print('\t');
                Serial.print(direction == IN ? "IN\t" : "OUT\t");
                Serial.print(sensor_reading);
                Serial.print('\t');
                Serial.print(sensor_reading - real_last_sensor_reading);
                Serial.print('\t');
                Serial.print(current_pwm);

#if 0
                {
                    Serial.print("%\t");
                    Serial.print(" Pressure: ");
                    Serial.print(analogRead(PRESSURE_SENSOR_1));
                    Serial.print("\t");
                    Serial.print(analogRead(PRESSURE_SENSOR_2));
                    Serial.print("\t");
                    Serial.print(analogRead(PRESSURE_SENSOR_3));
                    Serial.print("\t");
                    Serial.print(analogRead(PRESSURE_SENSOR_4));
                }
#endif

                Serial.print('\n');
                real_last_sensor_reading = sensor_reading;
            }
        }
#endif

        if (check_keypress()) {
            failed = 1;
            break;
        }
    }

    pwms_off();

    total_millis = stop_millis - start_millis;
    total_sensor = abs(stop_sensor - start_sensor);
/*    speed = total_millis / total_sensor;*/
    speed = (total_sensor * 1000) / total_millis;
    if ((stop_sensor == 0) || (stop_millis == 0))
        speed = 0;

    /* I want sensor movement/sec */

#if 0
    if (verbose) {
        Serial.print("# Done.\n");

        Serial.print("# Start sensor reading: ");
        Serial.print(start_sensor);
        Serial.print(", end sensor reading: ");
        Serial.print(stop_sensor);
        Serial.print(", total sensor movement: ");
        Serial.print(total_sensor);

        Serial.print("\n# Total milliseconds at full speed: ");
        Serial.print(total_millis);

        Serial.print("\n# Speed - sensor_units/sec at  ");
        Serial.print(pwm_goal);
        Serial.print("% pwm: ");
        Serial.print(speed);
        Serial.print('\n');

        if (failed)
            Serial.print("ERROR:  **************** FAILED ****************\n");
        if (no_movement)
            Serial.print("ERROR:  **************** JOINT DID NOT MOVE ****************\n");
    }

#endif
    Serial.print("----------------------------------------------------------------\n");
    Serial.print("# Done: ");
    if (failed) {
        Serial.print("Failure.\n");
        return -1;
    }
    if (no_movement) {
        Serial.print("Joint did not move.\n");
        return 0;
    }

    if (speed == 0) {
        Serial.print("**** I'm not sure this should be called success!\n");
    }

    Serial.print("Success: PWM = ");
    Serial.print(pwm_goal);
    Serial.print("%, Speed = ");
    Serial.print(speed);
    Serial.print('\n');

    return speed;

fail:
    pwms_off();
    Serial.print("Failed.\n");
    return -1;
}

/*
 * Measures the speed of a joint in a given direction.  The speed, in
 * sensor_units/sec, is returned.
 *
 * The joint is accelerated from the low_pwm_movement value to the
 * desired PWM, and is decelerated at the end of travel.  The speed is
 * measured only for the portion of travel that's at full speed.
 *
 * The joint is assumed to already be in an appropriate position for
 * this test.
 *
 * The acceleration and deceleration are still too abrupt - mostly
 * acceleration.
 *
 * This needs to be able to detect a non-moving joint!
 */

int measure_speed(int joint, int direction, int pwm_goal, int verbose)
{
    static int run_count = 0;
    int sensor_reading;
    int rc = 0;
    int i;
    int current_pwm;
    int valve;
    int start_millis = 0;
    int start_sensor = 0;
    int stop_millis = 0;
    int stop_sensor = 0;
    int low_pwm;
    int speed;
    int total_millis;
    int total_sensor;
    int high_sensor_decel, low_sensor_decel;
    int last_sensor;
    int old_pwm_scale;
    int old_int_state;
    uint32_t this_time, last_time;

    pwms_off();

    Serial.print("\n");
    run_count++;

    old_int_state = set_interrupt_state(0);

    old_pwm_scale = set_pwm_scale(100);
    enable_leg();

    /* Accelerate from the lowest PWM value that gives movement. */
    current_pwm = LOW_PWM_MOVEMENT(joint + direction);
    if (current_pwm > pwm_goal)
        current_pwm = pwm_goal;

    /* Decelerate down to this value when stopping. */
    low_pwm = LOW_PWM_MOVEMENT(joint + direction) + 5;
    if (low_pwm > pwm_goal)
        low_pwm = pwm_goal;

    /* Start accelerating/decelerating this far from the end of travel. */
    /*
     * XXX fixme: The distance from the end of travel should be based
     * on the difference between the desired PWM and LOW_PWM_MOVEMENT.
     * I don't think 30 is high enough for higher PWMs.
     */
#if 0
    Serial.print("SENSOR_HIGH = ");
    Serial.print(SENSOR_HIGH(joint));
    Serial.print(" SENSOR_LOW = ");
    Serial.print(SENSOR_LOW(joint));
    Serial.print('\n');
#endif

    high_sensor_decel = SENSOR_HIGH(joint) - 2400; /* When to start decelerating. */
    low_sensor_decel  = SENSOR_LOW(joint)  + 2400; /* When to start decelerating. */

    /* XXX fixme:  This should only be printed if verbose. */
    Serial.print("# Measuring speed of ");
    Serial.print(joint_names[joint]);
    Serial.print(direction == IN ? " in " : " out ");
    Serial.print(pwm_goal);
    Serial.print(" percent PWM (which might be scaled by the PWM module).\n");

    valve = joint + direction;

    /* out = decreasing sensor reading. */

    sensor_reading = read_sensor(joint);

    this_time = millis();
    last_time = this_time;

    Serial.print("# Accelerating from sensor reading ");
    Serial.print(sensor_reading);
    Serial.print("...\n");
#if 0
    Serial.print("Looking for ");
    Serial.print(direction == IN ? "low" : "high");
    Serial.print(" end of ");
    Serial.print(direction == IN ? low_sensor_decel : high_sensor_decel);
    Serial.print(" compared to current ");
    Serial.print(sensor_reading);
    Serial.print('\n');
#endif

    /* Accelerate. */
    for (i = 0;i < 30000;i++) {
        capped_pwm_write(valve, current_pwm);

        current_pwm++;
        if (current_pwm >= pwm_goal) {
            current_pwm = pwm_goal;
            break;
        }

        /* Only run through this loop every X milliseconds. */
        while ((this_time - last_time) < 2)
            this_time = millis();
        last_time = this_time;

        /* Check to see if we're too close to the end of travel. */
        if (((direction == IN)  && (sensor_reading < low_sensor_decel)) ||
            ((direction == OUT) && (sensor_reading > high_sensor_decel))) {
            Serial.print("# Prematurely reached end of travel at sensor reading ");
            Serial.print(sensor_reading);
            Serial.print('\n');
            rc = -1;
            goto decel;
        }
    }

    Serial.print("# Hit full PWM at sensor reading ");
    Serial.print(sensor_reading);
    Serial.print(".\n");

#if 0
    Serial.print("Looking for ");
    Serial.print(direction == IN ? "low" : "high");
    Serial.print(" end of ");
    Serial.print(direction == IN ? low_sensor_decel : high_sensor_decel);
    Serial.print(" compared to current ");
    Serial.print(sensor_reading);
    Serial.print('\n');
#endif

    capped_pwm_write(valve, pwm_goal);
    start_millis = millis();
    last_time = start_millis;
    sensor_reading = read_sensor(joint);
    start_sensor = sensor_reading;

    /* Running at full speed. */
    while (1) {
        if (check_keypress()) {
            rc = -1;
            goto failed;
        }

#if 0
        /* Frobulate the PWM value every X milliseconds. */
        this_time = millis();
        if ((this_time - last_time) > 0) {
            last_time = this_time;
            /* XXX Do something to current_pwm. */
            capped_pwm_write(valve, current_pwm);
        }
#endif

        sensor_reading = read_sensor(joint);

        /* Check to see if we're close to the end and need to decelerate. */
        if (((direction == IN)  && (sensor_reading < low_sensor_decel)) ||
            ((direction == OUT) && (sensor_reading > high_sensor_decel))) {
            break;
        }
    }

    stop_millis = millis();
    stop_sensor = sensor_reading;

    Serial.print("# Decelerating from sensor reading ");
    Serial.print(sensor_reading);
    Serial.print("...\n");

decel:
    last_sensor = read_sensor(joint);
    /* Decelerate - 1%/X ms.  Move until joint stops moving. */
    for (i = 0;i < 1000;i++) {
        if (check_keypress()) {
            rc = -1;
            goto failed;
        }

        if (current_pwm > low_pwm)
            current_pwm--;

        capped_pwm_write(valve, current_pwm);

        /* Only run through this loop every X milliseconds. */
        while ((this_time - last_time) < 2)
            this_time = millis();
        last_time = this_time;

        sensor_reading = read_sensor(joint);
        /* Check to see if the joint is still moving. */
        if (((direction == IN)  && (sensor_reading < last_sensor)) ||
            ((direction == OUT) && (sensor_reading > last_sensor))) {
            last_sensor = sensor_reading;
            i = 0;
        }
    }

    pwms_off();

    total_millis = stop_millis - start_millis;
    total_sensor = abs(stop_sensor - start_sensor);
    speed = (total_sensor * 1000) / total_millis;
    if ((stop_sensor == 0) || (stop_millis == 0))
        speed = 0;

    if (verbose) {
        Serial.print("# Done, PWM = ");
        Serial.print(pwm_goal);
        Serial.print("%, Speed = ");
        Serial.print(speed);
        Serial.print(", time = ");
        Serial.print(((float)total_millis) / 1000.0);
        Serial.print(" seconds.\n");

        Serial.print("# Start sensor reading: ");
        Serial.print(start_sensor);
        Serial.print(", end sensor reading: ");
        Serial.print(stop_sensor);
        Serial.print(", total sensor movement: ");
        Serial.print(total_sensor);
        Serial.print('\n');

        Serial.print("# Total milliseconds at full speed: ");
        Serial.print(total_millis);
        Serial.print('\n');

        Serial.print("Speed: ");
        Serial.print(speed);
        Serial.print("\tPWM: ");
        Serial.print(pwm_goal);
        Serial.print(direction == IN ? "  in " : " out ");

        Serial.print("\tRun: ");
        Serial.print(run_count);

        Serial.print("\n\n");
    }

failed:
    pwms_off();
    set_pwm_scale(old_pwm_scale);
    reset_current_location();
    set_interrupt_state(old_int_state);

    if (rc)
        return rc;
    else
        return speed;
}

/*
 * Find the lowest PWM value that results in joint movement.
 *
 * Joint movement is defined as the joint sensor moving by at least
 * JOINT_SENSOR_MOVEMENT in one second.
 *
 * This function may fail if the joint is already near the end of its
 * travel, so on failure the joint should be moved to the other end of
 * travel and a second attempt should be made.
 *
 * Returns the lowest PWM value that gave movement, 0 if no movement
 * is seen, -1 on failure.
 *
 * XXX fixme: This should use the same method to detect movement as
 * the pwm/sensor speed mapping routine.  If this routine detects
 * movement at a particular PWM value then that value should be in the
 * map.
 */

/* Minimum sensor value change that indicates movement. */
#define MIN_SENSOR_MOVEMENT	2400

int find_joint_first_movement(int joint, int direction, int pwm_val, int pwm_inc)
{
    int valve;
    int rc = -1;
    int start_sensor_val;
    int current_sensor_val;
    int sensor_diff;
    int n;

    pwms_off();

    Serial.print("# Looking for lowest ");
    Serial.print(joint_names[joint]);
    Serial.print(direction == OUT ? " out " : " in ");
    Serial.print("pwm value that produces movement.\n");

    valve = joint + direction;

    /* Try PWM values up to 70%. */
    for (;pwm_val <= 70;pwm_val += pwm_inc) {
        if (!check_deadman())
            goto failed;

        if (check_keypress())
            goto failed;

        start_sensor_val = read_sensor(joint);

        Serial.print("# Trying ");
        Serial.print(pwm_val);
        Serial.print("% PWM.\n");

        capped_pwm_write(valve, pwm_val);

        /* Movement should happen in a few seconds if it's going to happen. */
        for (n = 0;n < 3000;n++) {
            current_sensor_val = read_sensor(joint);
            sensor_diff = start_sensor_val - current_sensor_val;
            if (abs(sensor_diff) >= MIN_SENSOR_MOVEMENT) {
                Serial.print("Joint moved!\n");
                rc = pwm_val;
                break;
            }
            delay(1);	/* 1ms delay. */
        }

        if (rc != 0)
            break;

        Serial.print("# No joint movement:  Start sensor value: ");
        Serial.print(start_sensor_val);
        Serial.print(", ending sensor value: ");
        Serial.print(current_sensor_val);
        Serial.print(", movement was: ");
        Serial.print(sensor_diff);
        Serial.print("\n");

        pwms_off();

        delay(100);	/* No movement, so wait at 0% pwm for 100ms then try higher PWM. */
    }
    pwms_off();

    if (rc > 0) {
        Serial.print("# Found ");
        Serial.print(joint_names[joint]);
        Serial.print(" movement at ");
        Serial.print(pwm_val);
        Serial.print("% PWM.\n");
        Serial.print("# Start sensor value: ");
        Serial.print(start_sensor_val);
        Serial.print(", ending sensor value: ");
        Serial.print(current_sensor_val);
        Serial.print(", movement was: ");
        Serial.print(sensor_diff);
        Serial.print("\n");
    } else {
        Serial.print("# Failed to see any joint movemenrt.\n");
    }

    Serial.print("\n");

failed:
    pwms_off();
    reset_current_location();
    return rc;
}

/*
 * Returns 0 for success, -1 for failure.
 *
 * XXX fixme: If this is the thigh then it needs an artificial
 * end-of-travel to keep it from hitting the ground.
 */
int find_joint_sensor_limit(int joint, int direction, int pwm_val)
{
    int valve;
    int rc = 0;
    int sensor_val;
    int sensor_limit;
    int n;

    pwms_off();

    Serial.print("\n#************************************************\n");
    Serial.print("# Discovering ");
    Serial.print(direction == IN ? "low" : "high");
    Serial.print(" sensor value for ");
    Serial.print(joint_names[joint]);
    Serial.print(direction == IN ? " IN" : " OUT");
    Serial.print(" using ");
    Serial.print(pwm_val);
    Serial.print("% PWM.\n");
    Serial.print("#************************************************\n");

    valve = joint + direction;

    if (!check_deadman())
        return -1;

    if (check_keypress())
        return -1;

    if (direction == IN)
        sensor_limit = 1 << 30;
    else
        sensor_limit = 0;

    sensor_val = read_sensor(joint);
    Serial.print("# Sensor start = ");
    Serial.print(sensor_val);
    Serial.print(":");

    capped_pwm_write(valve, pwm_val);

    /* If there's no movement for N seconds then the joint must be at the end. */
    /* XXX fixme:  I should bail if the joint's moving very slow. */
    for (n = 0;n < 10;n++) {
        if (!check_deadman())
            goto failed;
        if (check_keypress())
            goto failed;

        sensor_val = read_sensor(joint);
        Serial.print("semsor = ");
        Serial.print(sensor_val);

        if (direction == IN) {
            Serial.print(" - is it less than ");
            Serial.print(sensor_limit);
            Serial.print("? ");
            /* Joint going IN, sensor reading going down. */
            if (sensor_val < sensor_limit) {
                Serial.print("yes.");
                sensor_limit = sensor_val;
                n = 0;	/* Restart one second wait. */
            }
        } else { /* direction == OUT */
            Serial.print(" - is it greater than ");
            Serial.print(sensor_limit);
            /* Joint going OUT, sensor reading going up. */
            Serial.print("? ");
            if (sensor_val > sensor_limit) {
                Serial.print("yes.");
                sensor_limit = sensor_val;
                n = 0;	/* Restart one second wait. */
            }
        }
        Serial.print("\n");
        if (n == 0) {
            Serial.print(" ");
            Serial.print(sensor_val);
        }

        /*
         * Artificial limit for thigh to avoid ground interference.
         *
         * XXX fixme: The kinematics need real low and high sensor
         * limits.  The high limit should be a constant offset from
         * the low limit, I just need to find out what it is.
         */
        if (joint == THIGH) {
            if (sensor_val > 38000) {
                Serial.print("\n**** Hit artificial thigh limit!\n\n");
                break;
            }
        }

        delay(100);	/* 100ms delay. */
    }

    pwms_off();

    Serial.print("\n");

    delay(100);

    Serial.print("# ");
    Serial.print(joint_names[joint]);
    Serial.print(" ");
    Serial.print(direction_names[direction]);
    Serial.print(direction == IN ? " high" : " low");
    Serial.print(" sensor reading: ");
    Serial.print(sensor_limit);
    Serial.print("\n\n");

    delay(100);

    if (direction == IN) {
        SENSOR_LOW(joint) = sensor_limit;
        Serial.print("Set SENSOR_LOW to ");
        Serial.print(SENSOR_LOW(joint));
        Serial.print('\n');
    } else {
        SENSOR_HIGH(joint) = sensor_limit;
        Serial.print("Set SENSOR_HIGH to ");
        Serial.print(SENSOR_HIGH(joint));
        Serial.print('\n');
    }

    Serial.print("\nDone with limit discovery.\n");

failed:
    pwms_off();
    reset_current_location();

    return rc;

}

/*
 * Find the speed (sensor_units/sec) corresponding to a specific PWM
 * value for the given joint, in both directions.
 *
 * Each joint is tested the specified (count) number of times, and an
 * average is taken of those runs, which is stored in the in-memory
 * leg info struct.
 *
 * Returns 0 on success, non-zero on failure.
 *
 * This tests movement going OUT, so it moves the joint IN first.
 */
int find_joint_pwm_speeds(int joint, int count)
{
    int pwm_val;
    int pwm_start;
    int rc;
    int direction;
    int other_direction;
    int dir;
    int speeds[11][10][2]; /* 11 PWM values (0-100% in tens), average up to 10 tries, 2 directions. */
    int iternum;

    if (count > 10)
        count = 10;

    memset(speeds, 0, sizeof(speeds));

    Serial.print("\n#****************************************************************\n");
    Serial.print("# Discovering PWM speeds for ");
    Serial.print(joint_names[joint]);
    Serial.print("IN/OUT\n");
    Serial.print("#****************************************************************\n");

    direction = OUT;

    for (iternum = 0;iternum < count;iternum++) {
        if (direction == OUT)
            other_direction = IN;
        else
            other_direction = OUT;

        Serial.print("\n# Moving joint to position.\n");
        if (exercise_joint(joint, other_direction, LOW_PWM_MOVEMENT(joint + other_direction) + 5, 0) == -1)
            return -1;
        Serial.print("# Joint in position, waiting...\n\n");
        delay(1000);

        Serial.print("\n# Iteration ");
        Serial.print(iternum);
        Serial.print(" of ");
        Serial.print(count);
        Serial.print('\n');

        /* Start at 10% below the lowest movement point. */
        pwm_start = LOW_PWM_MOVEMENT(joint + direction);
        pwm_start -= 10;
        pwm_start += 9;
        pwm_start = (pwm_start / 10) * 10;

        pwm_val = pwm_start;

        /* PWM value is from min movement to 100%. */
        while (pwm_val <= 100) {

            Serial.print("# Testing ");
            Serial.print(joint_names[joint]);
            Serial.print(direction == IN ? " in " : " out");
            Serial.print(" at ");
            Serial.print(pwm_val);
            Serial.print("%\n");

            rc = exercise_joint(joint, direction, pwm_val, 1);
            if (rc == -1) {
                Serial.print("exercise_joint() failed.\n");
                return -1;
            }

            Serial.print("PWM/SPEED: ");
            Serial.print("\t");
            Serial.print(pwm_val);
            Serial.print("% PWM gave ");
            Serial.print(joint_names[joint]);
            Serial.print(direction == IN ? " in " : " out");
            Serial.print("\tmovement at speed ");
            Serial.print(rc);
            Serial.print('\n');

            speeds[iternum][pwm_val / 10][direction ? 1 : 0] = rc;

            Serial.print("# Done, watiing...\n\n");
            delay(1000);

            /*
             * If the joint is moving OUT then switch to IN.  If the
             * joint is moving IN then increase the PWM and switch to
             * OUT.
             */
            if (direction == OUT)
                direction = IN;
            else {
                direction = OUT;
                pwm_val += 10;
            }
        }
    }

    pwms_off();
    reset_current_location();

    /*
     * Print out the results, average the results, and store it.
     */
    for (dir = 0;dir < 2;dir++) {
        direction = dir ? IN : OUT;

        for (pwm_val = 0;pwm_val < 11;pwm_val++) {
            int speed_total = 0;
            int speed_count = 0;

            Serial.print(joint_names[joint]);
            Serial.print(direction == IN ? "  in\t" : " out\t");

            Serial.print("Speeds: ");
            Serial.print(pwm_val * 10);
            Serial.print("%\t");
            Serial.print(direction ? "in \t" : "out\t");

            for (iternum = 0;iternum < count;iternum++) {
                if (speeds[iternum][pwm_val][dir] != 0) {
                    Serial.print(speeds[iternum][pwm_val][dir]);
                    Serial.print("\t");
                    speed_total += speeds[iternum][pwm_val][dir];
                    speed_count++;
                }
            }

            /* Save the average. */
            JOINT_SPEED(joint + direction, pwm_val) = speed_total / speed_count;

            Serial.print("= ");
            Serial.print(JOINT_SPEED(joint + direction, pwm_val));
            Serial.print('\n');
        }
    }

    Serial.print("\nDone finding joint PWM speeds\n\n");
    Serial.print("#****************************************************************\n");

    return 0;
}

/*
 * Find the first movement PWM value and sensor limit for a joint in a
 * specific direction.
 *
 * XXX fixme:  Maybe this should have speed passed in?
 *
 * XXX fixme: If this is the knee then it gets air in it if it's run
 * all the way up, so after being run all the way up it needs to be
 * run against the stop in the other direction.
 *
 * If the cylinder hasn't been de-aired then it (XXX which function?)
 * will prematurely detect stoppage at the gravity crossing.
 */
int find_joint_limits(int joint, int direction)
{
    int i;
    int rc = -1;

    /* XXX fixme: I need to move the joint into position if it's against the wrong end. */

    /* Find first movement to 10% granularity. */
    i = find_joint_first_movement(joint, direction, 10, 10);
    if (i <= 0)
        return -1;
    /* XXX fixme:  I should return the joint to the beginning of travel. */
    i -= 10;
    if (i < 1)
        i = 1;
    /* Now find first movement to 1% granularity. */
    i = find_joint_first_movement(joint, direction, i, 1);
    if (i <= 0)
        return -1;

    LOW_PWM_MOVEMENT(joint + direction) = i;

    if (i != 0) {
        /* Find the sensor limit. */
        if (find_joint_sensor_limit(joint, direction, LOW_PWM_MOVEMENT(joint + direction) + 10))
            goto fail;
    }
    rc = 0;

fail:
    pwms_off();
    reset_current_location();

    return rc;
}

int set_joint_limits(int joint)
{
    int rc = -1;
    int direction = IN;
    int old_pwm_scale;
    int old_int_state;

    pwms_off();

    if (!check_deadman()) {
        Serial.print("ERROR:  Deadman has leg disabled!\n");
        return -1;
    }

    old_int_state = set_interrupt_state(0);

    old_pwm_scale = set_pwm_scale(100);

    enable_leg();

    /* Get thigh and knee into place. */

    park_leg();

    switch(joint) {
    case HIP:
        /* This does IN first, so move the joint OUT. */
        if (move_joint_all_the_way(HIPPWM_OUT, 50) == -1)
            goto fail;
        break;

    case KNEE:
        /* Nothing special to do. */
        break;

    case THIGH:
        /* Move the knee all the way out first to avoid having the foot hit the ground. */
        /*
         * XXX fixme: The results will presumably be very different
         * with the knee extended vs. retracted.  What should I do
         * about that?  Presumably unweighted movement isn't the more
         * important than weighted movement, but it probably best
         * represents weighted movement.
         */
        if (move_joint_all_the_way(KNEEPWM_IN, 50) == -1)
            goto fail;
        direction = OUT;
        break;
    }

    if (joint == THIGH)
        direction = OUT;
    else
        direction = IN;

    /* XXX Maybe I should pass speed in? */
    Serial.print("\n# Finding joint limits for ");
    Serial.print(direction_names[direction]);
    Serial.print("...\n");

    if (find_joint_limits(joint, direction) == -1)
        ;
/*        goto fail;*/

    Serial.print("# Done, waiting...\n");
    delay(1000);

    /* And the same thing for the other directoon. */
    if (direction == OUT)
        direction = IN;
    else
        direction = OUT;
    Serial.print("\n# Finding joint limits for ");
    Serial.print(direction_names[direction]);
    Serial.print("...\n");

    if (find_joint_limits(joint, direction) == -1)
        goto fail;

    Serial.print("# Done, waiting...\n");
    delay(1000);

    /* XXX FIXME:  units_per_deg isn't linear over the travel! */
    UNITS_PER_DEG(joint) = (SENSOR_HIGH(joint) - SENSOR_LOW(joint)) / (ANGLE_HIGH(joint) - ANGLE_LOW(joint));

    rc = 0; /* Success! */

    Serial.print('\n');
    Serial.print("# Done with calibration.\n");
    Serial.print('\n');

    Serial.print("# Low sensor readings:  \t");
    Serial.print(joint_names[joint]);
    Serial.print("\t");
    Serial.print(SENSOR_LOW(joint));

    Serial.print('\n');
    Serial.print("# High sensor readings: \t");
    Serial.print(joint_names[joint]);
    Serial.print("\t");
    Serial.print(SENSOR_HIGH(joint));

    Serial.print('\n');
    Serial.print('\n');

    Serial.print("# ");
    Serial.print(joint_names[joint]);
    Serial.print(" first moved in at ");
    Serial.print(LOW_PWM_MOVEMENT(joint + IN));
    Serial.print(" percent\n");

    Serial.print("# ");
    Serial.print(joint_names[joint]);
    Serial.print(" first moved out at ");
    Serial.print(LOW_PWM_MOVEMENT(joint + OUT));
    Serial.print(" percent\n");

    Serial.print("\n");

    park_leg();

fail:
    pwms_off();
    set_interrupt_state(old_int_state);
    set_pwm_scale(old_pwm_scale);
    reset_current_location();

    return rc;
}

/*
 * Find the PWM->speed mappings for a joint in both directions.
 *
 * IN first, then OUT.
 *
 * rep_count is the number of times to repeat the PWM/speed discovery
 * to get averaged results.
 */

int calibrate_joint(int joint, int rep_count)
{
    int old_pwm_scale;
    int old_int_state;

    if (!check_deadman()) {
        Serial.print("ERROR:  Deadman has leg disabled!\n");
        return -1;
    }

    if (rep_count == 0)
        rep_count = 1;

    old_pwm_scale = set_pwm_scale(100);

    pwms_off();
    old_int_state = set_interrupt_state(0);
    enable_leg();

    /* Get thigh and knee into place. */
    park_leg();

    switch(joint) {
    case HIP:
        /* This does IN first, so move the joint OUT. */
        if (move_joint_all_the_way(HIPPWM_OUT, 50) == -1)
            goto fail;
        break;

    case KNEE:
        /* Nothing special to do. */
        break;

    case THIGH:
        /* Move the knee all the way out first to avoid having the foot hit the ground. */
        /*
         * XXX fixme: The results will presumably be very different
         * with the knee extended vs. retracted.  What should I do
         * about that?  Presumably unweighted movement isn't the more
         * important than weighted movement, but it probably best
         * represents weighted movement.
         */
        if (move_joint_all_the_way(KNEEPWM_IN, 50) == -1)
            goto fail;
        break;
    }

    pwms_off();

    /* Discover PWM -> speed mapppings for both directions. */
    if (find_joint_pwm_speeds(joint, rep_count) == -1)
        goto fail;

    Serial.print("# Done, waiting...\n");
    delay(1000);

    park_leg();

    Serial.print('\n');
    Serial.print("# Done with calibration.\n");
    Serial.print("\n");

fail:
    pwms_off();
    reset_current_location();
    set_pwm_scale(old_pwm_scale);
    set_interrupt_state(old_int_state);

    return 0;
}
