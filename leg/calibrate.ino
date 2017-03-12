/* ; -*- mode: C ;-*- */

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
 * How much PWM to move leg slowly?
 *
 * I should have adjustable slow start and deceleration.
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

/*
 * Data to save in flash:
 *
 * For each joint:
 * - High and low sensor limits.
 * - Minimum PWM at which joint moves.
 * - PWM-to-speed mappings.
 * - Temperature compensation data (?).
 * - Manufacturing tolerance compensation - compensation for joint
 *   differences from leg-to-leg.
 *
 */

/*
Done with calibration.

Low sensor readings:    hip     86      thigh   27      knee    818
High sensor readings:   hip     728     thigh   224     knee    945

This gives a range of 642 on the hip.

*/

/*
 * Move a joint associated with a specified valve until it stops
 * moving.
 *
 * Returns the last sensor reading from the joint, or -1 on failure.
 *
 * The valve number is 0-5.
 *
 * This should take a joint (0-2) and a direction.
 */
int move_joint_all_the_way(int valve, int pwm_percent)
{
    int out = 0;
    int no_change_count = 0;
    int sensor_num;
    int sensor_reading;
    int last_sensor_reading = 0;
    int i;

    sensor_num = valve;
    /* Sensor number is 0-2. */
    if (sensor_num >= 3) {
        sensor_num = sensor_num - 3;
        out = 1;	/* Sensor reading should be going up?  Hopefully? */
        Serial.println("Sensor value should be increasing.");
    } else {
        out = 0;
        Serial.println("Sensor value should be dropping.");
   }

    Serial.print("Calibrating valve ");
    Serial.print(valve);
    Serial.print(" PWM pin ");
    Serial.print(pwm_pins[valve]);
    Serial.print("\tsensor ");
    Serial.print(sensor_num);
    Serial.print(" sensor pin ");
    Serial.println(sensorPin[sensor_num]);

    /* Set the PWM to move the joint. */
/*    set_pwm(valve, pwm_percent);*/
    analogWrite(pwm_pins[valve], (pwm_percent * 1024) / 100);

    /* Look for movement, wait until it stops. */
    sensor_reading = read_sensor(sensorPin[sensor_num]);
    last_sensor_reading = sensor_reading;
    /* 100 readings of 100ms each. */
    for (i = 0;i < 600;i++) {
        check_deadman();

        no_change_count++;
        sensor_reading = read_sensor(sensorPin[sensor_num]);

        if (sensor_reading < sensor_lows[sensor_num])
            sensor_lows[sensor_num] = sensor_reading;
        if (sensor_reading > sensor_highs[sensor_num])
            sensor_highs[sensor_num] = sensor_reading;

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

        Serial.print(i);
        Serial.print("\tNo change: ");
        Serial.print(no_change_count);
        Serial.print("\tlast: ");
        Serial.print(last_sensor_reading);
        Serial.print("\tSensor: ");
        Serial.println(sensor_reading);

        /*
         * If we've been through this loop 10 times without movement
         * then we're at the end.
         */
        if (no_change_count > 9)
            break;

        delay(100);
    }

    if (i == 600) {
        Serial.println("Leg didn't stop moving!");
        return -1;
    } else
        return last_sensor_reading;
}

/* I need a function to test for movement. */

#define ACCELERATING	0
#define FULL_SPEED	1
#define DECELERATING	2

/* Start accelerating from this PWM percentage. */
#define LOW_ACCEL_PERCENT	40
/* Decelerate down to this PWM percentage. */
#define LOW_DECEL_PERCENT	40

/* If the joint hasn't moved after this many readings then it's not moving. */
#define NO_CHANGE_LIMIT		200

/*
 * I need to be able to tell this function to quit once it's detected the leg moving.
 *
 * And detect movement based on total sensor movement rather than just
 * some number of iterations without moving?  Or at least report total
 * sensor movement.
 */

int exercise_joint(int joint, int direction, int pwm_percent, int verbose)
{
    int sensor_reading;
    int i;
    int current_percent = LOW_ACCEL_PERCENT;
    int valve;
    int start_micros = 0;
    int stop_micros = 0;
    int state = ACCELERATING;
    int no_change_count = 0;
    int last_sensor_reading = 0;
    int real_last_sensor_reading = 0;
    float inc;
    float accel_percent = LOW_ACCEL_PERCENT;
    int low_percent = LOW_DECEL_PERCENT;

    if (current_percent < pwm_percent)
        current_percent = pwm_percent;

    /* Decelerate down to this value when stopping. */
    if (low_percent < pwm_percent)
        low_percent = pwm_percent;

    Serial.print("Using low percent of ");
    Serial.println(low_percent);

    /* Acceleration.  Accelerate over 30 steps.  */
    inc = (pwm_percent - current_percent) / 30.0;
    if (inc < 1)
        inc = 1;

    Serial.print("\n#================================================================\n# ");
    Serial.print(joint_names[joint]);
    Serial.print(direction == IN ? " in " : " out ");
    Serial.print(pwm_percent);
    Serial.println(" percent PWM (which might be scaled by the PWM module.");

    valve = joint + direction;

    /* Sensor number is 0-2. */
    if (verbose) {
        if (direction == OUT)
            Serial.println("# Sensor value should be decreasing.");
        else
            Serial.println("# Sensor value should be increasing.");

        Serial.print("# exercising valve ");
        Serial.print(valve);
        Serial.print(" PWM pin ");
        Serial.print(pwm_pins[valve]);
        Serial.print("\tsensor ");
        Serial.print(joint);
        Serial.print(" sensor pin ");
        Serial.println(sensorPin[joint]);
    }

    /* Look for movement, wait until it stops. */
    sensor_reading = read_sensor(sensorPin[joint]);
    last_sensor_reading = sensor_reading;
    real_last_sensor_reading = sensor_reading;

    for (i = 0;i < 10000;i++) {
        check_deadman();

        if ((deadMan != 0) || (!deadman_forced != 0)) {
/*            set_pwm(valve, current_percent);*/
            analogWrite(pwm_pins[valve], (current_percent * 1024) / 100);
        }

        /* Sleep 10ms. */
        delay(10);

        no_change_count++;

        sensor_reading = read_sensor(sensorPin[joint]);

        /* Check if the leg's moving. */
        if (direction == IN) {
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
        if (no_change_count > NO_CHANGE_LIMIT) {
            Serial.print("\n# Joint is not moving at ");
            Serial.print(current_percent);
            Serial.println("% pwm.\nAborting.\n");
            break;
        }

        /* Check to see if we've passed a limit. */
        if (direction == IN) {
            /* Joint going out, sensor reading going up. */
            if (sensor_reading > 720) {
                Serial.println("# Hit high end.");
                break;
            }
            if ((sensor_reading > 628) && (state <= FULL_SPEED)) {
                state = DECELERATING;
                stop_micros = micros();
                Serial.println("# Decelerating.");
                inc = (current_percent - 40) / 30;
                if (inc < 1)
                    inc = 1;
                accel_percent = (float)current_percent;
                Serial.print("Decelerating from ");
                Serial.println(accel_percent);
            }
        } else {
            /* Joint going in, sensor reading going down. */
            if (sensor_reading < 92) {
                Serial.println("# Hit low end.");
                break;
            }
            if ((sensor_reading < 186) && (state <= FULL_SPEED)) {
                state = DECELERATING;
                stop_micros = micros();
                Serial.println("# Decelerating.");
                inc = (current_percent - 40) / 30;
                if (inc < 1)
                    inc = 1;
                accel_percent = (float)current_percent;
                Serial.print("Decelerating from ");
                Serial.println(accel_percent);
            }
        }

        switch(state) {
        case ACCELERATING:
            accel_percent += inc;
            current_percent = (int)accel_percent;
            if (current_percent >= pwm_percent) {
                current_percent = pwm_percent;
                state = FULL_SPEED;
                Serial.println("# Hit full PWM.");
                start_micros = micros();
            }
            break;
        case FULL_SPEED:
            break;
        case DECELERATING:
            /* Decelerate. */
            accel_percent -= inc;
            current_percent = (int)accel_percent;
            if (current_percent < low_percent)
                current_percent = low_percent;
            break;
        }

        if (verbose) {
            if ((state != FULL_SPEED) || ((i % 10) == 0)) {
/*            if ((state != FULL_SPEED)) {*/
                Serial.print(i);
                Serial.print('\t');
                Serial.print(sensor_reading);
                Serial.print('\t');
                Serial.print(sensor_reading - real_last_sensor_reading);
                Serial.print('\t');
                Serial.print(current_percent);
/*
                Serial.print('\t');
                Serial.print((current_percent * 1024) / 100);
*/
                Serial.println("");
                real_last_sensor_reading = sensor_reading;
            }
        }

        if (Serial.available() > 0) {
            Serial.read();
            Serial.println("# Aborted by keypress.");
            break;
        }

    }

    analogWrite(pwm_pins[valve], 0);

    set_pwm(valve, 0);

    if (verbose) {
        Serial.println("# Done.");
        Serial.print("# Total time at full speed: ");
        Serial.println((float)(stop_micros - start_micros) / 1000000.0);
        Serial.print("# Final sensor readin: ");
        Serial.println(sensor_reading = read_sensor(sensorPin[joint]));
    }
    Serial.println("\n#================================================================\n");

    if (no_change_count > NO_CHANGE_LIMIT)
        return -1;
    else
        return 0;
}

int calibrate(void)
{
    int i;
    int rc;
    int first_move_in = -1;
    int first_move_out = -1;
    int start;

    set_pwm_scale(100);

    enable_leg();

    Serial.println("# Retracting thigh.");
    move_joint_all_the_way(THIGHPWM_UP, 50);

    Serial.println("# Retracting knee.");
    /* Move knee up. */
/*    move_joint_all_the_way(KNEEPWM_RETRACT); OOPS!  This appears to be backwards? */
    move_joint_all_the_way(KNEEPWM_EXTEND, 50);
/*    move_joint_all_the_way(KNEEPWM_RETRACT); Need tent corner removed for this. */

#if 0
    Serial.println("# Testing hip limits.\n");
    move_joint_all_the_way(HIPPWM_FORWARD, 30);
    move_joint_all_the_way(HIPPWM_REVERSE, 30);
#endif

    Serial.println("# Done, waiting...");
    delay(1000);

    Serial.println("# Discovering low PWM limit for in:");
    for (i = 30;i <= 50;i += 5) {
        Serial.println("# Moving joint to position.");
        exercise_joint(HIP, OUT, 50, 0);
        Serial.println("# Done , waiting...");
        delay(1000);

        Serial.println("");
        rc = exercise_joint(HIP, IN, i, 1);
        if (rc == -1) {
            Serial.println("Joint did not move.");
        } else {
            if (first_move_in == -1) {
                Serial.print("# Joint moved in at ");
                Serial.print(i);
                Serial.println(" percent");
                first_move_in = i;
                break;
            }
        }
        Serial.println("# Done, waiting...");
        delay(1000);
    }

    start = first_move_in - 10;
    first_move_in = -1;
    for (i = start;i <= start + 10;i++) {
        Serial.println("# Moving joint to position.");
        exercise_joint(HIP, OUT, 50, 0);
        Serial.println("# Done , waiting...");
        delay(1000);

        Serial.println("");
        rc = exercise_joint(HIP, IN, i, 1);
        if (rc == -1) {
            Serial.println("Joint did not move.");
        } else {
            if (first_move_in == -1) {
                Serial.print("# Joint moved in at ");
                Serial.print(i);
                Serial.println(" percent");
                first_move_in = i;
                break;
            }
        }
        Serial.println("# Done, waiting...");
        delay(1000);
    }

    Serial.println("# Discovering low PWM limit for out:");
    for (i = 30;i <= 50;i += 5) {
        Serial.println("# Moving joint to position.");
        exercise_joint(HIP, IN, 50, 0);
        Serial.println("# Done , waiting...");
        delay(1000);

        Serial.println("");
        rc = exercise_joint(HIP, OUT, i, 1);
        if (rc == -1) {
            Serial.println("Joint did not move.");
        } else {
            if (first_move_out == -1) {
                Serial.print("# Joint moved out at ");
                Serial.print(i);
                Serial.println(" percent");
                first_move_out = i;
                break;
            }
        }
        Serial.println("# Done, waiting...");
        delay(1000);
    }

    start = first_move_out - 10;
    first_move_out = -1;
    for (i = start;i <= start + 10;i++) {
        Serial.println("# Moving joint to position.");
        exercise_joint(HIP, IN, 50, 0);
        Serial.println("# Done , waiting...");
        delay(1000);

        Serial.println("");
        rc = exercise_joint(HIP, OUT, i, 1);
        if (rc == -1) {
            Serial.println("Joint did not move.");
        } else {
            if (first_move_out == -1) {
                Serial.print("# Joint moved out at ");
                Serial.print(i);
                Serial.println(" percent");
                first_move_out = i;
                break;
            }
        }
        Serial.println("# Done, waiting...");
        delay(1000);
    }

    set_pwm_scale(60);

    pwms_off();

    Serial.println("");
    Serial.println("# Done with calibration.");
    Serial.println("");

    Serial.print("# Low sensor readings:  ");
    for (i = 0;i < 3;i++) {
        Serial.print("\t");
        Serial.print(joint_names[i]);
        Serial.print("\t");
        Serial.print(sensor_lows[i]);
    }
    Serial.println("");
    Serial.print("# High sensor readings: ");
    for (i = 0;i < 3;i++) {
        Serial.print("\t");
        Serial.print(joint_names[i]);
        Serial.print("\t");
        Serial.print(sensor_highs[i]);
    }
    Serial.println("");
    Serial.println("");

    Serial.print("# Joint first moved in at ");
    Serial.print(first_move_in);
    Serial.println(" percent");
    Serial.print("# Joint first moved out at ");
    Serial.print(first_move_out);
    Serial.println(" percent");

    return 0;
}
