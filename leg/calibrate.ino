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
    set_pwm(valve, pwm_percent);

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

int exercise_joint(int joint, int direction, int pwm_percent, int verbose)
{
    int sensor_reading;
    int i;
    int current_percent = 20;
    int valve;
    int start_micros = 0;
    int stop_micros = 0;
    int state = ACCELERATING;
    int no_change_count = 0;
    int last_sensor_reading = 0;

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
        if (no_change_count > 150) {
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
            }
        }

        switch(state) {
        case ACCELERATING:
            int inc;
            /*
             * XXX fixme: Acceleration is wrong.  It should start out
             * slow and get faster, rather than vice versa.
             */
            inc = (pwm_percent - current_percent) / 10;
            if (inc == 0)
                inc = 1;
            current_percent += inc;
            if (current_percent == pwm_percent) {
                state = FULL_SPEED;
                Serial.println("# Hit full PWM.");
                start_micros = micros();
            }
            break;
        case FULL_SPEED:
            break;
        case DECELERATING:
            int dec;
            dec = (current_percent - 35) / 10;
            if (dec == 0)
                dec = 1;
            current_percent -= dec;
            if (current_percent < 35)
                current_percent = 35;
            break;
        }

        if (verbose) {
/*            if ((state != FULL_SPEED) || ((i % 10) == 0)) {*/
            if ((state != FULL_SPEED)) {
                Serial.print(i);
                Serial.print('\t');
                Serial.print(sensor_reading);
                Serial.print('\t');
                Serial.print(current_percent);
/*
                Serial.print('\t');
                Serial.print((current_percent * 1024) / 100);
*/
                Serial.println("");
            }
        }

        if (Serial.available() > 0) {
            Serial.read();
            Serial.println("# Aborted by keypress.");
            break;
        }

    }

    set_pwm(valve, 0);

    if (verbose) {
        Serial.println("# Done.");
        Serial.print("# Total time at full speed: ");
        Serial.println((float)(stop_micros - start_micros) / 1000000.0);
    }
    Serial.println("\n#================================================================\n");
    return 0;
}

int calibrate(void)
{
    int i;

    set_pwm_scale(100);

    enable_leg();

    Serial.println("# Retracting thigh.");
    move_joint_all_the_way(THIGHPWM_UP, 30);

    Serial.println("# Retracting knee.");
    /* Move knee up. */
/*    move_joint_all_the_way(KNEEPWM_RETRACT); OOPS!  This appears to be backwards? */
    move_joint_all_the_way(KNEEPWM_EXTEND, 30);
/*    move_joint_all_the_way(KNEEPWM_RETRACT); Need tent corner removed for this. */

#if 0
    Serial.println("# Testing hip limits.\n");
    move_joint_all_the_way(HIPPWM_FORWARD, 30);
    move_joint_all_the_way(HIPPWM_REVERSE, 30);
#endif

    Serial.println("# Done, waiting...");
    delay(1000);

    Serial.println("# Going into exercise loop:");
    for (i = 20;i < 100;i += 5) {
        Serial.println("# Moving joint to position.");
        exercise_joint(HIP, OUT, 50, 0);
        Serial.println("# Done , waiting...");
        delay(1000);

        Serial.println("");
        exercise_joint(HIP, IN, i, 1);
        Serial.println("# Done, waiting...");
        delay(1000);

        Serial.println("");
        exercise_joint(HIP, OUT, i, 1);
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

    return 0;
}
