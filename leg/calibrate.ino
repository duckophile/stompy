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

int check_keypress(void)
{
    int ch;

    if (Serial.available() > 0) {
        ch = Serial.read();
        if (ch == 3) {
            set_pwm_scale(60);
            pwms_off();
            disable_leg();
        }
        Serial.println("# Aborted by keypress.");
        return 1;
    }

    return 0;
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
    analogWrite(pwm_pins[valve], (pwm_percent * PWM_MAX) / 100);

    /* Look for movement, wait until it stops. */
    sensor_reading = read_sensor(sensor_num);
    last_sensor_reading = sensor_reading;
    /* 100 readings of 100ms each. */
    for (i = 0;i < 600;i++) {
        check_deadman();

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
    } else if (failed) {
        Serial.println("Aborted by keypress.");
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

int exercise_joint(int joint, int direction, int pwm_percent, int verbose)
{
    int sensor_reading;
    int i;
    int current_percent = LOW_ACCEL_PERCENT;
    int valve;
    int start_micros = 0;
    int start_sensor = 0;
    int stop_micros = 0;
    int stop_sensor = 0;
    int state = ACCELERATING;
    int no_change_count = 0;
    int last_sensor_reading = 0;
    int real_last_sensor_reading = 0;
    float inc;
    float accel_percent = LOW_ACCEL_PERCENT;
    int low_percent = LOW_DECEL_PERCENT;
    int failed = 0;
    int no_movement = 0;
    int speed;
    int total_micros;
    int total_sensor;

    if (current_percent < pwm_percent)
        current_percent = pwm_percent;

    /* Decelerate down to this value when stopping. */
    if (low_percent < pwm_percent)
        low_percent = pwm_percent;

    Serial.print("\n#================================================================ Begin\n# ");
    Serial.println("Exercising joint.");
    Serial.print("Using low percent of ");
    Serial.println(low_percent);

    /* Acceleration.  Accelerate over 30 steps.  */
    inc = (pwm_percent - current_percent) / 30.0;
    if (inc < 1)
        inc = 1;

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
    sensor_reading = read_sensor(joint);
    last_sensor_reading = sensor_reading;
    real_last_sensor_reading = sensor_reading;

    for (i = 0;i < 10000;i++) {
        check_deadman();

        if ((deadMan != 0) || (deadman_forced != 0)) {
/*            set_pwm(valve, current_percent);*/
            analogWrite(pwm_pins[valve], (current_percent * PWM_MAX) / 100);
        }

        /* Sleep 10ms. */
        delay(10);

        no_change_count++;

        sensor_reading = read_sensor(joint);

        /* Save min & max sensor values.  0xFFFF means blank flash. */
        if ((sensor_reading < SENSOR_LOW(joint)) ||(SENSOR_LOW(joint) == 0xFFFF))
            SENSOR_LOW(joint) = sensor_reading;
        if ((sensor_reading > SENSOR_HIGH(joint)) || (SENSOR_HIGH(joint) == 0xFFFF))
            SENSOR_HIGH(joint) = sensor_reading;

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
            no_movement = 1;
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
                stop_sensor = sensor_reading;
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
                stop_sensor = sensor_reading;
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
                start_sensor = sensor_reading;
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

                {
                    Serial.print("\t");
                    Serial.print(" Sensors: ");
                    Serial.print(analogRead(PRESSURE_SENSOR_1));
                    Serial.print("\t");
                    Serial.print(analogRead(PRESSURE_SENSOR_2));
                    Serial.print("\t");
                    Serial.print(analogRead(PRESSURE_SENSOR_3));
                    Serial.print("\t");
                    Serial.print(analogRead(PRESSURE_SENSOR_4));
                }

                Serial.println("");
                real_last_sensor_reading = sensor_reading;
            }
        }

        if (check_keypress()) {
            Serial.println("Aborting due to keypress.");
            failed = 1;
            break;
        }
    }

    analogWrite(pwm_pins[valve], 0);

    set_pwm(valve, 0);

    total_micros = stop_micros - start_micros;
    total_sensor = abs(stop_sensor - start_sensor);
    speed = total_micros / total_sensor;

    if (verbose) {
        Serial.println("# Done.");

        Serial.print("# Start sensor reading: ");
        Serial.println(start_sensor);
        Serial.print("# End sensor reading: ");
        Serial.println(stop_sensor);

        Serial.print("# Total seconds at full speed: ");
        Serial.println((float)total_micros / 1000000.0);

        Serial.print("# Total sensor movement: ");
        Serial.println(total_sensor);

        Serial.print("# Speed - microseconds / sensor unit at ");
        Serial.print(pwm_percent);
        Serial.print("% pwm: ");
        Serial.println(speed);

        if (failed || no_movement)
            Serial.println("# **************** FAILED ****************\n");
    }
    Serial.print("\n#================================================================ Done: ");

    if (failed) {
        Serial.println("Failure.\n");
        return -1;
    }
    if (no_movement) {
        Serial.println("No movement.\n");
        return 0;
    }

    Serial.print("Success: PWM = ");
    Serial.print(pwm_percent);
    Serial.print("% Speed = ");
    Serial.println(speed);
    Serial.println("");

    return speed;
}


/*
 * Find the lowest PWM value at which a joint moves.
 *
 * The PWM percentage is returned.
 */

/* Start testing for movement at this percentage. */
#define DISCOVERY_START_PWM	30

int find_joint_first_movement(int joint, int direction)
{
    float try_pwm;
    float pwm_inc;
    int other_direction;
    int rc;
    int last_movement_percent = -1;

    Serial.println("\n@ Finding first movement point for joint.\n");

    if (direction == IN)
        other_direction = OUT;
    else
        other_direction = IN;

    try_pwm = DISCOVERY_START_PWM;
    pwm_inc = try_pwm;
    do {
        pwm_inc = pwm_inc / 2;

        Serial.println("@ Moving joint to position.");
        if (exercise_joint(joint, other_direction, 50, 1) == -1) {
            Serial.print("@ Failed to move joint to starting position.\n");
            goto fail;
        }
        Serial.println("@ Done , waiting...");
        delay(1000);

        Serial.println("");
        rc = exercise_joint(joint, direction, (int)try_pwm, 1);
        if (rc == -1) {
            Serial.println("@ Aborting due to error return from exercise_joint.");
            goto fail;
        }
        Serial.println("@ Done , waiting...");
        delay(1000);

        Serial.print("@ ");
        Serial.print(joint_names[joint]);
        if (rc == 0) {
            try_pwm += pwm_inc; /* Didn't move, so go faster next time. */
            Serial.print(" didn't move");
        } else {
            last_movement_percent = (int)try_pwm;
            try_pwm -= pwm_inc; /* Did move, so go slower next time. */
            Serial.print(" did move");
        }
        Serial.print(direction == IN ? " in" : " out");
        Serial.print(" at ");
        Serial.print((int)try_pwm);
        Serial.println(" percent.");

    } while (pwm_inc >= 1);

    Serial.print("@ Found first movement point at ");
    Serial.print(last_movement_percent);
    Serial.println("%\n");

    /* Save to leg_info. */
    leg_info.valves[joint + direction].low_joint_movement = last_movement_percent;

    return last_movement_percent;

fail:
    Serial.println("\n@ Failed to find first movement point.\n");
    return -1;
}

int find_joint_pwm_speeds(int joint, int direction)
{
    int pwm_val;
    int other_direction;
    int rc;

    Serial.println("\nFinding joint PWM speeds...\n");

    if (direction == IN)
        other_direction = OUT;
    else
        other_direction = IN;

    if (direction == IN)
        pwm_val = first_move_in[joint];
    else
        pwm_val = first_move_out[joint];

    if (pwm_val == -1) {
        Serial.println("**************** I do not have a low PWM value for this joint!\n");
        return -1;
    }

    Serial.print("# Finding speeds for ");
    Serial.println(joint_names[joint]);
    Serial.print("# This joint will first move at ");
    Serial.print(pwm_val);
    Serial.println("%");

    pwm_val += 9;
    pwm_val = (pwm_val / 10) * 10;

    for (;pwm_val <= 100;pwm_val += 10) {
        Serial.println("# Moving joint to position.");
        if (exercise_joint(joint, other_direction, 50, 0))
            return -1;
        Serial.println("# Done , waiting...");
        delay(1000);

        Serial.println("");
        Serial.print("# Testing at ");
        Serial.print(pwm_val);
        Serial.println("%");
        rc = exercise_joint(joint, direction, pwm_val, 1);
        if (rc == -1)
            return -1;
        Serial.print("# ");
        Serial.print(pwm_val);
        Serial.print("% PWM gave ");
        Serial.print(joint_names[joint]);
        Serial.print(" movement at speed ");
        Serial.println(rc);
        Serial.println("# Done , waiting...");
        delay(1000);
    }

    Serial.println("\nDone finding joint PWM speeds...\n");

    return 0;
}

int calibrate(void)
{
    int i;

    set_pwm_scale(100);

    enable_leg();

    Serial.println("# Retracting thigh.");
    if (move_joint_all_the_way(THIGHPWM_UP, 50) == -1)
        return -1;

    Serial.println("# Retracting knee.");
    /* Move knee up. */
/*    move_joint_all_the_way(KNEEPWM_RETRACT); OOPS!  This appears to be backwards? */
    if (move_joint_all_the_way(KNEEPWM_EXTEND, 50) == -1)
        return -1;
/*    move_joint_all_the_way(KNEEPWM_RETRACT); Need tent corner removed for this. */

#if 0
    Serial.println("# Testing hip limits.\n");
    if (move_joint_all_the_way(HIPPWM_FORWARD, 30) == -1)
        return -1;
    if (move_joint_all_the_way(HIPPWM_REVERSE, 30) == -1)
        return -1;
#endif

    Serial.println("# Done, waiting...");
    delay(1000);

    /* HIP */
    Serial.println("\n# Discovering low PWM limit for HIP IN:\n");
    first_move_in[HIP] = find_joint_first_movement(HIP, IN);

    Serial.println("# Discovering PWM speeds for HIP IN:");
    find_joint_pwm_speeds(HIP, IN);

#if 0

    Serial.println("# Discovering low PWM limit for HIP OUT:");
    first_move_out[HIP] = find_joint_first_movement(HIP, OUT);

    Serial.println("# Discovering PWM speeds for HIP OUT:");
    find_joint_pwm_speeds(HIP, OUT);

    /* Park knee in center of travel. */

    Serial.println("# Done, waiting...");
    delay(1000);
#endif

#if 0
    /* THIGH */
    Serial.println("# Discovering low PWM limit for THIGH IN:");
    first_move_in[THIGH] = find_joint_first_movement(THIGH, IN);

    Serial.println("# Discovering low PWM limit for THIGH OUT:");
    first_move_out[THIGH] = find_joint_first_movement(THIGH, OUT);

    /* Pull the thigh in. */

    Serial.println("# Done, waiting...");
    delay(1000);
#endif

#if 0
    /* KNEE */
    Serial.println("# Discovering low PWM limit for KNEE IN:");
    first_move_in[KNEE] = find_joint_first_movement(KNEE, IN);

    Serial.println("# Discovering low PWM limit for KNEE OUT:");
    first_move_out[KNEE] = find_joint_first_movement(KNEE, OUT);

    /* Pull the knee in */

    Serial.println("# Done, waiting...");
    delay(1000);
#endif

    set_pwm_scale(60);

    pwms_off();

    UNITS_PER_DEG(HIP)   = (SENSOR_HIGH(HIP)   - SENSOR_LOW(HIP))   / (ANGLE_HIGH(HIP)   - ANGLE_LOW(HIP));
    UNITS_PER_DEG(THIGH) = (SENSOR_HIGH(THIGH) - SENSOR_LOW(THIGH)) / (ANGLE_HIGH(THIGH) - ANGLE_LOW(THIGH));
    UNITS_PER_DEG(KNEE)  = (SENSOR_HIGH(KNEE)  - SENSOR_LOW(KNEE))  / (ANGLE_HIGH(KNEE)  - ANGLE_LOW(KNEE));

    Serial.println("");
    Serial.println("# Done with calibration.");
    Serial.println("");

    Serial.print("# Low sensor readings:  ");
    for (i = 0;i < 3;i++) {
        Serial.print("\t");
        Serial.print(joint_names[i]);
        Serial.print("\t");
        Serial.print(SENSOR_LOW(i));
    }
    Serial.println("");
    Serial.print("# High sensor readings: ");
    for (i = 0;i < 3;i++) {
        Serial.print("\t");
        Serial.print(joint_names[i]);
        Serial.print("\t");
        Serial.print(SENSOR_HIGH(i));
    }
    Serial.println("");
    Serial.println("");

    for (i = 0;i < 3;i++) {
        Serial.print("# ");
        Serial.print(joint_names[i]);
        Serial.print(" first moved in at ");
        Serial.print(first_move_in[i]);
        Serial.println(" percent");

        Serial.print("# ");
        Serial.print(joint_names[i]);
        Serial.print(" first moved out at ");
        Serial.print(first_move_out[i]);
        Serial.println(" percent");
    }

    return 0;
}
