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
    if (Serial.available() > 0) {
        (void)Serial.read();
        set_pwm_scale(60);
        pwms_off();
        disable_leg();
        Serial.println("\n# Aborted by keypress.\n");
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

/*
 * THIS ISN'T ACTUALLY AS CONTORTED AS IT LOOKS!
 *
 * It's really pretty simple, it's just written out as simple as
 * possible, which makes it look big, plus there's a lot of debugging
 * output.
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
    int real_last_sensor_reading;
    int pwm_inc;
    int low_percent;
    int failed = 0;
    int no_movement = 0;
    int speed;
    int total_millis;
    int total_sensor;
    int high_sensor_decel, low_sensor_decel;
    int sensor_high_end, sensor_low_end;
    int32_t this_time, last_time;

    /* Accelerate from the lowest PWM value that gives movement. */
    current_pwm = LOW_PWM_MOVEMENT(joint + direction);
    if (current_pwm < pwm_goal)
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
    high_sensor_decel = SENSOR_HIGH(joint) - 30; /* When to start decelerating. */
    low_sensor_decel  = SENSOR_LOW(joint)  + 30; /* When to start decelerating. */
    sensor_high_end   = SENSOR_HIGH(joint) - 10; /* close enough to the end of travel. */
    sensor_low_end    = SENSOR_LOW(joint)  + 10;

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

    sensor_reading = read_sensor(joint);
    last_sensor_reading = sensor_reading;
    real_last_sensor_reading = sensor_reading;
    Serial.print("# Starting at sensor reading ");
    Serial.println(sensor_reading);

    this_time = millis();
    last_time = 0;

    Serial.print("# Writing raw PWM value ");
    Serial.println((current_pwm * PWM_MAX) / 100);

    for (i = 0;i < 30000;i++) {
        if (!check_deadman()) {
            failed = 1;
            break;
        }

        analogWrite(pwm_pins[valve], (current_pwm * PWM_MAX) / 100);

        /* Only run through this loop every 10ms. */
        while ((this_time - last_time) < 10)
            this_time = millis();
        last_time = this_time;

        sensor_reading = read_sensor(joint);

        /* Save min & max sensor values.  0xFFFF means blank flash. */
        if ((sensor_reading < SENSOR_LOW(joint)) || (SENSOR_LOW(joint) == 0xFFFF))
            SENSOR_LOW(joint) = sensor_reading;
        if ((sensor_reading > SENSOR_HIGH(joint)) || (SENSOR_HIGH(joint) == 0xFFFF))
            SENSOR_HIGH(joint) = sensor_reading;

        /* Check if the joint's moving. */
        no_change_count++;
        if (((direction == IN)  && (sensor_reading > last_sensor_reading)) ||
            ((direction == OUT) && (sensor_reading < last_sensor_reading))) {
            last_sensor_reading = sensor_reading;
            no_change_count = 0;
        }
        if (no_change_count > NO_CHANGE_LIMIT) {
            Serial.print("\n# Joint is not moving at ");
            Serial.print(current_pwm);
            Serial.println("% pwm.\nAborting.\n");
            no_movement = 1;
            break;
        }

        /* Check to see if we're close enough to the end of travel. */
        if (((direction == IN)  && (sensor_reading > sensor_high_end)) ||
            ((direction == OUT) && (sensor_reading < sensor_low_end))) {
            Serial.print("# Reached end of travel at sensor reading ");
            Serial.println(sensor_reading);
            break;
        }

        /* Check to see if we've passed the high or low limit. */
        /* This handles the transition from FULL_SPEED to DECELERATING. */
        if (state <= FULL_SPEED) {
            if (((direction == IN)  && (sensor_reading > high_sensor_decel)) ||
                ((direction == OUT) && (sensor_reading < low_sensor_decel))) {
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
                Serial.println(pwm_goal);
                current_pwm = pwm_goal;
                state = FULL_SPEED;
                start_millis = millis();
                start_sensor = sensor_reading;
                Serial.print("# Hit full PWM at sensor reading ");
                Serial.println(start_sensor);
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

                Serial.println("");
                real_last_sensor_reading = sensor_reading;
            }
        }

        if (check_keypress()) {
            failed = 1;
            break;
        }
    }

    analogWrite(pwm_pins[valve], 0);
    set_pwm(valve, 0);

    total_millis = stop_millis - start_millis;
    total_sensor = abs(stop_sensor - start_sensor);
/*    speed = total_millis / total_sensor;*/
    speed = (total_sensor * 1000) / total_millis;
    if ((stop_sensor == 0) || (stop_millis == 0))
        speed = 0;

    /* I want sensor movement/sec */

    if (verbose) {
        Serial.println("# Done.");

        Serial.print("# Start sensor reading: ");
        Serial.print(start_sensor);
        Serial.print(", end sensor reading: ");
        Serial.print(stop_sensor);
        Serial.print(", total sensor movement: ");
        Serial.println(total_sensor);

        Serial.print("# Total milliseconds at full speed: ");
        Serial.println(total_millis);

        Serial.print("# Speed - sensor_units/sec at  ");
        Serial.print(pwm_goal);
        Serial.print("% pwm: ");
        Serial.println(speed);

        if (failed)
            Serial.print("# **************** FAILED ****************\n");
        if (no_movement)
            Serial.print("# **************** JOINT DID NOT MOVE ****************\n");
    }

    Serial.print("# Done: ");
    if (failed) {
        Serial.print("Failure.\n");
        return -1;
    }
    if (no_movement) {
        Serial.print("Joint did not move.\n");
        return 0;
    }

    Serial.print("Success: PWM = ");
    Serial.print(pwm_goal);
    Serial.print("%, Speed = ");
    Serial.println(speed);

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
 */

/* Minimum sensor value change that indicates movement. */
#define MIN_SENSOR_MOVEMENT	15

int find_joint_first_movement(int joint, int direction)
{
    int valve;
    int pwm_val;
    int pwm_inc;
    int rc = 0;
    int start_sensor_val;
    int current_sensor_val;
    int sensor_diff;
    int n;

    Serial.print("# Looking for lowest ");
    Serial.print(joint_names[joint]);
    Serial.print(direction == OUT ? " out " : " in ");
    Serial.print("pwm value that produces movement.\n");

    valve = joint + direction;

    /* These values should be passed in. */
    pwm_val = 5; /* Start at 5%. */
    pwm_inc = 5; /* Increment by 5%. */

    /* Try PWM values up to 90%. */
    for (;pwm_val < 90;pwm_val += pwm_inc) {
        if (!check_deadman())
            return -1;

        if (check_keypress())
            return -1;

        start_sensor_val = read_sensor(joint);

        Serial.print("# Trying ");
        Serial.print(pwm_val);
        Serial.print("% PWM.\n");

        analogWrite(pwm_pins[valve], (pwm_val * PWM_MAX) / 100);

        /* Movement should happen in a second if it's going to happen. */
        for (n = 0;n < 1000;n++) {
            current_sensor_val = read_sensor(joint);
            sensor_diff = start_sensor_val - current_sensor_val;
            if (abs(sensor_diff) >= MIN_SENSOR_MOVEMENT) {
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

        analogWrite(pwm_pins[valve], 0);
        delay(100);	/* Wait at 0% pwm for 100ms. */
    }

    analogWrite(pwm_pins[valve], 0);

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
    return rc;
}

/*
 * Returns 0 for success, -1 for failure.
 */
int find_joint_sensor_limit(int joint, int direction, int pwm_val)
{
    int valve;
    int rc = 0;
    int sensor_val;
    int sensor_limit;
    int n;

    Serial.print("\n#************************************************\n");
    Serial.print("# Discovering ");
    Serial.print(direction == IN ? "low" : "high");
    Serial.print(" sensor value for ");
    Serial.print(joint_names[joint]);
    Serial.print(direction == IN ? "IN\n" : "OUT\n");
    Serial.print("#************************************************\n");

    valve = joint + direction;

    if (!check_deadman())
        return -1;

    if (check_keypress())
        return -1;

    if (direction == IN)
        sensor_limit = 0;
    else
        sensor_limit = 1 << 30;

    analogWrite(pwm_pins[valve], (pwm_val * PWM_MAX) / 100);

    /* If there's no movement for a second then the joint must be at the end. */
    for (n = 0;n < 10;n++) {
        if (!check_deadman())
            return -1;
        if (check_keypress())
            return -1;

        sensor_val = read_sensor(joint);
        Serial.print("Sensor = ");
        Serial.print(sensor_val);
        Serial.print(", limit ");
        Serial.print(sensor_limit);

        if (direction == IN) {
            Serial.print(" IN ");
            /* Joint going IN, sensor reading going up. */
            if (sensor_val > sensor_limit) {
                sensor_limit = sensor_val;
                n = 0;	/* Restart one second wait. */
                Serial.print(" new high.");
            }
        } else { /* direction == OUT */
            Serial.print(" OUT ");
            /* Joint going in, sensor reading going down. */
            if (sensor_val < sensor_limit) {
                sensor_limit = sensor_val;
                n = 0;	/* Restart one second wait. */
                Serial.print(" new low.");
            }
        }
        Serial.print("\n");

        delay(100);	/* 100ms delay. */
    }

    /* Stop joint. */
    analogWrite(pwm_pins[valve], 0);

    Serial.print("# ");
    Serial.print(joint_names[joint]);
    Serial.print(" ");
    Serial.print(direction_names[direction]);
    Serial.print(direction == IN ? " high" : " low");
    Serial.print(" sensor reading: ");
    Serial.print(sensor_limit);
    Serial.print("\n");

    if (direction == IN)
        SENSOR_HIGH(joint) = sensor_limit;
    else
        SENSOR_LOW(joint) = sensor_limit;

    return rc;
}

#if 0
/*
 * Find the lowest PWM value at which a joint moves.
 *
 * The PWM percentage is returned.
 */

/* Start testing for movement at this percentage. */
#define DISCOVERY_START_PWM	30

int old_find_joint_first_movement(int joint, int direction)
{
    float try_pwm;
    float pwm_inc;
    int other_direction;
    int rc;
    int last_movement_percent = -1;

    Serial.print("\n#************************************************\n");
    Serial.print("# Discovering low PWM limit for ");
    Serial.print(joint_mames[joint]);
    Serial.print(diretion == IN ? " IN\n" : " OUT\n");
    Serial.print("#************************************************\n");

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
#endif

/*
 * I should be able to specify a starting PWM value and an increment.
 */
int find_joint_pwm_speeds(int joint, int direction)
{
    int pwm_val;
    int other_direction;
    int rc;

    Serial.print("\n#****************************************************************\n");
    Serial.print("# Discovering PWM speeds for ");
    Serial.print(joint_names[joint]);
    Serial.print(" ");
    Serial.println(direction_names[direction]);
    Serial.print("#****************************************************************\n");

    if (direction == IN)
        other_direction = OUT;
    else
        other_direction = IN;

    pwm_val = LOW_PWM_MOVEMENT(joint + direction) - 10;

    if (pwm_val == -1) {
        Serial.println("**************** I do not have a low PWM value for this joint!\n");
        return -1;
    }
    Serial.print("# This joint will first move at ");
    Serial.print(pwm_val);
    Serial.print("%\n");

    pwm_val += 9;
    pwm_val = (pwm_val / 10) * 10;

    for (;pwm_val <= 100;pwm_val += 10) {
        Serial.println("# Moving joint to position.");
        /* This might fail if the joint's already in position. */
        if (exercise_joint(joint, other_direction, LOW_PWM_MOVEMENT(joint + direction) + 5, 0) == -1)
            return -1;
        Serial.print("# Done , waiting...\n");
        delay(1000);

        Serial.println("");
        Serial.print("# Testing at ");
        Serial.print(pwm_val);
        Serial.print("%\n");
        rc = exercise_joint(joint, direction, pwm_val, 1);
        if (rc == -1) {
            Serial.print("exercise_joint() failed.\n");
            return -1;
        }
        Serial.print("# ");
        Serial.print(pwm_val);
        Serial.print("% PWM gave ");
        Serial.print(joint_names[joint]);
        Serial.print(" movement at speed ");
        Serial.println(rc);
        /* XXX fixme:  joint speed needs to be in larger units, like sensor units/sec. */
        JOINT_SPEED(joint + direction, pwm_val / 10) = rc;
        Serial.print("# Done, waiting...\n\n");
        delay(1000);
    }

    Serial.println("\nDone finding joint PWM speeds\n\n");

    return 0;
}

int calibrate(void)
{
    int i;

    set_pwm_scale(100);

    enable_leg();

    /* Get thigh and knee into place. */

    /* I should use a low speed PWM value if I have one. */
    Serial.println("# Retracting thigh.");
    if (move_joint_all_the_way(THIGHPWM_UP, 50) == -1)
        return -1;

    Serial.println("# Done, waiting...");
    delay(1000);

    Serial.println("# Retracting knee.");
    /* Move knee up. */
    if (move_joint_all_the_way(KNEEPWM_EXTEND, 50) == -1)
        return -1;

    Serial.println("# Done, waiting...");
    delay(1000);

    /*******/
    /* HIP */
    /*******/

    /* Find the lowest PWM value at which the hip moves in. */
    /* This might fail if the joint's already against the stop. */
    i = find_joint_first_movement(HIP, IN);
    if (i == -1)
        return -1;
    LOW_PWM_MOVEMENT(HIP + IN) = i;
    if (i != 0) {
        /* Find the low sensor limit. */
        if (find_joint_sensor_limit(HIP, IN, LOW_PWM_MOVEMENT(HIP + IN)))
            return -1;
    }

    /* Find the lowest PWM value at which the hip moves out. */
    i = find_joint_first_movement(HIP, OUT);
    if (i == -1)
        return -1;
    LOW_PWM_MOVEMENT(HIP + OUT) = i;

    /* Find the high sensor limit. */
    if (find_joint_sensor_limit(HIP, OUT, LOW_PWM_MOVEMENT(HIP + OUT)))
        return -1;

    /*
     * The first try at HIP IN might have failed if the joint started
     * out already at its end limit, so retry if necessary.
     */
    if (LOW_PWM_MOVEMENT(HIP + IN) == 0) {
        i = find_joint_first_movement(HIP, IN);
        LOW_PWM_MOVEMENT(HIP + IN) = i;
        if (find_joint_sensor_limit(HIP, IN, LOW_PWM_MOVEMENT(HIP + IN)))
            return -1;
    }

    /* The hip could be in or out at this point. */

    /* XXX fixme:  I should have some way to park the hip in the middle of its travel. */

    /* discover PWM -> speed mapppings. */
    if (find_joint_pwm_speeds(HIP, IN) == -1)
        return -1;

    if (find_joint_pwm_speeds(HIP, OUT) == -1)
        return -1;

    Serial.println("# Done, waiting...");
    delay(1000);

    /***********/
    /* END HIP */
    /***********/

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
        Serial.print(LOW_PWM_MOVEMENT(i + IN));
        Serial.println(" percent");

        Serial.print("# ");
        Serial.print(joint_names[i]);
        Serial.print(" first moved out at ");
        Serial.print(LOW_PWM_MOVEMENT(i + OUT));
        Serial.println(" percent");
    }

    Serial.print("\n");

    return 0;
}
