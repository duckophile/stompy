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
 * Move a joint associated with a specified valve until it stops
 * moving.
 *
 * Returns the last sensor reading from the joint, or -1 on failure.
 *
 * The valve number is 0-5.
 *
 * This should take a joint (0-2) and a direction.
 */
int move_joint_all_the_way(int valve)
{
    int slow_pwm = 40; /* Some good slow running speed. */
    int out = 0;
    int no_change_count = 0;
    int sensor_num;
    int sensor_reading;
    int last_sensor_reading = 0;
    int i;

    sensor_num = valve;
    /* Sensor number is 0-2. */
    if (sensor_num > 3) {
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
    set_pwm(valve, slow_pwm);

    /* Look for movement, wait until it stops. */
    sensor_reading = read_sensor(sensorPin[sensor_num]);
    last_sensor_reading = sensor_reading;
    /* 100 readings of 100ms each. */
    for (i = 0;i < 100;i++) {
        check_deadman();

        no_change_count++;
        sensor_reading = read_sensor(sensorPin[sensor_num]);
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

    if (i == 100)
        return -1;
    else
        return last_sensor_reading;
}

int calibrate(void)
{
    Serial.println("Retracting knee.");
    /* Move knee up. */
    move_joint_all_the_way(KNEEPWM_RETRACT);

    disable_leg();

    Serial.println("Done with calibration.");

    return 0;
}
