/* ; -*- mode: C ;-*- */

/*
 * Things I want to store in flash:
 *
 * Sensor min and max for three sensors.
 *
 * PWM scaling factor to scale speed to PWM.
 *
 * Angle min and max (could be slightly different per leg?).
 *
 * Foot x,y,z min and max.
 *
 * Which joint I am.
 *
 * Sensor jitter?
 *
 * Firmware rev.
 */

/*
 * TODO:
 *
 * I should verify the forward and inverse kinematics every boot.
 */

/*
 * XXX fixme:  info command should print out more stuff - goals, sensor goals, etc.
 *
 * XXX fixme: When I exit joystick mode I should set the goal to the
 * current position!
 * Or, really, joystick mode should move the goal, not actuate the PWMs.
 *
 * The command interpreter should be able to use a regular serial terminal.
 */

/*  Inverse kinematics for stompy
 *   theta1 = arctan(y/x)
 *   theta2 = arctan(z/x1) + arccos([L2^2 + r^2 - L3^2] / [2*L2*r])
 *        where: x1 = [y/sin(theta1)] - L1
 *               r = z/[sin(beta)]
 *               beta = arctan(z/x1)
 *               alpha = arccos[(L2^2 + r^2 - L3^2) / (2*L2*r)]
 *   theta3 = arccos[(L3^2 + L2^2  - r^2) / [2*L3*L2]
 *
 *   There are three cases that must be caught for this math to work.
 *   1) If theta1 (hip angle) equals zero then x1 = (x - L1).
 *   2) If z equals zero then r = x1.
 *   3) If x = 0 then the hip angle is degenerate so I add .0001" to x when x = 0 to avoid these degenerate cases.
 */

/*
 * to convert deg to rad and rad to deg
 *  radians = (degrees * 71) / 4068
 *  degrees = (radians * 4068) / 71
 */

#define KNEE_SENSOR_PIN		17
#define THIGH_SENSOR_PIN	18
#define HIP_SENSOR_PIN		20

/*
 * XXX fixme: This should be ordered such that the first three have
 * the sensor reading going down and the last three have it going up,
 * but I'm not sure if that's true or if that can be made to work.
 */
#define HIPPWM_FORWARD		0
#define THIGHPWM_UP		1
#define KNEEPWM_RETRACT		2
#define HIPPWM_REVERSE		3
#define THIGHPWM_DOWN		4
#define KNEEPWM_EXTEND		5

//These are mapped to the right front leg
#define THIGHPWM_DOWN_PIN	3
#define THIGHPWM_UP_PIN		4
#define KNEEPWM_RETRACT_PIN	5
#define KNEEPWM_EXTEND_PIN	6
#define HIPPWM_FORWARD_PIN	9
#define HIPPWM_REVERSE_PIN	10

/*
 * *_up and *_down are poor names.
 *
 * These are ordered for the easiest joystick mapping.
 *
 * SHIT!  I reordered these to match the joystick, but that probably
 * messed up positional!
 */
#if 1
/* This is the old ordering. */
const int up_pwms[3]   = {HIPPWM_REVERSE_PIN,  THIGHPWM_UP_PIN,   KNEEPWM_EXTEND_PIN};
const int down_pwms[3] = {HIPPWM_FORWARD_PIN,  THIGHPWM_DOWN_PIN, KNEEPWM_RETRACT_PIN};
const int pwm_pins[6]  = {HIPPWM_REVERSE_PIN,  THIGHPWM_UP_PIN,   KNEEPWM_EXTEND_PIN,
                          HIPPWM_FORWARD_PIN,  THIGHPWM_DOWN_PIN, KNEEPWM_RETRACT_PIN};
#else
const int up_pwms[3]   = {HIPPWM_FORWARD_PIN,  THIGHPWM_UP_PIN,   KNEEPWM_RETRACT_PIN};
const int down_pwms[3] = {HIPPWM_REVERSE_PIN,  THIGHPWM_DOWN_PIN, KNEEPWM_EXTEND_PIN};
const int pwm_pins[6]  = {HIPPWM_FORWARD_PIN,  THIGHPWM_UP_PIN,   KNEEPWM_RETRACT_PIN,
                          HIPPWM_REVERSE_PIN,  THIGHPWM_DOWN_PIN, KNEEPWM_EXTEND_PIN};
#endif

#define DEADMAN_PIN		0

//enable pins for motor divers
#define ENABLE_PIN		1
#define ENABLE_PIN_HIP		2

#define M1FB_PIN		21
#define M2FB_PIN		22
#define M1FB_HIP_PIN		23


static int debug_flag = 0;  /* Enable verbose output. */
static int old_debug_flag = 0;
static int periodic_debug_flag = 0;
#define DEBUG   if(debug_flag)Serial.print
#define DEBUGLN if(debug_flag)Serial.println

#define HIP	0
#define THIGH	1
#define KNEE	2

const char *joint_names[]        = {"hip",     "thigh", "knee"};
const char *joint_up_actions[]   = {"back",    "up",    "out"};
const char *joint_down_actions[] = {"forward", "down",  "in"};

// Block for reading sensors and setting the PWM to drive the solenoids
#define BIT_RESOLUTION pow(2,10)-1

int sensorGoal[3]    = {0,0,0};
int goingHot[3]      = {0,0,0};
float xyz_goal[3]    = {0,0,0};
float goal_angles[3] = {0, 0, 0};

int sensorReading[3] = {0, 0, 0};

int sensorPin[3] = {HIP_SENSOR_PIN, THIGH_SENSOR_PIN, KNEE_SENSOR_PIN};
// this is the distance in sensor reading that is close enough for directed movement
// I am putting this here so we can avoid chasing our tails early in positional control
int closeEnough = 2;

int joystick_mode = 0;

//Deadman button -- with the joystick I'll be using a momentary switch
//on the panel.  It will need to be held down or jumpered to set the
//joystick as "hot".

int deadMan = 0; //JOYSTICK is OFF if pin is low (pull high to enable joystick).
int deadman_forced = 0; /* Ignore the deadman if this is set. */

int current_reading_pin[] = {M2FB_PIN, M1FB_PIN, M1FB_HIP_PIN};

//values for home position of leg
//x home is knee fully retracted (cylindar fully extended) i.e. large pot value
//y home is thigh fully up (cylindar fully retracted) i.e. small pot value
//z home is hip in the ~middle i.e. mid pot value
int homePosition[] = {900, 200, 350};

//Sensor reading to angle of joint block
//These sensor values are for the right front leg

int hipPotMax = 722;
int hipPotMin = 93;
float hipAngleMin = -40.46;
float hipAngleMax = 40.46;
float hipSensorUnitsPerDeg;

int thighPotMax = 917;
int thighPotMin = 34;
float thighAngleMin = -6;
float thighAngleMax = 84;
float thighSensorUnitsPerDeg;

int kneePotMax = 934;
int kneePotMin = 148;
float kneeAngleMin = 13;
float kneeAngleMax = 123;
float kneeSensorUnitsPerDeg;

/*
 * Angles in degrees -
 * Hip is zero when straight out from the robot body.
 * Hip angle is zero when the thigh is parallel to the ground
 * Knee angle is between thigh and calf (13 degrees fully retracted)
 */
float current_deg[3];
float current_rad[3];

// kinematics block
/*  Forward kinematics for stompy
    x = cos(theta1) * [L1 + L2*cos(theta2) + L3*cos(theta2 + theta3 -180deg)]
    y = x * tan(theta1)
    z = [L2 * sin(theta2)] + [L3 * sin(theta2 + theta3 - 180deg)]
*/

//leg link lengths hip, thigh, and knee

#define L1 11
#define L2 54
/* Knee to ankle */
#define L3 72

float currentX;
float currentY;
float currentZ;

/*
 * Take three threadings and return the middle one.  This should
 * remove some sensor jitter.
 */
int read_sensor(int pin)
{
    int r1, r2, r3;
    int tmp;

    r1 = analogRead(pin);
    r2 = analogRead(pin);
    r3 = analogRead(pin);

#define SWAP(x,y) {tmp = (x); (x) = (y); (y) = tmp; }

    if (r3 < r2)
        SWAP(r2, r3);
    if (r2 < r1)
        SWAP(r1, r2);
    if (r3 < r2)
        SWAP(r2, r3);

    return r2;
}

void print_xyz(float x, float y, float z)
{
    Serial.print("(");
    Serial.print(x);
    Serial.print("\t");
    Serial.print(y);
    Serial.print("\t");
    Serial.print(z);
    Serial.print(")");

    return;
}

int func_none(void)
{
    Serial.println("Not implemented.");

    return 0;
}

/* Set PWM scaling factor. */
int func_scale(void)
{
    int scale;

    scale = read_int();

    set_pwm_scale(scale);

    return 0;
}

/* Set PWM frequency. */
int func_freq(void)
{
    int freq;

    freq = read_int();

    set_pwm_freq(freq);

    return 0;
}

int func_debug(void)
{
    if (debug_flag)
        debug_flag = 0;
    else
        debug_flag = -1; /* -1 = Permanently on. */
    Serial.print("Debug turned ");
    Serial.println(debug_flag ? "on" : "off");

    return 0;
}

void dbg_n(int n)
{
    old_debug_flag = debug_flag;
    debug_flag = n;
}

int func_dbg(void)
{
    periodic_debug_flag = !periodic_debug_flag;

    return 0;
}

extern int current_pwms[];

void print_current(void)
{
    int i;

    Serial.print("Current pwms:   ");
    for (i = 0;i < 3;i++) {
        Serial.print("\t");
        Serial.print(joint_names[i]);
        Serial.print("\t");
        Serial.print(current_pwms[i]);
    }
    Serial.println("");
    Serial.print("Current pwms:   ");
    for (i = 3;i < 6;i++) {
        Serial.print("\t");
        Serial.print(joint_names[i - 3]);
        Serial.print("\t");
        Serial.print(current_pwms[i]);
    }
    Serial.println("");

    Serial.print("Current angles: ");
    for (i = 0; i < 3; i++) {
        Serial.print("\t");
        Serial.print(joint_names[i]);
        Serial.print("\t");
        Serial.print(current_deg[i]);
    }
    Serial.println("");

    Serial.print("Current sensors:");
    for (i = 0; i < 3; i++) {
        Serial.print("\t");
        Serial.print(joint_names[i]);
        Serial.print("\t");
        Serial.print(sensorReading[i]);
    }
    Serial.println("");

    Serial.print("Current XYZ:    \t");
    Serial.print(currentX);
    Serial.print("\t");
    Serial.print(currentY);
    Serial.print("\t");
    Serial.print(currentZ);
    Serial.println("");

    return;
}

void print_goals(void)
{
    int i;

    Serial.print("Sensor Goals:    ");
    for (i = 0;i < 3;i++) {
        Serial.print("\t");
        Serial.print(joint_names[i]);
        Serial.print("\t");
        Serial.print(sensorGoal[i]);
    }
    Serial.println("");

    Serial.print("XYZ goal:        ");
    for (i = 0;i < 3;i++) {
        Serial.print("\t");
        Serial.print(xyz_goal[i]);
    }
    Serial.println("");

    return;
}

int func_info(void)
{
    int i;

    Serial.println("================================================================");
    Serial.println("Current info:\n");
    Serial.print("deadMan: ");
    Serial.print(deadMan);
    Serial.print(" deadman forced: ");
    Serial.print(deadman_forced);
    Serial.print(" goingHot: ");
    for (i = 0;i < 3;i++)
        Serial.print(goingHot[i]);
    Serial.println("");

    print_current();
    Serial.println("");
    print_goals();

    Serial.println("================================================================");
    Serial.println("");

    return 0;
}

int func_deadman(void)
{
    deadman_forced = !deadman_forced;

    Serial.print("Deadman ");
    Serial.println(deadman_forced ? "disabled" : "enabled");

    return 0;
}

int func_joystick(void)
{
    return toggle_joystick_mode();
}

/*
 * Stick the leg straight out and wiggle it up and down in hopes of
 * making it easier for air to get out.
 */
int func_bleed(void)
{
    return 0;
}

int func_stop(void)
{
    disable_leg();

    return 0;
}

int func_calibrate(void)
{
    return calibrate();
}

int func_go(void);

struct {
    const char *name;
    int (*func)();
} cmd_table[] = {
    { "bleed",     func_bleed     },
    { "calibrate", func_calibrate },
    { "deadman",   func_deadman   }, /* Ignore the deadman. */
    { "dbg",       func_dbg       }, /* Enable debug once. */
    { "debug",     func_debug     },
    { "freq",      func_none      }, /* Set PWM frequency. */
    { "help",      func_help      },
    { "dither",    func_none      }, /* Set the dither amount. */
    { "scale",     func_scale     }, /* Set max PWM value. */
    { "go",        func_go        }, /* goto given x, y, z. */
    { "home",      func_none      }, /* Some neutral position?  Standing positioon maybe? */
    { "info",      func_info      },
    { "joystick",  func_joystick  }, /* Enable jpoystick mode. */
    { "park",      func_none      }, /* Move leg to parked position. */
    { "stop",      func_stop      }, /* Stop moving. */
    { "where",     func_none      }, /* Print current x, y, z, and degrees. */
    { NULL,        NULL           }
};

char cmd_buf[128];
int cmd_len = 0;
char *cmd_ptr;

int read_int(void)
{
    int n;
    char *p;

    n = strtol(cmd_ptr, &p, 0);
    cmd_ptr = p;

    return n;
}

float read_float(void)
{
    float f;
    char *p;

    f = strtof(cmd_ptr, &p);
    cmd_ptr = p;

    return f;
}

void read_cmd(void)
{
    int len;
    int n;
    int caught = 0;
    int ch = 0;

    if (Serial.available() <= 0)
        return;

    while (Serial.available() > 0) {
        ch = Serial.read();

        if (ch == 24) { /* ^X */
            func_dbg();
            continue;
        }

        if (ch == 127) {
            if (cmd_len > 0) {
                cmd_len--;
                cmd_buf[cmd_len] = 0;
                Serial.print("\b \b");
            }
            continue;
        }

        if (cmd_len > 127)
            continue;

        Serial.print((char)ch);
        if (ch == '\r')
            Serial.print('\n');

        if ((ch == '\r') || (ch == '\n'))
            break;

        cmd_buf[cmd_len++] = ch;
        cmd_buf[cmd_len] = 0;
    }
    if ((ch != '\r') && (ch != '\n'))
        return;

    /* We have a newline, so read the cmd out of the buffer. */

    if (cmd_len == 0) {
        func_info();
        return;
    }

    for (n = 0; cmd_table[n].name != NULL;n++) {
        len = strlen(cmd_table[n].name);
        if (!strncmp(cmd_buf, cmd_table[n].name, len)) {
            cmd_ptr = cmd_buf + len;
            cmd_table[n].func();
            caught = 1;
        }
    }

    if (!caught) {
        Serial.print("Unknown command ");
        Serial.println(cmd_buf);
    }

    cmd_len = 0;

    return;
}

#if 0
int read_cmd(void)
{
    static char cmd[16];
    static char ch;
    static int size = 0;
    int done = 0;
    int caught = 0;
    int n;

    if (Serial.available() <= 0)
        return 0;

    while (Serial.available() > 0) {
        ch = Serial.read();
        if ((ch == ' ') || (ch == '\r') || (ch == '\n') || (size == 15)) {
            done = 1;
            break;
        }
        cmd[size++] = ch;
    }
    cmd[size] = 0;

    if (!done)
        return 0;

    if (size == 0) {
        func_info();
        return 0;
    }

    for (n = 0; cmd_table[n].name != NULL;n++)
        if (!strcmp(cmd, cmd_table[n].name)) {
            cmd_table[n].func();
            caught = 1;
        }

    if (!caught) {
        Serial.print("Unknown command ");
        Serial.println(cmd);
    }

    size = 0;
    if ((ch != '\r') || (ch == '\n')){
        while (((ch = Serial.read()) != '\r') && (ch != '\n'))
            Serial.print(ch);
        Serial.println(" <-- extra chars.");
    }

    return 1;
}
#endif

int func_help(void)
{
    for (int i = 0;cmd_table[i].name != NULL;i++) {
        Serial.print(cmd_table[i].name);
        Serial.print(" ");
    }
    Serial.println("");

    return 0;
}

/*
 * Takes an X,Y,Z location and calculates the joint angles in degrees
 * and radians that will give that location.
 *
 * Also Sets the sensor goals that correspond to those angles.
 *
 * Input is (x,y,z)
 * Output is desired  sensor readings in sensorGoal[HIP, THIGH, KNEE].
 *
 * XXX fixme: This should be two functions, one to convert (x,y,z) to
 * three angles, and another to convert that to sensor values.
 */
void inverse_kin(float x, float y, float z)
{
    float theta1R, theta2R, theta3R; /* Goal angles in degrees. */
    float theta1, theta2, theta3; /* Goal angles in radians. */
    float hipGoal, thighGoal, kneeGoal; /* Goal sensor values. */

    Serial.print("New goal (x,y,z): ");
    Serial.print("(");
    Serial.print(x);
    Serial.print(", ");
    Serial.print(y);
    Serial.print(", ");
    Serial.print(z);
    Serial.println(")");

    /* HIP - theta1 */
    theta1R = atan(y/x);
    //convert to degrees
    theta1 = (theta1R * 4068) / 71;
    //angle goal to pot reading
    hipGoal = ((theta1 - hipAngleMin) * hipSensorUnitsPerDeg) + hipPotMin;
    if (hipGoal < hipPotMin || hipGoal > hipPotMax) {
        Serial.println("HIP GOAL OUT OF RANGE!!");
        hipGoal = constrain(hipGoal, hipPotMin, hipPotMax);
        Serial.print("constrained goal: ");
        Serial.println(hipGoal);
    }
    goingHot[HIP] = 1;
    Serial.println("Hip going hot.");

    /* Thigh - theta2 */
    float r;
    float x1;
    if (theta1R == 0) {
        x1 = (x - L1);
    }
    else {
        x1 = (y/sin(theta1R)) - L1;
    }
    x1 = abs(x1);
    float beta = atan(z/x1);
    if (x == L1) {
        beta = -(PI/2);
    }
    else if (x < L1) {
        if (z == 0) {
            r = x1;
        }
        else {
            r = z/sin(beta);
        }
        r = abs(r);
        float gama = asin(x1/r);
        beta = -(gama + PI/2);
    }
    else {
        beta = atan(z/x1);

    }
    if (z == 0) {
        r = x1;
    }
    else {
        r = z/sin(beta);
    }
    r = abs(r);
    theta2R = beta + acos((sq(L2) + sq(r) - sq(L3))/(2*L2*r));
    theta2 = (theta2R * 4068) / 71;
    //thighGoal is sensor reading at goal angle
    thighGoal = thighPotMin + ((thighAngleMax - theta2) * thighSensorUnitsPerDeg);
    if (thighGoal < thighPotMin || thighGoal > thighPotMax) {
        Serial.println("THIGH GOAL OUT OF RANGE!!");
        thighGoal = constrain(thighGoal, thighPotMin, thighPotMax);
        Serial.print("constrained goal: ");
        Serial.println(thighGoal);
    }
    goingHot[THIGH] = 1;
    Serial.println("Thigh going hot.");

    /* Knee - theta3 */
    theta3R = acos( (sq(L3) + sq(L2) - sq(r)) / (float)(2*L3*L2));
    theta3 = (theta3R * 4068) / 71;
#if 0
    Serial.print("knee rad = ");
    Serial.print(theta3R);
    Serial.print(" which is from acos( ");
    Serial.print(sq(L3) + sq(L2) - sq(r));
    Serial.print(" / ");
    Serial.print((2*L3*L2));
    Serial.print(" ), or acos( ");
    Serial.print((sq(L3) + sq(L2) - sq(r)) / (float)(2*L3*L2));
    Serial.print(" ) ");

    Serial.print(" and ");
    Serial.print((2*L3*L2));
    Serial.print(" theta3 = ");
    Serial.print(theta3);
#endif
    kneeGoal= ((kneeAngleMax - theta3) * kneeSensorUnitsPerDeg) + kneePotMin;
    //kneeGoal = ((theta3 - kneeAngleMin) * kneeSensorUnitsPerDeg) + kneePotMin;
    if (kneeGoal < kneePotMin || kneeGoal > kneePotMax) {
        Serial.print("(KNEE GOAL OUT OF RANGE) ");
        // kneeGoal = constrain(kneeGoal, kneePotMin, kneePotMax);
        // Serial.print("constrained goal: ");
    }
    goingHot[KNEE] = 1;
    Serial.println("Knee going hot.");

    sensorGoal[HIP]   = hipGoal;
    sensorGoal[THIGH] = thighGoal;
    sensorGoal[KNEE]  = kneeGoal;

    goal_angles[HIP]   = theta1;
    goal_angles[THIGH] = theta2;
    goal_angles[KNEE]  = theta3;

    Serial.print("\nNew Goals: (x,y,z):\t");
    print_xyz(x, y, z);
    Serial.println("");
    Serial.print("Goal angles (deg):\t");
    print_xyz(theta1, theta2, theta3);
    Serial.println("");
    Serial.print("Sensor goals:\thip\t");
    Serial.print(sensorGoal[HIP]);
    Serial.print("\tthigh\t");
    Serial.print(sensorGoal[THIGH]);
    Serial.print("\tknee\t");
    Serial.print(sensorGoal[KNEE]);
    Serial.println("");

    return;
}

/*
 * Forward kinematics.
 *
 * Calculate joint angles in both degrees and radians, based on the
 * last sensor readings.
 *
 * Takes sensor readings from sensorReadings[] and calculates the
 * angles in degrees into current_deg[] and radians into
 * current_rad[].
 *
 * XXX fixme:  This should have the sensor readings passed in and return the angles.
 */
void calculate_angles(void)
{
    current_deg[HIP]   = ((sensorReading[HIP] - hipPotMin) / hipSensorUnitsPerDeg) + hipAngleMin;
    current_deg[THIGH] = thighAngleMax - ((sensorReading[THIGH] - thighPotMin) / thighSensorUnitsPerDeg);
    current_deg[KNEE]  = kneeAngleMax  - ((sensorReading[KNEE]  - kneePotMin)  / kneeSensorUnitsPerDeg);

    for (int i = 0;i < 3;i++)
        current_rad[i] = (current_deg[i] * 71) / 4068;

    DEBUG("Angles:\t");
    for (int i = 0; i < 3; i++) {
        /* Read sensors even if we're not moving. */
        DEBUG("\t");
        DEBUG(joint_names[i]);
        DEBUG("\t");
        DEBUG(current_deg[i]);
    }
    DEBUGLN("");

    return;
}

/*
 * Forward kinematics.
 *
 * Takes the current angles from current_rad[] and converts to the
 * current (x,y,z).
 */
void calculate_xyz(void)
{
    currentX = cos(current_rad[HIP]) * (L1 + L2*cos(current_rad[THIGH]) + L3*cos(current_rad[THIGH] + current_rad[KNEE] - PI));
    currentY = currentX * tan(current_rad[HIP]);
    currentZ = (L2 * sin(current_rad[THIGH])) + (L3 * sin(current_rad[THIGH] + current_rad[KNEE] - PI));

    DEBUG("Current\t\tX:\t");
    DEBUG(currentX);
    DEBUG("\tY:\t");
    DEBUG(currentY);
    DEBUG("\tZ:\t");
    DEBUG(currentZ);
    DEBUGLN("");

    return;
}

/*
 * Reads the sensors and sets the x,y,z goal to the current location.
 */
void reset_current_location(void)
{
    /* Set our goal to the current position. */
    read_sensors();
    calculate_angles();
    calculate_xyz();

    xyz_goal[0] = currentX;
    xyz_goal[1] = currentY;
    xyz_goal[2] = currentZ;
}

void setup(void)
{
    Serial.begin(9600);
    Serial.setTimeout(10);

    //setup analog wite resolution and PWM frequency
    analogWriteResolution(10);

    //max PWM freqency of the motor driver board is 20kHz
    set_pwm_freq(20000);

    //setup deadman, enable, and error digital I/O pins
    pinMode(DEADMAN_PIN, INPUT);
    pinMode(ENABLE_PIN, OUTPUT);
    digitalWrite(ENABLE_PIN, LOW);
    pinMode(ENABLE_PIN_HIP, OUTPUT);
    digitalWrite(ENABLE_PIN_HIP, LOW);

    pinMode(HIPPWM_REVERSE_PIN, OUTPUT);

    //sensor units per deg
    hipSensorUnitsPerDeg = (hipPotMax - hipPotMin) / (hipAngleMax - hipAngleMin);
    thighSensorUnitsPerDeg = (thighPotMax - thighPotMin) / (thighAngleMax - thighAngleMin);
    kneeSensorUnitsPerDeg = (kneePotMax - kneePotMin) / (kneeAngleMax - kneeAngleMin);

    interrupt_setup();

    disable_leg();

    delay(2000); /* Give the serial console time to come up. */

    reset_current_location();

    Serial.println("Ready to rock and roll!\n");

    return;
}

void print_reading(int i, int sensor, int goal, const char *joint, const char *action)
{
    Serial.print(i);
    Serial.print('\t');
    Serial.print(sensor);
    Serial.print('\t');
    Serial.print(goal);
    Serial.print('\t');
    Serial.print(joint);
    Serial.print(' ');
    Serial.println(action);
}

/* Reads the current leg position sensors. */
void read_sensors(void)
{
    DEBUG("Sensors:");
    for (int i = 0; i < 3; i++) {
        /* Read sensors even if we're not moving. */
/*        sensorReading[i] = analogRead(sensorPin[i]);*/
        sensorReading[i] = read_sensor(sensorPin[i]);
        DEBUG("\t");
        DEBUG(joint_names[i]);
        DEBUG("\t");
        DEBUG(sensorReading[i]);
    }
    DEBUGLN("");

    return;
}

/*
 * Write the PWMs based on the last read sensor values and the
 * position goal.
 */
void write_pwms(void)
{
    static int dbg_msg = 0;

    if ((deadMan == 0) && (!deadman_forced)) {
        if (!dbg_msg) {
            DEBUGLN("Deadman has leg disabled.");
            dbg_msg = 1;
        }
        return;
    }
    dbg_msg = 0;

    for (int i = 0; i < 3; i++) {
        DEBUG(joint_names[i]);
        if (goingHot[i] == 0) {
            DEBUGLN(" goingHot = 0");
            continue;
        }

        DEBUG(" sensor ");
        DEBUG(i);
        DEBUG(":\t");
        DEBUG(sensorReading[i]);
        DEBUG("\tgoal:\t");
        DEBUG(sensorGoal[i]);
        //compare sensor reading to goal and only move if not close enough
        if (abs(sensorReading[i] - sensorGoal[i]) >= closeEnough) {
            DEBUGLN("\tJoint is not close enough.");
            if (sensorReading[i] > sensorGoal[i]) {
                set_pwm_goal(i, 100); /* 100% is ambitious. */
/*                print_reading(i, sensorReading[i], sensorGoal[i], joint_names[i], joint_up_actions[i]);*/
            } else if (sensorReading[i] < sensorGoal[i]) {
                set_pwm_goal(i + 3, 100);
/*                print_reading(i, sensorReading[i], sensorGoal[i], joint_names[i], joint_down_actions[i]);*/
            }
        } else {
            Serial.print(joint_names[i]);
            Serial.print(" joint is close enough: ");
            Serial.print(" sensor ");
            Serial.print(i);
            Serial.print(":\t");
            Serial.print(sensorReading[i]);
            Serial.print("\tgoal:\t");
            Serial.print(sensorGoal[i]);

            /*
             * if joint was in motion and the goal was reached - turn
             * motion flag and solenoids off.
             *
             * I don't think the leg should go cold when it reaches
             * its destination.  It should set the PWMs based on the
             * desired speed, and the desired speed is 0.
             */
            goingHot[i] = 0;
            set_pwm_goal(i, 0);
            set_pwm_goal(i + 3, 0);

            Serial.println("\treached goal - going cold.");
        }
    }
}

int read_xyz(float *x, float *y, float *z)
{
    // look for first valid integar to be x
    *x = read_float();
    *y = read_float();
    *z = read_float();

    if (*x == 0) {
        *x = *x + .0001;
        //Serial.println("x = 0 so .0001 was added to avoid degenerate case");
    }

#if 0
    Serial.println("");
    Serial.print("I was given a goal of (x,y,z): ");
    print_xyz(*x, *y, *z);
    Serial.println("");
#endif

    return 0;
}

void check_deadman(void)
{
    int i;

    i = digitalRead(DEADMAN_PIN) || deadman_forced;

    if (i != deadMan) {
        /* Disable deadman override whenever deadman button changes. */
/*        deadman_forced = 0;*/
        if (i == 0) {
            /*
             * if deadman is off -- joystick is deactivated, then disable
             * driver boards and write zero to PWM lines
             */
            disable_leg();
            Serial.println("Deadman off - leg disabled.");
        } else {
            enable_leg();
            Serial.println("Deadman on - leg enabled.");
        }
        deadMan = i;
    }

    return;
}

void thing(int n)
{
    static int done = 0;

    if (joystick_mode == 1)
        done = 0;

    if (done)
        return;
    Serial.print(n);
    Serial.print(" ");
    Serial.print(joystick_mode);
    Serial.println("");
    if (joystick_mode == 0)
        done = 1;
}

void loop()
{
    static int last_seconds = 0;
    int seconds;
    static float last_x_inches, last_y_inches, last_z_inches;

    /*
     * If debug_flag > 0 then it specifies the number of times through
     * this loop to print debugging output.
     */
    if (debug_flag > 0) {
        debug_flag--;
        if (debug_flag == 0) {
            debug_flag = old_debug_flag;
            old_debug_flag = 0;
        }
    }

    if (periodic_debug_flag) {
        seconds = micros() / 500000;
        if (last_seconds != seconds) {
            dbg_n(1);
            last_seconds = seconds;
        }
    }

    /* Check for any input on the console. */
    read_cmd();

    check_deadman();

    /* Read the sensors. */
    read_sensors();
    /* Turn sensor readings into joint angles. */
    calculate_angles();
    /* Turn joint angles into (x,y,z). */
    calculate_xyz();

    /* Joystic mode should move the desired x,y,z, but only after I fix positional. */
    if (joystick_mode) {
        /* In joystick mode the joystick controls the PWMs. */
        do_joystick();
    } else {
        /* Write something to the PWMs to make the leg move. */
        write_pwms();
    }

    /*
     * If the foot has moved an inch in any direction then print the
     * current locatioin.
     */
    if ((abs(currentX - last_x_inches) > 1) ||
        (abs(currentY - last_y_inches) > 1) ||
        (abs(currentZ - last_z_inches) > 1))
    {
#if 0
        Serial.print("Current (x,y,z): ");
        print_xyz(currentX, currentY, currentZ);
        Serial.print(" Goal: ");
        print_xyz(xyz_goal[0], xyz_goal[1], xyz_goal[2]);
        Serial.println("");
#endif
/*
        Serial.print(" current angles (hip, thigh, knee): ");
        print_xyz(current_deg[HIP], current_deg[THIGH], current_deg[KNEE]);
        Serial.println("");
*/

        last_x_inches = currentX;
        last_y_inches = currentY;
        last_z_inches = currentZ;
    }

    // A short delay so I can read the serial while programing.
/*    delay(10);*/

    return;
}

/*
 * The 'go' command.
 *
 * Takes 3 floats for (x,y,z) goal.
 */
int func_go(void)
{
    float x_goal, y_goal, z_goal;

    joystick_mode = 0;

    Serial.println("\n----------------------------------------------------------------");

    old_debug_flag = debug_flag;
    debug_flag = 1;	/* Enable debugging for one loop. */

    /* Read x,y,z from console, and update thetas if there's a new one. */
    read_xyz(&x_goal, &y_goal, &z_goal);
    inverse_kin(x_goal, y_goal, z_goal);

    xyz_goal[0] = x_goal;
    xyz_goal[1] = y_goal;
    xyz_goal[2] = z_goal;

    for (int i = 0; i < 3; i++)
        goingHot[i] = 1;

    Serial.println("----------------------------------------------------------------\n");

    return 0;
}

/*
 * This makes sure the driver board is not enabled if the joystick is
 * turned off (deadMan is low).
 *
 * This sets goingHot to 0, which means the leg will not try to move
 * until it's given another goal, even if it's the same as the current
 * goal.
 */
void disable_leg()
{
    digitalWrite(ENABLE_PIN, LOW);
    digitalWrite(ENABLE_PIN_HIP, LOW);
    pwms_off();
    for (int i = 0; i < 3; i++)
        goingHot[i] = 0;
    //Serial.println("Joystick is deactivated -- all off!");
    Serial.println("Leg disabled.");
}

void enable_leg(void)
{
    digitalWrite(ENABLE_PIN, HIGH);
    digitalWrite(ENABLE_PIN_HIP, HIGH);

    Serial.println("Leg enabled.");

    return;
}
