/* ; -*- mode: C ;-*- */

#include <stdio.h>
#include <stdint.h>
#include <Arduino.h>
#include <math.h>
#include "comm.h"
#include "interrupt.h"
#include "kinematics.h"

static point_list_t *point_list;
static leg_cmd_t *leg_cmd;

static int point_list_size;
static int point_list_idx;

#if 0
typedef struct {
    uint8_t type;		/* Some non-ASCII magic number to identify the payload of the packet. */
    uint8_t cksum;		/* 8 bit checksum for the whole command packet. */
    uint8_t size;		/* Size of the packet, in 32 bit words, including this header. */
    uint8_t condiment;		/* Robin suggested this, in case the packet doesn't taste good. */

    uint8_t data[];		/* The data for the packet described by 'type'. */
} leg_cmd_t;

typedef struct {
    float xyz[3];
    uint8_t sequence;
    uint8_t speed;
} point_list_t;
#endif

/*
 * I need some way to free the memory occupied by a point list when
 * it's done, or some method to notify that it's done.  Or maintain a
 * circular list of points.
 */

/*
 * Report how many points are queued.
 */
int queued_point_count(void)
{
    return point_list_size - point_list_idx;
}

/*
 * Get the next point from the point list.
 */
int get_next_point(double xyz[3], int *speed, int do_repeat)
{
    int i;

    if (point_list_size == 0)    /* Nothing queued. */
        return -1;

    for (i = 0;i < 3;i++)
        xyz[i] = point_list[point_list_idx].xyz[i];
    *speed = point_list[point_list_idx].speed;

    point_list_idx++;
    if (point_list_idx == point_list_size) {
        Serial.print("# Repeating point list.\n");
        if (do_repeat) {
            point_list_idx = 0;
        } else {
            Serial.print("# Done with point list.\n");
            point_list_size = 0;
            point_list_idx = 0;
            point_list = NULL;
            leg_cmd = NULL;
        }
    }

    return 0;
}

/*
 * Puts a point list into the queue for execution.
 */
int queue_point_list(point_list_t *pl, int pl_size)
{
    int old_int_state;

    old_int_state = set_interrupt_state(0);

    point_list = pl;
    point_list_size = pl_size;
    point_list_idx = 0;

    set_interrupt_state(old_int_state);

    return 0;
}

void cancel_point_list(void)
{
    if (point_list != NULL)
        Serial.print("# Cancelling point list.\n");

    point_list_size = 0;
    point_list_idx = 0;
    point_list = NULL;
    leg_cmd = NULL;

    return;
}

/*
 * Generates a point list describing a circle.
 */
int generate_circle_point_list(point_list_t *pl, int speed)
{
    double deg;
    double xyz[3];
    int idx = 0;
    int sensors[3];
    double degrees[3];

    for (deg = 0; deg < (PI * 2); deg += ((PI * 2) / 64)) {
        /* The center of the circle is roughly the center of the sensors. */
#if 0
        xyz[0] = 0;
        xyz[1] = 74;
        xyz[2] = -35;
#endif
        xyz[0] = 85;
        xyz[1] = 0;
        xyz[2] = 0;
        /* 10" radius circle. */
        xyz[0] += 0;
        xyz[1] += (sin(deg) * 20) + 10;
        xyz[2] += (cos(deg) * 20) + 10;

        if (inverse_kin(xyz, sensors, degrees)) /* Just to sanity check the point. */
            Serial.print("ERROR: Point is outside valid range!\n");

        pl[idx].xyz[0] = xyz[0];
        pl[idx].xyz[1] = xyz[1];
        pl[idx].xyz[2] = xyz[2];
        pl[idx].speed = speed;

        Serial.print("# Point ");
        Serial.print(idx);
        Serial.print(" = (");
        Serial.print(xyz[0]);
        Serial.print(", ");
        Serial.print(xyz[1]);
        Serial.print(", ");
        Serial.print(xyz[2]);
        Serial.print(")\n");

        idx++;
    }

    queue_point_list(pl, 64);

    return idx;
}

/*
 * Generates a point list describing a triangle
 */
int generate_triangle_point_list(point_list_t *pl, int speed)
{
    double deg;
    double xyz[3];
    int idx = 0;
    int sensors[3];
    double degrees[3];

    for (deg = 0; deg <= (PI * 2); deg += ((PI * 2) / 3)) {
        /* The center of the triangle is roughly the center of the sensors. */
#if 0
        xyz[0] = 0;
        xyz[1] = 74;
        xyz[2] = -35;
#endif
        xyz[0] = 85;
        xyz[1] = 0;
        xyz[2] = 0;
        /* 10" radius triangle. */
        xyz[0] += 0;
        xyz[1] += (sin(deg) * 20) + 10;
        xyz[2] += (cos(deg) * 20) + 10;

        if (inverse_kin(xyz, sensors, degrees)) /* Just to sanity check the point. */
            Serial.print("ERROR: Point is outside valid range!\n");

        pl[idx].xyz[0] = xyz[0];
        pl[idx].xyz[1] = xyz[1];
        pl[idx].xyz[2] = xyz[2];
        pl[idx].speed = speed;

        Serial.print("# Point ");
        Serial.print(idx);
        Serial.print(" = (");
        Serial.print(xyz[0]);
        Serial.print(", ");
        Serial.print(xyz[1]);
        Serial.print(", ");
        Serial.print(xyz[2]);
        Serial.print(")\n");

        idx++;
    }

    queue_point_list(pl, 4);

    return idx;
}
