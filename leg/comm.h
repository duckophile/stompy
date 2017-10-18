/* ; -*- mode: C ;-*- */

#ifndef __COMM_H__
#define __COMM_H__

/* Controller/leg Comm protocol */

/*
 * This is a proposal for a communication protocol to talk to a teensy
 * on a leg.  It's meant to be both simple in the simplest casse yet
 * extensible to any level of complexity we want.
 *
 * There are shortcuts (indicated by "*** SHORTCUT") to make a simple
 * implementation easy with a minimum of effort and error handling.
 *
 * (*** SHORTCUTS):
 *   command packet:
 *     type has to be set.
 *     cksum = 0 menas don't check checksum.
 *
 *   point list:
 *     sequence = 0 means go to this point if possible and hope it
 *     works, regardless of preious sequence number.
 *
 */

/*
 * Main command packet - this encapsulates every request sent from the
 * control node to a leg.
 *
 * type:
 *
 * The 'type' field can double as a packet type or protocol vrsion
 * number.  The important part is that it's untypeable non-ASCII
 * (presumably meaning between decimal 129 and 254 (0x81 - 0xFE),
 * allowing command packets and typed ascii commands to be sent over
 * the same serial line.
 *
 * One type of packet, for example, will provide an array of points
 * for the leg to move to.  Another might query the leg to find its
 * current location.  Other possibilities include resetting the leg,
 * telling it to go to its home position, querying its calibration
 * parameters, setting up the lights for the leg, etc.
 *
 * For protocol versioning, If the 'array of points' command, for
 * example, turns out to have shortcomings, then, if necessary, we
 * could define a new command type for the new improved command and
 * obsolete the old one, but most likely we'd just update the
 * controller and all legs simultaneously and not need to screw with
 * versioning.
 *
 * 'type' could be point_list, point_list_reply, query, query_reply,
 * calibration_params, calibration_params_reply, reset, reset_reply,
 * home, home_reply, etc.
 *
 * cksum:
 *
 * This is an 8 bit checksum of the whole packet to verify packet
 * integrity.  If the checksum is non-zero and does not match then
 * (insert handwaving here).  Probably some sort of error response.
 * (*** SHORTCUT) If the field is zero then it is ignored.
 *
 * size:
 *
 * The size of the packet, in 32 bit words.  The packet will be padded
 * to the next 32 bit boundary and padded with zeros, or, if we're
 * lazy, whatever crap already happens to be in memory, as long as
 * it's included in the checksumm (if the checksum is being used).
 * 256 words limits us to a 1024 byte packet.  Is this big enough?
 *
 * condiment:
 *
 * A leftover byte.  Maybe we can expand the size to 16 bits, or send
 * Burma Shave jingles one byte at a time.
 *
 * data:
 *
 * This holds the packet described by the 'type' field, as listed
 * below.
 */

typedef struct {
    uint8_t type;		/* Some non-ASCII magic number to identify the payload of the packet. */
    uint8_t cksum;		/* 8 bit checksum for the whole command packet. */
    uint8_t size;		/* Size of the packet, in 32 bit words, including this header. */
    uint8_t condiment;		/* Robin suggested this, in case the packet doesn't taste good. */

    uint8_t data[];		/* The data for the packet described by 'type'. */
} leg_cmd_t;

/*
 * Types of packet:
 *  Point list
 *  Query
 *  Stop
 *  Emergency stop
 */

/*
 * POINT_LIST:
 *
 * A packet describing a list of points for the leg to move to and a
 * speed at which to move.
 *
 * sequence:
 *
 * This indicates the next point in a sequence.  Points can be
 * replaced by subsequent packets with the same sequence numbers.
 * I.E., if one packet has a list of points numbered {1, 2, 3, 4, 5,
 * 6, 7} and the next packet has a list of points numbered {4, 5, 6,
 * 7, 8} then the points {4, 5, 6, 7} from the first packet will be
 * replaced by the same numbered points from the second packet, for
 * the points in the first packet that haven't been started yet.
 *
 * (*** SHORTCUT) Sequence 0 is always executed next.  A bunch of
 * packets with sequeence 0 will be executed in order in the most
 * brainless manner possible.  A point list with a bunch of sequence 0
 * points will hopefully be exectuted in order, but maybe not, no
 * guarantees.
 *
 * The fields should be self-explanatory.  Speed is either 0-100 or
 * 0-255, depending on what we think is best.  I don't think that any
 * finer granularity is warranted, and 0 doesn't make sense.  X, Y, Z
 * are 32 bit floats describing foot position in inches, presumably in
 * robot space.
 *
 * 73 of these can fit in 1024 bytes.
 */



typedef struct {
    float xyz[3];
    uint8_t sequence;
    uint8_t speed;
} point_list_t;

/*
 * POJNT_LIST_REPLY:
 *
 * This returns the squence number of first point in the last
 * point_list command that was accepted (I.E. the leg had not already
 * started movement to the position described by that point - or maybe
 * we can have uit abort motion in progress in the case of long
 * movements?) as well as the last sequence number that was accepted
 * (perhaps the leg node maintains a smaller list of points than the
 * controller would like?  Or one of the points was out of range?)
 *
 * (*** SORTCUT) Return sequence number 0 for either first_cmd or
 * last_cmd as and way of saying "Ok, I'll try to do what you asked, I
 * hope it works our, but I'm not making any promises".
 */

typedef struct {
    uint8_t status;		/* Success or failure? */
    uint8_t first_cmd;		/* First point sequence number accepted. */
    uint8_t last_cmd;		/* Last point sequence number  accepted. */
} point_list_reply_t;


/*
 * Status/info query?
 *
 * Maybe there should be multiple types of query?
 */

typedef struct query {
    /* No data needed? */
} query_t;

typedef struct query_reply {
    /*
     * Return:
     *
     * current position in inches?
     * Current sensor values?
     * Current angles?
     * Last sequence number started?
     * Last sequence number in queue?
     * Whips, chains, whistles, dildos, and a book.
     */
} query_reply_t;

/*
 * Other shit.
 *
 * Stop, emergency stop, calibrate, control leg lights, etc.
 */

int get_next_point(double xyz[3], int *speed, int do_repeat);
int queued_point_count(void);
int queue_point_list(point_list_t *pl, int pl_size);
void cancel_point_list(void);
int generate_circle_point_list(point_list_t *pl, int speed);
int generate_triangle_point_list(point_list_t *pl, int speed);

#endif /* __COMM_H__ */
