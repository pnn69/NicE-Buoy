#ifndef COMPASS_H_
#define COMPASS_H_

#include <Arduino.h>
#include <RoboCompute.h>

extern QueueHandle_t compass;
extern QueueHandle_t compassIn;

template <typename T>
struct vector_t
{
    T x, y, z;
};

// Stores min and max magnetometer values from calibration

bool InitCompass(void);
void initcompassQueue(void);
bool CalibrateCompass(void);
float GetHeading(void);
float GetHeadingIron(void);
float GetHeadingRaw(void);
float CompassAverage(float in);
void CompassTask(void *arg);
int linMagCalib(int *corr);

// Guided eight point compass calibration. See the block comment in compass.cpp: the rules live
// there once, and every interface drives the same session rather than repeating the arithmetic.
//
// seq is the press serial, and it is what makes a press safe on a link that repeats it. Every LoRa
// receiver repeats what it hears once, and a Top bridges a UDP command it is not the target of onto
// the air - so one press was measured coming back six more times across 8 seconds. A copy arriving
// later than the Top's own duplicate window used to be applied a SECOND time, and for BEGIN that
// means silently wiping every capture the operator has already made. Numbering all four actions,
// not just SET, is what lets a stale copy be told from a fresh press.
//
// Pass 0 for a press with no retry behind it, such as the Sub's own web page: 0 means "unnumbered"
// and is always applied. Returns false when the press was rejected as a repeat or out of step.
bool cal8Begin(unsigned int seq = 0);
bool cal8Cancel(unsigned int seq = 0);

// Why a capture did not happen. Negative so cal8Set() can return the filled index on success, and
// distinct so an interface can say which of them it was - "the press did nothing" is the one
// answer an operator cannot act on.
typedef enum
{
    CAL8_SET_NO_SESSION = -1,  // no run is armed
    CAL8_SET_BAD_LEG    = -2,  // not the direction being asked for, and not one already captured
    CAL8_SET_DUPLICATE  = -3,  // this press serial has already been applied, or is out of step
    CAL8_SET_OFF_ANCHOR = -4   // N pressed while Imag is further than CAL8_ANCHOR_TOL_DEG from zero
} cal8_set_result_t;

// Capture the direction named by leg (0..7 = N, NE, ... NW), returning leg or a cal8_set_result_t.
// seq is the press serial that makes a retried press safe; pass 0 when there is no retry behind the
// press. See cal8Set() in compass.cpp.
int  cal8Set(int leg, unsigned int seq = 0);
bool cal8Save(unsigned int seq = 0); // commits the table; never touches compassOffset

extern bool cal8_active;
extern int cal8_next;        // the direction being asked for, 0..7, or 8 when every one is in
extern uint8_t cal8_mask;    // bit i set once direction i has been captured
extern uint16_t cal8_seq;    // serial of the last press applied
// The serial a presser should give its next capture. Skips 0, which means "unnumbered".
uint16_t cal8NextSeq(uint16_t s);
extern float cal8_captured[8];

// The one door every eight point table goes through: writes it to NVS, records which filtering mode
// it was measured in, validates the ordering and re-enables the correction. Returns false if the
// stored table is not usable (not in increasing order) - it is stored anyway, so the caller can
// report the truth rather than silently leaving the buoy uncorrected.
//
// It does NOT touch compassOffset. It used to, because the offset was applied before the table and
// north's deviation had to be folded out of entry 0 into the offset; the offset is applied after
// the table now, so the two are independent. See the block comment in compass.cpp.
bool storeInterpolationTable(const float *eight);

// Which convention the stored table was measured under.
//
//   1  the table is indexed by (sensor + compassOffset), and entry 0 was forced to zero with the
//      rotation folded into the offset. Everything calibrated before the chain was reordered.
//   2  the table is indexed by the iron-corrected heading alone and is stored exactly as measured;
//      compassOffset is applied after it. See the pipeline in CompassTask.
//
// A revision 1 table applied to a revision 2 chain is wrong at every heading by roughly the offset
// it was measured against - and it looks completely valid: eight plausible numbers in increasing
// order, passing every other check there is. So it is refused rather than applied, and the buoy
// sails uncorrected until it is measured again. Uncorrected is a known error; this would be a
// hidden one.
#define INTERP_TABLE_REV 2
extern int interp_table_rev;
bool interpTableRevOk(void);

// Which filtering mode the stored table was measured in (-1 = not recorded, older calibration), and
// whether that is the mode running now. A table measured in another mode is applied to a different
// heading source entirely - see the comment on interp_table_mode in compass.cpp.
// The attitude this hull sits at when it is level, and the raw reading it was taken from. See
// memMountLevel() - the sensor may not be bolted in flat, and the bubble is about the hull.
extern float mount_pitch;
extern float mount_roll;
extern volatile float global_pitch_raw;
extern volatile float global_roll_raw;

extern int interp_table_mode;
bool interpTableModeMatches(void);

#endif /* COMPASS_H_ */