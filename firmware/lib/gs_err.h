/* Error codes (RESULT message), field ids, fault bits, event codes. See PROTOCOL.md. */
#ifndef GS_ERR_H
#define GS_ERR_H

enum gs_err {
    GS_OK = 0,
    GS_ERR_UNKNOWN_CMD = 1,
    GS_ERR_BAD_LENGTH = 2,
    GS_ERR_BAD_VALUE = 3,       /* detail = gs_field id */
    GS_ERR_LIMIT_WIDTH = 4,     /* strobe width outside [min,max]; detail = width us */
    GS_ERR_LIMIT_DUTY = 5,      /* worst-case duty too high; detail = duty ppm (rounded up) */
    GS_ERR_LIMIT_PPS = 6,       /* worst-case pulses/s too high; detail = pulses in 1 s */
    GS_ERR_LIMIT_TIMING = 7,    /* spacing/length limits; detail = gs_field id */
    GS_ERR_BUSY_ARMED = 8,      /* configuration changes need disarmed state */
    GS_ERR_NOT_ARMED = 9,
    GS_ERR_FAULT_LATCHED = 10,  /* clear faults first; detail = fault mask */
    GS_ERR_LOCKOUT = 11,        /* fire() inside trigger lockout; detail = remaining us */
    GS_ERR_BAD_MAGIC = 12,
    GS_ERR_HW = 13              /* hardware layer refused (e.g. clock mismatch) */
};

enum gs_field {
    GS_FIELD_NONE = 0,
    GS_FIELD_MODE = 1,
    GS_FIELD_PULSES = 2,
    GS_FIELD_BURST_EXPOSURES = 3,
    GS_FIELD_WIDTH = 4,
    GS_FIELD_INTERVAL = 5,
    GS_FIELD_FIRST_DELAY = 6,
    GS_FIELD_TAIL = 7,
    GS_FIELD_FRAME_PERIOD = 8,
    GS_FIELD_TRIGGER_DELAY = 9,
    GS_FIELD_EXPOSURE = 10,     /* derived exposure length */
    GS_FIELD_LOCKOUT = 11,
    GS_FIELD_POLARITY = 12,
    GS_FIELD_VDIV = 13,
    GS_FIELD_STATUS_PERIOD = 14
};

/* Latched fault bits (STATUS.faults). Cleared by CLEAR_FAULTS. */
#define GS_FAULT_STROBE_OVERWIDTH (1u << 0)  /* watchdog forced strobe low */
#define GS_FAULT_DUTY_RUNTIME     (1u << 1)  /* measured duty above limit */
#define GS_FAULT_PPS_RUNTIME      (1u << 2)  /* measured pulses/s above limit */
#define GS_FAULT_INTERLOCK_TRIPPED (1u << 3) /* STROBE_EN went low (informational, latched) */
#define GS_FAULT_CLOCK            (1u << 4)  /* sysclk != GS_CLK_HZ */
#define GS_FAULT_WDT_RESET        (1u << 5)  /* last reset was the hardware watchdog */
#define GS_FAULT_TRIG_QUEUE       (1u << 6)  /* trigger event queue overflowed */
#define GS_FAULT_FIRE_TIMEOUT     (1u << 7)  /* software trigger did not reach the PIO */
/* Faults that stop the strobe and disarm: */
#define GS_FAULT_STOPPING (GS_FAULT_STROBE_OVERWIDTH | GS_FAULT_DUTY_RUNTIME | GS_FAULT_PPS_RUNTIME | GS_FAULT_CLOCK)

enum gs_event {
    GS_EVT_BOOT = 1,             /* value = 1 if watchdog reset else 0 */
    GS_EVT_FAULT = 2,            /* value = newly raised fault bits */
    GS_EVT_INTERLOCK = 3,        /* value = 1 closed (strobe allowed), 0 open */
    GS_EVT_DISARMED = 4          /* value = fault bits that caused it */
};

#endif
