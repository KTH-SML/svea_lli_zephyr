#ifndef REMOTE_H
#define REMOTE_H

#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>

#define NUM_RC_CHANNELS 4

typedef enum {
    RC_STEER = 0,
    RC_HIGH_GEAR = 1,
    RC_THROTTLE = 2,
    RC_OVERRIDE = 3
} rc_channel_t;

typedef struct {
    union {
        struct {
            uint32_t steer;
            uint32_t high_gear_us;
            uint32_t throttle;
            uint32_t override_us;
        };
        uint32_t fields[NUM_RC_CHANNELS];
    };
} RemoteState;

BUILD_ASSERT(ARRAY_SIZE(((RemoteState *)NULL)->fields) == NUM_RC_CHANNELS,
             "RemoteState.fields[] must match NUM_RC_CHANNELS");

extern struct k_msgq rc_q;

void remote_init(void);
void remote_report(int ch, uint32_t us);
void remote_link_lost(int ch);
bool remote_is_valid(void);
bool remote_all_channels_valid(void);

#endif // REMOTE_H
