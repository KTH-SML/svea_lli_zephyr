#ifndef REMOTE_H
#define REMOTE_H

#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>

#define NUM_RC_CHANNELS 4

typedef enum {
    RC_STEER = 0,
    RC_GEAR = 1,
    RC_THROTTLE = 2,
    RC_OVERRIDE = 3
} rc_channel_t;

typedef struct {
    union {
        struct {
            uint32_t steer;
            uint32_t gear;
            uint32_t throttle;
            uint32_t override_us;
        };
        uint32_t fields[NUM_RC_CHANNELS];
    };
} RemoteState;

extern bool rc_valid;
extern struct k_msgq rc_q;

void remote_init(void);
void remote_report(int ch, uint32_t us);
void remote_link_lost(int ch);
bool remote_all_channels_valid(void);

#endif // REMOTE_H