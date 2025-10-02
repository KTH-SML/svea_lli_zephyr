// wake_chip.h
#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

int wake_chip_init(void);
int wake_chip_pulse_ms(uint32_t ms);
int wake_chip_release(void);

#ifdef __cplusplus
}
#endif