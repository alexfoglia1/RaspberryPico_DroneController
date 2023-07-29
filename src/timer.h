#ifndef TIMER_H
#define TIMER_H

#include <stdint.h>
#include <pico/stdlib.h>

#define SECONDS_TO_US 1000000

extern bool CPU1_WaitFlashWritten;

bool CPU0_TIMER_Init(int frequency_hz);
bool CPU0_TIMER_Loop(repeating_timer_t* timer);

bool CPU1_TIMER_Init(int frequency_hz);
bool CPU1_TIMER_Loop(repeating_timer_t* timer);


#endif //TIMER_H