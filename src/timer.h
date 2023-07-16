#ifndef TIMER_H
#define TIMER_H

#include <stdint.h>
#include "pico/stdlib.h"

#define SECONDS_TO_US 1000000

bool TIMER_Init(int frequency_hz);
void TIMER_Loop(void);

bool __TIMER_ISR__(repeating_timer_t* timer);


#endif //TIMER_H