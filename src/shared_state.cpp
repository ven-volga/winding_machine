// shared_state.cpp
//
// Визначення глобального стану і Mutex.
// Підключається один раз — більше ніде не визначати.

#include "shared_state.h"

SharedState       shared;
SemaphoreHandle_t sharedMutex;

void shared_state_init()
{
    sharedMutex = xSemaphoreCreateMutex();
}