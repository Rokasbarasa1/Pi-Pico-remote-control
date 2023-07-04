#pragma once
#include <pico/stdlib.h>
#include <stdio.h>
#include <hardware/adc.h>
#include <string.h>
#include <pico/time.h>

// struct button
// {
//     void (*m_button_callback)();
//     uint m_button_pin
// };

void init_button(void (*button_callback)(), uint button_pin);