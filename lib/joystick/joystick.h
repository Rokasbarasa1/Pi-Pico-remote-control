#pragma once
#include <pico/stdlib.h>
#include <stdio.h>
#include <hardware/adc.h>
#include <string.h>
#include <pico/time.h>

bool init_joystick(uint x_axis_pin, uint y_axis_pin, uint button_pin, void (*button_callback_temp)());
uint16_t get_x();
uint16_t get_y();