#pragma once
#include <pico/stdlib.h>
#include <stdio.h>
#include <hardware/adc.h>
#include <string.h>
#include <pico/time.h>

void init_joystick();
uint16_t joystick_get_throttle_raw();
uint16_t joystick_get_yaw_raw();
uint16_t joystick_get_pitch_raw();
uint16_t joystick_get_roll_raw();
float joystick_get_throttle_percent();
float joystick_get_yaw_percent();
float joystick_get_pitch_percent();
float joystick_get_roll_percent();

float joystick_get_throttle_volts();
float joystick_get_yaw_volts();
float joystick_get_pitch_volts();
float joystick_get_roll_volts();