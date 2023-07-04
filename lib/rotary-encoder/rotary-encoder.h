#pragma once
#include <pico/stdlib.h>
#include <stdio.h>
#include <hardware/adc.h>
#include <string.h>
#include <pico/time.h>

struct rotary_encoder
{
    bool m_clk_state;
    bool m_clk_state_previous;
    bool m_dt_state;
    bool m_clockwise;
    int32_t m_counter;
    uint8_t m_clk_pin;
    uint8_t m_dt_pin;
};

uint8_t init_rotary_encoder(uint8_t clk_pin, uint8_t dt_pin);
bool rotary_encoder_get_is_clockwise(uint8_t encoder_index);
int32_t rotary_encoder_get_counter(uint8_t encoder_index);
void rotary_encoder_reset_counter(uint8_t encoder_index);