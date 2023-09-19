#pragma once
#include "hardware/i2c.h"
#include <pico/stdlib.h>


// LOWER COLUMN ADDRESS 0x00 - 0x0F

// HIGHER COLUMN ADDRESS 0x10 - 0x1F

#define SET_ADDRESSING_MODE 0x20
enum t_memory_addressing_mode {
    MODE_PAGE_ADDRESSING = 0x10,
    MODE_HORIZONTAL_ADDRESSING = 0x00,
    MODE_VERTICAL_ADDRESSING = 0x01,
};

#define SET_COLUMN_ADDRESS 0x21
#define SET_PAGE_ADDRESS 0x22

// SET DISPLAY START LINE 0x40 - 0x7F

#define SET_CONTRAST_CONTROL_BANK0 0x81
#define SET_SEGMENT_REMAP 0xA0
#define SET_SEGMENT_REMAP 0xA1
#define SET_ENTIRE_DISPLAY_ON 0xA5
#define SET_ENTIRE_DISPLAY_NORMAL 0xA4
#define SET_DISPLAY_NORMAL 0xA6
#define SET_DISPLAY_INVERSE 0xA7
#define SET_MULTIPLEXER_RATIO 0xA8
#define SET_DISPLAY_ON 0xAF
#define SET_DISPLAY_OFF 0xAE

// SET START PAGE ADDRESS FOR PAGE ADDRESSING 0xB0 - 0xB7

#define SET_COM_OUTPUT_SCAN_DIRECTION 0xC0
#define SET_COM_OUTPUT_SCAN_DIRECTION 0xC8


void init_oled_display(i2c_inst_t *i2c_port, uint8_t sda_pin, uint8_t scl_pin);
void oled_write_command(uint8_t command);
void oled_write_data(uint8_t data);
void oled_turn_on();
void oled_set_mode_normal();
void oled_set_mode_inverted();
void oled_set_page_mode();
void oled_full_clear();
void oled_set_cursor(uint8_t x, uint8_t y);
void oled_canvas_clear();
void oled_canvas_write(const char *string, uint8_t string_length, bool flip_letters);
void oled_canvas_show();
void oled_canvas_invert_row(uint8_t row_index);
