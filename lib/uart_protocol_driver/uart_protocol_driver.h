#pragma once

#include "pico/stdlib.h"

#include <avr/io.h>


void init_uart_protocol(uint8_t serial_device, uint32_t baud_rate);

uint8_t uart_write_ready(uint8_t serial_device);
uint8_t uart_read_ready(uint8_t serial_device);

uint8_t uart_read_byte(uint8_t serial_device);
void uart_write_byte(uint8_t serial_device, uint8_t data);
void uart_write_string(uint8_t serial_device, char *string, uint8_t free_string);