#pragma once
#include <pico/stdlib.h>
// #include <stdio.h>
#include "hardware/spi.h"
// #include <string.h>

void init_receiver(uint spi_port, uint spi_frequency, uint pin_mosi, uint pin_miso, uint pin_sck, uint pin_cs);
void init_transmitter(uint spi_port, uint spi_frequency, uint pin_mosi, uint pin_miso, uint pin_sck, uint pin_cs);
