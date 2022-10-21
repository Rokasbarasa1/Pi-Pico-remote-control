#pragma once
#include <pico/stdlib.h>
#include <stdio.h>
#include <hardware/spi.h>
#include <string.h>
#include <pico/time.h>

bool adxl345_init(spi_inst_t *spi_temp, uint pin_cs, bool init_spi);