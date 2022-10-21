#pragma once
#include <pico/stdlib.h>
#include <stdio.h>
#include <hardware/spi.h>
#include <string.h>
#include <pico/time.h>

bool nrf24_init(spi_inst_t *spi_temp, uint pin_csn_temp, uint pin_ce_temp, bool init_spi);