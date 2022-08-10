#pragma once
#include "pico/stdlib.h"
#include <stdio.h>
#include "hardware/uart.h"
#include <string.h>



// Instructions:
// * Connect rx of pico to tx of the esp 01
// * Connect tx of pico to rx of the esp 01
// * Connect the enable pin that you select to the enable pin of the pico

bool init_esp_01(uart_inst_t *uart, uint enable_pin);
bool esp_01_connect_wifi(uart_inst_t *uart, char *wifi_name, char *wifi_password);
bool esp_01_send_http(uart_inst_t *uart, char *ADDRESS, char *PORT, char *command);
