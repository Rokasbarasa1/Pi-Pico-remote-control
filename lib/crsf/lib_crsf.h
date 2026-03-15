// /**
//  * @file crsf.h
//  * @author Britannio Jarrett
//  * @brief
//  * @version 0.1
//  * @date 2024-04-13
//  *
//  * @copyright Copyright (c) Britannio Jarrett 2024
//  *
//  * @section LICENSE
//  * Licensed under the MIT License.
//  * See https://github.com/britannio/pico_crsf/blob/main/LICENSE for more information.
//  *
//  */
// #pragma once
// #include <stdint.h>
// #include "pico/stdlib.h"

// typedef struct {
// 	uint8_t buffer[60];
// 	uint8_t length;
// } crsf_payload_custom_t;

// // BEGIN gen_frames.dart
// // The values are CRSF channel values (0-1984). CRSF 172 represents 988us, CRSF 992 represents 1500us, and CRSF 1811 represents 2012us.
// typedef struct __attribute__((packed)) {
// 	unsigned channel0 : 11;
// 	unsigned channel1 : 11;
// 	unsigned channel2 : 11;
// 	unsigned channel3 : 11;
// 	unsigned channel4 : 11;
// 	unsigned channel5 : 11;
// 	unsigned channel6 : 11;
// 	unsigned channel7 : 11;
// 	unsigned channel8 : 11;
// 	unsigned channel9 : 11;
// 	unsigned channel10 : 11;
// 	unsigned channel11 : 11;
// 	unsigned channel12 : 11;
// 	unsigned channel13 : 11;
// 	unsigned channel14 : 11;
// 	unsigned channel15 : 11;
// } crsf_payload_rc_channels_packed_t;

// typedef struct {
// 	// voltage in dV (Big Endian)
// 	uint16_t voltage;
// 	// current in dA (Big Endian)
// 	uint16_t current;
// 	// used capacity in mAh
// 	uint32_t capacity;
// 	// estimated battery remaining in percent (%)
// 	uint8_t percent;
// } crsf_payload_battery_sensor_t;

// typedef struct
// {
// 	crsf_payload_rc_channels_packed_t rc_channels_packed;
// 	crsf_payload_battery_sensor_t battery_sensor;
// 	crsf_payload_link_statistics_t link_statistics;
// 	crsf_payload_custom_t custom;
// } telemetry_t;

// // END gen_frames.dart

// typedef struct
// {
//     uint8_t *buffer;
//     size_t capacity;
//     size_t offset;
// } buffer_t;

// typedef struct
// {
//     uint8_t rssi;
//     uint8_t link_quality;
//     int8_t snr;
//     uint16_t tx_power;
// } link_statistics_t;

// #define TICKS_TO_US(x) ((x - 992.0f) * 5.0f / 8.0f + 1500.0f)

// #ifdef __cplusplus
// extern "C"
// {
// #endif

//     void crsf_telem_set_battery_data(uint16_t voltage, uint16_t current, uint32_t capacity, uint8_t percent);
// 	void crsf_telem_set_custom_payload(uint8_t *data, uint8_t length);
//     void crsf_set_link_quality_threshold(uint8_t threshold);
//     void crsf_set_rssi_threshold(uint8_t threshold);
//     void crsf_set_on_rc_channels(void (*callback)(const uint16_t channels[16]));
//     void crsf_set_on_link_statistics(void (*callback)(const link_statistics_t link_stats));
//     void crsf_set_on_failsafe(void (*callback)(const bool failsafe));
//     void crsf_begin(uart_inst_t *uart, uint8_t rx, uint8_t tx);
//     void crsf_end();
//     void crsf_process_frames();
// 	void crsf_send_telem();
// 	bool crsf_process_frame(uint8_t *frameIndex, uint8_t *frameLength, uint8_t *crcIndex, uint8_t currentByte);


// #ifdef __cplusplus
// }
// #endif