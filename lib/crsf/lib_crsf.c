// /**
//  * @file crsf.c
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

//  #include "crsf.h"
//  #include <hardware/uart.h>
//  #include <hardware/gpio.h>
//  #include <stdlib.h>
//  #include <string.h>
 
//  #define BAUD_RATE 420000
//  #define CRSF_MAX_CHANNELS 16
//  #define CRSF_MAX_FRAME_SIZE 64
//  #define CRSF_DEBUG 0
//  #if CRSF_DEBUG
//  #include <stdio.h>
//  #define DEBUG_WARN(...) fprintf(stderr, __VA_ARGS__)
//  #define DEBUG_INFO(...) printf(__VA_ARGS__)
//  #else
//  #define DEBUG_WARN(...)
//  #define DEBUG_INFO(...)
//  #endif
 
//  uart_inst_t *_uart = NULL;
//  uint8_t _incoming_frame[CRSF_MAX_FRAME_SIZE];
//  uint16_t _rc_channels[CRSF_MAX_CHANNELS];
//  link_statistics_t _link_statistics;
//  bool _failsafe = true;
//  uint8_t _link_quality_threshold = 70;
//  uint8_t _rssi_threshold = 105;
 
//  void (*rc_channels_callback)(const uint16_t channels[]);
//  void (*link_statistics_callback)(const link_statistics_t link_stats);
//  void (*failsafe_callback)(const bool failsafe);
 
//  uint8_t _telem_buf_data[CRSF_MAX_FRAME_SIZE];
//  buffer_t _telem_buf = {
//      .buffer = _telem_buf_data,
//      .capacity = CRSF_MAX_FRAME_SIZE,
//      .offset = 0,
//  };
//  telemetry_t _telemetry;
 
//  enum
//  {
//    CRSF_BATTERY_INDEX = 0,
//    CRSF_CUSTOM_PAYLOAD_INDEX = 1,
//    // Add new frame types above
//    TELEMETRY_FRAME_TYPES
//  };
 
//  bool frameHasData[TELEMETRY_FRAME_TYPES] = {false};
 
//  /**
//   * Sets the callback function to be called when RC channels are received.
//   *
//   * @param callback A function pointer to the callback function that takes an array of uint16_t channels as input.
//   */
//  void crsf_set_on_rc_channels(void (*callback)(const uint16_t channels[16]))
//  {
//    rc_channels_callback = callback;
//  }
 
//  /**
//   * Sets the callback function for link statistics.
//   *
//   * This function sets the callback function that will be called when link statistics are available.
//   *
//   * @param callback A pointer to the callback function.
//   */
//  void crsf_set_on_link_statistics(void (*callback)(const link_statistics_t link_stats))
//  {
//    link_statistics_callback = callback;
//  }
 
//  /**
//   * Sets the callback function to be called when a failsafe event occurs.
//   *
//   * @param callback A function pointer to the callback function that takes a boolean parameter indicating the failsafe status.
//   */
//  void crsf_set_on_failsafe(void (*callback)(const bool failsafe))
//  {
//    failsafe_callback = callback;
//  }
 
//  /**
//   * Sets the link quality threshold for CRSF communication.
//   *
//   * The link quality threshold determines the minimum acceptable link quality for CRSF communication.
//   * A lower threshold allows for more frames to be lost before the failsafe is triggered.
//   *
//   * @param threshold The link quality threshold value, ranging from 0 to 100.
//   */
//  void crsf_set_link_quality_threshold(uint8_t threshold)
//  {
//    _link_quality_threshold = threshold;
//  }
 
//  /**
//   * Sets the RSSI (Received Signal Strength Indicator) threshold for CRSF communication.
//   *
//   * @param threshold The RSSI threshold value to set.
//   */
//  void crsf_set_rssi_threshold(uint8_t threshold)
//  {
//    _rssi_threshold = threshold;
//  }

 
//  const uint16_t tx_power_table[9] = {
//      0,    // 0 mW
//      10,   // 10 mW
//      25,   // 25 mW
//      100,  // 100 mW
//      500,  // 500 mW
//      1000, // 1 W
//      2000, // 2 W
//      250,  // 250 mW
//      50    // 50 mW
//  };
 
//  void buf_reset(buffer_t *buf)
//  {
//    if (buf)
//    {
//      buf->offset = 0;
//    }
//  }
 
//  // Write an uint8_t to the buffer
//  void buf_write_ui8(buffer_t *buf, uint8_t data)
//  {
//    if (buf && (buf->offset + sizeof(uint8_t) <= buf->capacity))
//    {
//      buf->buffer[buf->offset] = data;
//      buf->offset += sizeof(uint8_t);
//    }
//  }
 
//  // Write an int8_t to the buffer
//  void buf_write_i8(buffer_t *buf, int8_t data)
//  {
//    if (buf && (buf->offset + sizeof(int8_t) <= buf->capacity))
//    {
//      buf->buffer[buf->offset] = data;
//      buf->offset += sizeof(int8_t);
//    }
//  }
 
//  // Write an uint16_t to the buffer
//  void buf_write_ui16(buffer_t *buf, uint16_t data)
//  {
//    if (buf && (buf->offset + sizeof(uint16_t) <= buf->capacity))
//    {
//      buf->buffer[buf->offset] = (data >> 8) & 0xFF;
//      buf->buffer[buf->offset + 1] = data & 0xFF;
//      buf->offset += sizeof(uint16_t);
//    }
//  }
 
//  // Write an int16_t to the buffer
//  void buf_write_i16(buffer_t *buf, int16_t data)
//  {
//    if (buf && (buf->offset + sizeof(int16_t) <= buf->capacity))
//    {
//      buf->buffer[buf->offset] = (data >> 8) & 0xFF;
//      buf->buffer[buf->offset + 1] = data & 0xFF;
//      buf->offset += sizeof(int16_t);
//    }
//  }
 
//  // Write an uint24_t to the buffer
//  void buf_write_ui24(buffer_t *buf, uint32_t data)
//  {
//    if (buf && (buf->offset + 3 <= buf->capacity))
//    {
//      buf->buffer[buf->offset] = (data >> 16) & 0xFF;
//      buf->buffer[buf->offset + 1] = (data >> 8) & 0xFF;
//      buf->buffer[buf->offset + 2] = data & 0xFF;
//      buf->offset += 3;
//    }
//  }
 
//  // Write an int24_t to the buffer
//  void buf_write_i24(buffer_t *buf, int32_t data)
//  {
//    if (buf && (buf->offset + 3 <= buf->capacity))
//    {
//      buf->buffer[buf->offset] = (data >> 16) & 0xFF;
//      buf->buffer[buf->offset + 1] = (data >> 8) & 0xFF;
//      buf->buffer[buf->offset + 2] = data & 0xFF;
//      buf->offset += 3;
//    }
//  }
 
//  // Write an uint32_t to the buffer
//  void buf_write_ui32(buffer_t *buf, uint32_t data)
//  {
//    if (buf && (buf->offset + sizeof(uint32_t) <= buf->capacity))
//    {
//      buf->buffer[buf->offset] = (data >> 24) & 0xFF;
//      buf->buffer[buf->offset + 1] = (data >> 16) & 0xFF;
//      buf->buffer[buf->offset + 2] = (data >> 8) & 0xFF;
//      buf->buffer[buf->offset + 3] = data & 0xFF;
//      buf->offset += sizeof(uint32_t);
//    }
//  }
 
//  // Write an int32_t to the buffer
//  void buf_write_i32(buffer_t *buf, int32_t data)
//  {
//    if (buf && (buf->offset + sizeof(int32_t) <= buf->capacity))
//    {
//      buf->buffer[buf->offset] = (data >> 24) & 0xFF;
//      buf->buffer[buf->offset + 1] = (data >> 16) & 0xFF;
//      buf->buffer[buf->offset + 2] = (data >> 8) & 0xFF;
//      buf->buffer[buf->offset + 3] = data & 0xFF;
//      buf->offset += sizeof(int32_t);
//    }
//  }
 
//  void _begin_frame()
//  {
//    buf_reset(&_telem_buf);
//    // Write sync byte
//    buf_write_ui8(&_telem_buf, 0xC8);
//  }
 
//  void _end_frame()
//  {
//    // Skip sync byte and frame length
//    const uint8_t bytesToSkip = 2;
//    const uint8_t *start = _telem_buf.buffer + bytesToSkip;
//    const uint8_t length = _telem_buf.offset - bytesToSkip;
//    const uint8_t crc = crsf_crc8(start, length);
 
//    buf_write_ui8(&_telem_buf, crc);
//  }
 
//  // BEGIN gen_frames.dart
//  void _write_battery_sensor_payload()
//  {
//    buf_write_ui8(&_telem_buf, 10);                            // Frame length
//    buf_write_ui8(&_telem_buf, CRSF_FRAMETYPE_BATTERY_SENSOR); // Frame type
//    buf_write_ui16(&_telem_buf, _telemetry.battery_sensor.voltage);
//    buf_write_ui16(&_telem_buf, _telemetry.battery_sensor.current);
//    buf_write_ui24(&_telem_buf, _telemetry.battery_sensor.capacity);
//    buf_write_ui8(&_telem_buf, _telemetry.battery_sensor.percent);
//  }
//  // END gen_frames.dart
 
//  bool crsf_telem_update()
//  {
//    bool updated = false;
//    static int currentFrameType = 0;
 
//    for (int i = 0; i < TELEMETRY_FRAME_TYPES; i++)
//    {
//      int frameTypeIndex = (currentFrameType + i) % TELEMETRY_FRAME_TYPES;
 
//      if (frameHasData[frameTypeIndex])
//      {
//        _begin_frame();
//        switch (frameTypeIndex)
//        {
//        case CRSF_BATTERY_INDEX:
//          _write_battery_sensor_payload();
//          break;
//        case CRSF_CUSTOM_PAYLOAD_INDEX:
//          buf_write_ui8(&_telem_buf, _telemetry.custom.length + 2);  // Frame length
//          buf_write_ui8(&_telem_buf, CRSF_FRAMETYPE_CUSTOM_PAYLOAD); // Frame type
//          for (size_t i = 0; i < _telemetry.custom.length; i++)
//          {
//            buf_write_ui8(&_telem_buf, _telemetry.custom.buffer[i]);
//          }
//          break;
//        }
//        _end_frame();
//        updated = true;
//        currentFrameType = (currentFrameType + 1) % TELEMETRY_FRAME_TYPES;
//        break;
//      }
//    }
 
//    return updated;
//  }

 
//  void crsf_send_telem()
//  {
//    // Send telemetry
//    if (crsf_telem_update())
//    {
//      DEBUG_INFO("Sending telemetry frame");
//      for (size_t i = 0; i < _telem_buf.offset; i++)
//      {
//        uart_putc(_uart, _telem_buf.buffer[i]);
//      }
//    }
//  }
 
//  /**
//   * Sets the battery data in the telemetry structure.
//   *
//   * @param voltage The battery voltage in dv
//   * @param current The battery current in dA
//   * @param capacity The battery capacity in mAH
//   * @param percent The battery percentage remaining.
//   */
//  void crsf_telem_set_battery_data(uint16_t voltage, uint16_t current, uint32_t capacity, uint8_t percent)
//  {
//    _telemetry.battery_sensor.voltage = voltage;
//    _telemetry.battery_sensor.current = current;
//    _telemetry.battery_sensor.capacity = capacity;
//    _telemetry.battery_sensor.percent = percent;
//    frameHasData[CRSF_BATTERY_INDEX] = true;
//  }
 
//  void crsf_telem_set_custom_payload(uint8_t *data, uint8_t length)
//  {
//    if (length > 60)
//    {
//      return;
//    }
//    memcpy(_telemetry.custom.buffer, data, length);
//    _telemetry.custom.length = length;
//    frameHasData[CRSF_CUSTOM_PAYLOAD_INDEX] = true;
//  }