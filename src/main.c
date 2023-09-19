#include "pico/stdlib.h"
#include <stdio.h>
#include "hardware/uart.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"

#include "../lib/esp_01/esp_01.h"
#include "../lib/joystick/joystick.h"
#include "../lib/button/button.h"
#include "../lib/rotary-encoder/rotary-encoder.h"
#include "../lib/adxl345/adxl345.h"
#include "../lib/nrf24l01/nrf24l01.h"
#include "../lib/oled-display/oled-display.h"

#define MAX_MESSAGE_LENGTH 100

void button1_callback();
void button2_callback();

unsigned char* int_to_string(uint number);
unsigned char* generate_message_joystick_esp01(uint x, uint y, unsigned char *ADDRESS);
unsigned char* generate_message_joystick_nrf24(uint throttle, uint yaw, uint pitch, uint roll);

/**
 * SPI0 RADIO nRF24L01+
 * 
 * GP19 TX
 * GP18 SCK
 * GP16 RX
 * 
 * GP7 CSN
 * GP8 CE
 * GP16 CS
 */

/**
 * ADC for joysticks
 * 
 * GP28 ADC2
 * GP27 ADC1
 * GP26 ADC0 2 in 1
 * 
 * Use GP3 and GP6 to control the multiplexer MC14052BCP (same as CD4052) 
 * and tell it which pin to read from the joysticks and pass to ADC0 
 * 
 */

/**
 * Reset button
 * 
 * RUN (PIN28, bellow GP26)
 */
 
/**
 * Rotary encoder 1 KY-040
 * 
 * Button GP22
 * CLK GP12
 * DT GP13
 */

/**
 * Rotary encoder 2 KY-040
 * 
 * Button GP21
 * CLK GP10
 * DT GP11
 */

/**
 * Display 128x64
 * 
 * SDA GP4
 * SCL GP5
 */

bool disable_repeated_send = false;
volatile bool send_data = false;

volatile uint throttle = 0;
volatile uint yaw = 0;
volatile uint pitch = 0;
volatile uint roll = 0;

enum t_mode {
    MODE_CONTROL,
    MODE_PID_TUNE,
    MODE_MAIN
};
uint8_t* mode_select_strings[] = {
    (uint8_t*)"Control mode",
    (uint8_t*)"PID tune mode"
};

enum t_control_mode {
    CONTROL_MODE_NONE
};
uint8_t* control_mode_strings[] = {
    (uint8_t*)"Back",
};

enum t_pid_tune_mode {
    PID_TUNE_MODE_NONE,
    PID_TUNE_MODE_EDIT_PID_VALUES,
    PID_TUNE_MODE_SYNC_SLAVE_WITH_REMOTE,
    PID_TUNE_MODE_SYNC_REMOTE_WITH_SLAVE
};
uint8_t* pid_tune_mode_strings[] = {
    (uint8_t*)"Back",
    (uint8_t*)"Edit PID Values",
    (uint8_t*)"Sync Slave To Remote",
    (uint8_t*)"Sync Remote To Slave"
};

enum t_pid_tune_mode_edit {
    PID_TUNE_MODE_EDIT_NONE,
    PID_TUNE_MODE_EDIT_PROPORTIONAL,
    PID_TUNE_MODE_EDIT_INTEGRAL,
    PID_TUNE_MODE_EDIT_DERIVATIVE,
    PID_TUNE_MODE_EDIT_MASTER_GAIN,
};
uint8_t* pid_tune_mode_edit_strings[] = {
    (uint8_t*)"Back",
    (uint8_t*)"Proportional",
    (uint8_t*)"Integral",
    (uint8_t*)"Derivative",
    (uint8_t*)"Master Gain"
};

enum t_mode current_mode = MODE_MAIN;
enum t_mode old_mode = MODE_CONTROL;

enum t_control_mode current_control = CONTROL_MODE_NONE;
enum t_control_mode old_control = CONTROL_MODE_NONE;

enum t_pid_tune_mode current_pid_tune = PID_TUNE_MODE_NONE;
enum t_pid_tune_mode old_pid_tune = PID_TUNE_MODE_NONE;

enum t_pid_tune_mode_edit current_pid_tune_edit = PID_TUNE_MODE_EDIT_NONE;
enum t_pid_tune_mode_edit old_pid_tune_edit = PID_TUNE_MODE_EDIT_NONE;


uint8_t rotary_encoder_1 = -99;
uint8_t rotary_encoder_2 = -99;

int32_t rotary_encoder_1_old_value = 0;
int32_t rotary_encoder_2_old_value = 0;

int32_t rotary_encoder_1_new_value = 0;
int32_t rotary_encoder_2_new_value = 0;

double base_proportional = 7.4;
double base_integral = 3.4;
double base_derivative = 2000;
double base_master_gain = 1.0;

double added_proportional = 0;
double added_integral = 0;
double added_derivative = 0;
double added_master_gain = 0;

#define DIAL_PRECISION 0.1

uint16_t positive_mod(int32_t value, uint16_t value_modal){
    int32_t result = value % value_modal;
    if (result < 0) {
        result += value_modal;
    }
    return result;
}

volatile char string_buffer[100];
volatile uint8_t string_length = 0;

bool screen_enabled = true;
bool screen_enabled_old = true;

int main() {
    stdio_init_all();

    // Sleep a so you actually have time to read some of the serial outputs
    // sleep_ms(2500); // For debugging
    printf("STARTING PROGRAM\n");

    // ########################################################## status led
    gpio_init(2);
    gpio_set_dir(2, GPIO_OUT);
    gpio_put(2, 1);
    printf("Gpio led initialized\n");

    // ########################################################## Joysticks
    init_joystick();
    printf("Joystick initialized\n");

    // ########################################################## Button interrupts
    init_button(button1_callback, 21);
    init_button(button2_callback, 22);

    printf("Buttons initialized\n");

    // ########################################################## Rotary encoders
    rotary_encoder_1 = init_rotary_encoder(12, 13);
    rotary_encoder_2 = init_rotary_encoder(10, 11);

    rotary_encoder_1_old_value = rotary_encoder_get_counter(rotary_encoder_1);
    rotary_encoder_2_old_value = rotary_encoder_get_counter(rotary_encoder_2);

    printf("Rotary encoders initialized\n");

    // ########################################################## Oled display
    init_oled_display(i2c_default, PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN);

    //oled_canvas_clear();
    sleep_ms(1000);
    //oled_canvas_write("123\n321\n456\n584\n116\n455", 23, true);
    //oled_canvas_show();

    printf("Oled initialized\n");

    // ########################################################## Setup radio communication
    if(nrf24_init(spi_default, 7, 8, true)){
        printf("nrf24 setup succeeded\n");
    }else{
        printf("nrf24 setup failed\n");
    }

    uint8_t tx_address[5] = {0xEE, 0xDD, 0xCC, 0xBB, 0xAA};
    nrf24_tx_mode(tx_address, 10);

    printf("Radio initialized\n");

    // ########################################################## Main loop
    printf("\n\n====START OF LOOP====\n\n");
    while (true) {


        
        if( (current_mode != old_mode || 
            current_control != old_control || 
            current_pid_tune != old_pid_tune || 
            current_pid_tune_edit != old_pid_tune_edit) && screen_enabled
        ){

            if(current_mode != old_mode){
                old_mode = current_mode;
                old_pid_tune = current_pid_tune;
                old_control = current_control;

                if(current_mode == MODE_MAIN){
                    oled_canvas_clear();

                    uint8_t selected_row = 0;
                    uint8_t size_mode_select_strings = sizeof(mode_select_strings) / sizeof(mode_select_strings[0]);
                    for (size_t i = 0; i < size_mode_select_strings; i++)
                    {
                        sprintf(string_buffer, "%s", mode_select_strings[i]);
                        string_length = strlen(string_buffer);
                        oled_canvas_write(string_buffer, string_length, true);
                        memset(string_buffer, 0, string_length);
                        if(i == 0){
                            selected_row = i;
                            // oled_canvas_write(" XXXX", 5, true);
                        }
                        oled_canvas_write("\n", 1, true);
                    }

                    for (size_t i = 0; i < 5-size_mode_select_strings; i++){
                        oled_canvas_write("\n", 1, true);
                    }

                    oled_canvas_invert_row(selected_row);
                    oled_canvas_show();
                }else if(current_mode == MODE_CONTROL){
                    oled_canvas_clear();

                    uint8_t selected_row = 0;
                    uint8_t size_control_mode_strings = sizeof(control_mode_strings) / sizeof(control_mode_strings[0]);
                    for (size_t i = 0; i < size_control_mode_strings; i++)
                    {
                        sprintf(string_buffer, "%s", control_mode_strings[i]);
                        string_length = strlen(string_buffer);
                        oled_canvas_write(string_buffer, string_length, true);
                        memset(string_buffer, 0, string_length);
                        if(i == 0){
                            selected_row = i;
                            //oled_canvas_write(" XXXX", 5, true);
                        }
                        oled_canvas_write("\n", 1, true);
                    }
                    for (size_t i = 0; i < 5-size_control_mode_strings; i++){
                        oled_canvas_write("\n", 1, true);
                    }

                    oled_canvas_invert_row(selected_row);
                    oled_canvas_show();
                }else if(current_mode == MODE_PID_TUNE){
                    oled_canvas_clear();
                    
                    uint8_t selected_row = 0;
                    uint8_t size_pid_tune_mode_strings = sizeof(pid_tune_mode_strings) / sizeof(pid_tune_mode_strings[0]);
                    for (size_t i = 0; i < size_pid_tune_mode_strings; i++)
                    {
                        sprintf(string_buffer, "%s", pid_tune_mode_strings[i]);
                        string_length = strlen(string_buffer);
                        oled_canvas_write(string_buffer, string_length, true);
                        memset(string_buffer, 0, string_length);
                        if(i == 0){
                            selected_row = i;
                            // oled_canvas_write(" XXXX", 5, true);
                        }
                        oled_canvas_write("\n", 1, true);
                    }
                    for (size_t i = 0; i < 5-size_pid_tune_mode_strings; i++){
                        oled_canvas_write("\n", 1, true);
                    }

                    oled_canvas_invert_row(selected_row);
                    oled_canvas_show();
                }
            }else if(current_control != old_control){
                old_control = current_control;
                // This doesn't have any features
            }else if(current_pid_tune != old_pid_tune){
                old_pid_tune = current_pid_tune;
                old_pid_tune_edit = current_pid_tune_edit;

                if(current_pid_tune == PID_TUNE_MODE_NONE){
                    oled_canvas_clear();

                    uint8_t selected_row = 0;
                    uint8_t size_pid_tune_mode_strings = sizeof(pid_tune_mode_strings) / sizeof(pid_tune_mode_strings[0]);
                    for (size_t i = 0; i < size_pid_tune_mode_strings; i++)
                    {
                        sprintf(string_buffer, "%s", pid_tune_mode_strings[i]);
                        string_length = strlen(string_buffer);
                        oled_canvas_write(string_buffer, string_length, true);
                        memset(string_buffer, 0, string_length);
                        if(i == 0){
                            selected_row = i;
                            // oled_canvas_write(" XXXX", 5, true);
                        }
                        oled_canvas_write("\n", 1, true);
                    }
                    for (size_t i = 0; i < 5-size_pid_tune_mode_strings; i++){
                        oled_canvas_write("\n", 1, true);
                    }

                    oled_canvas_invert_row(selected_row);
                    oled_canvas_show();
                }else if(current_pid_tune == PID_TUNE_MODE_EDIT_PID_VALUES){
                    oled_canvas_clear();

                    uint8_t selected_row = 0;
                    uint8_t size_pid_tune_mode_edit_strings = sizeof(pid_tune_mode_edit_strings) / sizeof(pid_tune_mode_edit_strings[0]);
                    for (size_t i = 0; i < size_pid_tune_mode_edit_strings; i++)
                    {
                        sprintf(string_buffer, "%s", pid_tune_mode_edit_strings[i]);
                        string_length = strlen(string_buffer);
                        oled_canvas_write(string_buffer, string_length, true);
                        memset(string_buffer, 0, string_length);
                        if(i == 0){
                            selected_row = i;
                            // oled_canvas_write(" XXXX", 5, true);
                        }
                        oled_canvas_write("\n", 1, true);
                    }
                    for (size_t i = 0; i < 5-size_pid_tune_mode_edit_strings; i++){
                        oled_canvas_write("\n", 1, true);
                    }

                    oled_canvas_invert_row(selected_row);
                    oled_canvas_show();
                }
                // Other values dont have their own page

            }else if(current_pid_tune_edit != old_pid_tune_edit){
                old_pid_tune_edit = current_pid_tune_edit;

                if(current_pid_tune_edit == PID_TUNE_MODE_EDIT_NONE){
                    oled_canvas_clear();

                    uint8_t selected_row = 0;
                    uint8_t size_pid_tune_mode_edit_strings = sizeof(pid_tune_mode_edit_strings) / sizeof(pid_tune_mode_edit_strings[0]);
                    for (size_t i = 0; i < size_pid_tune_mode_edit_strings; i++)
                    {
                        sprintf(string_buffer, "%s", pid_tune_mode_edit_strings[i]);
                        string_length = strlen(string_buffer);
                        oled_canvas_write(string_buffer, string_length, true);
                        memset(string_buffer, 0, string_length);
                        if(i == 0){
                            selected_row = i;
                            // oled_canvas_write(" XXXX", 5, true);
                        }
                        oled_canvas_write("\n", 1, true);
                    }
                    for (size_t i = 0; i < 5-size_pid_tune_mode_edit_strings; i++){
                        oled_canvas_write("\n", 1, true);
                    }

                    oled_canvas_invert_row(selected_row);
                    oled_canvas_show();
                }else if(current_pid_tune_edit == PID_TUNE_MODE_EDIT_PROPORTIONAL){
                    oled_canvas_clear();

                    oled_canvas_write("\n", 1, true);
                    oled_canvas_write("\n", 1, true);

                    sprintf(string_buffer, "P base: %3.3f\n\nP added: %3.3f\n", base_proportional, added_proportional);
                    string_length = strlen(string_buffer);
                    oled_canvas_write(string_buffer, string_length, true);
                    memset(string_buffer, 0, string_length);

                    oled_canvas_write("\n", 1, true);
                    oled_canvas_write("\n", 1, true);

                    oled_canvas_show();
                }else if(current_pid_tune_edit == PID_TUNE_MODE_EDIT_INTEGRAL){
                    oled_canvas_clear();

                    oled_canvas_write("\n", 1, true);
                    oled_canvas_write("\n", 1, true);

                    sprintf(string_buffer, "I base: %3.3f\n\nI added: %3.3f\n", base_integral, added_integral);
                    string_length = strlen(string_buffer);
                    oled_canvas_write(string_buffer, string_length, true);
                    memset(string_buffer, 0, string_length);

                    oled_canvas_write("\n", 1, true);
                    oled_canvas_write("\n", 1, true);

                    oled_canvas_show();
                }else if(current_pid_tune_edit == PID_TUNE_MODE_EDIT_DERIVATIVE){
                    oled_canvas_clear();

                    oled_canvas_write("\n", 1, true);
                    oled_canvas_write("\n", 1, true);

                    sprintf(string_buffer, "D base: %3.3f\n\nD added: %3.3f\n", base_derivative, added_derivative);
                    string_length = strlen(string_buffer);
                    oled_canvas_write(string_buffer, string_length, true);
                    memset(string_buffer, 0, string_length);
                    
                    oled_canvas_write("\n", 1, true);
                    oled_canvas_write("\n", 1, true);

                    oled_canvas_show();
                }else if(current_pid_tune_edit == PID_TUNE_MODE_EDIT_MASTER_GAIN){
                    oled_canvas_clear();

                    oled_canvas_write("\n", 1, true);
                    oled_canvas_write("\n", 1, true);

                    sprintf(string_buffer, "M base: %3.3f\n\nM added: %3.3f\n", base_master_gain, added_master_gain);
                    string_length = strlen(string_buffer);
                    oled_canvas_write(string_buffer, string_length, true);
                    memset(string_buffer, 0, string_length);

                    oled_canvas_write("\n", 1, true);
                    oled_canvas_write("\n", 1, true);

                    oled_canvas_show();
                }
            }
        }
        
        if ((rotary_encoder_get_counter(rotary_encoder_1) != rotary_encoder_1_old_value && screen_enabled) || screen_enabled_old == false && screen_enabled == true){
            rotary_encoder_1_new_value = rotary_encoder_get_counter(rotary_encoder_1);

            if(current_mode == MODE_MAIN){
                oled_canvas_clear();

                uint8_t selected_row = 0;
                uint8_t size_mode_select_strings = sizeof(mode_select_strings) / sizeof(mode_select_strings[0]);
                for (size_t i = 0; i < size_mode_select_strings; i++)
                {
                    sprintf(string_buffer, "%s", mode_select_strings[i]);
                    string_length = strlen(string_buffer);
                    oled_canvas_write(string_buffer, string_length, true);
                    memset(string_buffer, 0, string_length);
                    if(positive_mod(rotary_encoder_1_new_value, size_mode_select_strings) == i){
                        selected_row = i;
                        // oled_canvas_write(" XXXX", 5, true);
                    }
                    oled_canvas_write("\n", 1, true);
                }
                for (size_t i = 0; i < 5-size_mode_select_strings; i++){
                    oled_canvas_write("\n", 1, true);
                }

                oled_canvas_invert_row(selected_row);
                oled_canvas_show();
            }else if(current_mode == MODE_CONTROL){

                if(current_control == CONTROL_MODE_NONE){
                    oled_canvas_clear();

                    uint8_t selected_row = 0;
                    uint8_t size_control_mode_strings = sizeof(control_mode_strings) / sizeof(control_mode_strings[0]);
                    for (size_t i = 0; i < size_control_mode_strings; i++)
                    {
                        sprintf(string_buffer, "%s", control_mode_strings[i]);
                        string_length = strlen(string_buffer);
                        oled_canvas_write(string_buffer, string_length, true);
                        memset(string_buffer, 0, string_length);
                        if(positive_mod(rotary_encoder_1_new_value, size_control_mode_strings) == i){
                            selected_row = i;
                            // oled_canvas_write(" XXXX", 5, true);
                        }
                        oled_canvas_write("\n", 1, true);
                    }
                    for (size_t i = 0; i < 5-size_control_mode_strings; i++){
                        oled_canvas_write("\n", 1, true);
                    }

                    oled_canvas_invert_row(selected_row);
                    oled_canvas_show();
                }

                // No features for this

            }else if(current_mode == MODE_PID_TUNE){

                if(current_pid_tune == PID_TUNE_MODE_NONE){
                    oled_canvas_clear();

                    uint8_t selected_row = 0;
                    uint8_t size_pid_tune_mode_strings = sizeof(pid_tune_mode_strings) / sizeof(pid_tune_mode_strings[0]);
                    for (size_t i = 0; i < size_pid_tune_mode_strings; i++)
                    {
                        sprintf(string_buffer, "%s", pid_tune_mode_strings[i]);
                        string_length = strlen(string_buffer);
                        oled_canvas_write(string_buffer, string_length, true);
                        memset(string_buffer, 0, string_length);
                        if(positive_mod(rotary_encoder_1_new_value, size_pid_tune_mode_strings) == i){
                            selected_row = i;
                            // oled_canvas_write(" XXXX", 5, true);
                        }
                        oled_canvas_write("\n", 1, true);
                    }
                    for (size_t i = 0; i < 5-size_pid_tune_mode_strings; i++){
                        oled_canvas_write("\n", 1, true);
                    }

                    oled_canvas_invert_row(selected_row);
                    oled_canvas_show();
                }else if (current_pid_tune == PID_TUNE_MODE_EDIT_PID_VALUES){

                    if(current_pid_tune_edit == PID_TUNE_MODE_EDIT_NONE){
                        oled_canvas_clear();

                        uint8_t selected_row = 0;
                        uint8_t size_pid_tune_mode_edit_strings = sizeof(pid_tune_mode_edit_strings) / sizeof(pid_tune_mode_edit_strings[0]);
                        for (size_t i = 0; i < size_pid_tune_mode_edit_strings; i++)
                        {
                            sprintf(string_buffer, "%s", pid_tune_mode_edit_strings[i]);
                            string_length = strlen(string_buffer);
                            oled_canvas_write(string_buffer, string_length, true);
                            memset(string_buffer, 0, string_length);
                            if(positive_mod(rotary_encoder_1_new_value, size_pid_tune_mode_edit_strings) == i){
                                selected_row = i;
                                // oled_canvas_write(" XXXX", 5, true);
                            }
                            oled_canvas_write("\n", 1, true);
                        }
                        for (size_t i = 0; i < 5-size_pid_tune_mode_edit_strings; i++){
                            oled_canvas_write("\n", 1, true);
                        }

                        oled_canvas_invert_row(selected_row);
                        oled_canvas_show();
                    }
                    if(current_pid_tune_edit == PID_TUNE_MODE_EDIT_PROPORTIONAL){
                        oled_canvas_clear();

                        added_proportional = added_proportional + ((rotary_encoder_1_new_value - rotary_encoder_1_old_value) * DIAL_PRECISION);
                        oled_canvas_write("\n", 1, true);
                        oled_canvas_write("\n", 1, true);

                        sprintf(string_buffer, "P base: %3.3f\n\nP added: %3.3f\n", base_proportional, added_proportional);
                        string_length = strlen(string_buffer);
                        oled_canvas_write(string_buffer, string_length, true);
                        memset(string_buffer, 0, string_length);

                        oled_canvas_write("\n", 1, true);
                        oled_canvas_write("\n", 1, true);

                        oled_canvas_show();
                    }else if(current_pid_tune_edit == PID_TUNE_MODE_EDIT_INTEGRAL){
                        oled_canvas_clear();

                        added_integral = added_integral + ((rotary_encoder_1_new_value - rotary_encoder_1_old_value) * DIAL_PRECISION);
                        oled_canvas_write("\n", 1, true);
                        oled_canvas_write("\n", 1, true);

                        sprintf(string_buffer, "I base: %3.3f\n\nI added: %3.3f\n", base_integral, added_integral);
                        string_length = strlen(string_buffer);
                        oled_canvas_write(string_buffer, string_length, true);
                        memset(string_buffer, 0, string_length);

                        oled_canvas_write("\n", 1, true);
                        oled_canvas_write("\n", 1, true);

                        oled_canvas_show();
                    }else if(current_pid_tune_edit == PID_TUNE_MODE_EDIT_DERIVATIVE){
                        oled_canvas_clear();

                        added_derivative = added_derivative + ((rotary_encoder_1_new_value - rotary_encoder_1_old_value) * DIAL_PRECISION);
                        oled_canvas_write("\n", 1, true);
                        oled_canvas_write("\n", 1, true);

                        sprintf(string_buffer, "D base: %3.3f\n\nD added: %3.3f\n", base_derivative, added_derivative);
                        string_length = strlen(string_buffer);
                        oled_canvas_write(string_buffer, string_length, true);
                        memset(string_buffer, 0, string_length);

                        oled_canvas_write("\n", 1, true);
                        oled_canvas_write("\n", 1, true);

                        oled_canvas_show();
                    }else if(current_pid_tune_edit == PID_TUNE_MODE_EDIT_MASTER_GAIN){
                        oled_canvas_clear();

                        added_master_gain = added_master_gain + ((rotary_encoder_1_new_value - rotary_encoder_1_old_value) * DIAL_PRECISION);
                        oled_canvas_write("\n", 1, true);
                        oled_canvas_write("\n", 1, true);

                        sprintf(string_buffer, "M base: %3.3f\n\nM added: %3.3f\n", base_master_gain, added_master_gain);
                        string_length = strlen(string_buffer);
                        oled_canvas_write(string_buffer, string_length, true);
                        memset(string_buffer, 0, string_length);

                        oled_canvas_write("\n", 1, true);
                        oled_canvas_write("\n", 1, true);

                        oled_canvas_show();
                    }

                }
                // Other features here dont have screens
            }



            rotary_encoder_1_old_value = rotary_encoder_1_new_value;
        }

        //if (rotary_encoder_get_counter(rotary_encoder_2) != rotary_encoder_2_old_value && screen_enabled){
        //    rotary_encoder_2_new_value = rotary_encoder_get_counter(rotary_encoder_2);
        //    rotary_encoder_2_old_value = rotary_encoder_2_new_value;
        //}

        if(screen_enabled_old != screen_enabled){
            screen_enabled_old = screen_enabled;

            if(!screen_enabled){
                oled_canvas_clear();
                oled_canvas_show();
            }
        }

        if(send_data){
            throttle = (uint)joystick_get_throttle_percent();
            yaw = (uint)joystick_get_yaw_percent();
            pitch = (uint)joystick_get_pitch_percent();
            roll = (uint)joystick_get_roll_percent();
            //printf("Current Joystick: %d %d %d %d\n", throttle, yaw, pitch, roll);
        
            char *string = generate_message_joystick_nrf24(throttle, yaw, pitch, roll);
            printf("'%s'\n", string);
            if(nrf24_transmit(string)){
                gpio_put(2, 1);
            }
            
            free(string);
        }
        
        sleep_ms(5);
        gpio_put(2, 0);
    }
}

unsigned char* generate_message_joystick_nrf24(uint throttle, uint yaw, uint pitch, uint roll){
    // calculate the length of the resulting string
    int length = snprintf(NULL, 0, "/%u/%u/%u/%u/  ", throttle, yaw, pitch, roll);
    
    // allocate memory for the string
    unsigned char *string = malloc(length + 1); // +1 for the null terminator

    // format the string
    snprintf((char*)string, length + 1, "/%u/%u/%u/%u/  ", throttle, yaw, pitch, roll);

    return string;
}

void button1_callback(){
    printf("\nCALLED 1\n");

    if(!screen_enabled){
        return;
    }

    // GOOD
    if(current_mode == MODE_MAIN){

        uint8_t size_mode_select_strings = sizeof(mode_select_strings) / sizeof(mode_select_strings[0]);
        uint16_t selected = positive_mod(rotary_encoder_1_new_value, size_mode_select_strings);
        current_mode = (mode_t)selected;

    // GOOD
    }else if(current_mode == MODE_CONTROL){

        if(current_control == CONTROL_MODE_NONE){
            uint8_t size_control_mode_strings = sizeof(control_mode_strings) / sizeof(control_mode_strings[0]);
            uint16_t selected = positive_mod(rotary_encoder_1_new_value, size_control_mode_strings);
            current_control = selected;
            if(selected == 0){
                current_mode = MODE_MAIN;
            }
        }

        // No features in the control page currently
        
    }else if(current_mode == MODE_PID_TUNE){

        // GOOD
        if(current_pid_tune == PID_TUNE_MODE_NONE){
            uint8_t size_pid_tune_mode_strings = sizeof(pid_tune_mode_strings) / sizeof(pid_tune_mode_strings[0]);
            uint16_t selected = positive_mod(rotary_encoder_1_new_value, size_pid_tune_mode_strings);
            current_pid_tune = selected;
            if(selected == 0){
                current_mode = MODE_MAIN;
            }

            if(current_pid_tune == PID_TUNE_MODE_SYNC_REMOTE_WITH_SLAVE){
                printf("SYNC REMOTE WITH SLAVE\n");
                current_pid_tune = PID_TUNE_MODE_NONE;
            }

            if(current_pid_tune == PID_TUNE_MODE_SYNC_SLAVE_WITH_REMOTE){
                printf("SYNC SLAVE WITH REMOTEn");
                current_pid_tune = PID_TUNE_MODE_NONE;
            }

        }else if (current_pid_tune == PID_TUNE_MODE_EDIT_PID_VALUES){

            if(current_pid_tune_edit == PID_TUNE_MODE_EDIT_NONE){
                uint8_t size_pid_tune_mode_edit_strings = sizeof(pid_tune_mode_edit_strings) / sizeof(pid_tune_mode_edit_strings[0]);
                uint16_t selected = positive_mod(rotary_encoder_1_new_value, size_pid_tune_mode_edit_strings);
                current_pid_tune_edit = selected;
                if(selected == 0){
                    current_pid_tune = PID_TUNE_MODE_NONE;

                    printf("Going back to pid mode\n");
                }
            }else if(current_pid_tune_edit == PID_TUNE_MODE_EDIT_PROPORTIONAL){
                current_pid_tune_edit = PID_TUNE_MODE_EDIT_NONE;
            }else if(current_pid_tune_edit == PID_TUNE_MODE_EDIT_INTEGRAL){
                current_pid_tune_edit = PID_TUNE_MODE_EDIT_NONE;
            }else if(current_pid_tune_edit == PID_TUNE_MODE_EDIT_DERIVATIVE){
                current_pid_tune_edit = PID_TUNE_MODE_EDIT_NONE;
            }else if(current_pid_tune_edit == PID_TUNE_MODE_EDIT_MASTER_GAIN){
                current_pid_tune_edit = PID_TUNE_MODE_EDIT_NONE;
            }
        }
    }

    // printf("Before encoder 1 - %d - %d\n", rotary_encoder_1, rotary_encoder_get_counter(rotary_encoder_1));
    // printf("Before encoder 2 - %d - %d\n", rotary_encoder_2, rotary_encoder_get_counter(rotary_encoder_2));
    rotary_encoder_reset_counter(rotary_encoder_1);
    rotary_encoder_reset_counter(rotary_encoder_2);

    rotary_encoder_1_old_value = 0;
    rotary_encoder_1_new_value = 0;
    rotary_encoder_2_old_value = 0;
    rotary_encoder_2_new_value = 0;


    // printf("After encoder 1 - %d - %d\n", rotary_encoder_1, rotary_encoder_get_counter(rotary_encoder_1));
    // printf("After encoder 2 - %d - %d\n", rotary_encoder_2, rotary_encoder_get_counter(rotary_encoder_2));

}

void button2_callback(){
    printf("\nCALLED 2\n");
    // send_data = !send_data;
    screen_enabled = !screen_enabled;
}