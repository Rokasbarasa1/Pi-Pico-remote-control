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
#include "hardware/timer.h"

#include "../lib/esp_01/esp_01.h"
#include "../lib/joystick/joystick.h"
#include "../lib/button/button.h"
#include "../lib/rotary-encoder/rotary-encoder.h"
#include "../lib/adxl345/adxl345.h"
#include "../lib/nrf24l01/nrf24l01.h"
#include "../lib/oled-display/oled-display.h"

#define MAX_MESSAGE_LENGTH 100
#define DIAL_PRECISION 0.1

void button1_callback();
void button2_callback();
void screen_menu_logic();
void apply_pid_to_slave();
void sync_remote_with_slave();

unsigned char* int_to_string(uint number);
unsigned char* generate_message_joystick_nrf24(uint throttle, uint yaw, uint pitch, uint roll);
unsigned char* generate_message_pid_values_nrf24(double added_proportional, double added_integral, double added_derivative, double added_master_gain);
uint16_t positive_mod(int32_t value, uint16_t value_modal);

void extract_pid_values(char *request, uint8_t request_size, double *base_proportional, double *base_integral, double *base_derivative, double *base_master);

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


// Menu structure #######################################################
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
    (uint8_t*)"Sync Slave To This",
    (uint8_t*)"Sync This To Slave"
};

enum t_pid_tune_mode_edit {
    PID_TUNE_MODE_EDIT_NONE,
    PID_TUNE_MODE_EDIT_PROPORTIONAL,
    PID_TUNE_MODE_EDIT_INTEGRAL,
    PID_TUNE_MODE_EDIT_DERIVATIVE,
    PID_TUNE_MODE_EDIT_MASTER_GAIN,
    PID_TUNE_MODE_EDIT_APPLY_TO_SLAVE,
};
uint8_t* pid_tune_mode_edit_strings[] = {
    (uint8_t*)"Back",
    (uint8_t*)"Proportional  ",
    (uint8_t*)"Integral      ",
    (uint8_t*)"Derivative    ",
    (uint8_t*)"Master Gain   ",
    (uint8_t*)"Apply to slave",
};

// State of what menu is showing #####################################################
enum t_mode current_mode = MODE_MAIN;
enum t_mode old_mode = MODE_CONTROL;

enum t_control_mode current_control = CONTROL_MODE_NONE;
enum t_control_mode old_control = CONTROL_MODE_NONE;

enum t_pid_tune_mode current_pid_tune = PID_TUNE_MODE_NONE;
enum t_pid_tune_mode old_pid_tune = PID_TUNE_MODE_NONE;

enum t_pid_tune_mode_edit current_pid_tune_edit = PID_TUNE_MODE_EDIT_NONE;
enum t_pid_tune_mode_edit old_pid_tune_edit = PID_TUNE_MODE_EDIT_NONE;


// State of the rotary encoder and changes ###############################################

// These are just the identifiers for the encoders. Badly named
uint8_t rotary_encoder_1 = -99;
uint8_t rotary_encoder_2 = -99;

int32_t rotary_encoder_1_old_value = 0;
int32_t rotary_encoder_2_old_value = 0;

int32_t rotary_encoder_1_new_value = 0;
int32_t rotary_encoder_2_new_value = 0;

// State of the pid menu
volatile double m_base_proportional = 7.4;
volatile double m_base_integral = 3.4;
volatile double m_base_derivative = 2000;
volatile double m_base_master_gain = 1.0;

volatile double m_added_proportional = 0;
volatile double m_added_integral = 0;
volatile double m_added_derivative = 0;
volatile double m_added_master_gain = 0;

// State of triggered actions
bool action_apply_pid_to_slave = false;
bool action_sync_remote_to_slave = false;

volatile char string_buffer[100];
volatile uint8_t string_length = 0;

bool screen_enabled = true;
bool screen_enabled_old = true;

bool current_remote_synced_to_slave = false;
bool old_remote_synced_to_slave = false;

uint8_t tx_address[5] = {0xEE, 0xDD, 0xCC, 0xBB, 0xAA};
char rx_data[32];

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

    sleep_ms(1000);
    printf("Oled initialized\n");

    // ########################################################## Setup radio communication
    if(nrf24_init(spi_default, 7, 8, true)){
        printf("nrf24 setup succeeded\n");
    }else{
        printf("nrf24 setup failed\n");
    }

    nrf24_tx_mode(tx_address, 10);

    printf("Radio initialized\n");

    // ########################################################## Main loop
    printf("\n\n====START OF LOOP====\n\n");
    while (true) {

        screen_menu_logic();

        if(current_mode == MODE_CONTROL){
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

uint16_t positive_mod(int32_t value, uint16_t value_modal){
    int32_t result = value % value_modal;
    if (result < 0) {
        result += value_modal;
    }
    return result;
}

unsigned char* generate_message_joystick_nrf24(uint throttle, uint yaw, uint pitch, uint roll){
    // calculate the length of the resulting string
    int length = snprintf(NULL, 0, "/joystick/%u/%u/%u/%u/  ", throttle, yaw, pitch, roll);
    
    // allocate memory for the string
    unsigned char *string = malloc(length + 1); // +1 for the null terminator

    // format the string
    snprintf((char*)string, length + 1, "/joystick/%u/%u/%u/%u/  ", throttle, yaw, pitch, roll);

    return string;
}

unsigned char* generate_message_pid_values_nrf24(double added_proportional, double added_integral, double added_derivative, double added_master_gain){
    // calculate the length of the resulting string
    int length = snprintf(
        NULL, 
        0, 
        "/pid/%.2f/%.2f/%.2f/%.2f/  ", 
        added_proportional, 
        added_integral, 
        added_derivative, 
        added_master_gain
    );
    
    // allocate memory for the string
    unsigned char *string = malloc(length + 1); // +1 for the null terminator

    // format the string
    snprintf(
        (char*)string, 
        length + 1, 
        "/pid/%.2f/%.2f/%.2f/%.2f/  ", 
        added_proportional, 
        added_integral, 
        added_derivative, 
        added_master_gain
    );

    return string;
}


void screen_menu_logic(){
    // Print out the new screen after button click. Also is triggered when screen goes from off to on
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
                    }
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
                    }
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
                    }

                    // No better way to do it
                    if(i == 3 && current_remote_synced_to_slave){
                        oled_canvas_write(" X", 2, true);
                    }
                    
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
                    }

                    // No better way to do it
                    if(i == 3 && current_remote_synced_to_slave){
                        oled_canvas_write(" X", 2, true);
                    }
                    oled_canvas_write("\n", 1, true);
                }

                oled_canvas_invert_row(selected_row);
                oled_canvas_show();
            }else if(current_pid_tune == PID_TUNE_MODE_EDIT_PID_VALUES){
                oled_canvas_clear();

                uint8_t selected_row = 0;
                uint8_t size_pid_tune_mode_edit_strings = sizeof(pid_tune_mode_edit_strings) / sizeof(pid_tune_mode_edit_strings[0]);

                double gains[4] = {
                    m_base_proportional+m_added_proportional,
                    m_base_integral+m_added_integral,
                    m_base_derivative+m_added_derivative,
                    m_base_master_gain+m_added_master_gain
                };
                
                for (size_t i = 0; i < size_pid_tune_mode_edit_strings; i++)
                {
                    sprintf(string_buffer, "%s", pid_tune_mode_edit_strings[i]);
                    string_length = strlen(string_buffer);
                    oled_canvas_write(string_buffer, string_length, true);
                    memset(string_buffer, 0, string_length);
                    if(i == 0){
                        selected_row = i;
                    }

                    if(i >= 1 && i <= 4){
                        sprintf(string_buffer, "%.2f", gains[i-1]);
                        string_length = strlen(string_buffer);
                        oled_canvas_write(string_buffer, string_length, true);
                        memset(string_buffer, 0, string_length);
                    }

                    oled_canvas_write("\n", 1, true);
                }

                oled_canvas_invert_row(selected_row);
                oled_canvas_show();
            }
            // Other values dont have their own page

        }else if(current_pid_tune_edit != old_pid_tune_edit){
            old_pid_tune_edit = current_pid_tune_edit;

            // print out the default menu if nothing is selected
            if(current_pid_tune_edit == PID_TUNE_MODE_EDIT_NONE){
                oled_canvas_clear();

                uint8_t selected_row = 0;
                uint8_t size_pid_tune_mode_edit_strings = sizeof(pid_tune_mode_edit_strings) / sizeof(pid_tune_mode_edit_strings[0]);

                double gains[4] = {
                    m_base_proportional+m_added_proportional,
                    m_base_integral+m_added_integral,
                    m_base_derivative+m_added_derivative,
                    m_base_master_gain+m_added_master_gain
                };

                for (size_t i = 0; i < size_pid_tune_mode_edit_strings; i++)
                {
                    sprintf(string_buffer, "%s", pid_tune_mode_edit_strings[i]);
                    string_length = strlen(string_buffer);
                    oled_canvas_write(string_buffer, string_length, true);
                    memset(string_buffer, 0, string_length);
                    if(i == 0){
                        selected_row = i;
                    }
                    
                    // Addition that prints the values of gains next to the menu items of them
                    if(i >= 1 && i <= 4){
                        sprintf(string_buffer, "%.2f", gains[i-1]);
                        string_length = strlen(string_buffer);
                        oled_canvas_write(string_buffer, string_length, true);
                        memset(string_buffer, 0, string_length);
                    }

                    oled_canvas_write("\n", 1, true);
                }

                oled_canvas_invert_row(selected_row);
                oled_canvas_show();
            }else if(current_pid_tune_edit == PID_TUNE_MODE_EDIT_PROPORTIONAL){
                oled_canvas_clear();

                oled_canvas_write("\n", 1, true);
                oled_canvas_write("\n", 1, true);

                sprintf(string_buffer, "P base: %3.3f\n\nP added: %3.3f\n", m_base_proportional, m_added_proportional);
                string_length = strlen(string_buffer);
                oled_canvas_write(string_buffer, string_length, true);
                memset(string_buffer, 0, string_length);

                oled_canvas_show();
            }else if(current_pid_tune_edit == PID_TUNE_MODE_EDIT_INTEGRAL){
                oled_canvas_clear();

                oled_canvas_write("\n", 1, true);
                oled_canvas_write("\n", 1, true);

                sprintf(string_buffer, "I base: %3.3f\n\nI added: %3.3f\n", m_base_integral, m_added_integral);
                string_length = strlen(string_buffer);
                oled_canvas_write(string_buffer, string_length, true);
                memset(string_buffer, 0, string_length);

                oled_canvas_show();
            }else if(current_pid_tune_edit == PID_TUNE_MODE_EDIT_DERIVATIVE){
                oled_canvas_clear();

                oled_canvas_write("\n", 1, true);
                oled_canvas_write("\n", 1, true);

                sprintf(string_buffer, "D base: %3.3f\n\nD added: %3.3f\n", m_base_derivative, m_added_derivative);
                string_length = strlen(string_buffer);
                oled_canvas_write(string_buffer, string_length, true);
                memset(string_buffer, 0, string_length);
                
                oled_canvas_show();
            }else if(current_pid_tune_edit == PID_TUNE_MODE_EDIT_MASTER_GAIN){
                oled_canvas_clear();

                oled_canvas_write("\n", 1, true);
                oled_canvas_write("\n", 1, true);

                sprintf(string_buffer, "M base: %3.3f\n\nM added: %3.3f\n", m_base_master_gain, m_added_master_gain);
                string_length = strlen(string_buffer);
                oled_canvas_write(string_buffer, string_length, true);
                memset(string_buffer, 0, string_length);

                oled_canvas_show();
            }
        }
    }
    
    // Print out the new screen after rotation of the rotary encoder
    if (((rotary_encoder_get_counter(rotary_encoder_1) != rotary_encoder_1_old_value && screen_enabled) || screen_enabled_old == false || current_remote_synced_to_slave != old_remote_synced_to_slave) && screen_enabled == true){
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
                }
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
                    }
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
                    }

                    // No better way to do it
                    if(i == 3 && current_remote_synced_to_slave){
                        oled_canvas_write(" X", 2, true);
                    }
                    oled_canvas_write("\n", 1, true);
                }

                oled_canvas_invert_row(selected_row);
                oled_canvas_show();
            }else if (current_pid_tune == PID_TUNE_MODE_EDIT_PID_VALUES){

                if(current_pid_tune_edit == PID_TUNE_MODE_EDIT_NONE){
                    oled_canvas_clear();

                    uint8_t selected_row = 0;
                    uint8_t size_pid_tune_mode_edit_strings = sizeof(pid_tune_mode_edit_strings) / sizeof(pid_tune_mode_edit_strings[0]);

                    double gains[4] = {
                        m_base_proportional+m_added_proportional,
                        m_base_integral+m_added_integral,
                        m_base_derivative+m_added_derivative,
                        m_base_master_gain+m_added_master_gain
                    };
                    for (size_t i = 0; i < size_pid_tune_mode_edit_strings; i++)
                    {
                        sprintf(string_buffer, "%s", pid_tune_mode_edit_strings[i]);
                        string_length = strlen(string_buffer);
                        oled_canvas_write(string_buffer, string_length, true);
                        memset(string_buffer, 0, string_length);
                        if(positive_mod(rotary_encoder_1_new_value, size_pid_tune_mode_edit_strings) == i){
                            selected_row = i;
                        }

                        // Addition that prints the values of gains next to the menu items of them
                        if(i >= 1 && i <= 4){
                            sprintf(string_buffer, "%.2f", gains[i-1]);
                            string_length = strlen(string_buffer);
                            oled_canvas_write(string_buffer, string_length, true);
                            memset(string_buffer, 0, string_length);
                        }
                        oled_canvas_write("\n", 1, true);
                    }

                    oled_canvas_invert_row(selected_row);
                    oled_canvas_show();
                }
                if(current_pid_tune_edit == PID_TUNE_MODE_EDIT_PROPORTIONAL){
                    oled_canvas_clear();

                    m_added_proportional = m_added_proportional + ((rotary_encoder_1_new_value - rotary_encoder_1_old_value) * DIAL_PRECISION);
                    oled_canvas_write("\n", 1, true);
                    oled_canvas_write("\n", 1, true);

                    sprintf(string_buffer, "P base: %3.3f\n\nP added: %3.3f\n", m_base_proportional, m_added_proportional);
                    string_length = strlen(string_buffer);
                    oled_canvas_write(string_buffer, string_length, true);
                    memset(string_buffer, 0, string_length);

                    oled_canvas_show();
                }else if(current_pid_tune_edit == PID_TUNE_MODE_EDIT_INTEGRAL){
                    oled_canvas_clear();

                    m_added_integral = m_added_integral + ((rotary_encoder_1_new_value - rotary_encoder_1_old_value) * DIAL_PRECISION);
                    oled_canvas_write("\n", 1, true);
                    oled_canvas_write("\n", 1, true);

                    sprintf(string_buffer, "I base: %3.3f\n\nI added: %3.3f\n", m_base_integral, m_added_integral);
                    string_length = strlen(string_buffer);
                    oled_canvas_write(string_buffer, string_length, true);
                    memset(string_buffer, 0, string_length);

                    oled_canvas_show();
                }else if(current_pid_tune_edit == PID_TUNE_MODE_EDIT_DERIVATIVE){
                    oled_canvas_clear();

                    m_added_derivative = m_added_derivative + ((rotary_encoder_1_new_value - rotary_encoder_1_old_value) * DIAL_PRECISION);
                    oled_canvas_write("\n", 1, true);
                    oled_canvas_write("\n", 1, true);

                    sprintf(string_buffer, "D base: %3.3f\n\nD added: %3.3f\n", m_base_derivative, m_added_derivative);
                    string_length = strlen(string_buffer);
                    oled_canvas_write(string_buffer, string_length, true);
                    memset(string_buffer, 0, string_length);

                    oled_canvas_show();
                }else if(current_pid_tune_edit == PID_TUNE_MODE_EDIT_MASTER_GAIN){
                    oled_canvas_clear();

                    m_added_master_gain = m_added_master_gain + ((rotary_encoder_1_new_value - rotary_encoder_1_old_value) * DIAL_PRECISION);
                    oled_canvas_write("\n", 1, true);
                    oled_canvas_write("\n", 1, true);

                    sprintf(string_buffer, "M base: %3.3f\n\nM added: %3.3f\n", m_base_master_gain, m_added_master_gain);
                    string_length = strlen(string_buffer);
                    oled_canvas_write(string_buffer, string_length, true);
                    memset(string_buffer, 0, string_length);

                    oled_canvas_show();
                }

            }
        }

        // Update the old value to trigger the function next time
        rotary_encoder_1_old_value = rotary_encoder_1_new_value;
        old_remote_synced_to_slave = current_remote_synced_to_slave;
    }

    // React to screen turn on and off
    if(screen_enabled_old != screen_enabled){
        screen_enabled_old = screen_enabled;

        // Turn the screen off
        if(!screen_enabled){
            oled_canvas_clear();
            oled_canvas_show();
        }
    }

    // Handle actions ############################################
    if(action_apply_pid_to_slave){
        action_apply_pid_to_slave = false;
        apply_pid_to_slave();
    }

    if(action_sync_remote_to_slave){
        action_sync_remote_to_slave = false;
        sync_remote_with_slave();
    }
}

void button1_callback(){
    printf("\nCALLED 1\n");

    bool refresh_page = true;


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
                action_sync_remote_to_slave = true;

                current_pid_tune = PID_TUNE_MODE_NONE;
                refresh_page = false;
            }

            if(current_pid_tune == PID_TUNE_MODE_SYNC_SLAVE_WITH_REMOTE){
                printf("SYNC SLAVE WITH REMOTE");

                current_pid_tune = PID_TUNE_MODE_NONE;
                refresh_page = false;
            }

        }else if (current_pid_tune == PID_TUNE_MODE_EDIT_PID_VALUES){

            if(current_pid_tune_edit == PID_TUNE_MODE_EDIT_NONE){
                uint8_t size_pid_tune_mode_edit_strings = sizeof(pid_tune_mode_edit_strings) / sizeof(pid_tune_mode_edit_strings[0]);
                uint16_t selected = positive_mod(rotary_encoder_1_new_value, size_pid_tune_mode_edit_strings);
                current_pid_tune_edit = selected;
                if(selected == 0){
                    current_pid_tune = PID_TUNE_MODE_NONE;
                }

                if(current_pid_tune_edit == PID_TUNE_MODE_EDIT_APPLY_TO_SLAVE){
                    printf("APPLY TO SLAVE\n");
                    action_apply_pid_to_slave = true;

                    current_pid_tune_edit = PID_TUNE_MODE_NONE;
                    refresh_page = false;
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

    if(refresh_page){
        // Make the first item in the options be selected
        rotary_encoder_reset_counter(rotary_encoder_1);
        rotary_encoder_reset_counter(rotary_encoder_2);

        // Make sure the values are updated also so they dont trigger updates.
        rotary_encoder_1_old_value = 0;
        rotary_encoder_1_new_value = 0;
        rotary_encoder_2_old_value = 0;
        rotary_encoder_2_new_value = 0;
    }
}

void button2_callback(){
    printf("\nCALLED 2\n");
    // send_data = !send_data;
    screen_enabled = !screen_enabled;
}

void apply_pid_to_slave(){

    char *string = generate_message_pid_values_nrf24(
        m_added_proportional, 
        m_added_integral, 
        m_added_derivative, 
        m_added_master_gain
    );
    
    printf("'%s'\n", string);
    if(nrf24_transmit(string)){
        gpio_put(2, 1);
    }
    
    free(string);
}

#define NUM_COPIES 25
#define STRING_LENGTH 32 // assuming the string length is 32

void sync_remote_with_slave(){

    if(nrf24_transmit("/remoteSyncBase/")){
        gpio_put(2, 1);
    }

    // Turn on receive mode to wait for response from slave
    nrf24_rx_mode(tx_address, 10);

    uint8_t base_received_messages = 0;
    char base_received_copies[NUM_COPIES][STRING_LENGTH];
    char base_corrected_string[STRING_LENGTH];

    // Receive as many broken responses as possible
    for (size_t i = 0; i < 2000; i++)
    {
        if(nrf24_data_available(1)){
            nrf24_receive(rx_data);
            if(base_received_messages < NUM_COPIES-1){
                strncpy(base_received_copies[base_received_messages], rx_data, STRING_LENGTH);
            }
            base_received_messages++;
        }
        sleep_ms(1);
    }

    // If anything received do error correction on the data 
    if(base_received_messages > 10){
        // Check one character at a time
        for (int i = 0; i < STRING_LENGTH; i++) {
            int frequency[256] = {0}; // ASCII characters frequency

            // How many times does each character show up
            for (int j = 0; j < base_received_messages; j++) {
                frequency[(unsigned char) base_received_copies[j][i]]++;
            }
            
            int max_freq = 0;
            char most_frequent_char = 0;
            // Find the most abundant character
            for (int k = 0; k < 256; k++) {
                if (frequency[k] > max_freq) {
                    max_freq = frequency[k];
                    most_frequent_char = (char)k;
                }
            }
            // Set most abundant character to the finished string
            base_corrected_string[i] = most_frequent_char;
        }
        base_corrected_string[STRING_LENGTH - 1] = '\0'; // Null-terminate the string

        printf("Base - GOOD one '%s' samples - %d\n", base_corrected_string, base_received_messages);
    }else if (base_received_messages == 0){
        printf("Base - Timedout \n");
        nrf24_tx_mode(tx_address, 10);
        return;
    }else{
        printf("Base - Not enough samples - %d\n", base_received_messages);
        nrf24_tx_mode(tx_address, 10);
        return;
    }

    extract_pid_values(
        base_corrected_string, 
        strlen(base_corrected_string),
        &m_base_proportional,
        &m_base_integral,
        &m_base_derivative,
        &m_base_master_gain
    );

    sleep_ms(2000);
    // Toggle quickly to flush out trash
    nrf24_rx_mode(tx_address, 10);
    nrf24_tx_mode(tx_address, 10);
    sleep_ms(2000);


    if(nrf24_transmit("/remoteSyncAdded/")){
        gpio_put(2, 1);
    }

    // Turn on receive mode to wait for response from slave
    nrf24_rx_mode(tx_address, 10);

    uint8_t added_received_messages = 0;
    char added_received_copies[NUM_COPIES][STRING_LENGTH];
    char added_corrected_string[STRING_LENGTH];

    // Receive as many broken responses as possible
    for (size_t i = 0; i < 2000; i++)
    {
        if(nrf24_data_available(1)){
            nrf24_receive(rx_data);
            if(added_received_messages < NUM_COPIES-1){
                strncpy(added_received_copies[added_received_messages], rx_data, STRING_LENGTH);
            }
            added_received_messages++;
        }
        sleep_ms(1);
    }

    // If anything received do error correction on the data 
    if(added_received_messages > 10){
        // Check one character at a time
        for (int i = 0; i < STRING_LENGTH; i++) {
            int frequency[256] = {0}; // ASCII characters frequency

            // How many times does each character show up
            for (int j = 0; j < added_received_messages; j++) {
                frequency[(unsigned char) added_received_copies[j][i]]++;
            }
            
            int max_freq = 0;
            char most_frequent_char = 0;
            // Find the most abundant character
            for (int k = 0; k < 256; k++) {
                if (frequency[k] > max_freq) {
                    max_freq = frequency[k];
                    most_frequent_char = (char)k;
                }
            }
            // Set most abundant character to the finished string
            added_corrected_string[i] = most_frequent_char;
        }
        added_corrected_string[STRING_LENGTH - 1] = '\0'; // Null-terminate the string

        printf("Added - GOOD one '%s' samples - %d\n", added_corrected_string, added_received_messages);
    }else if (added_received_messages == 0){
        printf("Added - Timedout \n");
        nrf24_tx_mode(tx_address, 10);
        return;
    }else{
        printf("Added - Not enough samples - %d\n", added_received_messages);
        nrf24_tx_mode(tx_address, 10);
        return;
    }

    extract_pid_values(
        added_corrected_string, 
        strlen(added_corrected_string),
        &m_added_proportional,
        &m_added_integral,
        &m_added_derivative,
        &m_added_master_gain
    );

    current_remote_synced_to_slave = true;

    // Turn transmit back on to send out data to slaves as usual
    nrf24_tx_mode(tx_address, 10);
}

void extract_pid_values(char *request, uint8_t request_size, double *proportional, double *integral, double *derivative, double *master){
    // Skip the request type
    char* start = strchr(request, '/') + 1;
    char* end = strchr(start, '/');

    start = strchr(end, '/') + 1;
    end = strchr(start, '/');
    if(start == NULL || end == NULL ) return;
    int length = end - start;
    char proportional_string[length + 1];
    strncpy(proportional_string, start, length);
    proportional_string[length] = '\0';
    *proportional = strtod(proportional_string, NULL);
    //printf("'%s'\n", added_proportional);

    start = strchr(end, '/') + 1;
    end = strchr(start, '/');
    if(start == NULL || end == NULL ) return;
    length = end - start;
    char integral_string[length + 1];
    strncpy(integral_string, start, length);
    integral_string[length] = '\0';
    *integral = strtod(integral_string, NULL);
    //printf("'%s'\n", added_integral);

    start = strchr(end, '/') + 1;
    end = strchr(start, '/');
    if(start == NULL || end == NULL ) return;
    length = end - start;
    char derivative_string[length + 1];
    strncpy(derivative_string, start, length);
    derivative_string[length] = '\0';
    *derivative = strtod(derivative_string, NULL);
    //printf("'%s'\n", added_derivative);

    start = strchr(end, '/') + 1;
    end = strchr(start, '/');
    if(start == NULL || end == NULL ) return;
    length = end - start;
    char master_string[length + 1];
    strncpy(master_string, start, length);
    master_string[length] = '\0';
    *master = strtod(master_string, NULL);
    //printf("'%s'\n", added_master);
}