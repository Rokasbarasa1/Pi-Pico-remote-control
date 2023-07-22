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

volatile bool send_data = false;

void button_callback(){
    printf("\nCALLED\n");
    send_data = !send_data;
}

unsigned char* int_to_string(uint number);
unsigned char* generate_message_joystick_esp01(uint x, uint y, unsigned char *ADDRESS);
unsigned char* generate_message_joystick_nrf24(uint throttle, uint yaw, uint pitch, uint roll);
bool disable_repeated_send = false;

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
 * Display
 * 
 * SDA GP4
 * SCL GP5
 */

volatile uint throttle = 0;
volatile uint yaw = 0;
volatile uint pitch = 0;
volatile uint roll = 0;

int main() {
    stdio_init_all();

    // Sleep a so you actually have time to read some of the serial outputs
    sleep_ms(2500);
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
    init_button(button_callback, 22);
    init_button(button_callback, 21);
    printf("Buttons initialized\n");

    // ########################################################## Rotary encoders
    uint8_t rotary_encoder_1 = init_rotary_encoder(12, 13);
    uint8_t rotary_encoder_2 = init_rotary_encoder(10, 11);
    printf("Rotary encoders initialized\n");

    // ########################################################## Oled display
    init_oled_display(i2c_default, PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN);
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