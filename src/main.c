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
    // float throttle = 0;
    // float yaw = 0;
    // float pitch = 0;
    // float roll = 0;

    uint throttle = 0;
    uint yaw = 0;
    uint pitch = 0;
    uint roll = 0;
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
            printf("Current Joystick: %d %d %d %d\n", throttle, yaw, pitch, roll);

            // throttle = joystick_get_throttle_volts();
            // yaw = joystick_get_yaw_volts();
            // pitch = joystick_get_pitch_volts();
            // roll = joystick_get_roll_volts();
            // printf("Current Joystick: %6.2f %6.2f %6.2f %6.2f\n", throttle, yaw, pitch, roll);
        
            char *string = generate_message_joystick_nrf24(throttle, yaw, pitch, roll);
            printf("'%s'\n", string);
            if(nrf24_transmit(string)){
                gpio_put(2, 1);
            }else{
                gpio_put(0, 1);
                printf("BAD\n");
            }
            
            gpio_put(2, 1);

            free(string);
        }
        
        sleep_ms(5);
        gpio_put(2, 0);
    }
}

unsigned char* int_to_string(uint number){
    uint8_t negative = 0;
    // if (number < 0){
    //     negative = 1;
    //     number *= -1;
    // }
    if(number == 0){
        unsigned char *string = malloc(2);
        string[0] = '0';
        string[1] = '\0'; 
        return string;
    }
    
    // get digit count
    uint8_t count = 0;   // variable declaration 
    int number_temp = number; 
    while(number_temp!=0)  {  
        number_temp=number_temp/10;
        count++;  
    }    

    unsigned char *string = malloc(count+negative+1);
    
    for(uint8_t i = 0+negative; i < count+negative; i++){
        uint32_t divisor = 1;
        for(uint8_t j = 0; j < count-1-i; j++){
            divisor = divisor * 10;
        }

        string[i] = 48 + (number / divisor);
        number =  number % divisor;
    }  

    if(negative){
        string[0] = '-';
    }

    string[count] = '\0';

    return string;
}

unsigned char* generate_message_joystick_nrf24(uint throttle, uint yaw, uint pitch, uint roll){
    unsigned char* throttle_str = int_to_string(throttle);
    unsigned char* yaw_str = int_to_string(yaw);
    unsigned char* pitch_str = int_to_string(pitch);
    unsigned char* roll_str = int_to_string(roll);

    uint connection_length = strlen("/////") + strlen(throttle_str) + strlen(yaw_str) + strlen(pitch_str) + strlen(roll_str) + 2;
    unsigned char connection[connection_length];
    memset(connection, 0, connection_length * sizeof(unsigned char));

    strcat(connection, "/");
    strcat(connection, throttle_str);
    strcat(connection, "/");
    strcat(connection, yaw_str);
    strcat(connection, "/");
    strcat(connection, pitch_str);
    strcat(connection, "/");
    strcat(connection, roll_str);
    strcat(connection, "/");
    strcat(connection, "  ");

    free(throttle_str);
    free(yaw_str);
    free(pitch_str);
    free(roll_str);

    unsigned char *string = malloc(connection_length+1);

    for(uint8_t i = 0; i < connection_length-1; i++){
        string[i] = connection[i];
    }
    string[connection_length] = '\0';

    return string;
}