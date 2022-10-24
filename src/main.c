#include "pico/stdlib.h"
#include <stdio.h>
#include "hardware/uart.h"
#include <string.h>
#include <stdlib.h>

#include "../lib/esp_01/esp_01.h"
#include "../lib/joystick/joystick.h"
#include "../lib/adxl345/adxl345.h"
#include "../lib/nrf24l01/nrf24l01.h"

char *wifi_name = "ESP32_wifi";
char *wifi_password = "1234567890";
char *server_ip = "192.168.8.1";
char *server_port = "3500";

volatile bool send_data = false;

void button_callback(){
    printf("\nCALLED\n");
    send_data = !send_data;
}

char* int_to_string(uint number);
char* generate_message_joystick(uint x, uint y, char *ADDRESS);


// esp 01
int main() {
    stdio_init_all();

    // status led
    gpio_init(2);
    gpio_set_dir(2, GPIO_OUT);
    gpio_put(2, 1);

    // sleep because the program executes too quick
    // and doesn't cache what is sent
    sleep_ms(2500);
    printf("STARTING PROGRAM\n");

    init_joystick(26,27,22, button_callback);

    bool adxl345_setup = adxl345_init(spi_default, 6, true);
    bool nrf24_setup = nrf24_init(spi_default, 7, 8, true);


    if(adxl345_setup){
        printf("adxl345 setup succeeded\n");
    }else{
        printf("adxl345 setup failed\n");
    }

    if(nrf24_setup){
        printf("nrf24 setup succeeded\n");
    }else{
        printf("nrf24 setup failed\n");
    }

    uint8_t tx_address[5] = {0xEE, 0xDD, 0xCC, 0xBB, 0xAA};
    uint8_t tx_data[] = "hello world!\n";

    // nrf24_tx_mode(tx_address, 10);
    nrf24_rx_mode(tx_address, 10);

    // init_esp_01_client(uart1, 3, false);
    
    bool connected = false;
    // while (!connected){
    //     connected = esp_01_client_connect_wifi(uart1, wifi_name, wifi_password, false);
    //     if(!connected){
    //         printf("FAILED TO CONNECT\n");
    //         sleep_ms(2000);
    //     }
    // }
    printf("CONNECTED TO WIFI\n");
    

    uint x_previous = 0;
    uint y_previous = 0;
    uint x = 1;
    uint y = 1;

    uint error_count = 0;
    uint error_count_limit = 10;
    uint8_t rx_data[32];
    printf("\n\n====START OF LOOP====\n\n");
    while (true) {
        if(send_data){
            x = (uint)get_x_percentage();
            y = (uint)get_y_percentage();

            // if( x != x_previous || y != y_previous){
            //     printf("\nCurrent x \%: %d\n", (uint)get_x_percentage());
            //     printf("Current y \%: %d\n", (uint)get_y_percentage());
            //     // printf("\nCurrent x: %d\n", (uint)get_x());
            //     // printf("Current y: %d\n", (uint)get_y());
            
            //     x_previous = x;
            //     y_previous = y;

            //     char *string = generate_message_joystick((uint)get_x_percentage(), (uint)get_y_percentage(), server_ip);
            //     bool result = esp_01_client_send_http(
            //         uart1, 
            //         server_ip, 
            //         server_port, 
            //         string,
            //         false
            //     );
            //     free(string);

            //     // check the connection
            //     if(result){
            //         printf("Transmission: OK\n");
            //     }else{
            //         printf("Transmission: ERROR\n");
            //         error_count++;
            //         if(error_count == error_count_limit){
            //             printf("Transmission: Trying to reconnect\n");
            //             connected = false;
            //             // send_data = false;
            //             error_count = 0;
            //             while (!connected){
            //                 connected = esp_01_client_connect_wifi(uart1, wifi_name, wifi_password, false);
            //                 if(!connected){
            //                     printf("FAILED TO CONNECT\n");
            //                     sleep_ms(2000);
            //                 }
            //             }
            //             printf("CONNECTED TO WIFI\n");
            //         }
            //     }
            // }   
        }
        

        printf("Receiving: ");
        if(nrf24_data_available(1)){
            nrf24_receive(rx_data);
            printf("The data: ");
            for(uint8_t i = 0; i < strlen((char*) rx_data); i++ ){
                printf("%c", ((char*) rx_data)[i]);
            }
            printf("\n");

            printf("The data: %s\n", ((char*) rx_data));
            gpio_put(2, 1);
        }else{
            printf("\n");
        }


        // printf("Transmitting: ");
        // if(nrf24_transmit(tx_data)){
        //     printf("TX success\n");
        //     gpio_put(2, 1);
        // }else{
        //     printf("TX failed\n");
        // }


        sleep_ms(500);
        gpio_put(2, 0);
        sleep_ms(500);
    }
}

char* int_to_string(uint number){
    uint8_t negative = 0;
    // if (number < 0){
    //     negative = 1;
    //     number *= -1;
    // }
    if(number == 0){
        char *string = malloc(2);
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

    char *string = malloc(count+negative+1);
    
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

char* generate_message_joystick(uint x, uint y, char *ADDRESS){
    char* x_str = int_to_string(x);
    char* y_str = int_to_string(y);

    uint connection_length = strlen("GET //") + strlen(" HTTP/1.1\r\nHost: ")+ strlen("\r\n") + strlen(ADDRESS) + strlen(x_str) + strlen(y_str);
    char connection[connection_length];
    memset(connection, 0, connection_length * sizeof(char));

    strcat(connection, "GET /");
    strcat(connection, x_str);
    strcat(connection, "/");
    strcat(connection, y_str);
    strcat(connection, " HTTP/1.1\r\nHost: ");
    strcat(connection, ADDRESS);
    strcat(connection, "\r\n");

    free(x_str);
    free(y_str);

    char *string = malloc(connection_length+1);

    for(uint8_t i = 0; i < connection_length-1; i++){
        string[i] = connection[i];
    }
    string[connection_length] = '\0';

    return string;
}
