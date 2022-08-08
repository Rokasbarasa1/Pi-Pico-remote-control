#include "pico/stdlib.h"
#include <stdio.h>
#include "hardware/uart.h"
#include <string.h>

bool first_transmission = true;
char *wifi_name = "Stofa70521";
char *wifi_password = "bis56lage63";


bool sendAT(uart_inst_t *uart, char *command , char *ack){
    // put the closing tags to the command... pain in the ass
    char command_result[strlen(command) + 2];
    memset( command_result, 0, (strlen(command) + 2)*sizeof(char) );
    strcat(command_result, command);
    strcat(command_result, "\r\n");

    uart_puts(uart, command_result);

    // dont waste read attempts while nothing is there
    while (!uart_is_readable(uart));

    char *error = "ERROR";
    uint e = 0;
    uint error_length = strlen(error);

    uint ack_length = strlen(ack);
    uint u = 0;

    // skip 32 garbage characters for first time uart
    // dont know why this happens
    if(first_transmission){
        first_transmission = false;
        for(uint i = 0; i<32; i++){
            char data = uart_getc(uart1);
        }
    }

    for(uint i = 0; i<2000; i++){
            char data = uart_getc(uart1);
            printf("%c", data);

            // Check if ack reached
            if(data == ack[u]){
                u++;
                if(u == ack_length){
                    printf("\n");
                    return true;
                }
            }else{
                u = 0;
            }

            if(data == error[e]){
                e++;
                if(e == error_length){
                    printf("\n");
                    return false;
                }
            }else{
                e = 0;
            }
    }
    return false;
}

bool init_esp_01(uint enable_pin, uart_inst_t *uart){

    // enable the device 
    gpio_init(enable_pin);
    gpio_set_dir(enable_pin, GPIO_OUT);
    gpio_put(enable_pin, 1);

    uart_init(uart, 115200);
    if(uart == uart0){
        gpio_set_function(1, GPIO_FUNC_UART);
        gpio_set_function(2, GPIO_FUNC_UART);
    }else if(uart == uart1){
        gpio_set_function(4, GPIO_FUNC_UART);
        gpio_set_function(5, GPIO_FUNC_UART);
    }

    bool result1 = sendAT(uart1, "AT+RST", "ready");
    bool result2 = sendAT(uart1, "AT+CWMODE=1", "OK");
    bool result3 = sendAT(uart1, "AT+CWLAP", "OK");

    return result1 && result2 && result3;
}

bool esp_01_connect_wifi(char *wifi_name, char *wifi_password){
    char result[30] = "AT+CWJAP=\"";
    strcat(result, wifi_name);
    strcat(result, "\",\"");
    strcat(result, wifi_password);
    strcat(result, "\"\r\n");

    sendAT(uart1, result, "OK");
    // sendAT(uart1, "AT+CWJAP=\"Stofa70521\",\"bis56lage63\"\r\n", "OK");

    return true;
}

// bool send(char *IP, char *PORT, )

int main() {
    stdio_init_all();
    
    // led
    gpio_init(2);
    gpio_set_dir(2, GPIO_OUT);
    gpio_put(2, 1);

    // init esp 01

    // init_esp_01(3, uart1);
    // uart AT enable pin
    gpio_init(3);
    gpio_set_dir(3, GPIO_OUT);
    gpio_put(3, 1);

    // uart
    uart_init(uart1, 115200);
    gpio_set_function(4, GPIO_FUNC_UART);
    gpio_set_function(5, GPIO_FUNC_UART);
    sleep_ms(5000);

    // sendAT(uart1, "AT+RST\r\n", "ready");
    // sendAT(uart1, "AT+CWMODE=1\r\n", "OK");
    // sendAT(uart1, "AT+CWLAP\r\n", "OK");
    // sendAT(uart1, "AT+CWJAP=\"Stofa70521\",\"bis56lage63\"\r\n", "OK");
    // sendAT(uart1, "AT+CIPSTART=\"TCP\",\"192.168.87.178\",4000\r\n", "OK");
    // // How many characters the next command will be including the next line stuff
    // sendAT(uart1, "AT+CIPSEND=45\r\n", "OK");
    // sendAT(uart1, "GET /users HTTP/1.1\r\nHost: 192.168.87.178\r\n\r\n", "CLOSED");

    sendAT(uart1, "AT+RST", "ready");
    sendAT(uart1, "AT+CWMODE=1", "OK");
    sendAT(uart1, "AT+CWLAP", "OK");
    sendAT(uart1, "AT+CWJAP=\"Stofa70521\",\"bis56lage63\"", "OK");
    sendAT(uart1, "AT+CIPSTART=\"TCP\",\"192.168.87.178\",4000", "OK");
    // How many characters the next command will be including the next line stuff
    sendAT(uart1, "AT+CIPSEND=45", "OK");
    sendAT(uart1, "GET /users HTTP/1.1\r\nHost: 192.168.87.178\r\n", "CLOSED");
    while (true) {
        gpio_put(2, 1);
        sleep_ms(100);

        gpio_put(2, 0);
        sleep_ms(100);
    }
}

// Blink led and print to serial monitor
// #include "pico/stdlib.h"
// #include <stdio.h>

// int main() {
//     stdio_init_all();
//     const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    
//     gpio_init(LED_PIN);
//     gpio_set_dir(LED_PIN, GPIO_OUT);

//     while (true) {
//         gpio_put(LED_PIN, 1);
//         sleep_ms(100);
//         printf("ON\n");

//         gpio_put(LED_PIN, 0);
//         sleep_ms(100);
//         printf("OFF\n");
//     }
// }