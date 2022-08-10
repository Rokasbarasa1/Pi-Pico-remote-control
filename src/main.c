#include "pico/stdlib.h"
#include <stdio.h>
#include "hardware/uart.h"
#include <string.h>

#include "../lib/esp_01/esp_01.h"

// should put in defines
char *wifi_name = "Stofa70521";
char *wifi_password = "bis56lage63";
char *server_ip = "192.168.87.178";
char *server_port = "4000";

// esp 01
int main() {
    stdio_init_all();
    
    // led
    gpio_init(2);
    gpio_set_dir(2, GPIO_OUT);
    gpio_put(2, 1);

    // sleep because the program executes too quick
    // and doesn't cache what is sent
    sleep_ms(5000);

    // init esp 01
    init_esp_01(uart1, 3);
    esp_01_connect_wifi(uart1, wifi_name, wifi_password);
    esp_01_send_http(
        uart1, 
        server_ip, 
        server_port, 
        "GET /users HTTP/1.1\r\nHost: 192.168.87.178\r\n"
    );

    esp_01_send_http(
        uart1, 
        server_ip, 
        server_port, 
        "GET / HTTP/1.1\r\nHost: 192.168.87.178\r\n"
    );

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