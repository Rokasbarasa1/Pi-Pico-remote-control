#include "./esp_01.h"

bool first_transmission = true;

bool sendAT(uart_inst_t *uart, char *command , char *ack, uint timeout_ms, bool logging){
    // put the closing tags to the command... pain in the ass
    char command_result[strlen(command) + 2];
    memset(command_result, 0, (strlen(command) + 2)*sizeof(char));
    strcat(command_result, command);
    strcat(command_result, "\r\n");

    uart_puts(uart, command_result);
    // free(command_result);
    // dont waste read attempts while nothing is there
    // while (!uart_is_readable(uart));
    if(!uart_is_readable_within_us(uart, timeout_ms*1000)){
        return false;
    }
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
            if(!uart_is_readable_within_us(uart, timeout_ms*1000)){
                return false;
            }
            char data = uart_getc(uart1);
        }
    }

    for(uint i = 0; i<2000; i++){
            if(!uart_is_readable_within_us(uart, timeout_ms*1000)){
                break;
            }
            char data = uart_getc(uart1);
            if(logging){
                printf("%c", data);
            }

            // Check if ack reached
            if(data == ack[u]){
                u++;
                if(u == ack_length){
                    if(logging){
                        printf("\n");
                    }
                    
                    return true;
                }
            }else{
                u = 0;
            }

            if(data == error[e]){
                e++;
                if(e == error_length){
                    if(logging){
                        printf("\n");
                    }
                    return false;
                }
            }else{
                e = 0;
            }
    }
    return false;
}

bool init_esp_01_client(uart_inst_t *uart, uint enable_pin){

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

    bool result1 = sendAT(uart, "AT+RST", "ready", 500, true);
    bool result2 = sendAT(uart, "AT+CWMODE=1", "OK", 500, true);
    bool result3 = sendAT(uart, "AT+CWLAP", "OK", 5000, true);

    return result1 && result2 && result3;
}

bool esp_01_connect_wifi(uart_inst_t *uart, char *wifi_name, char *wifi_password){
    uint connect_command_length = strlen(wifi_name) + strlen(wifi_password) + 14;
    char connect_command[connect_command_length];
    memset(connect_command, 0, connect_command_length * sizeof(char));

    strcat(connect_command, "AT+CWJAP=\"");
    strcat(connect_command, wifi_name);
    strcat(connect_command, "\",\"");
    strcat(connect_command, wifi_password);
    strcat(connect_command, "\"");

    // sendAT(uart1, "AT+CWJAP=\"Stofa70521\",\"bis56lage63\"\r\n", "OK");

    return  sendAT(uart1, connect_command, "OK", 20000, true);
}

int numPlaces (int n) {
    if (n < 0) return 0;
    if (n < 10) return 1;
    if (n < 100) return 2;
    if (n < 1000) return 3;
    if (n < 10000) return 4;
    if (n < 100000) return 5;
    if (n < 1000000) return 6;
    if (n < 10000000) return 7;
    if (n < 100000000) return 8;
    if (n < 1000000000) return 9;
    /*      2147483647 is 2^31-1 - add more ifs as needed
       and adjust this final return as well. */
    return 10;
}

bool esp_01_send_http(uart_inst_t *uart, char *ADDRESS, char *PORT, char *command){

    // connect to the server
    // sendAT(uart1, "AT+CIPSTART=\"TCP\",\"192.168.87.178\",4000", "OK");

    uint connect_server_length = strlen(ADDRESS) + strlen(PORT) + strlen("AT+CIPSTART=\"TCP\",\""); // 21
    char connect_server[connect_server_length];
    memset(connect_server, 0, connect_server_length * sizeof(char));

    strcat(connect_server, "AT+CIPSTART=\"TCP\",\"");
    strcat(connect_server, ADDRESS);
    strcat(connect_server, "\",");
    strcat(connect_server, PORT);
 
    sendAT(uart, connect_server, "OK", 2000, false);

    // send how many bytes the message will be
    // sendAT(uart1, "AT+CIPSEND=45", "OK");
    // the plus 2 is for the /r/n that gets added to every message that is very important
    uint command_length = strlen(command) + 2;                      // length is: 10 + 2
    char number[numPlaces(command_length)];                         // the lenghth of 12 is 2
    memset(number, 0, numPlaces(command_length) * sizeof(char));    // allocate the memory for it
    sprintf(number, "%d", command_length);                          // then convert the number into char

    // get characters for the whole command
    uint bytes_length = strlen("AT+CIPSEND=") + numPlaces(command_length);
    char bytes_command[bytes_length];
    memset(bytes_command, 0, bytes_length * sizeof(char));

    strcat(bytes_command, "AT+CIPSEND=");
    strcat(bytes_command, number);
    sendAT(uart, bytes_command, "OK", 2000, false);

    // send the actual command
    // sendAT(uart, "GET /users HTTP/1.1\r\nHost: 192.168.87.178\r\n", "CLOSED");

    sendAT(uart, command, "CLOSED", 10000, false);
    printf("Transmission sent\n");
    return true;
}