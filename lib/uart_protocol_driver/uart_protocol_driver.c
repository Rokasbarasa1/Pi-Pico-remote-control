#include "./uart_protocol_driver.h"

#include <string.h>
#include <stdlib.h>

// Modes:
//  Async normal - data is transmitted by pulses of baud rate
//  Async double speed - like first mode but double speed. Uses when accurate baud rate 
//  sync - uses clock pulses to transfer data

typedef struct uart_t{
	 volatile uint8_t* baud_low;
	 volatile uint8_t* baud_high;
	 volatile uint8_t* control_register_c;
	 volatile uint8_t* control_register_b;
	 volatile uint8_t* control_register_a;
	 volatile uint8_t* data;
} uart_t;

volatile uart_t serials[]={
    {&UBRR0L, &UBRR0H, &UCSR0C, &UCSR0B, &UCSR0A, &UDR0},
	{&UBRR1L, &UBRR1H, &UCSR1C, &UCSR1B, &UCSR1A, &UDR1},
    {&UBRR2L, &UBRR2H, &UCSR2C, &UCSR2B, &UCSR2A, &UDR2},
    {&UBRR3L, &UBRR3H, &UCSR3C, &UCSR3B, &UCSR3A, &UDR3}
};

uint32_t baud_rates[] = {
    300
    ,1200
    ,2400
    ,4800
    ,9600
    ,19200
    ,38400
    ,57600
    ,74880
    ,115200
    ,250000
    ,500000
    ,1000000
    ,2000000
};

uint8_t baud_rate_valid(uint32_t baud_rate){
    uint8_t included = 0;
    for(uint8_t i = 0; i < 14; i++){
        if(baud_rates[i] == baud_rate){
            included = 1;
            break;
        }
    }
    return included;
}

/**
 * There are serial devices:
 *  0
 *  1
 *  2
 *  3
 * 
 *  Baud rate modes:
 *  300
 *  1200
 *  2400
 *  4800
 *  9600
 *  19200
 *  38400
 *  57600
 *  74880
 *  115200
 *  250000
 *  500000
 *  1000000
 *  2000000
 */
void init_uart_protocol(uint8_t serial_device, uint32_t baud_rate){
    if(serial_device < 3 && baud_rate_valid(baud_rate) == 1){

        // Calculate value using baud rate
        uint16_t baud_temp = (F_CPU / (8*baud_rate)) -1;
        *serials[serial_device].control_register_a |= _BV(U2X1);

        // Set the baud rate value
        *serials[serial_device].baud_low = baud_temp;
        *serials[serial_device].baud_high = baud_temp >> 8;

        // set mode as async normal
        *serials[serial_device].control_register_c &= ~(_BV(UMSEL10) | _BV(UMSEL10));

        // set to use one stop bit
        *serials[serial_device].control_register_c &= ~(_BV(USBS1));

        // set data frame format as 8 bit size
        *serials[serial_device].control_register_c |= _BV(UCSZ11) | _BV(UCSZ10);

        // // set interrupts to happen when data 
        // if(serial_device == 2){
        //     *serials[serial_device].control_register_b |= _BV(RXCIE1) | _BV(UDRIE1);
        // }

        // enable receive and transmit
        *serials[serial_device].control_register_b |= _BV(TXEN1) | _BV(RXEN1);

    }
}

// #define RX_BUFFER_SIZE 128
// volatile static uint8_t rx_buffer[8] = {0};
// volatile static uint16_t rx_count = 0;	
// volatile static uint8_t uart_tx_busy = 1;

// ISR(USART2_RX_vect){
	
// 	volatile static uint16_t rx_write_pos = 0;
	
// 	rx_buffer[rx_write_pos] = UDR0;
// 	rx_count++;
// 	rx_write_pos++;
// 	if(rx_write_pos >= RX_BUFFER_SIZE){
// 		rx_write_pos = 0;
// 	}
	
// }


// ISR(USART2_TX_vect){
// 	uart_tx_busy = 1;
// }



uint8_t uart_read_ready(uint8_t serial_device){
    // Check the receive complete bit
    // Get the bit from the register and represent it as 1 or 0
    return (*serials[serial_device].control_register_a & _BV(RXC1)) >> RXC1;
}

uint8_t uart_write_ready(uint8_t serial_device){
    // Check the data register empty bit
    // Get the bit from the register and represent it as 1 or 0
    return (*serials[serial_device].control_register_a & _BV(UDRE1)) >> UDRE1;
}

void uart_write_byte(uint8_t serial_device, uint8_t data){
    while (!uart_write_ready(serial_device));
    
    // This is possible because the register is split into two. With separate read, write
    *serials[serial_device].data = data;
}

void uart_write_string(uint8_t serial_device, char *string, uint8_t free_string){
    for(uint8_t i = 0; i < strlen(string); i++){
        uart_write_byte(serial_device, string[i]);
    }
    if(free_string){
        free(string);
    }
}

uint8_t uart_read_byte(uint8_t serial_device){
    while (!uart_read_ready(serial_device));

    // This is possible because the register is split into two. With separate read, write
    return *serials[serial_device].data;
}


// void uart_send_byte(uint8_t c){
// 	while(uart_tx_busy == 0);
// 	uart_tx_busy = 0;
// 	UDR0 = c;
// }

// uint8_t uart_read(void){
// 	static uint16_t rx_read_pos = 0;
// 	uint8_t data = 0;
	
// 	data = rx_buffer[rx_read_pos];
// 	rx_read_pos++;
// 	rx_count--;
// 	if(rx_read_pos >= RX_BUFFER_SIZE){
// 		rx_read_pos = 0;
// 	}
// 	return data;
// }