#include "./nrf24l01.h"

volatile uint pin_cs = 0;
// volatile spi_inst_t instance = NULL;


// const byte address[6] = "00001";

void init_receiver(uint spi_port, uint spi_frequency, uint pin_mosi, uint pin_miso, uint pin_sck, uint pin_cs_temp){
    init_spi(spi0, spi_frequency);
    // instance = spi_port;

    // cs pin high
    gpio_init(pin_cs);
    gpio_set_function(pin_mosi, GPIO_OUT);
    gpio_set_dir(pin_cs, 0);
    pin_cs = pin_cs_temp;

    // mosi
    gpio_set_function(pin_mosi, GPIO_FUNC_SPI);

    // miso 
    gpio_set_function(pin_miso, GPIO_FUNC_SPI);
    
    // sck
    gpio_set_function(pin_sck, GPIO_FUNC_SPI);


    // spi_write_blocking()
    return;
}

void init_transmitter(uint spi_port, uint spi_frequency, uint pin_mosi, uint pin_miso, uint pin_sck, uint pin_cs){

    return;
}