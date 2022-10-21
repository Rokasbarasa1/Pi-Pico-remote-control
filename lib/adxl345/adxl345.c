#include "./adxl345.h"

// adxl345 has these bits that need to be there to write stuff to or read registers
#define SPIBUS_READ     (0x80)  /*!< addr | SPIBUS_READ  */
#define SPIBUS_WRITE    (0x7F)  /*!< addr & SPIBUS_WRITE */

volatile uint cs_pin = 0;
volatile spi_inst_t *spi = NULL;

static void cs_deselect(){
    asm volatile("nop \n nop \n nop");
    gpio_put(cs_pin, 1);
    asm volatile("nop \n nop \n nop");
}

static void cs_select(){
    asm volatile("nop \n nop \n nop");
    gpio_put(cs_pin, 0);
    asm volatile("nop \n nop \n nop");
}

static void write_register(uint8_t reg, uint8_t data) {

    reg &= SPIBUS_WRITE;

    uint8_t buf[2];
    buf[0] = reg & 0x7f;  // remove read bit as this is a write
    buf[1] = data;
    cs_select();
    sleep_ms(10);

    spi_write_blocking(spi_default, buf, 2);
    sleep_ms(10);

    cs_deselect();
}

static void read_registers(uint8_t reg, uint8_t *buf, uint16_t len) {
    // For this particular device, we send the device the register we want to read
    // first, then subsequently read from the device. The register is auto incrementing
    // so we don't need to keep sending the register we want, just the first.
    reg |= SPIBUS_READ;

    cs_select();

    spi_write_blocking(spi_default, &reg, 1);
    sleep_ms(10);
    spi_read_blocking(spi_default, 0, buf, len);
    
    cs_deselect();
    sleep_ms(10);

}

bool adxl345_test(){
    
    write_register(0x1E, 0b00000000);
    write_register(0x1E, 0b00000000); // For some reason i need to write twice to this

        
    uint8_t data[1] = {0};
    read_registers(0x00, data, 1);
    printf("%d\n", data[0]);

    if(data[0] != 229){ // device id should be 229
        return false;
    }

    uint8_t data2[1] = {0};
    read_registers(0x1E, data2, 1);
    printf("%d\n", data2[0]);

    if(data2[0] != 0){ // device id should be 229
        return false;
    }
    // uint8_t write2[1] = {0b10101010};
    write_register(0x1E, 0b10101010);

    uint8_t data1[1] = {0};
    read_registers(0x1E, data1, 1);
    printf("%d\n\n", data1[0]);

    if(data1[0] != 170){ // device id should be 229
        return false;
    }

    return true;
}

// This library is not a work in progress as i wont use it on the pi pico 
bool adxl345_init(spi_inst_t *spi_temp, uint pin_cs_temp, bool init_spi){
    cs_pin = pin_cs_temp;
    spi = spi_temp;

    if(init_spi){
        spi_init(spi, 4500000); // 4.5 Mhz
        spi_set_format(spi, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST); // 8 bits , cpol 1, cpha 1, MSB first

        // set pin functons to default spi
        gpio_set_function(PICO_DEFAULT_SPI_RX_PIN, GPIO_FUNC_SPI);
        gpio_set_function(PICO_DEFAULT_SPI_SCK_PIN, GPIO_FUNC_SPI);
        gpio_set_function(PICO_DEFAULT_SPI_TX_PIN, GPIO_FUNC_SPI);
    }

    // init cs pin and set it to high to make the device not be selected 
    gpio_init(cs_pin);
    gpio_set_dir(cs_pin, GPIO_OUT); // Outputs
    cs_deselect();

    sleep_ms(10);
    
    // while(true){
    //     write_register(0x1E, 0b00000000);
        
    //     uint8_t data[1] = {0};
    //     read_registers(0x00, data, 1);
    //     printf("%d\n", data[0]);
    

    //     uint8_t data2[1] = {0};
    //     read_registers(0x1E, data2, 1);
    //     printf("%d\n", data2[0]);

    //     // uint8_t write2[1] = {0b10101010};
    //     write_register(0x1E, 0b10101010);

    //     uint8_t data1[1] = {0};
    //     read_registers(0x1E, data1, 1);
    //     printf("%d\n\n", data1[0]);

    //     sleep_ms(500);
    // }

    return adxl345_test();
}