#include "./nrf24l01.h"

#define SPIBUS_READ     (0b00000000)  /*!< addr | SPIBUS_READ  */
#define SPIBUS_WRITE    (0b00100000)  /*!< addr & SPIBUS_WRITE */
#define CONFIG          0x00
#define SETUP_AW        0x03
#define RF_CH           0x05

static volatile uint csn_pin = 0;
static volatile uint ce_pin = 0;
static volatile spi_inst_t *spi = NULL;

static void cs_deselect(){
    asm volatile("nop \n nop \n nop");
    gpio_put(csn_pin, 1);
    asm volatile("nop \n nop \n nop");
}

static void cs_select(){
    asm volatile("nop \n nop \n nop");
    gpio_put(csn_pin, 0);
    asm volatile("nop \n nop \n nop");
}

static void ce_enable(){
    asm volatile("nop \n nop \n nop");
    gpio_put(ce_pin, 1);
    asm volatile("nop \n nop \n nop");
}

static void ce_disable(){
    asm volatile("nop \n nop \n nop");
    gpio_put(ce_pin, 0);
    asm volatile("nop \n nop \n nop");
}

static void write_register(uint8_t reg, uint8_t data) {

    reg |= SPIBUS_WRITE;

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
    // reg |= SPIBUS_READ;

    cs_select();

    spi_write_blocking(spi_default, &reg, 1);
    sleep_ms(10);
    spi_read_blocking(spi_default, 0, buf, len);
    
    cs_deselect();
    sleep_ms(10);

}

bool nrf24_test(){
    
    // write_register(SETUP_AW, 0b00000000);
    // write_register(SETUP_AW, 0b00000000); // For some reason i need to write twice to this
        
    // uint8_t data[1] = {0};
    // read_registers(0x00, data, 1);
    // printf("%d\n", data[0]);

    // if(data[0] != 0){
    //     return false;
    // }

    // write_register(SETUP_AW, 0b00000011); 

    // uint8_t data1[1] = {0};
    // read_registers(SETUP_AW, data1, 1);
    // printf("%d\n", data1[0]);

    // if(data1[0] != 3){
    //     return false;
    // }

    write_register(CONFIG, 0b00000010);// Turn it on i guess
    write_register(RF_CH, 0b00000000);  // Clear this register
        
    uint8_t data[1] = {0};
    read_registers(RF_CH, data, 1);
    printf("%d\n", data[0]);

    if(data[0] != 0){ // check that it is empty
        return false;
    }

    write_register(RF_CH, 0b01010101); // write to it 

    uint8_t data1[1] = {0};
    read_registers(RF_CH, data1, 1);
    printf("%d\n", data1[0]);

    if(data1[0] != 0b01010101){ // check that it is the same value as writen
        return false;
    }


    return true;
}

bool nrf24_init(spi_inst_t *spi_temp, uint pin_csn_temp, uint pin_ce_temp, bool init_spi){
    csn_pin = pin_csn_temp;
    ce_pin = pin_ce_temp;
    spi = spi_temp;

    if(init_spi){
        spi_init(spi, 500000); // 0.5 Mhz

        // POLARITY AND PHASE HAVE TO BE BOTH 0 IMPORTANT
        spi_set_format(spi, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST); // 8 bits , cpol 0, cpha 0, MSB first

        // set pin functons to default spi
        gpio_set_function(PICO_DEFAULT_SPI_RX_PIN, GPIO_FUNC_SPI);
        gpio_set_function(PICO_DEFAULT_SPI_SCK_PIN, GPIO_FUNC_SPI);
        gpio_set_function(PICO_DEFAULT_SPI_TX_PIN, GPIO_FUNC_SPI);
    }

    // init cs pin and set it to high to make the device not be selected 
    gpio_init(csn_pin);
    gpio_set_dir(csn_pin, GPIO_OUT); // Outputs
    cs_deselect();

    gpio_init(ce_pin);
    gpio_set_dir(ce_pin, GPIO_OUT); // Outputs
    ce_disable();

    return nrf24_test();
}