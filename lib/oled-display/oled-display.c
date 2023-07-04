#include "./oled-display.h"

#define OLED_I2C_ADDRESS 0x3C

#define COMMAND_REG 0x80
#define DATA_REG 0x40

#define NORMAL_DISPLAY_CMD 0xAF
#define PAGE_ADDRESSING_MODE 0x02

volatile i2c_inst_t *m_i2c_port = NULL;

void init_oled_display(i2c_inst_t *i2c_port, uint8_t sda_pin, uint8_t scl_pin){

    // Init on default i2c
    i2c_init(i2c_port, 100000); // Specify the desired I2C clock speed (e.g., 100 kHz)

    gpio_set_function(sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(scl_pin, GPIO_FUNC_I2C);
    gpio_pull_up(sda_pin);
    gpio_pull_up(scl_pin);

    // Enable the I2C interface
    i2c_set_slave_mode(i2c_port, false, 0x00);
    //i2c_set_master_mode(i2c_port, true);

    m_i2c_port = i2c_port;

    oled_turn_on();
    oled_set_mode_normal();
    oled_set_page_mode();
    oled_write_command(0x8d);
    oled_write_command(0x14);
    oled_full_clear();
    oled_set_cursor(0, 0);
    
    for(uint page = 0; page < 8; page++){
        oled_set_cursor(0, page);
        for(uint column = 0; column < 128; column++){
            oled_write_data(0b11001100);
        }
    }
    oled_set_cursor(0, 0);

}

void oled_write_command(uint8_t command){
    uint8_t buf[2];
    buf[0] = COMMAND_REG;
    buf[1] = command;

    i2c_write_blocking(m_i2c_port, OLED_I2C_ADDRESS, buf, 2, false);
}

void oled_write_data(uint8_t data){
    uint8_t buf[2];
    buf[0] = DATA_REG;
    buf[1] = data;
    i2c_write_blocking(m_i2c_port, OLED_I2C_ADDRESS, buf, 2, false);
}

void oled_turn_on(){
    oled_write_command(SET_DISPLAY_ON);
}

void oled_turn_off(){
    oled_write_command(SET_DISPLAY_OFF);
}

void oled_set_mode_normal(){
    // Inverted or normal mode
    oled_write_command(SET_DISPLAY_NORMAL);
}

void oled_set_mode_inverted(){
    // Inverted or normal mode
    oled_write_command(SET_DISPLAY_INVERSE);
}

void oled_set_page_mode(){
    oled_write_command(SET_ADDRESSING_MODE); // Set addressing mode
    oled_write_command(MODE_PAGE_ADDRESSING); // set page addressing mode
}

void oled_full_clear(){
    for(uint page = 0; page < 8; page++){
        oled_set_cursor(0, page);
        for(uint column = 0; column < 128; column++){
            oled_write_data(0x00);
        }
    }
    oled_set_cursor(0, 0);
}

void oled_set_cursor(uint8_t x, uint8_t y){
    oled_write_command(0x00 + (x & 0x0F));
    oled_write_command(0x10 + ((x >> 4) & 0x0F));
    oled_write_command(0xB0 + y);
}