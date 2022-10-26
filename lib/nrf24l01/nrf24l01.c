#include "./nrf24l01.h"

#define SPIBUS_READ     (0b00000000)  /*!< addr | SPIBUS_READ  */
#define SPIBUS_WRITE    (0b00100000)  /*!< addr & SPIBUS_WRITE */

static volatile uint csn_pin = 0;
static volatile uint ce_pin = 0;
static volatile spi_inst_t *spi = NULL;

#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)  \
  (byte & 0x80 ? '1' : '0'), \
  (byte & 0x40 ? '1' : '0'), \
  (byte & 0x20 ? '1' : '0'), \
  (byte & 0x10 ? '1' : '0'), \
  (byte & 0x08 ? '1' : '0'), \
  (byte & 0x04 ? '1' : '0'), \
  (byte & 0x02 ? '1' : '0'), \
  (byte & 0x01 ? '1' : '0') 

// Set spi to use nrf24 settings
void set_spi_settings(){
    // POLARITY AND PHASE HAVE TO BE BOTH 0 IMPORTANT
    spi_set_format(spi, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST); // 8 bits , cpol 0, cpha 0, MSB first
    spi_set_baudrate(spi, 500000); // 0.5 Mhz
}

// Slave deselect equivalent
static void cs_deselect(){
    gpio_put(csn_pin, 1);
}

// Slave select equivalent
static void cs_select(){
    gpio_put(csn_pin, 0);
}

// Ce pin is mainly used to change settings. If it is enabled you cant change them(cant write to the registers using spi)
static void ce_enable(){
    gpio_put(ce_pin, 1);
}

static void ce_disable(){
    gpio_put(ce_pin, 0);
}

// Spi write one byte to specific register
static void write_register(uint8_t reg, uint8_t data) {
    set_spi_settings(); // Make sure the correct settings are used. Maybe something

    // The nrf24 uses a specific bit that is added to the register address to 
    reg |= SPIBUS_WRITE;

    uint8_t buf[2];
    buf[0] = reg & 0x7f;  // remove read bit as this is a write
    buf[1] = data;
    cs_select(); // Select device
    spi_write_blocking(spi_default, buf, 2);
    cs_deselect();
}

// Read one register and return it from function
static uint8_t read_register(uint8_t reg) {
    set_spi_settings();

    uint8_t buf[1] = { 0 };

    cs_select();
    spi_write_blocking(spi_default, &reg, 1);
    spi_read_blocking(spi_default, 0, buf, 1);
    cs_deselect();

    return buf[0];
}

// write multiple bytes to register
static void write_register_multiple(uint8_t reg, uint8_t * data, uint size, bool command) {
    set_spi_settings();

    uint8_t buf[1];
    if(!command){
        reg |= SPIBUS_WRITE;
        reg = (reg & 0x7f);
    }
    buf[0] = reg;

    cs_select();
    spi_write_blocking(spi_default, buf, 1);
    spi_write_blocking(spi_default, data, size);
    cs_deselect();
}

// send a specific command. Basically write and ignore response
static void send_command(uint8_t command){
    set_spi_settings();

    uint8_t buf[1] = { command };

    cs_select();
    spi_write_blocking(spi_default, buf, 1);
    cs_deselect();
}

// read multiple registers from specified registers
static void read_register_multiple(uint8_t reg, uint8_t *buf, uint16_t len) {
    set_spi_settings();

    cs_select();
    spi_write_blocking(spi_default, &reg, 1);
    spi_read_blocking(spi_default, 0, buf, len);
    cs_deselect();
}

// Test to see if spi setup is working for nrf24
bool nrf24_test(){
    write_register(CONFIG, 0b00000010);// Turn it on i guess
    write_register(RF_CH, 0b00000000);  // Clear this register
        
    uint8_t data[1] = {0};
    read_register_multiple(RF_CH, data, 1);
    // printf("%d\n", data[0]);

    if(data[0] != 0b00000000){ // check that it is empty
        return false;
    }

    write_register(RF_CH, 0b01010101); // write to it 

    uint8_t data1[1] = {0};
    read_register_multiple(RF_CH, data1, 1);
    // printf("%d\n", data1[0]);

    if(data1[0] != 0b01010101){ // check that it is the same value as writen
        return false;
    }

    return true;
}


// reset some registers on the nrf24
void nrf24_reset(uint8_t REG)
{
	if (REG == STATUS){
		write_register(STATUS, 0x00);
	}

	else if (REG == FIFO_STATUS){
		write_register(FIFO_STATUS, 0x11);
	}else {
        write_register(CONFIG, 0x08);
        write_register(EN_AA, 0x3F);
        write_register(EN_RXADDR, 0x03);
        write_register(SETUP_AW, 0x03);
        write_register(SETUP_RETR, 0x03);
        write_register(RF_CH, 0x02);
        write_register(RF_SETUP, 0x0E);
        write_register(STATUS, 0x00);
        write_register(OBSERVE_TX, 0x00);
        write_register(CD, 0x00);
        uint8_t rx_addr_p0_def[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
        write_register_multiple(RX_ADDR_P0, rx_addr_p0_def, 5, false);
        uint8_t rx_addr_p1_def[5] = {0xC2, 0xC2, 0xC2, 0xC2, 0xC2};
        write_register_multiple(RX_ADDR_P1, rx_addr_p1_def, 5, false);
        write_register(RX_ADDR_P2, 0xC3);
        write_register(RX_ADDR_P3, 0xC4);
        write_register(RX_ADDR_P4, 0xC5);
        write_register(RX_ADDR_P5, 0xC6);
        uint8_t tx_addr_def[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
        write_register_multiple(TX_ADDR, tx_addr_def, 5, false);
        write_register(RX_PW_P0, 0);
        write_register(RX_PW_P1, 0);
        write_register(RX_PW_P2, 0);
        write_register(RX_PW_P3, 0);
        write_register(RX_PW_P4, 0);
        write_register(RX_PW_P5, 0);
        write_register(FIFO_STATUS, 0x11);
        write_register(DYNPD, 0);
        write_register(FEATURE, 0);
	}
}

// Set nrf24 to transmit mode
void nrf24_tx_mode(uint8_t *address, uint8_t channel){
	// disable the chip before configuring the device
	ce_disable();

	write_register(RF_CH, channel);  // select the channel

	write_register_multiple(TX_ADDR, address, 5, false);  // Write the TX address


	// power up the device
	uint8_t config = read_register(CONFIG);
    //	config = config | (1<<1);   // write 1 in the PWR_UP bit
	config = config & (0xF2);    // write 0 in the PRIM_RX, and 1 in the PWR_UP, and all other bits are masked
	write_register(CONFIG, config);

	// Enable the chip after configuring the device
	ce_enable();
}

// set nrf24 to receive mode
void nrf24_rx_mode(uint8_t *address, uint8_t channel){
	// disable the chip before configuring the device
	ce_disable();

	nrf24_reset(STATUS);

	write_register(RF_CH, channel);  // select the channel

    // setup pipe 1
    uint8_t en_rxaddr = read_register(EN_RXADDR);
	en_rxaddr = en_rxaddr | 0b00000010;
	write_register(EN_RXADDR, en_rxaddr); // set to use pipe 1
    write_register_multiple(RX_ADDR_P1, address, 5, false); // write address to pipe 1 address register
    write_register(RX_PW_P1, 32); // receive 32 bytes
    

    // setup pipe 2
    // en_rxaddr = read_register(EN_RXADDR);
	// en_rxaddr = en_rxaddr | (1<<2);
	// write_register(EN_RXADDR, en_rxaddr);
    // write_register(RX_ADDR_P2, 0xEE);
    // write_register(RX_PW_P2, 32);




    // dont need pipe 2
	// // select data pipe 2
	// uint8_t en_rxaddr = read_register(EN_RXADDR);
	// en_rxaddr = en_rxaddr | (1<<2);
	// write_register(EN_RXADDR, en_rxaddr);

	// /* We must write the address for Data Pipe 1, if we want to use any pipe from 2 to 5
	//  * The Address from DATA Pipe 2 to Data Pipe 5 differs only in the LSB
	//  * Their 4 MSB Bytes will still be same as Data Pipe 1
	//  *
	//  * For Eg->
	//  * Pipe 1 ADDR = 0xAABBCCDD11
	//  * Pipe 2 ADDR = 0xAABBCCDD22
	//  * Pipe 3 ADDR = 0xAABBCCDD33
	//  *
	//  */
	// write_register_multiple(RX_ADDR_P1, address, 5);  // Write the Pipe1 address
	// write_register(RX_ADDR_P2, 0xEE);  // Write the Pipe2 LSB address

	// write_register(RX_PW_P2, 32);   // 32 bit payload size for pipe 2


	// power up the device in Rx mode
	uint8_t config = read_register(CONFIG);
	config = config | (1<<1) | (1<<0);
	write_register(CONFIG, config);


    // uint8_t data[5] = {0,0,0,0,0};
    // read_register_multiple(RX_ADDR_P1, data, 5);

    // printf("EN_RXADDR "BYTE_TO_BINARY_PATTERN"\n", BYTE_TO_BINARY(data[0]));
    // printf("EN_RXADDR "BYTE_TO_BINARY_PATTERN"\n", BYTE_TO_BINARY(data[1]));
    // printf("EN_RXADDR "BYTE_TO_BINARY_PATTERN"\n", BYTE_TO_BINARY(data[2]));
    // printf("EN_RXADDR "BYTE_TO_BINARY_PATTERN"\n", BYTE_TO_BINARY(data[3]));
    // printf("EN_RXADDR "BYTE_TO_BINARY_PATTERN"\n", BYTE_TO_BINARY(data[4]));

	ce_enable();
}

// perform the transmission with specified data
bool nrf24_transmit(uint8_t *data){
    write_register_multiple(W_TX_PAYLOAD, data, 32, true);
	sleep_ms(1);

	uint8_t fifo_status = read_register(FIFO_STATUS);

	// check the fourth bit of FIFO_STATUS to know if the TX fifo is empty
	if ((fifo_status&(1<<4)) && (!(fifo_status&(1<<3)))){
		send_command(FLUSH_TX);
		nrf24_reset(FIFO_STATUS);// reset FIFO_STATUS
		return true;
	}

	return false;
}

// Check if data is available on the specified pipe
bool nrf24_data_available(int pipe_number){
	uint8_t status = read_register(STATUS);
    printf("STATUS "BYTE_TO_BINARY_PATTERN"\n", BYTE_TO_BINARY(status));

	if ((status&(1<<6))&&(status&(pipe_number<<1))){
		write_register(STATUS, (1<<6));
		return true;
	}

	return false;
}

// receive the data form the nrf24 into the specified array
void nrf24_receive(uint8_t *data){
	// payload command
    read_register_multiple(R_RX_PAYLOAD, data, 32);
    sleep_ms(1);

	send_command(FLUSH_RX);
}

// read all the registers on the nrf24
void nrf24_read_all(uint8_t *data){
	for (int i=0; i<10; i++){
		*(data+i) = read_register(i);
	}

	read_register_multiple(RX_ADDR_P0, (data+10), 5);
	read_register_multiple(RX_ADDR_P1, (data+15), 5);

	*(data+20) = read_register(RX_ADDR_P2);
	*(data+21) = read_register(RX_ADDR_P3);
	*(data+22) = read_register(RX_ADDR_P4);
	*(data+23) = read_register(RX_ADDR_P5);

	read_register_multiple(RX_ADDR_P0, (data+24), 5);

	for (int i=29; i<38; i++){
		*(data+i) = read_register(i-12);
	}
}

bool nrf24_init(spi_inst_t *spi_temp, uint pin_csn_temp, uint pin_ce_temp, bool init_spi){
    csn_pin = pin_csn_temp;
    ce_pin = pin_ce_temp;
    spi = spi_temp;

    if(init_spi){
        spi_init(spi, 500000); // 0.5 Mhz

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
    ce_disable(); // disable to start changing registers on nrf24

    if(!nrf24_test()){ // test if spi works
        return false;
    }


    nrf24_reset(0);
    write_register(CONFIG, 0b00000010);// Will come back
    printf("CONFIG "BYTE_TO_BINARY_PATTERN"\n", BYTE_TO_BINARY(read_register(CONFIG)));
    write_register(EN_AA, 0b00000000); // No auto ACK
    printf("EN_AA "BYTE_TO_BINARY_PATTERN"\n", BYTE_TO_BINARY(read_register(EN_AA)));
    write_register(EN_RXADDR, 0b00000000); // Will come back
    printf("EN_RXADDR "BYTE_TO_BINARY_PATTERN"\n", BYTE_TO_BINARY(read_register(EN_RXADDR)));
    write_register(SETUP_AW, 0b00000011); // 5 bytes rx/tx address field
    printf("SETUP_AW "BYTE_TO_BINARY_PATTERN"\n", BYTE_TO_BINARY(read_register(SETUP_AW)));
    write_register(SETUP_RETR, 0b00000000); // no ACK being used
    printf("SETUP_RETR "BYTE_TO_BINARY_PATTERN"\n", BYTE_TO_BINARY(read_register(SETUP_RETR)));
    write_register(RF_CH, 0b00000000);   // will come back
    printf("RF_CH "BYTE_TO_BINARY_PATTERN"\n", BYTE_TO_BINARY(read_register(RF_CH)));
    write_register(RF_SETUP, 0b00001110); // 0db power and data rate 2Mbps
    printf("RF_SETUP "BYTE_TO_BINARY_PATTERN"\n", BYTE_TO_BINARY(read_register(RF_SETUP)));

    ce_enable();
    return true;
}

//Examples

// If both receive and transmit are on the same address and pipe 1 then this should work 

// Receiver

// uint8_t address[5] = {0xEE, 0xDD, 0xCC, 0xBB, 0xAA};
// uint8_t rx_data[32];
// bool nrf24_setup = nrf24_init(spi_default, 7, 8, true);
// nrf24_rx_mode(address, 10);

// printf("Transmitting: ");
// if(nrf24_transmit(tx_data)){
//     printf("TX success\n");
// }else{
//     printf("TX failed\n");
// }


// Transmitter

// uint8_t address[5] = {0xEE, 0xDD, 0xCC, 0xBB, 0xAA};
// uint8_t tx_data[] = "hello world!\n";
// bool nrf24_setup = nrf24_init(spi_default, 7, 8, true);
// nrf24_tx_mode(address, 10);

// printf("Receiving: ");
// if(nrf24_data_available(1)){
//     nrf24_receive(rx_data);
//     for(uint8_t i = 0; i < strlen((char*) rx_data); i++ ){
//         printf("%c", ((char*) rx_data)[i]);
//     }
//     printf("\n");

// }else{
//     printf("no data\n");
// }