#include "./rotary-encoder.h"

struct rotary_encoder rotary_encoders[5];
uint8_t amount_of_encoders_in_array = 0;
bool timer_initialized = false;

struct repeating_timer rotary_encoder_timer;

void rotary_encoder_read_timer_callback(){
    for(uint8_t i = 0; i< amount_of_encoders_in_array; i++){

        rotary_encoders[i].m_clk_state = gpio_get(rotary_encoders[i].m_clk_pin);
        rotary_encoders[i].m_dt_state = gpio_get(rotary_encoders[i].m_dt_pin);

        if(rotary_encoders[i].m_clk_state != rotary_encoders[i].m_clk_state_previous){

            // only read when clk is high
            if(rotary_encoders[i].m_clk_state == true){
                if(rotary_encoders[i].m_dt_state != rotary_encoders[i].m_clk_state){
                    // Counter-clockwise
                    rotary_encoders[i].m_counter--;
                    rotary_encoders[i].m_clockwise = true;
                    printf("Counter-clockwise %d %d\n", i, rotary_encoders[i].m_counter);
                }else{
                    // Clockwise
                    rotary_encoders[i].m_counter++;
                    rotary_encoders[i].m_clockwise = false;
                    printf("Clockwise %d %d\n", i, rotary_encoders[i].m_counter);
                }
            }
        }

        rotary_encoders[i].m_clk_state_previous = rotary_encoders[i].m_clk_state;
    }
}

uint8_t init_rotary_encoder(uint8_t clk_pin, uint8_t dt_pin){
    struct rotary_encoder new_rotary_encoder;

    // Init the pins 
    gpio_init(clk_pin);
    gpio_set_dir(clk_pin, GPIO_IN);

    gpio_init(dt_pin);
    gpio_set_dir(dt_pin, GPIO_IN);

    // Configure the struct
    new_rotary_encoder.m_clk_state = gpio_get(clk_pin);
    new_rotary_encoder.m_dt_state = gpio_get(dt_pin);
    new_rotary_encoder.m_counter = 0;
    new_rotary_encoder.m_clk_state_previous = false;
    new_rotary_encoder.m_clockwise = false;
    new_rotary_encoder.m_clk_pin = clk_pin;
    new_rotary_encoder.m_dt_pin = dt_pin;

    // Store the struct in the array
    rotary_encoders[amount_of_encoders_in_array] = new_rotary_encoder;
    amount_of_encoders_in_array++;

    if(!timer_initialized){
        timer_initialized = true;

        // setup timer to switch between them;
        if(!add_repeating_timer_ms(2, rotary_encoder_read_timer_callback, NULL, &rotary_encoder_timer)){
            printf("Failed to initialize timer for rotary encoders\n");
        }
    }

    // Return the index of the encoder in array
    return amount_of_encoders_in_array - 1;
}

bool rotary_encoder_get_is_clockwise(uint8_t encoder_index){
    return rotary_encoders[encoder_index].m_clockwise;
}

int32_t rotary_encoder_get_counter(uint8_t encoder_index){
    return rotary_encoders[encoder_index].m_counter;
}

void rotary_encoder_reset_counter(uint8_t encoder_index){
    rotary_encoders[encoder_index].m_counter = 0;
}