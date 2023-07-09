#include "./joystick.h"

volatile uint adc_throttle = 0;
volatile uint adc_yaw = 0;
volatile uint adc_pitch = 0;
volatile uint adc_roll = 0;

// Keep track of which joystick/joystick axis is being read
volatile uint axis_index = 0;
struct repeating_timer joystick_timer;

/**
 * @brief Effectively a loop that keeps switching between each of the joystick inputs and reading the data from them
 * 
 */
bool joystick_repeating_timer_callback(struct repeating_timer *t){
    switch (axis_index)
    {
        case 0:
            adc_select_input(0);
            adc_throttle = adc_read();
            // Change which one to use after reading
            // so the changes have propagated by the next read
            gpio_put(3, 0);
            gpio_put(6, 1);
            break;
        case 1:
            adc_select_input(0);
            adc_yaw = adc_read();
            gpio_put(3, 0);
            gpio_put(6, 0);
            break;
        case 2:
            adc_select_input(1);
            adc_pitch = adc_read();
            break;
        case 3:
            adc_select_input(2);
            adc_roll = adc_read();
            break;
        default:
            break;
    }

    // Go to next joystick/axis of joystick
    axis_index++;
    axis_index = axis_index % 4;
    return true;
}

/**
 * @brief Initialize the pins, adc's and timers used by joystick
 * 
 */
void init_joystick(){
    // TODO: make joystick stuff more abstract and less hard coded to be 4 values of throttle, yaw and so on
    adc_init();
    adc_gpio_init(26); 
    adc_gpio_init(27);
    adc_gpio_init(28);

    // adc multiplexer initialization
    gpio_init(3);
    gpio_set_dir(3, GPIO_OUT);
    gpio_put(3, 0);

    gpio_init(6);
    gpio_set_dir(6, GPIO_OUT);
    gpio_put(6, 0);

    // setup timer to switch between them;
    if(!add_repeating_timer_ms(2, joystick_repeating_timer_callback, NULL, &joystick_timer)){
        printf("Failed to initialize timer for joystick\n");
    }
}

uint16_t joystick_get_throttle(){
    return adc_throttle;
}

uint16_t joystick_get_yaw(){
    return adc_yaw;
}

uint16_t joystick_get_pitch(){
    return adc_pitch;
}

uint16_t joystick_get_roll(){
    return adc_roll;
}

float joystick_get_throttle_percent(){
    return ((float)adc_throttle*100.0)/4095.0;
}

float joystick_get_yaw_percent(){
    return ((float)adc_yaw*100.0)/4095.0;
}

float joystick_get_pitch_percent(){
    return ((float)adc_pitch*100.0)/4095.0;
}

float joystick_get_roll_percent(){
    return ((float)adc_roll*100.0)/4095.0;
}

float joystick_get_throttle_volts(){
    return ((float)adc_throttle/4095.0) * 3.3;
}

float joystick_get_yaw_volts(){
    return ((float)adc_yaw/4095.0) * 3.3;
}

float joystick_get_pitch_volts(){
    return ((float)adc_pitch/4095.0) * 3.3;
}

float joystick_get_roll_volts(){
    return ((float)adc_roll/4095.0) * 3.3;
}