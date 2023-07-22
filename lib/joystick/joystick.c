#include "./joystick.h"

// First i tried having potentiometer joysticks and they were garbage. 
// They seemed to be drifting due to being worn down. Super inconsistent.

// Then i bough betafpv zero joysticks with hall effect sensors. And these work great
// but the ground (black wire) on them is the output and the output (yellow wire) is the output
// I measured the ranges of voltage output from these and put the data next to the values of adc input

// Min      - 1469
// Min high - 1489
// Max low  - 2872
// Max      - 2936
volatile uint adc_throttle = 0;

// Min      - 1534
// High min - 1568      high Min to avg - 945
// Mid low  - 2428
// Avg      - 2479      (1534+((945 + 761)/2)) - 2479 = -92 The calculation made sense but in practice the i adjusted the value to -107
// MId high - 2548
// Low max  - 3163      low Max to avg - 761
// Max      - 3240
volatile uint adc_yaw = 0;

// Min      - 1278
// High min - 1305      high Min to avg - 762
// Mid low  - 2002
// Avg      - 2040      (1278+((829 + 762)/2)) - 2040 = 34 The calculation made sense but in practice the i adjusted the value to 67.0
// MId high - 2104
// Low max  - 2810      low Max to avg - 829
// Max      - 2869
volatile uint adc_pitch = 0;

// Min      - 1457
// High min - 1480      high Min to avg - 740
// Mid low  - 2170
// Avg      - 2197      (1457+((740 + 745)/2))-2197 = 3 The calculation made sense but in practice the i adjusted the value to 67.0
// MId high - 2227
// Low max  - 2874      low Max to avg - 745
// Max      - 2942
volatile uint adc_roll = 0;

volatile float deadzone = 5;


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

uint16_t joystick_get_throttle_raw(){
    return adc_throttle;
}

uint16_t joystick_get_yaw_raw(){
    return adc_yaw;
}

uint16_t joystick_get_pitch_raw(){
    return adc_pitch;
}

uint16_t joystick_get_roll_raw(){
    return adc_roll;
}


float joystick_get_throttle_percent(){
    // Calculate percent value
    float percent_value = (((float)adc_throttle-1489.0)*100.0)/(2872.0-1489.0);
    
    // Doesn't have a deadzone in center

    // Cap value to range
    if(percent_value < 0.0){
        percent_value = 0.0;
    }else if(percent_value > 100.0){
        percent_value = 100.0;
    }


    return percent_value;
}

float joystick_get_yaw_percent(){
    // Calculate percent value
    float percent_value = (((float)adc_yaw-1568.0-107.0)*100.0)/(3163.0-1568.0);
    
    // Calculate deadzone
    if(percent_value < 50.0 + deadzone && percent_value > 50.0 - deadzone){
        percent_value = 50.0;
    }else if(percent_value >= 50.0 + deadzone){
        percent_value = 50.0 + ((percent_value - (50.0 + deadzone)) * 100.0) / ((50.0 - deadzone)*2.0);
    }else{
        percent_value = (percent_value * 100.0) / ((50.0 - deadzone)*2.0);
    }

    // Cap value to range
    if(percent_value < 0.0){
        percent_value = 0.0;
    }else if(percent_value > 100.0){
        percent_value = 100.0;
    }

    return percent_value;
}

float joystick_get_pitch_percent(){
    // Calculate percent value
    float percent_value = (((float)adc_pitch-1305.0-4.0)*100.0)/(2810.0-1305.0);
    
    // Calculate deadzone
    if(percent_value < 50.0 + deadzone && percent_value > 50.0 - deadzone){
        percent_value = 50.0;
    }else if(percent_value >= 50.0 + deadzone){
        percent_value = 50.0 + ((percent_value - (50.0 + deadzone)) * 100.0) / ((50.0 - deadzone)*2.0);
    }else{
        percent_value = (percent_value * 100.0) / ((50.0 - deadzone)*2.0);
    }

    // Cap value to range
    if(percent_value < 0.0){
        percent_value = 0.0;
    }else if(percent_value > 100.0){
        percent_value = 100.0;
    }

    return percent_value;
}

float joystick_get_roll_percent(){
    // Calculate percent value
    float percent_value = (((float)adc_roll-1568.0+67.0)*100.0)/(2874.0-1480.0);
    
    // Calculate deadzone
    if(percent_value < 50.0 + deadzone && percent_value > 50.0 - deadzone){
        percent_value = 50.0;
    }else if(percent_value >= 50.0 + deadzone){
        percent_value = 50.0 + ((percent_value - (50.0 + deadzone)) * 100.0) / ((50.0 - deadzone)*2.0);
    }else{
        percent_value = (percent_value * 100.0) / ((50.0 - deadzone)*2.0);
    }

    // Cap value to range
    if(percent_value < 0.0){
        percent_value = 0.0;
    }else if(percent_value > 100.0){
        percent_value = 100.0;
    }

    return percent_value;
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