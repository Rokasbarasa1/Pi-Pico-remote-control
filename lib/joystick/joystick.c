#include "./joystick.h"

// First i tried having potentiometer joysticks and they were garbage. 
// They seemed to be drifting due to being worn down. Super inconsistent.

// Then i bough betafpv zero joysticks with hall effect sensors. And these work great
// but the ground (black wire) on them is the output and the output (yellow wire) is the output
// I measured the ranges of voltage output from these and put the data next to the values of adc input

// OLD
// Min      - 1469
// Min high - 1489
// Max low  - 2872
// Max      - 2936

// NEW
float adc_throttle_min = 1469.0f;
float adc_throttle_high_min = 1489.0f;
float adc_throttle_max_low = 2872.0f;
float adc_throttle_max = 2936.0f;

float adc_throttle_mid_adjustment = 0.0f;

volatile uint adc_throttle = 0;
volatile uint adc_throttle_array[MAX_AVERAGING_SAMPLE_SIZE];

// OLD
// Min      - 1534
// High min - 1568      high Min to avg - 945
// Mid low  - 2428
// Avg      - 2479      (1534+((945 + 761)/2)) - 2479 = -92 The calculation made sense but in practice the i adjusted the value to -107
// MId high - 2548
// Low max  - 3163      low Max to avg - 761
// Max      - 3240

// NEW
float adc_yaw_min = 1534.0f;
float adc_yaw_high_min = 1568.0f;
float adc_yaw_mid_low = 2428.0f;
float adc_yaw_avg = 2479.0f;
float adc_yaw_mid_high = 2548.0f;
float adc_yaw_low_max = 3163.0f;
float adc_yaw_max = 3240.0f;

float adc_yaw_mid_adjustment = -107.0f;

volatile uint adc_yaw = 0;
volatile uint adc_yaw_array[MAX_AVERAGING_SAMPLE_SIZE];

// OLD
// Min      - 1278
// High min - 1305      high Min to avg - 762
// Mid low  - 2002
// Avg      - 2040      (1278+((829 + 762)/2)) - 2040 = 34 The calculation made sense but in practice the i adjusted the value to 67.0
// MId high - 2104
// Low max  - 2810      low Max to avg - 829
// Max      - 2869

// NEW
float adc_pitch_min = 1457.0f;
float adc_pitch_high_min = 1480.0f;
float adc_pitch_mid_low = 2170.0f;
float adc_pitch_avg = 2197.0f;
float adc_pitch_mid_high = 2227.0f;
float adc_pitch_low_max = 2874.0f;
float adc_pitch_max = 2942.0f;

float adc_pitch_mid_adjustment = 67.0f;

volatile uint adc_pitch = 0;
volatile uint adc_pitch_array[MAX_AVERAGING_SAMPLE_SIZE];

// OLD
// Min      - 1457
// High min - 1480      high Min to avg - 740
// Mid low  - 2170
// Avg      - 2197      (1457+((740 + 745)/2))-2197 = 3 The calculation made sense but in practice the i adjusted the value to 67.0
// MId high - 2227
// Low max  - 2874      low Max to avg - 745
// Max      - 2942

// NEW
float adc_roll_min = 1278.0f;
float adc_roll_high_min = 1305.0f;
float adc_roll_mid_low = 2002.0f;
float adc_roll_avg = 2040.0f;
float adc_roll_mid_high = 2104.0f;
float adc_roll_low_max = 2810.0f;
float adc_roll_max = 2869.0f;

float adc_roll_mid_adjustment = -4.0f;

volatile uint adc_roll = 0;
volatile uint adc_roll_array[MAX_AVERAGING_SAMPLE_SIZE];

volatile float deadzone = 0;

volatile uint16_t averaging_sample_size = 20;
volatile uint16_t averaging_sample_array_index = 0;


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
            for(uint8_t i = 0; i < averaging_sample_size; i++){
                adc_throttle = adc_read();
                adc_throttle_array[i] = adc_throttle;
            }
            // Change which one to use after reading
            // so the changes have propagated by the next read
            gpio_put(3, 0);
            gpio_put(6, 1);
            break;
        case 1:
            adc_select_input(0);
            for(uint8_t i = 0; i < averaging_sample_size; i++){
                adc_yaw = adc_read();
                adc_yaw_array[i] = adc_yaw;
            }

            gpio_put(3, 0);
            gpio_put(6, 0);
            break;
        case 2:
            adc_select_input(1);
            for(uint8_t i = 0; i < averaging_sample_size; i++){
                adc_roll = adc_read();
                adc_roll_array[i] = adc_roll;
            }

            break;
        case 3:
            adc_select_input(2);
            for(uint8_t i = 0; i < averaging_sample_size; i++){
                adc_pitch = adc_read();
                adc_pitch_array[i] = adc_pitch;
            }

            break;
        default:
            break;
    }

    // Go to next joystick/axis of joystick
    axis_index++;


    // if(axis_index == 4){
    //     // Increment the averaging index
    //     averaging_sample_array_index++;
    //     averaging_sample_array_index = averaging_sample_array_index % averaging_sample_size;
    // }
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

    // setup timer to switch between them. 0.5 ms delay
    if(!add_repeating_timer_us(500, joystick_repeating_timer_callback, NULL, &joystick_timer)){
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

// enum t_calibration_step {
//     MAX_DEFLECT,
//     MIN_DEFLECT,
//     CENTER_DEFLECT,
// };

// void calibrate_joystick_pair(uint8_t mode){

//     // The sticks are already deflected
    
//     uint16_t samples = 1000;
//     // Get 50 samples of each
//     float samples_joystick_A[samples];
//     float samples_joystick_B[samples];

//     float average_throttle_sum = 0.0;
//     float average_yaw_sum = 0.0;

//     for(uint8_t i = 0; i < samples; i++){
//         average_throttle_sum = average_throttle_sum + (float) adc_throttle_array[i];
//         average_yaw_sum = average_yaw_sum + (float) adc_yaw_array[i];
//     }

//     float average_throttle = average_throttle_sum / (float) samples;
//     float average_yaw = average_yaw_sum / (float) samples;

//     switch (expression)
//     {
//     case constant expression:
//         /* code */
//         break;
    
//     default:
//         break;
//     }
//     if(mode == 0)

// }


float joystick_get_throttle_percent(){
    // Calculate the average adc value
    float average_throttle_sum = 0.0;
    for(uint8_t i = 0; i < averaging_sample_size; i++){
        average_throttle_sum = average_throttle_sum + (float) adc_throttle_array[i];
    }
    float average_throttle = average_throttle_sum / (float) averaging_sample_size;

    // Calculate percent value
    float percent_value = (((float)average_throttle-adc_throttle_high_min+adc_throttle_mid_adjustment)*100.0)/(adc_throttle_max_low-adc_throttle_high_min);
    
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
    // Calculate the average adc value
    float average_yaw_sum = 0.0;
    for(uint8_t i = 0; i < averaging_sample_size; i++){
        average_yaw_sum = average_yaw_sum + (float) adc_yaw_array[i];
    }
    float average_yaw = average_yaw_sum / (float) averaging_sample_size;

    // Calculate average value
    float percent_value = (((float)average_yaw-adc_yaw_high_min+adc_yaw_mid_adjustment)*100.0)/(adc_yaw_low_max-adc_yaw_high_min);
    
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
    // Calculate the average adc value
    float average_pitch_sum = 0.0;
    for(uint8_t i = 0; i < averaging_sample_size; i++){
        average_pitch_sum = average_pitch_sum + (float) adc_pitch_array[i];
    }
    float average_pitch = average_pitch_sum / (float) averaging_sample_size;

    // Calculate percent value
    float percent_value = (((float)average_pitch-adc_pitch_high_min+adc_pitch_mid_adjustment)*100.0)/(adc_pitch_low_max-adc_pitch_high_min);
    
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
    // Calculate the average adc value
    float average_roll_sum = 0.0;
    for(uint8_t i = 0; i < averaging_sample_size; i++){
        average_roll_sum = average_roll_sum + (float) adc_roll_array[i];
    }
    float average_roll = average_roll_sum / (float) averaging_sample_size;

    // Calculate percent value
    float percent_value = (((float)average_roll-adc_roll_high_min+adc_roll_mid_adjustment)*100.0)/(adc_roll_low_max-adc_roll_high_min);
    
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

void joystick_set_averaging_sample_size(uint8_t sample_size){
    if(sample_size == 0 || sample_size > MAX_AVERAGING_SAMPLE_SIZE){
        return;
        printf("Sample size value is not allowed: %d\n", sample_size);
    }
    
    averaging_sample_size = sample_size;
    averaging_sample_array_index = 0;
}

void joystick_set_deadzone(float deadzone_value){
    if(deadzone_value < 0.0 || deadzone_value > 50.0){
        return;
        printf("Deadzone value is not allowed: %f\n", deadzone_value);
    }

    deadzone = deadzone_value;
}