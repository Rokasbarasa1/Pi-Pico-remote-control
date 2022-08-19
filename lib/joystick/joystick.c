#include "./joystick.h"

volatile uint adc_x = 999;
volatile uint adc_y = 999;
volatile uint current_adc = 0;
struct repeating_timer timer;
volatile uint16_t current_x = 0;
volatile uint16_t current_y = 0;

bool state;
const uint LED_PIN = 25;
unsigned long time = 0; 
const int delayTime = 200; // Delay for every push button may vary

static void (*button_callback)() = 0;

bool repeating_timer_callback(struct repeating_timer *t){
    adc_select_input(current_adc);
    if(current_adc == adc_x){
        current_x = adc_read();
        current_adc = adc_y;
    }else{
        current_y = adc_read();
        current_adc = adc_x;
    }
    return true;
}

void button_interrupt(uint gpio, uint32_t events) {
    // debounce the button.
    if ((to_ms_since_boot(get_absolute_time())-time)>delayTime) {
        time = to_ms_since_boot(get_absolute_time());
        
        // printf("GPIO %d\n", gpio);
        button_callback();
    }
}

bool init_joystick(uint x_axis_pin, uint y_axis_pin, uint button_pin, void (*button_callback_temp)()){

    // adc 1
    switch (x_axis_pin){
        case 28:
            adc_x = 2;
            break;
        case 27:
            adc_x = 1;
            break;
        case 26:
            adc_x = 0;
            break;
        default:
            return false;
    }
    // adc 2
    switch (y_axis_pin){
        case 28:
            adc_y = 2;
            break;
        case 27:
            adc_y = 1;
            break;
        case 26:
            adc_y = 0;
            break;
        default:
            return false;
    }

    // Check if it is the same
    if(adc_x == adc_y){
        return false;
    }

    adc_init();
    adc_gpio_init(x_axis_pin); 
    adc_gpio_init(y_axis_pin);

    // pull pin 28 high
    gpio_init(28);
    gpio_set_dir(28, GPIO_OUT);
    gpio_put(28, 1);

    current_adc = adc_x;

    if(button_pin != 999){
        gpio_set_irq_enabled_with_callback(button_pin, GPIO_IRQ_EDGE_FALL, true, &button_interrupt);

        gpio_init(button_pin);
        gpio_set_dir(button_pin, GPIO_IN);
        gpio_put(button_pin, 1);
        gpio_pull_up(button_pin);

        time = to_ms_since_boot(get_absolute_time());
        button_callback = button_callback_temp;
    }

    // setup timer to switch between them;
    add_repeating_timer_ms(20, repeating_timer_callback, NULL, &timer);
    return true;
}


uint16_t get_x(){
    return current_x;
}

uint16_t get_y(){
    return current_y;
}

float get_x_percentage(){
    return ((float)current_x*100)/4095;
}

float get_y_percentage(){
    return ((float)current_y*100)/4095;
}