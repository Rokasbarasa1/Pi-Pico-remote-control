#include "./button.h"


unsigned long time = 0; 
const int delayTime = 200; // Delay for every push button may vary

#define MAX_GPIO_PINS 30 // set this to your maximum GPIO pin number
void (*button_callbacks[MAX_GPIO_PINS])() = {NULL}; // Initialize all pointers to NULL

void button_interrupt(uint gpio, uint32_t events) {
    // debounce the button.
    if ((to_ms_since_boot(get_absolute_time())-time)>delayTime) {
        time = to_ms_since_boot(get_absolute_time());
        if (button_callbacks[gpio] != NULL) {
            button_callbacks[gpio]();
        }
        printf("BUTTON GPIO %d\n", gpio);
    }
}

void init_button(void (*button_callback)(), uint button_pin){
    if(button_pin != 999){
        gpio_set_irq_enabled_with_callback(button_pin, GPIO_IRQ_EDGE_FALL, true, &button_interrupt);

        gpio_init(button_pin);
        gpio_set_dir(button_pin, GPIO_IN);
        gpio_put(button_pin, 1);
        gpio_pull_up(button_pin);

        time = to_ms_since_boot(get_absolute_time());

        // Store the button callback in an array and use the gpio pin as index
        button_callbacks[button_pin] = button_callback;
    }
}