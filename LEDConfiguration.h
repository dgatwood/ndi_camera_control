// LED duty cycle configuration.  Adjust as
// needed for your resistor configuration
// to compensate for differences in
// brightness from one LED to the next.
#define WHITE_DUTY_CYCLE 0.2     // Button 0 (Set)
#define RED_DUTY_CYCLE 0.3       // Button 1
#define YELLOW_DUTY_CYCLE 0.5    // Button 2
#define GREEN_DUTY_CYCLE 0.3     // Button 3
#define BLUE_DUTY_CYCLE 0.5      // Button 4
#define PURPLE_DUTY_CYCLE 1.0    // Button 5
#define RGB_RED_DUTY_CYCLE 0.5   // Tally Program
#define RGB_GREEN_DUTY_CYCLE 0.7 // Tally Preview
#define RGB_BLUE_DUTY_CYCLE 0.5  // Tally Malfunction

// LED pin configuration.  Adjust as needed for
// your wiring configuration.
#ifdef USE_ROCKPI_PINOUTS
    // Rock Pi 4
    #define LED_PIN_WHITE 29     // GPIO2_B2 GPIO 74
    #define LED_PIN_RED 31       // GPIO2_B1 GPIO 73
    #define LED_PIN_YELLOW 33    // GPIO2_B4 GPIO 76
    #define LED_PIN_GREEN 35     // GPIO4_A5 GPIO 133
    #define LED_PIN_BLUE 37      // GPIO4_D6 GPIO 158
    #define LED_PIN_PURPLE 32    // GPIO3_C0 GPIO 112

    #define LED_PIN_RGB_RED 36   // GPIO4_A4 GPIO 132
    #define LED_PIN_RGB_GREEN 38 // GPIO4_A6 GPIO 134
    #define LED_PIN_RGB_BLUE 40  // GPIO4_A7 GPIO 135
#else
#error RPI
    // Raspberry Pi 4
    #define LED_PIN_WHITE 5
    #define LED_PIN_RED 6
    #define LED_PIN_YELLOW 13 
    #define LED_PIN_GREEN 19 
    #define LED_PIN_BLUE 26
    #define LED_PIN_PURPLE 12

    #define LED_PIN_RGB_RED 16
    #define LED_PIN_RGB_GREEN 20
    #define LED_PIN_RGB_BLUE 21
#endif

