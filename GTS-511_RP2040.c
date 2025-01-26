#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"

#define I2C_PORT i2c0
#define SCL_PIN 21
#define SDA_PIN 20
#define PWDN_PIN 2
#define RSTB_PIN 3
#define LED_CON_PIN 14
#define MCLK_PIN 13
#define VSYNC_PIN 12
#define HSYNC_PIN 13
#define PCLK_PIN 19
#define D0_PIN 15 // D0-D7 are contiguous GPIO pins
#define SENSOR_I2C_ADDR 0x42

#define IMAGE_WIDTH 320
#define IMAGE_HEIGHT 240

void init_gpio() {
    // Power Down and Reset Pins
    gpio_init(PWDN_PIN);
    gpio_init(RSTB_PIN);
    gpio_set_dir(PWDN_PIN, GPIO_OUT);
    gpio_set_dir(RSTB_PIN, GPIO_OUT);
    gpio_put(PWDN_PIN, 1); // Power Down initially
    gpio_put(RSTB_PIN, 1); // Reset initially high

    // LED Control Pin
    gpio_init(LED_CON_PIN);
    gpio_set_dir(LED_CON_PIN, GPIO_OUT);
    gpio_put(LED_CON_PIN, 0); // Turn off LED

    // Data Pins D0-D7
    for (int pin = D0_PIN; pin <= D0_PIN + 7; ++pin) {
        gpio_init(pin);
        gpio_set_dir(pin, GPIO_IN);
    }

    // VSYNC, HSYNC, and PCLK Pins
    gpio_init(VSYNC_PIN);
    gpio_init(HSYNC_PIN);
    gpio_init(PCLK_PIN);
    gpio_set_dir(VSYNC_PIN, GPIO_IN);
    gpio_set_dir(HSYNC_PIN, GPIO_IN);
    gpio_set_dir(PCLK_PIN, GPIO_IN);
}

void init_i2c() {
    i2c_init(I2C_PORT, 100000); // 100 kHz
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SCL_PIN);
    gpio_pull_up(SDA_PIN);
}

void init_mclk() {
    uint slice_num = pwm_gpio_to_slice_num(MCLK_PIN);
    gpio_set_function(MCLK_PIN, GPIO_FUNC_PWM);

    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&cfg, 1.0f);       // Adjust clock divider
    pwm_set_wrap(slice_num, 41);            // Adjust for 24 MHz
    pwm_init(slice_num, &cfg, true);        // Initialize and start PWM
}

void write_sensor_register(uint8_t reg, uint8_t value) {
    uint8_t data[] = {reg, value};
    i2c_write_blocking(I2C_PORT, SENSOR_I2C_ADDR, data, 2, false);
}

void sensor_init() {
    // Power on the sensor
    gpio_put(PWDN_PIN, 0); // Enable sensor
    sleep_ms(10);
    gpio_put(RSTB_PIN, 0); // Reset sensor
    sleep_ms(5);
    gpio_put(RSTB_PIN, 1); // Release reset
    sleep_ms(10);

    // I2C Initialization Sequence
    write_sensor_register(0xFC, 0x16); // Soft reset
    write_sensor_register(0xFE, 0x80);
    write_sensor_register(0xFE, 0x00);
    write_sensor_register(0xFC, 0x16);
    write_sensor_register(0xF1, 0x01);
    write_sensor_register(0xF0, 0x07);
    write_sensor_register(0x24, 0x3F);
    write_sensor_register(0x46, 0x02);

    // Additional initialization as per datasheet
    // Add further I2C register writes as required
}

void capture_image(uint8_t *image_buffer) {
    // Wait for frame synchronization
    while (!gpio_get(VSYNC_PIN)); // Wait for VSYNC high
    while (gpio_get(VSYNC_PIN));  // Wait for VSYNC low

    for (int row = 0; row < IMAGE_HEIGHT; ++row) {
        while (!gpio_get(HSYNC_PIN)); // Wait for HSYNC high

        for (int col = 0; col < IMAGE_WIDTH; ++col) {
            while (!gpio_get(PCLK_PIN)); // Wait for PCLK high
            uint8_t pixel = 0;
            for (int bit = 0; bit < 8; ++bit) {
                pixel |= (gpio_get(D0_PIN + bit) << bit);
            }
            image_buffer[row * IMAGE_WIDTH + col] = pixel;
            while (gpio_get(PCLK_PIN)); // Wait for PCLK low
        }

        while (gpio_get(HSYNC_PIN)); // Wait for HSYNC low
    }
}

int main() {
    stdio_init_all();
    init_gpio();
    init_i2c();
    init_mclk();
    sensor_init();

    uint8_t image_buffer[IMAGE_WIDTH * IMAGE_HEIGHT];

    while (true) {
        printf("Capturing image...\n");
        capture_image(image_buffer);
        printf("Image captured. Process it as needed.\n");

        sleep_ms(1000); // Delay between captures
    }

    return 0;
}
