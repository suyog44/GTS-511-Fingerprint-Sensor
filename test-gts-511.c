#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include "ff.h"

// I2C Pins
#define I2C_SCL_PIN 6
#define I2C_SDA_PIN 7
#define SENSOR_ADDR 0x42 // Assuming the I2C address of the CMOS sensor

// GPIO pin definitions for control and data
#define VSYNC_PIN    17  // Frame synchronization
#define HSYNC_PIN    16  // Line synchronization
#define PCLK_PIN     19  // Pixel clock
#define RSTB_PIN     21  // Reset
#define PWDN_PIN     22  // Power-down
#define MCLK_PIN     18  // Master clock (PWM)

#define D0_PIN       8   // Data bus bit 0
#define D1_PIN       9   // Data bus bit 1
#define D2_PIN       10  // Data bus bit 2
#define D3_PIN       11  // Data bus bit 3
#define D4_PIN       12  // Data bus bit 4
#define D5_PIN       13  // Data bus bit 5
#define D6_PIN       14  // Data bus bit 6
#define D7_PIN       15  // Data bus bit 7

// SD card SPI pin definitions
#define SD_CS_PIN    17  // Chip select
#define SD_CLK_PIN   18  // Clock
#define SD_MISO_PIN  16  // MISO
#define SD_MOSI_PIN  15  // MOSI

// Initialize the I2C interface
void i2c_init_sensor() {
    i2c_init(i2c0, 100 * 1000); // 100kHz clock for I2C
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SCL_PIN);
    gpio_pull_up(I2C_SDA_PIN);
}

// Function to write data to the I2C register
void i2c_write(uint8_t reg, uint8_t value) {
    uint8_t data[2] = {reg, value};
    i2c_write_blocking(i2c0, SENSOR_ADDR, data, 2, false);  // Write register and data to the sensor
}

// Function to initialize the sensor registers
void sensor_init() {
    // Reset sensor
    gpio_put(PWDN_PIN, 1); // PWDN High
    gpio_put(RSTB_PIN, 0); // RESTB Low
    sleep_ms(5);
    gpio_put(PWDN_PIN, 0); // PWDN Low
    sleep_ms(1);
    gpio_put(RSTB_PIN, 1); // RESTB High

    // Initialize sensor registers
    i2c_write(0xFC, 0x16); // Soft reset
    i2c_write(0xFE, 0x80); // Select page 0x80
    i2c_write(0xFE, 0x00); // Select page 0x00

    // Add additional initialization steps for the sensor as required
    i2c_write(0xF1, 0x01);
    i2c_write(0xF0, 0x07);
    i2c_write(0xFA, 0x11);
    i2c_write(0x24, 0x3F);
    i2c_write(0x46, 0x02);
    i2c_write(0x09, 0x00);
    i2c_write(0x0A, 0x04);
    i2c_write(0x0B, 0x00);
    i2c_write(0x0C, 0x00);
    i2c_write(0x0D, 0x01);
    i2c_write(0x0E, 0xE8);
    i2c_write(0x0F, 0x02);
    i2c_write(0x10, 0x88);
    i2c_write(0x11, 0x2A);
    i2c_write(0x44, 0xA2);
    i2c_write(0x50, 0x01);
    i2c_write(0x51, 0x00);
    i2c_write(0x52, 0x00);
    i2c_write(0x53, 0x00);
    i2c_write(0x54, 0x00);
    i2c_write(0x55, 0x00);
    i2c_write(0x56, 0xF0);
    i2c_write(0x57, 0x01);
    i2c_write(0x58, 0x40);
    i2c_write(0x59, 0x22);
    i2c_write(0x5A, 0x03);
    i2c_write(0x5B, 0x00);
    i2c_write(0x5C, 0x00);
    i2c_write(0x5D, 0x00);
    i2c_write(0x5E, 0x00);
    i2c_write(0x5F, 0x00);
    i2c_write(0x60, 0x00);
    i2c_write(0x61, 0x00);
    i2c_write(0x62, 0x00);
    i2c_write(0x03, 0x01); // Example exposure time (update with correct values)
    i2c_write(0x04, 0x02); // Example exposure time (update with correct values)
}

// Initialize GPIO pins for sensor control
void gpio_init_sensor() {
    for (int i = D0_PIN; i <= D7_PIN; i++) {
        gpio_init(i);
        gpio_set_dir(i, GPIO_IN);
    }

    gpio_init(VSYNC_PIN);
    gpio_set_dir(VSYNC_PIN, GPIO_IN);

    gpio_init(HSYNC_PIN);
    gpio_set_dir(HSYNC_PIN, GPIO_IN);

    gpio_init(PCLK_PIN);
    gpio_set_dir(PCLK_PIN, GPIO_IN);

    gpio_init(RSTB_PIN);
    gpio_set_dir(RSTB_PIN, GPIO_OUT);

    gpio_init(PWDN_PIN);
    gpio_set_dir(PWDN_PIN, GPIO_OUT);

    gpio_init(MCLK_PIN);
    gpio_set_dir(MCLK_PIN, GPIO_OUT);
}

// Function to configure MCLK using PWM
void pwm_config_mclk() {
    gpio_set_function(MCLK_PIN, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(MCLK_PIN);
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 1.0f); // Divider for maximum frequency
    pwm_config_set_wrap(&config, 100);   // Set wrap value for 24 MHz
    pwm_init(slice_num, &config, true);
}

// Function to capture image
void get_image(uint8_t* ptr_img) {
    while (!gpio_get(VSYNC_PIN)); // Wait for VSYNC high (frame start)
    while (gpio_get(VSYNC_PIN));  // Wait for VSYNC low

    for (int j = 0; j < 240; j++) {
        while (!gpio_get(HSYNC_PIN)); // Wait for HSYNC high (line start)
        for (int i = 0; i < 320; i++) {
            while (!gpio_get(PCLK_PIN)); // Wait for PCLK high
            uint8_t pixel = 0;
            for (int bit = 0; bit < 8; bit++) {
                if (gpio_get(D0_PIN + bit)) {
                    pixel |= (1 << bit);
                }
            }
            *ptr_img++ = pixel;
            while (gpio_get(PCLK_PIN)); // Wait for PCLK low
        }
        while (gpio_get(HSYNC_PIN)); // Wait for HSYNC low
    }
}

// Initialize SD card SPI
int init_sd_card() {
    if (sdcard_init()) {
        printf("SD card initialization failed\n");
        return -1;
    }
    printf("SD card initialized successfully\n");

    // Mount file system
    FATFS fs;
    FRESULT res = f_mount(&fs, "", 1);
    if (res != FR_OK) {
        printf("Failed to mount file system\n");
        return -1;
    }

    return 0;
}

// Function to save image to SD card
void save_image_to_sd(uint8_t *image_data, size_t size) {
    FATFS fs;
    FIL file;
    FRESULT res;
    uint32_t bytes_written;

    // Open file for writing
    res = f_open(&file, "captured_image.raw", FA_WRITE | FA_CREATE_ALWAYS);
    if (res != FR_OK) {
        printf("Error opening file for writing\n");
        return;
    }

    // Write image data to file
    res = f_write(&file, image_data, size, &bytes_written);
    if (res != FR_OK) {
        printf("Error writing image data\n");
    }

    // Close the file
    f_close(&file);
    printf("Image saved to SD card as captured_image.raw\n");
}

int main() {
    stdio_init_all();
    gpio_init_sensor();
    i2c_init_sensor();
    pwm_config_mclk();
    sensor_init();

    // Initialize SD card
    if (init_sd_card() != 0) {
        return -1;
    }

    // Buffer to store image
    uint8_t image_buffer[320 * 240];
    get_image(image_buffer);

    // Save captured image to SD card
    save_image_to_sd(image_buffer, sizeof(image_buffer));

    while (1) {
        sleep_ms(1000);
    }

    return 0;
}
