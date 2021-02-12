/* UART Echo Example modified by Sanskriti Sharma
*/

#include <stdio.h>
#include "motorcontroller.h"
#include <stdlib.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "sdkconfig.h"
#include "esp_adc_cal.h"

#define TXD GPIO_NUM_10
#define RXD GPIO_NUM_9
#define RTS GPIO_NUM_11
#define CTS GPIO_NUM_6


#define UART_PORT_NUM      UART_NUM_1
#define UART_BAUD_RATE     115200
#define TASK_STACK_SIZE    2048

#define BUF_SIZE (1024)

#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   64          //Multisampling

static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channel1 = ADC_CHANNEL_6;     //GPIO34 if ADC1
static const adc_channel_t channel2 = ADC_CHANNEL_7;     //GPIO35 if ADC1

static const adc_atten_t atten = ADC_ATTEN_DB_11;       //Allows ESP to read up to 3.9V
static const adc_unit_t unit = ADC_UNIT_1;

#if CONFIG_IDF_TARGET_ESP32
static void check_efuse(void)
{
    //Check TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("eFuse Two Point: NOT supported\n");
    }

    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        printf("eFuse Vref: Supported\n");
    } else {
        printf("eFuse Vref: NOT supported\n");
    }
}

static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        printf("Characterized using Two Point Value\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        printf("Characterized using eFuse Vref\n");
    } else {
        printf("Characterized using Default Vref\n");
    }
}
#endif

void app_main(void)
{
#if CONFIG_UART_ISR_IN_IRAM
    intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif

#if CONFIG_IDF_TARGET_ESP32
    //Check if Two Point or Vref are burned into eFuse
    check_efuse();
#endif

    //Configure ADC
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(channel1, atten);
    adc1_config_channel_atten(channel2, atten);


#if CONFIG_IDF_TARGET_ESP32
    //Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);
#endif

    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    int intr_alloc_flags = 0;

// Error checking
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, TXD, RXD, RTS, CTS));

float current_theta = 0;
float last_theta = 0;

//Continuously sample ADC1
    while (1) {
        uint32_t adc_reading1 = 0;
        uint32_t adc_reading2 = 0;
        //Multisampling
        for (int i = 0; i < NO_OF_SAMPLES; i++) {
            adc_reading1 += adc1_get_raw((adc1_channel_t)channel1);
        }
        adc_reading1 /= NO_OF_SAMPLES;
        //Multisampling
        for (int i = 0; i < NO_OF_SAMPLES; i++) {
            adc_reading2 += adc1_get_raw((adc1_channel_t)channel2);
        }
        adc_reading2 /= NO_OF_SAMPLES;
        float x = (float) adc_reading1;
        float y = (float) adc_reading2;
        float r = sqrt(x*x + y*y);
        float theta = atan(y/x);
        float r_conv = 0.5/r;
        float theta_conv = 1.0/theta;
        
        last_theta = current_theta;
        current_theta = theta_conv;
        float movement = current_theta-last_theta;

        // Declare buffers
        char buf0[50];
        char buf1[50];

        // Read data from the UART
        //int len = uart_read_bytes(UART_PORT_NUM, encoder_data, BUF_SIZE, 20 / portTICK_RATE_MS);
        // Initialize data
        int len0 = snprintf(buf0,50,"p0 %f\n", movement);
        int len1 = snprintf(buf1,50,"v1 %f\n", r_conv);
        // Write data to the UART
        uart_write_bytes(UART_PORT_NUM, buf0, len0);
        uart_write_bytes(UART_PORT_NUM, buf1, len1);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}