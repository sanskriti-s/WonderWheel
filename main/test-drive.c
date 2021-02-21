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

void app_main(void){
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

    char buf[50];
    uint8_t* encoder_data=0;

    while(1){
        // Read data from the UART
        uart_read_bytes(UART_PORT_NUM, encoder_data, BUF_SIZE, 20 / portTICK_RATE_MS);
        printf("Encoder=%p\n", encoder_data);
        // Initialize data
        int len = snprintf(buf,50,"v1 %f\n", 0.1);
        // Write data to the UART
        uart_write_bytes(UART_PORT_NUM, buf, len);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}