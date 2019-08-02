
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/ledc.h>
#include <driver/i2c.h>
#include <driver/gpio.h>
#include <esp_log.h>
#include <sdkconfig.h>


static const char *PROJECT_NAME = "xbox-leds";
#define DATA_LENGTH (size_t) 2

#define I2C_SLAVE_SCL_IO GPIO_NUM_25                     /*!< gpio number for i2c slave clock */
#define I2C_SLAVE_SDA_IO GPIO_NUM_26                     /*!< gpio number for i2c slave data */
#define I2C_SLAVE_NUM I2C_NUM_0                 /*!< I2C port number for slave dev */
#define I2C_SLAVE_TX_BUF_LEN (1024 * DATA_LENGTH)  /*!< I2C slave tx buffer size */
#define I2C_SLAVE_RX_BUF_LEN (1024 * DATA_LENGTH)  /*!< I2C slave rx buffer size */

#define LED_SET_COLOUR(ch, value) ledc_set_duty(ledc_channel[ch].speed_mode, ledc_channel[ch].channel, value * 5000/255);\
                    ledc_update_duty(ledc_channel[ch].speed_mode, ledc_channel[ch].channel);
#define LED_FADE(ch, to) ledc_set_fade_with_time(ledc_channel[ch].speed_mode, ledc_channel[ch].channel, to * 5000/255, 1500);
#define LED_FADE_START__WAIT_TILL_END() ledc_fade_start(ledc_channel[LED_RED].speed_mode, ledc_channel[LED_RED].channel, LEDC_FADE_NO_WAIT);\
                    ledc_fade_start(ledc_channel[LED_GREEN].speed_mode, ledc_channel[LED_GREEN].channel, LEDC_FADE_NO_WAIT);\
                    ledc_fade_start(ledc_channel[LED_BLUE].speed_mode, ledc_channel[LED_BLUE].channel, LEDC_FADE_WAIT_DONE);
#define LED_FADE_START() ledc_fade_start(ledc_channel[LED_RED].speed_mode, ledc_channel[LED_RED].channel, LEDC_FADE_NO_WAIT);\
                    ledc_fade_start(ledc_channel[LED_GREEN].speed_mode, ledc_channel[LED_GREEN].channel, LEDC_FADE_NO_WAIT);\
                    ledc_fade_start(ledc_channel[LED_BLUE].speed_mode, ledc_channel[LED_BLUE].channel, LEDC_FADE_NO_WAIT);

#define LED_RED 0
#define LED_GREEN 1
#define LED_BLUE 2

#define ESP_SLAVE_ADDR 0x66 >> 1


typedef enum {
    STATE_PASSTHROUGH = 0x00,
    STATE_SOLID = 0x01,
    STATE_CYCLE = 0x02,
    STATE_PULSE = 0x03,
    STATE_OFF = 0x04,
    STATE_INITIAL
} STATE_MACHINE;

typedef enum {
    CMD_STATE = 0x00,
    CMD_COLOR0_RED = 0x01,
    CMD_COLOR0_GREEN = 0x02,
    CMD_COLOR0_BLUE = 0x03
} I2C_COMMANDS;


typedef struct {
	uint8_t reg; //number of the register to be written to
    uint8_t data;
} I2CPacket;


typedef struct {
	uint8_t red; //number of the register to be written to
    uint8_t green;
    uint8_t blue;
} Color;


i2c_port_t i2c_slave_port = I2C_SLAVE_NUM;
SemaphoreHandle_t print_mux = NULL;
STATE_MACHINE state_current = STATE_SOLID;
STATE_MACHINE state_previous = STATE_INITIAL;
Color color0;


static void i2c_handle_packets(void *arg) {
    I2CPacket packet;
    
    packet.reg = 0;
    packet.data = 0;

    i2c_reset_rx_fifo(i2c_slave_port);

    while(true) {
        if(0 < i2c_slave_read_buffer(i2c_slave_port, &packet.reg, 1, 1000 / portTICK_RATE_MS)) {
            if(0 < i2c_slave_read_buffer(i2c_slave_port, &packet.data, 1, 1000 / portTICK_RATE_MS)) {
                switch (packet.reg)
                {
                case CMD_STATE:
                    state_current = (STATE_MACHINE) packet.data;
                    break;
                case CMD_COLOR0_RED:
                    color0.red = packet.data;
                    state_previous = STATE_INITIAL;
                    break;
                case CMD_COLOR0_GREEN:
                    color0.green = packet.data;
                    state_previous = STATE_INITIAL;
                    break;
                case CMD_COLOR0_BLUE:
                    color0.blue = packet.data;
                    state_previous = STATE_INITIAL;
                    break;
                default:
                    xSemaphoreTake(print_mux, portMAX_DELAY);
                    ESP_LOGW(PROJECT_NAME, "I2C[r: %x d: %x] unknown command", packet.reg, packet.data);
                    xSemaphoreGive(print_mux);
                    break;
                }
            } else {
                xSemaphoreTake(print_mux, portMAX_DELAY);
                ESP_LOGE(PROJECT_NAME, "I2C only got 1 byte -> resetting buffer");
                xSemaphoreGive(print_mux);
                //Reset, as i2c broke itself!
                i2c_reset_rx_fifo(i2c_slave_port);
                packet.reg = 0;
                packet.data = 0;
                vTaskDelay(10 / portTICK_RATE_MS); // wait 10ms
            }
        }
    }
}

static esp_err_t i2c_slave_init()
{
    i2c_config_t conf_slave;
    conf_slave.sda_io_num = I2C_SLAVE_SDA_IO;
    conf_slave.sda_pullup_en = GPIO_PULLUP_DISABLE;
    conf_slave.scl_io_num = I2C_SLAVE_SCL_IO;
    conf_slave.scl_pullup_en = GPIO_PULLUP_DISABLE;
    conf_slave.mode = I2C_MODE_SLAVE;
    conf_slave.slave.addr_10bit_en = 0;
    conf_slave.slave.slave_addr = ESP_SLAVE_ADDR;
    i2c_param_config(i2c_slave_port, &conf_slave);
    return i2c_driver_install(i2c_slave_port, conf_slave.mode,
                              I2C_SLAVE_RX_BUF_LEN,
                              I2C_SLAVE_TX_BUF_LEN, 0);
}

static void task_led_output(void *arg) {
    ledc_timer_config_t ledc_timer;
    bool changed = true;
    uint8_t ch;

    ledc_timer.duty_resolution = LEDC_TIMER_13_BIT; // resolution of PWM duty
    ledc_timer.freq_hz = 5000;                      // frequency of PWM signal
    ledc_timer.speed_mode = LEDC_HIGH_SPEED_MODE;           // timer mode
    ledc_timer.timer_num = LEDC_TIMER_0;            // timer index
    
    // Set configuration of timer0 for high speed channels
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel[3];

    ledc_channel[LED_RED].channel    = LEDC_CHANNEL_0;
    ledc_channel[LED_RED].duty       = 0;
    ledc_channel[LED_RED].gpio_num   = 18;
    ledc_channel[LED_RED].speed_mode = LEDC_HIGH_SPEED_MODE;
    ledc_channel[LED_RED].hpoint     = 0;
    ledc_channel[LED_RED].timer_sel  = LEDC_TIMER_0;

    ledc_channel[LED_GREEN].channel    = LEDC_CHANNEL_1;
    ledc_channel[LED_GREEN].duty       = 0;
    ledc_channel[LED_GREEN].gpio_num   = 19;
    ledc_channel[LED_GREEN].speed_mode = LEDC_HIGH_SPEED_MODE;
    ledc_channel[LED_GREEN].hpoint     = 0;
    ledc_channel[LED_GREEN].timer_sel  = LEDC_TIMER_0;
    
    ledc_channel[LED_BLUE].channel    = LEDC_CHANNEL_2;
    ledc_channel[LED_BLUE].duty       = 0;
    ledc_channel[LED_BLUE].gpio_num   = 5;
    ledc_channel[LED_BLUE].speed_mode = LEDC_HIGH_SPEED_MODE;
    ledc_channel[LED_BLUE].hpoint     = 0;
    ledc_channel[LED_BLUE].timer_sel  = LEDC_TIMER_0;
    
    // Set LED Controller with previously prepared configuration
    for (ch = 0; ch < 3; ch++) {
        ledc_channel_config(&ledc_channel[ch]);
    }

    // Initialize fade service.
    ledc_fade_func_install(0);

    while(true) {
        // TODO: Check if error occured and go into passthrough
        switch(state_current) {
        case STATE_PASSTHROUGH:
            // implement readout and output
            if(changed) {
                xSemaphoreTake(print_mux, portMAX_DELAY);
                ESP_LOGI(PROJECT_NAME, "State: Passthrough");
                xSemaphoreGive(print_mux);
            }
            break;
        case STATE_SOLID:
            if(changed) {
                LED_SET_COLOUR(LED_RED, color0.red);
                LED_SET_COLOUR(LED_GREEN, color0.green);
                LED_SET_COLOUR(LED_BLUE, color0.blue);

                xSemaphoreTake(print_mux, portMAX_DELAY);
                ESP_LOGI(PROJECT_NAME, "State: Solid");
                xSemaphoreGive(print_mux);
            }
            break;
        case STATE_PULSE:
            if(changed) {
                xSemaphoreTake(print_mux, portMAX_DELAY);
                ESP_LOGI(PROJECT_NAME, "State: Pulse");
                xSemaphoreGive(print_mux);
            }

            LED_FADE(LED_RED, 0);
            LED_FADE(LED_GREEN, 0);
            LED_FADE(LED_BLUE, 0);
            LED_FADE_START();
            vTaskDelay(2500 / portTICK_PERIOD_MS);
            if(state_previous != state_current) break;

            LED_FADE(LED_RED, color0.red);
            LED_FADE(LED_GREEN, color0.green);
            LED_FADE(LED_BLUE, color0.blue);
            LED_FADE_START();
            vTaskDelay(2500 / portTICK_PERIOD_MS);
            break;
        case STATE_CYCLE:
            if(changed) {
                xSemaphoreTake(print_mux, portMAX_DELAY);
                ESP_LOGI(PROJECT_NAME, "State: Cycle");
                xSemaphoreGive(print_mux);
            }

            LED_FADE(LED_RED, 255);
            LED_FADE(LED_GREEN, 0);
            LED_FADE(LED_BLUE, 0);
            LED_FADE_START();
            vTaskDelay(2500 / portTICK_PERIOD_MS);
            if(state_previous != state_current) break;

            LED_FADE(LED_RED, 255);
            LED_FADE(LED_GREEN, 255);
            LED_FADE(LED_BLUE, 0);
            LED_FADE_START();
            vTaskDelay(2500 / portTICK_PERIOD_MS);
            if(state_previous != state_current) break;

            LED_FADE(LED_RED, 0);
            LED_FADE(LED_GREEN, 255);
            LED_FADE(LED_BLUE, 0);
            LED_FADE_START();
            vTaskDelay(2500 / portTICK_PERIOD_MS);
            if(state_previous != state_current) break;

            LED_FADE(LED_RED, 0);
            LED_FADE(LED_GREEN, 255);
            LED_FADE(LED_BLUE, 255);
            LED_FADE_START();
            vTaskDelay(2500 / portTICK_PERIOD_MS);
            if(state_previous != state_current) break;

            LED_FADE(LED_RED, 0);
            LED_FADE(LED_GREEN, 0);
            LED_FADE(LED_BLUE, 255);
            LED_FADE_START();
            vTaskDelay(2500 / portTICK_PERIOD_MS);
            if(state_previous != state_current) break;

            LED_FADE(LED_RED, 255);
            LED_FADE(LED_GREEN, 0);
            LED_FADE(LED_BLUE, 255);
            LED_FADE_START();
            vTaskDelay(2500 / portTICK_PERIOD_MS);
            break;
        case STATE_OFF:
        default:
            if(changed) {
                LED_SET_COLOUR(LED_RED, 0);
                LED_SET_COLOUR(LED_GREEN, 0);
                LED_SET_COLOUR(LED_BLUE, 0);

                xSemaphoreTake(print_mux, portMAX_DELAY);
                ESP_LOGI(PROJECT_NAME, "State: Off");
                xSemaphoreGive(print_mux);
            }
            break;
        }
        vTaskDelay(5);
        changed = state_previous != state_current;
        state_previous = state_current;
    }
}

extern "C" {
    void app_main(void) {
        print_mux = xSemaphoreCreateMutex();
        color0.red = 200;
        color0.green = 200;
        color0.blue = 200;

        ESP_ERROR_CHECK(i2c_slave_init());

        xTaskCreate(i2c_handle_packets, "i2c_handle_packets_0", 1024 * 2, (void *)0, 10, NULL);
        xTaskCreate(task_led_output, "task_led_output_0", 1024 * 2, (void *)0, 10, NULL);

	    xSemaphoreTake(print_mux, portMAX_DELAY);
        ESP_LOGI(PROJECT_NAME, "I2C Slave started with addr %x", ESP_SLAVE_ADDR);
	    xSemaphoreGive(print_mux);
    }

}