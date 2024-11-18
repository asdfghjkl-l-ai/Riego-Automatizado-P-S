#include <stdio.h>
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "string.h"
#include "esp_timer.h"   
#include "driver/adc.h"


#define TRIGGER_PIN GPIO_NUM_4
#define ECHO_PIN GPIO_NUM_5
#define LED_RED GPIO_NUM_15
#define LED_YELLOW GPIO_NUM_16
#define LED_GREEN GPIO_NUM_17
#define PUMP_RELAY GPIO_NUM_18

#define I2C_MASTER_SCL_IO GPIO_NUM_22
#define I2C_MASTER_SDA_IO GPIO_NUM_21
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000
#define LCD_ADDR 0x27 

#define WATER_LEVEL_SENSOR_PIN GPIO_NUM_34  //usamos un pin ADC


void init_gpio() {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << TRIGGER_PIN) | (1ULL << LED_RED) | (1ULL << LED_YELLOW) | (1ULL << LED_GREEN) | (1ULL << PUMP_RELAY),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    io_conf.pin_bit_mask = (1ULL << ECHO_PIN);
    io_conf.mode = GPIO_MODE_INPUT;
    gpio_config(&io_conf);

    io_conf.pin_bit_mask = (1ULL << WATER_LEVEL_SENSOR_PIN); // Configuración de entrada para el sensor de agua
    io_conf.mode = GPIO_MODE_INPUT;
    gpio_config(&io_conf);

    gpio_set_level(PUMP_RELAY, 0); // Apaga la bomba al inicio
}

int read_water_level() {
    return adc1_get_raw(ADC1_CHANNEL_6);  // Leer valor en el pin ADC (GPIO34 en este caso)
}

void i2c_master_init() {
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

void lcd_send_command(uint8_t cmd) {
    uint8_t data[2];
    data[0] = cmd;
    data[1] = 0x00; // Indica que es un comando
    i2c_master_write_to_device(I2C_MASTER_NUM, LCD_ADDR, data, sizeof(data), 1000 / portTICK_PERIOD_MS);
}

void lcd_send_data(uint8_t data) {
    uint8_t data_buf[2];
    data_buf[0] = data;
    data_buf[1] = 0x40; // Indica que es un dato
    i2c_master_write_to_device(I2C_MASTER_NUM, LCD_ADDR, data_buf, sizeof(data_buf), 1000 / portTICK_PERIOD_MS);
}

void lcd_init() {
    vTaskDelay(pdMS_TO_TICKS(50));
    lcd_send_command(0x38);  // Configura la interfaz de 8 bits, 2 líneas, 5x8 píxeles
    lcd_send_command(0x0C);  // Enciende la pantalla y el cursor
    lcd_send_command(0x06);  // Modo de incremento del cursor
    lcd_send_command(0x01);  // Limpia la pantalla
    vTaskDelay(pdMS_TO_TICKS(5));
}

void lcd_print(const char *str) {
    while (*str) {
        lcd_send_data(*str++);
    }

}


float measure_distance() {
    gpio_set_level(TRIGGER_PIN, 1);
    esp_rom_delay_us(10);  // Retraso de 10 microsegundos
    gpio_set_level(TRIGGER_PIN, 0);

    // Espera hasta que el pin ECHO este en alto
    while (gpio_get_level(ECHO_PIN) == 0) {}
    uint32_t start_time = esp_timer_get_time();  // Tiempo de inicio

    // Espera hasta que el pin ECHO esté en bajo
    while (gpio_get_level(ECHO_PIN) == 1) {}
    uint32_t end_time = esp_timer_get_time();  // Tiempo de fin

    // Calcula la distancia en centímetros
    float distance = (end_time - start_time) * 0.034 / 2;
    return distance;
}


void app_main() {
    init_gpio();
    i2c_master_init();
    lcd_init();

    while (1) {
        float distance = measure_distance();
        int water_level = read_water_level();  // Lee el nivel de agua
        char buffer[16];
        snprintf(buffer, sizeof(buffer), "Dist: %.1f cm", distance);

        lcd_send_command(0x01);  // Limpia la pantalla
        lcd_print(buffer);       // Muestra la distancia medida

        // PILUCHA Hay que establecer umbrales para el sensor de nivel de agua 
        // El valor de "water_level" variará entre 0 y 1023
        // Puedes calibrar estos valores según el comportamiento de tu sensor

        if (water_level < 200) {  // Si el nivel de agua es bajo (valor cercano a 0)
            gpio_set_level(LED_RED, 1);  // Enciende LED rojo
            gpio_set_level(LED_YELLOW, 0);  // Apaga LED amarillo
            gpio_set_level(LED_GREEN, 0);  // Apaga LED verde
            gpio_set_level(PUMP_RELAY, 0);  // Apaga la bomba
            lcd_send_command(0xC0);  // Mueve el cursor a la segunda línea
            lcd_print("Nivel de agua: BAJO");

        } else if (water_level >= 200 && water_level < 700) {  // Nivel medio (ajustar el valor según la calibración)
            gpio_set_level(LED_RED, 0);  // Apaga LED rojo
            gpio_set_level(LED_YELLOW, 1);  // Enciende LED amarillo
            gpio_set_level(LED_GREEN, 0);  // Apaga LED verde
            gpio_set_level(PUMP_RELAY, 1);  // Activa la bomba
            lcd_send_command(0xC0);
            lcd_print("Nivel de agua: CRITICO");

        } else {  // Nivel alto (cerca de 1023)
            gpio_set_level(LED_RED, 0);  // Apaga LED rojo
            gpio_set_level(LED_YELLOW, 0);  // Apaga LED amarillo
            gpio_set_level(LED_GREEN, 1);  // Enciende LED verde
            gpio_set_level(PUMP_RELAY, 1);  // Activa la bomba
            lcd_send_command(0xC0);  // Mueve el cursor a la segunda línea
            lcd_print("Nivel de agua: ALTO");
        }

        if (distance > 12) {  
            gpio_set_level(LED_RED, 1);  
            gpio_set_level(LED_YELLOW, 0);  
            gpio_set_level(LED_GREEN, 0);  
            gpio_set_level(PUMP_RELAY, 0);  
            lcd_send_command(0xC0);
            lcd_print("Nivel: ALARMANTE");

        } else if (distance >= 6 && distance <= 12) {  
            gpio_set_level(LED_RED, 0); 
            gpio_set_level(LED_YELLOW, 1);  
            gpio_set_level(LED_GREEN, 0);  
            gpio_set_level(PUMP_RELAY, 1); 
            lcd_send_command(0xC0);
            lcd_print("Nivel: CRITICO");

        } else {  // Menor a 6 cm
            gpio_set_level(LED_RED, 0); 
            gpio_set_level(LED_YELLOW, 0); 
            gpio_set_level(LED_GREEN, 1);  
            gpio_set_level(PUMP_RELAY, 1);  
            lcd_send_command(0xC0);
            lcd_print("Nivel: NORMAL");
        }

        vTaskDelay(pdMS_TO_TICKS(2000));  
   }
}

