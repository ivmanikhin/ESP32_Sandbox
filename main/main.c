#include <stdio.h>
#include "ssd1306.h"
#include "driver/i2c.h"
#include "esp_timer.h"

#define I2C_MASTER_SCL_IO 22        /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 21        /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_0    /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 200000   /*!< I2C master clock frequency */



#define USONIC_TRIGGER 5
#define USONIC_ECHO 18
#define SOUND_SPEED 0.0343
#define I2C_HOST  0
double cm;
int16_t start_time, stop_time, duration;


static ssd1306_handle_t ssd1306_dev = NULL;
char data_str_1[35] = {0};
char data_str_2[16] = {0};


void setup()
{
	esp_rom_gpio_pad_select_gpio(USONIC_TRIGGER);
	gpio_set_direction(USONIC_TRIGGER, GPIO_MODE_OUTPUT);
	esp_rom_gpio_pad_select_gpio(USONIC_ECHO);
	gpio_set_direction(USONIC_ECHO, GPIO_MODE_INPUT);
	gpio_install_isr_service(0);
}

void get_signal()
{
	esp_timer_init();
	start_time = esp_timer_get_time();
	do{
		stop_time = esp_timer_get_time();
	} while (gpio_get_level(USONIC_ECHO) == 1);
};

double mesure_distance()
{
	gpio_isr_handler_add(USONIC_ECHO, get_signal, NULL);
	gpio_set_intr_type(USONIC_ECHO, 1);
	gpio_set_level(USONIC_TRIGGER, 0);
	esp_rom_delay_us(2);
	gpio_set_level(USONIC_TRIGGER,  1);
	esp_rom_delay_us(10);
	gpio_set_level(USONIC_TRIGGER, 0);
	gpio_intr_enable(USONIC_ECHO);
	vTaskDelay(pdMS_TO_TICKS(50));
	gpio_intr_disable(USONIC_ECHO);
	duration = stop_time - start_time;
	return duration * SOUND_SPEED / 2;
}



void app_main(void)
{
	setup();
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = (gpio_num_t)I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = (gpio_num_t)I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    conf.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL;

    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);

    ssd1306_dev = ssd1306_create(I2C_MASTER_NUM, SSD1306_I2C_ADDRESS);
    ssd1306_refresh_gram(ssd1306_dev);
    sprintf(data_str_1, "Distance:");
//    int i;
    while(1)
    {
//    	cm = 0;
//    	for (i=0; i<10; i++)
//    	{
    		cm = mesure_distance();
//    		vTaskDelay(50 / portTICK_PERIOD_MS);
//    	}
//    	cm =  0.1 * cm;
        sprintf(data_str_2, "%0.02f cm", cm);
    	ssd1306_clear_screen(ssd1306_dev, 0x00);
		ssd1306_draw_string(ssd1306_dev, 8, 8, (const uint8_t *)data_str_1, 16, 1);
		ssd1306_draw_string(ssd1306_dev, 8, 28, (const uint8_t *)data_str_2, 16, 1);
		ssd1306_refresh_gram(ssd1306_dev);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
