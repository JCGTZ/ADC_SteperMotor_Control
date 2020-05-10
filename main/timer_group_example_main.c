#include <stdio.h>
#include "esp_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"
#include "driver/gpio.h"

#include "driver/adc.h"
#include "esp_adc_cal.h"

#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   64          //Multisampling

#define frecuency 10
xQueueHandle main_queue;
xQueueHandle adc_timer;
xQueueHandle adc_queue;


typedef struct {
	int ide;
    int step;
    int adc1;
    int adc2;
} Queue_data;


/********************************ADC*******************************************/
/******************************************************************************/


static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channel = ADC_CHANNEL_6;     //GPIO34 if ADC1, GPIO14 if ADC2
static const adc_atten_t atten = ADC_ATTEN_DB_0;
static const adc_unit_t unit = ADC_UNIT_1;

static void check_efuse(){
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

static void print_char_val_type(esp_adc_cal_value_t val_type){
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        printf("Characterized using Two Point Value\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        printf("Characterized using eFuse Vref\n");
    } else {
        printf("Characterized using Default Vref\n");
    }
}

static void ADC_evt_task(void *arg){
Queue_data ADC_data;
	while(1){
	xQueueReceive(adc_timer, &ADC_data.step, portMAX_DELAY); ////ADC-Timer Queue
	ADC_data.ide=20;
		if(4==ADC_data.step){
		    //Multisampling
		    for (int i = 0; i < NO_OF_SAMPLES; i++) {
		        if (unit == ADC_UNIT_1) {
		            ADC_data.adc1 += adc1_get_raw((adc1_channel_t)channel);
		        } else {
		            int raw;
		            adc2_get_raw((adc2_channel_t)channel, ADC_WIDTH_BIT_12, &raw);
		            ADC_data.adc1 += raw;
		        }
		    }
		    ADC_data.adc1 /= NO_OF_SAMPLES;
			xQueueSend(main_queue, &ADC_data, NULL);
			//printf("ADC read: %d \n",ADC_data.adc1);
		}	
	}
}



/******************************************************************************/
/******************************************************************************/

/*****************************Timer********************************************/
/******************************************************************************/

void IRAM_ATTR timer_isr(void *para){
static Queue_data Timer_data;
	
	Timer_data.ide=1;
	Timer_data.step++;
	
	if(5==Timer_data.step){
		Timer_data.step=1;
	}

    TIMERG0.int_clr_timers.t0 = 1;
    TIMERG0.hw_timer[TIMER_0].config.alarm_en = TIMER_ALARM_EN;

    xQueueSendFromISR(main_queue, &Timer_data, NULL);
    xQueueSendFromISR(adc_timer, &Timer_data.step, NULL);
}


static void tg0_timer_init(void){
    /* Select and initialize basic parameters of the timer */
    timer_config_t config;
    config.divider = 800; // CLK 100 KHz
    config.alarm_en = true;
    config.counter_en =false;
    config.intr_type = TIMER_INTR_LEVEL;
    config.counter_dir = TIMER_COUNT_UP;
    config.auto_reload =true;
    timer_init(TIMER_GROUP_0, TIMER_0, &config);

    timer_set_counter_value(TIMER_GROUP_0,TIMER_0, 0x00000000ULL);
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, (1.0/frecuency)/(1e-5*4.0));
    timer_enable_intr(TIMER_GROUP_0, TIMER_0);
    timer_isr_register(TIMER_GROUP_0, TIMER_0, &timer_isr,TIMER_0, ESP_INTR_FLAG_IRAM, NULL);
    timer_start(TIMER_GROUP_0,TIMER_0);
}


static void timer_evt_task(void *arg){
static Queue_data Task1_data;
static int Enable=1;
    while (1) {

		xQueueReceive(main_queue,&Task1_data, 10);	
		
		if(0!=Task1_data.adc1){
			if((20==Task1_data.ide)&& (1000<Task1_data.adc1)){
			Enable =1;}else{
			Enable =0;
			}
		}

			printf("Enable: %d \n",Enable);
			if(1==Task1_data.ide && 1==Enable ){
				switch(Task1_data.step){
				case 1:
					gpio_set_level(GPIO_NUM_5, 0); //signal 1
					gpio_set_level(GPIO_NUM_18,1); //signal 2
					gpio_set_level(GPIO_NUM_19,0); //signal 3
					gpio_set_level(GPIO_NUM_21,1); //signal 4
					break;	

				case 2:
					gpio_set_level(GPIO_NUM_5, 0); //signal 1
					gpio_set_level(GPIO_NUM_18,1); //signal 2
					gpio_set_level(GPIO_NUM_19,1); //signal 3
					gpio_set_level(GPIO_NUM_21,0); //signal 4
					break;

				case 3:
					gpio_set_level(GPIO_NUM_5, 1); //signal 1
					gpio_set_level(GPIO_NUM_18,0); //signal 2
					gpio_set_level(GPIO_NUM_19,1); //signal 3
					gpio_set_level(GPIO_NUM_21,0); //signal 4
					break;

				case 4:
					gpio_set_level(GPIO_NUM_5, 1); //signal 1
					gpio_set_level(GPIO_NUM_18,0); //signal 2
					gpio_set_level(GPIO_NUM_19,0); //signal 3
					gpio_set_level(GPIO_NUM_21,1); //signal 4
					break;
				}
			}
		
    }
}
/******************************************************************************/
/******************************************************************************/
void app_main(){
		gpio_set_direction(GPIO_NUM_5, GPIO_MODE_OUTPUT);
		gpio_set_direction(GPIO_NUM_18, GPIO_MODE_OUTPUT);
		gpio_set_direction(GPIO_NUM_19, GPIO_MODE_OUTPUT);
		gpio_set_direction(GPIO_NUM_21, GPIO_MODE_OUTPUT);
    tg0_timer_init();
    main_queue = xQueueCreate(5, sizeof(Queue_data));
    xTaskCreate(timer_evt_task, "timer_evt_task", 2048, NULL, 1, NULL);

    //Check if Two Point or Vref are burned into eFuse
    check_efuse();
    //Configure ADC
    if (unit == ADC_UNIT_1) {
        adc1_config_width(ADC_WIDTH_BIT_12);
        adc1_config_channel_atten(channel, atten);
    } else {
        adc2_config_channel_atten((adc2_channel_t)channel, atten);
    }

    //Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);


    adc_timer = xQueueCreate(5, sizeof(int));
    xTaskCreate(ADC_evt_task, "ADC_evt_task", 2048, NULL, 1, NULL);
	
}



