//https://madhusudan.live/blog/diy-smoke-detector-system-mq135-and-arduino
//https://www.elprocus.com/mq135-air-quality-sensor/
//https://www.olimex.com/Products/Components/Sensors/Gas/SNS-MQ135/resources/SNS-MQ135.pdf
//https://files.waveshare.com/upload/2/24/MQ-135-Gas-Sensor-UserManual.pdf

#include "stm32f0xx.h"
#include <math.h>   // for M_PI

#include <stdint.h>

#include <stdio.h>

// Pin definitions and configurations
#define AIR_QUALITY_ADC_CHANNEL ADC_CHANNEL_1  // MQ-135 connected to ADC Channel 1
#define FAN_PWM_CHANNEL TIM_CHANNEL_1          // Fan controlled by PWM on Timer Channel 1

// Threshold for poor air quality
#define AIR_QUALITY_THRESHOLD 300              // Threshold value for air quality control

uint16_t readAirQuality(void) {
    
    static uint8_t preheat= 0;  // checks whether preheated for 20sec.

    // Check 
    if (!preheat) {
        HAL_Delay(20000);  // Wait for 20 seconds for the sensor to stabilize
        preheat = 1;  
    }
    ADC_ChannelConfTypeDef sConfig = {0}; // defines the ADC stucture (settings for ADC), sConfig holds ADC channel number and conversion rank
    sConfig.Channel = AIR_QUALITY_ADC_CHANNEL; // Channel = channel connected to MQ-135
    sConfig.Rank = 1; //Only 1 channel
    HAL_ADC_ConfigChannel(&hadc, &sConfig); // configures the channel that the adc reads from & other stuff in sConfig
    HAL_ADC_Start(&hadc); //converts analog to digital 
    HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY); //waits until conversion completes, will wait forever until done
    uint16_t airQualityValue = HAL_ADC_GetValue(&hadc); // converts value to int
    HAL_ADC_Stop(&hadc); //stops adc
    return airQualityValue; //returns air quality as a 16 bit int
}