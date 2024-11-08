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

// Function prototypes
void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_ADC_Init(void);
void MX_TIM_Init(void);
void MX_SPI_Init(void);
void displayAirQualityOnTFT(uint16_t airQuality);

int main(void) {
    // HAL_Init();                // Initialize HAL library
    // SystemClock_Config();      // Configure system clock
    // MX_GPIO_Init();            // Initialize GPIO
    // MX_ADC_Init();             // Initialize ADC for air quality sensor
    // MX_TIM_Init();             // Initialize Timer for PWM control
    // MX_SPI_Init();             // Initialize SPI for TFT display

    // HAL_TIM_PWM_Start(&htim1, FAN_PWM_CHANNEL);  // Start PWM for fan

    while (1) {
        // Read Air Quality Sensor (MQ-135) data
        uint16_t airQualityValue = readAirQuality();

        // Display air quality data on TFT display
        displayAirQualityOnTFT(airQualityValue);

        // Control fan speed based on air quality
        controlFanSpeed(airQualityValue);

        // Small delay for stability in reading and display
        HAL_Delay(500);
    }
}

uint16_t readAirQuality(void) {
    static uint8_t preheat = 0;  

    if (!preheat) {
        for (volatile int i = 0; i < 2000000; ++i);  // Roughly 20 seconds delay
        preheat = 1;  
    }

    // Start ADC conversion
    ADC1->CR |= ADC_CR_ADSTART;

    // Wait for ADC conversion to complete
    while (!(ADC1->ISR & ADC_ISR_EOC));       // Wait until end of conversion

    // Retrieve ADC conversion result
    uint16_t adcValue = ADC1->DR;

    // Convert ADC value to voltage
    float voltage = (adcValue / (float)ADC_MAX_VALUE) * SENSOR_VOLTAGE_MAX;

    // Estimate PPM based on sensor's output characteristics
    uint16_t ppm = (voltage / SENSOR_VOLTAGE_MAX) * PPM_MAX;

    // Return the estimated PPM value
    return ppm;
}