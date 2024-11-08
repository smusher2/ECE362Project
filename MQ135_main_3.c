//https://madhusudan.live/blog/diy-smoke-detector-system-mq135-and-arduino
//https://www.elprocus.com/mq135-air-quality-sensor/
//https://www.olimex.com/Products/Components/Sensors/Gas/SNS-MQ135/resources/SNS-MQ135.pdf
//https://files.waveshare.com/upload/2/24/MQ-135-Gas-Sensor-UserManual.pdf


#include "stm32f0xx.h"

#include <math.h>   // for M_PI

#include <stdint.h>

#include <stdio.h>

#define MQOut ADC_CHANNEL_1          // ADC channel for MQ-135 sensor
#define SENSOR_THRESHOLD 170         // Threshold for safe/warning levels

// Function prototypes
void ADC_Init(void);
uint16_t readAnalogSensor(void);
void delay(uint32_t milliseconds);

int main(void) {
    SystemClock_Config();    // Configure the system clock
    ADC_Init();              // Initialize ADC for smoke sensor reading

    while (1) {

        // Read the analog value from the MQ sensor
        uint16_t analogSensor = readAnalogSensor();

        // Offset applied as in original Arduino code
        int adjustedValue = analogSensor - 50;

        // Process the adjusted value (you may add code here to transmit data, log it, etc.)

        delay(600);  // Delay before the next reading
    }
}

/**
 * @brief Initializes the ADC for reading the MQ sensor analog output.
 */
void ADC_Init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;        // Enable ADC1 clock
    ADC1->CFGR1 &= ~ADC_CFGR1_RES;             // Set ADC resolution to 12 bits
    ADC1->CFGR1 |= ADC_CFGR1_CONT;             // Enable continuous conversion mode
    ADC1->CHSELR = (1 << MQOut);               // Select MQ sensor channel
    ADC1->CR |= ADC_CR_ADEN;                   // Enable the ADC
    while (!(ADC1->ISR & ADC_ISR_ADRDY));      // Wait for ADC to be ready
}

/**
 * @brief Reads the analog value from the MQ sensor connected to the ADC.
 * 
 * @return The raw analog value from the sensor.
 */
uint16_t readAnalogSensor(void) {

    static uint8_t preheat = 0;  // preheated check 

    if (!preheat) {
        for (volatile int i = 0; i < 2000000; ++i);  // Roughly 20 seconds delay
        preheat = 1;  // preheated
    }

    // Start ADC conversion
    ADC1->CR |= ADC_CR_ADSTART;

    // Wait for conversion to complete
    while (!(ADC1->ISR & ADC_ISR_EOC));   // Wait until end of conversion

    // Read and return the ADC value
    return ADC1->DR;
}

/**
 * @brief Simple delay function.
 * @param milliseconds Number of milliseconds to delay.
 */
void delay(uint32_t milliseconds) {
    uint32_t count = milliseconds * 8000;  // Rough delay count for STM32 at ~8MHz
    while (count--) {
        __asm__("nop");
    }
}

