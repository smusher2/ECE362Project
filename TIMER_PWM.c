#include "stm32f0xx.h"  // Include STM32 register definitions

volatile int temperature = 25;  // Placeholder for temperature variable

void setup_tim3(void) {

    RCC -> APB1ENR |= RCC_APB1ENR_TIM3EN;
    RCC -> AHBENR |= RCC_AHBENR_GPIOCEN;
    GPIOC->MODER |= 0xAA000;
    GPIOC->AFR[0] |= (0x0 << (6 * 4)) | (0x0 << (7 * 4));
    GPIOC->AFR[1] |= (0x0 << (0 * 4)) | (0x0 << (1 * 4));

    TIM3->PSC = 47999;
    TIM3->ARR = 999;

    TIM3->CCMR1 |= (0x6 << 4) | (1 << 3);
    TIM3->CCMR1 |= (0x6 << 12) | (1 << 11);
    TIM3->CCMR2 |= (0x6 << 4) | (1 << 3);
    TIM3->CCMR2 |= (0x6 << 12) | (1 << 11);

    TIM3->CCER |= (1 << 0) | (1 << 4) | (1 << 8) | (1 << 12);

    TIM3->CCR1 = 800;
    TIM3->CCR2 = 600;
    TIM3->CCR3 = 400;
    TIM3->CCR4 = 200;

    TIM3->CR1 |= TIM_CR1_CEN; 
}

// Function to set the PWM duty cycle for the fan based on temperature
void Adjust_Fan_Speed(void) {
    uint16_t dutyCycle;

    // Map temperature to PWM duty cycle on Channel 1 (PC6)
    if (temperature <= 25) {
        dutyCycle = 0;  // Fan off below 25°C
    } else if (temperature >= 40) {
        dutyCycle = 1000;  // Fan at full speed at 40°C or higher
    } else {

        dutyCycle = (temperature - 25) * (1000 / 15);
    }

    // Set the PWM duty cycle on TIM3 Channel 1 (PC6)
    TIM3->CCR3 = dutyCycle;
}

int main(void) {
    setup_tim3(); 

    while (1) {
        temperature = 50; 

        // Adjust fan speed
        Adjust_Fan_Speed();

        // Delay
        for (volatile int i = 0; i < 100000; i++); 
    }
}