/**
  ******************************************************************************
  * @file    main.c
  * @author  Weili An, Niraj Menon
  * @date    Feb 7, 2024
  * @brief   ECE 362 Lab 7 student template
  ******************************************************************************
*/

/*******************************************************************************/

// Fill out your username!  Even though we're not using an autotest, 
// it should be a habit to fill out your username in this field now.
const char* username = "362 Project";

/*******************************************************************************/ 

#include "stm32f0xx.h"
#include <stdint.h>
#include "commands.h"
#include <stdio.h>

#include <stdio.h>
#include "fifo.h"
#include "tty.h"

// TODO DMA data structures
#define FIFOSIZE 16
char serfifo[FIFOSIZE];
int seroffset = 0;

void internal_clock();

// .-.-.-..-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-
// SETUP-TFT-SPI-DISPLAY
// Zay Linn Htet
// .-.-.-..-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-
void init_usart5() {
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN; // Enable GPIOC Clock
    RCC->AHBENR |= RCC_AHBENR_GPIODEN; // Enable GPIOD Clock

    GPIOC->MODER &= ~GPIO_MODER_MODER12; // Clear GPIOC-12 mode
    GPIOC->MODER |= GPIO_MODER_MODER12_1; // Set GPIOC-12 to alternate function (10)
    GPIOC->AFR[1] &= ~GPIO_AFRH_AFRH4; // Clear GPIOC-12 AF
    GPIOC->AFR[1] |= (0x2 << 16); // Shift 0010 (AF2) to bit 16 (AFR12[3:0])

    GPIOD->MODER &= ~GPIO_MODER_MODER2; // Clear GPIOD-2 mode
    GPIOD->MODER |= GPIO_MODER_MODER2_1; // Set GPIOD-2 to alternate function (10)
    GPIOD->AFR[0] &= ~GPIO_AFRL_AFRL2; // Clear GPIOD-2 AF
    GPIOD->AFR[0] |= (0x2 << 8); // Shift 0010 (AF2) to bit 8 (AFR2[3:0])

    RCC->APB1ENR |= RCC_APB1ENR_USART5EN; // Enable USART5 Clock

    USART5->CR1 &= ~USART_CR1_UE; // Disable USART5 by turning off UE bit
    USART5->CR1 &= ~USART_CR1_M1; // Set M1 to 0 for 1 start bit, 8 data bits, and n stop bits
    USART5->CR1 &= ~USART_CR1_M0; // Set M0 to 0 for same function as M1
    USART5->CR2 &= ~USART_CR2_STOP; // Set STOP[1:0] to 00 to have 1 stop bit

    USART5->CR1 &= ~USART_CR1_PCE; // Disable Parity Control with 0 at PCE

    USART5->CR1 &= ~USART_CR1_OVER8; // Set oversampling x16 with 0 at OVER8
    USART5->BRR = 0x1a1; // Baud Rate set to 115200 at BRR[3:0]

    USART5->CR1 |= USART_CR1_TE; // Enables Transmitter
    USART5->CR1 |= USART_CR1_RE; // Enables Receiver
    USART5->CR1 |= USART_CR1_UE; // Enables USART

    while( (!(USART5->ISR & USART_ISR_TEACK)) && (!(USART5->ISR & USART_ISR_REACK))) {
        // While USART5_ISR_TEACK and USART5_ISR_REACK both don't show an acknowledge flag,
        // the function will wait before proceeding
    }
}

void enable_tty_interrupt(void) {
    
    RCC->APB1ENR |= RCC_APB1ENR_USART5EN;
    NVIC->ISER[0] |= 1 << USART3_8_IRQn;
    USART5->CR1 |= USART_CR1_RXNEIE;
    USART5->CR3 |= USART_CR3_DMAR;

    RCC->AHBENR |= RCC_AHBENR_DMA2EN;
    DMA2->CSELR |= DMA2_CSELR_CH2_USART5_RX;
    DMA2_Channel2->CCR &= ~DMA_CCR_EN;  // First make sure DMA is turned off
    
// The DMA channel 2 configuration goes here
    DMA2_Channel2->CMAR = serfifo; // Channel Mememory Address Register to adress of serfifo array
    DMA2_Channel2->CPAR = &(USART5->RDR); // Channel Peripheral Address Register to address of USART5->RDR
    DMA2_Channel2->CNDTR = FIFOSIZE; // Channel Data Register to FIFOSIZE
    DMA2_Channel2->CCR &= ~DMA_CCR_DIR; // Set Data Transfer Direction to "From Peripheral"
    DMA2_Channel2->CCR &= ~DMA_CCR_HTIE; // Don't enable Half Transfer Interrupt
    DMA2_Channel2->CCR &= ~DMA_CCR_TCIE; // Don't enable Complete Transfer Interrupt
    DMA2_Channel2->CCR &= ~DMA_CCR_MSIZE; // Set Memory Size to 8 bits
    DMA2_Channel2->CCR &= ~DMA_CCR_PSIZE; // Set Peripheral Size to 8 bits
    DMA2_Channel2->CCR |= DMA_CCR_MINC; // Enable Memory Increment Mode
    DMA2_Channel2->CCR &= ~DMA_CCR_PINC; // Don't enable Peripheral Increment Mode
    DMA2_Channel2->CCR |= DMA_CCR_CIRC; // Enable Circular Transfers
    DMA2_Channel2->CCR &= ~DMA_CCR_MEM2MEM; // Don't enable Memory to Memory Mode
    DMA2_Channel2->CCR |= DMA_CCR_PL; // Set Priority Level to Highest 11: Very High

    DMA2_Channel2->CCR |= DMA_CCR_EN;
    // TODO
}

// Works like line_buffer_getchar(), but does not check or clear ORE nor wait on new characters in USART
char interrupt_getchar() {
    while(fifo_newline(&input_fifo) == 0) {
        asm volatile ("wfi"); // wait for an interrupt
    }
    char ch = fifo_remove(&input_fifo);
    return ch;
}

int __io_putchar(int c) {
    while(!(USART5->ISR & USART_ISR_TXE));
    if(c == '\n') {
        while(!(USART5->ISR & USART_ISR_TXE)) {}
        USART5->TDR = '\r';
    }
    while(!(USART5->ISR & USART_ISR_TXE)) {}
    USART5->TDR = c;
    return c;
}

int __io_getchar(void) {
    // TODO Use interrupt_getchar() instead of line_buffer_getchar()
    int c = interrupt_getchar();
    return c;
}

// TODO Copy the content for the USART5 ISR here
void USART3_8_IRQHandler(void) {
    while(DMA2_Channel2->CNDTR != sizeof serfifo - seroffset) {
        if (!fifo_full(&input_fifo))
            insert_echo_char(serfifo[seroffset]);
        seroffset = (seroffset + 1) % sizeof serfifo;
    }
}

void init_spi1_slow() {
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    // PB3 SCK; PB4 MISO; PB5 MOSI
    GPIOB->MODER &= ~(GPIO_MODER_MODER3 | GPIO_MODER_MODER4 | GPIO_MODER_MODER5);
    GPIOB->MODER |= (GPIO_MODER_MODER3_1 | GPIO_MODER_MODER4_1 | GPIO_MODER_MODER5_1);

    GPIOB->AFR[0] &= ~(GPIO_AFRL_AFRL3 | GPIO_AFRL_AFRL4 | GPIO_AFRL_AFRL5);
    SPI1->CR1 |= SPI_CR1_BR;
    SPI1->CR1 |= SPI_CR1_MSTR;
    SPI1->CR2 &= ~(SPI_CR2_DS);
    SPI1->CR2 |= (SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0);
    SPI1->CR1 |= SPI_CR1_SSM;
    SPI1->CR1 |= SPI_CR1_SSI;
    SPI1->CR2 |= SPI_CR2_FRXTH;
    SPI1->CR1 |= SPI_CR1_SPE;
}

void enable_sdcard() {
    GPIOB->ODR &= ~GPIO_ODR_2;
}

void disable_sdcard() {
    GPIOB->ODR |= GPIO_ODR_2;
}

void init_sdcard_io() {
    init_spi1_slow();
    GPIOB->MODER &= ~GPIO_MODER_MODER2;
    GPIOB->MODER |= GPIO_MODER_MODER2_0;
    disable_sdcard();
}

void sdcard_io_high_speed() {
    SPI1->CR1 &= ~SPI_CR1_SPE;
    SPI1->CR1 &= ~SPI_CR1_BR;
    SPI1->CR1 |= SPI_CR1_BR_0;
    SPI1->CR1 |= SPI_CR1_SPE;
}

void init_lcd_spi() {
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    GPIOB->MODER &= ~(GPIO_MODER_MODER8 | GPIO_MODER_MODER11 | GPIO_MODER_MODER14);
    GPIOB->MODER |= (GPIO_MODER_MODER8_0 | GPIO_MODER_MODER11_0 | GPIO_MODER_MODER14_0);
    init_spi1_slow();
    sdcard_io_high_speed();
}
// .-.-.-..-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-


// .-.-.-..-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-
// SETUP-TEMPERATURE-HUMIDITY-SENSORS
// Ashwin Patel
// .-.-.-..-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-
volatile int temperature = 25;  // Placeholder for temperature variable
volatile int humidity = 0;

// Initialize USART5 for UART communication
void init_usart5_temp_hum(void) {
    // Enable clocks
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
    RCC->AHBENR |= RCC_AHBENR_GPIODEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART5EN;

    // Configure PC12 as USART5_TX (AF2)
    GPIOC->MODER &= ~(0x3 << (12 * 2));
    GPIOC->MODER |= (0x2 << (12 * 2));
    GPIOC->AFR[1] &= ~(0xF << ((12 - 8) * 4));
    GPIOC->AFR[1] |= (0x2 << ((12 - 8) * 4));

    // Configure PD2 as USART5_RX (AF2)
    GPIOD->MODER &= ~(0x3 << (2 * 2));
    GPIOD->MODER |= (0x2 << (2 * 2));
    GPIOD->AFR[0] &= ~(0xF << (2 * 4));
    GPIOD->AFR[0] |= (0x2 << (2 * 4));

    // USART configuration
    USART5->CR1 &= ~USART_CR1_UE;        // Disable USART
    USART5->CR1 &= ~USART_CR1_M0;        // Word length: 8 bits
    USART5->CR1 &= ~USART_CR1_M1;
    USART5->CR2 &= ~(0x3 << 12);         // Stop bits: 1
    USART5->CR1 &= ~USART_CR1_PCE;       // No parity
    USART5->CR1 &= ~USART_CR1_OVER8;     // Oversampling by 16
    USART5->BRR = 48000000 / 115200;     // Baud rate: 115200 bps
    USART5->CR1 |= USART_CR1_RE | USART_CR1_TE;  // Enable RX and TX
    USART5->CR1 |= USART_CR1_UE;         // Enable USART
    while (!(USART5->ISR & USART_ISR_TEACK));
    while (!(USART5->ISR & USART_ISR_REACK));
}

// Function to initialize TIM2 for microsecond timing
void TIM2_Init(void) {
    // Enable clock for TIM2
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    // Set prescaler to get 1 MHz counter clock (1 us per count)
    TIM2->PSC = 48 - 1;  // Prescaler value
    TIM2->ARR = 0xFFFF;  // Max auto-reload value
    TIM2->CNT = 0;       // Reset counter

    // Start timer
    TIM2->CR1 |= TIM_CR1_CEN;
}

// Function to provide microsecond delay using TIM2
void delay_us(uint32_t us) {
    uint32_t start = TIM2->CNT;
    while ((TIM2->CNT - start) < us);
}

// GPIO initialization for DHT22 on PB6
void DHT22_GPIO_Init(void) {
    // Enable GPIOB clock
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

    // Set PB6 as output open-drain
    GPIOB->MODER &= ~(3 << (6 * 2));  // Clear mode bits for PB6
    GPIOB->MODER |= (1 << (6 * 2));   // Set mode to output (01)
    GPIOB->OTYPER |= (1 << 6);        // Set output type to open-drain
    GPIOB->OSPEEDR |= (1 << (6 * 2)); // Set speed to medium
    GPIOB->PUPDR &= ~(3 << (6 * 2));  // No pull-up, no pull-down
}

// Set PB6 as input
void DHT22_Set_Pin_Input(void) {
    GPIOB->MODER &= ~(3 << (6 * 2));  // Set mode to input (00)
}

// Set PB6 as output
void DHT22_Set_Pin_Output(void) {
    GPIOB->MODER &= ~(3 << (6 * 2));  // Clear mode bits
    GPIOB->MODER |= (1 << (6 * 2));   // Set mode to output (01)
}

// Function to read data from DHT22 sensor
int DHT22_Read(void) {
    uint8_t data[5] = {0};
    uint32_t start_time;
    uint32_t timeout;

    // Send start signal
    DHT22_Set_Pin_Output();
    GPIOB->ODR &= ~(1 << 6);  // Pull the pin LOW
    delay_us(1200);           // Wait at least 1 ms (1200 µs)

    GPIOB->ODR |= (1 << 6);   // Pull the pin HIGH
    delay_us(20);             // Wait 20 µs

    DHT22_Set_Pin_Input();    // Set pin as input

    // DHT22 response sequence
    timeout = 100;
    while ((GPIOB->IDR & (1 << 6))) {  // Wait for the line to go LOW
        delay_us(1);
        if (--timeout == 0) return 1;  // Timeout
    }

    timeout = 100;
    while (!(GPIOB->IDR & (1 << 6))) {  // Wait for the line to go HIGH
        delay_us(1);
        if (--timeout == 0) return 1;  // Timeout
    }

    timeout = 100;
    while ((GPIOB->IDR & (1 << 6))) {  // Wait for the line to go LOW again
        delay_us(1);
        if (--timeout == 0) return 1;  // Timeout
    }

    // Reading 40 bits (5 bytes) of data
    for (int i = 0; i < 40; i++) {
        // Wait for the pin to go HIGH
        timeout = 100;
        while (!(GPIOB->IDR & (1 << 6))) {
            delay_us(1);
            if (--timeout == 0) return 1;  // Timeout
        }

        // Measure the length of the HIGH signal
        start_time = TIM2->CNT;
        while ((GPIOB->IDR & (1 << 6))) {
            if ((TIM2->CNT - start_time) > 100) break;  // Safety break after 100 µs
        }

        // Determine if bit is 0 or 1
        if ((TIM2->CNT - start_time) > 40) {
            data[i / 8] <<= 1;
            data[i / 8] |= 1;  // Bit is '1'
        } else {
            data[i / 8] <<= 1;  // Bit is '0'
        }
    }

    // Checksum validation
    uint8_t sum = data[0] + data[1] + data[2] + data[3];
    if (sum != data[4]) {
        return 2;  // Checksum error
    }

    // Parse humidity and temperature data
    uint16_t raw_humidity = ((uint16_t)data[0] << 8) | data[1];
    uint16_t raw_temperature = ((uint16_t)data[2] << 8) | data[3];

    // Convert to human-readable format
    humidity = raw_humidity / 10;
    if (raw_temperature & 0x8000) {  // Negative temperature
        raw_temperature &= 0x7FFF;
        temperature = -((int)raw_temperature / 10);
    } else {
        temperature = raw_temperature / 10;
    }

    return 0;  // Success
}

void setup_tim3(void) {

    RCC -> APB1ENR |= RCC_APB1ENR_TIM3EN;
    RCC -> AHBENR |= RCC_AHBENR_GPIOCEN;
    GPIOC->MODER |= 0xAA000;
    GPIOC->AFR[0] |= (0x0 << (6 * 4)) | (0x0 << (7 * 4));
    GPIOC->AFR[1] |= (0x0 << (0 * 4)) | (0x0 << (1 * 4));

    TIM3->PSC = 7;
    TIM3->ARR = 999;

    TIM3->CCMR1 |= (0x6 << 4) | (1 << 3);
    TIM3->CCMR1 |= (0x6 << 12) | (1 << 11);
    TIM3->CCMR2 |= (0x6 << 4) | (1 << 3);
    TIM3->CCMR2 |= (0x6 << 12) | (1 << 11);

    TIM3->CCER |= (1 << 0) | (1 << 4) | (1 << 8) | (1 << 12);

    //TIM3->CCR1 = 800;
    //TIM3->CCR2 = 600;
    //TIM3->CCR3 = 400;
    //TIM3->CCR4 = 200;

    TIM3->CR1 |= TIM_CR1_CEN; 
}

// Function to set the PWM duty cycle for the fan based on temperature
void Adjust_Fan_Speed(void) {
    uint16_t dutyCycle;

    // Map temperature to PWM duty cycle on Channel 1 (PC6)
    if (temperature <= 25) {
        dutyCycle = 0;  // Fan off below 25°C
    } else if (temperature >= 30) {
        dutyCycle = 1000;  // Fan at full speed at 40°C or higher
    } else {

        dutyCycle = (temperature - 25) * (1000 / 5);
    }

    // Set the PWM duty cycle on TIM3 Channel 1 (PC6)
    TIM3->CCR3 = dutyCycle;
}

int ret_temp() {
    int val = DHT22_Read();
    return temperature;
}

int ret_hum() {
    int val = DHT22_Read();
    return humidity;
}
// .-.-.-..-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-



// .-.-.-..-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-
// SETUP-LIGHT-SENSORS
// Akash Amalarasan
// .-.-.-..-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-

// void ADC_Init(void);
// uint16_t Read_ADC(void);
// void TIM15_PWM_Init(void);
// void Set_LED_Brightness(uint16_t brightness);

void ADC_Init_LED(void) {
    // Enable clock for GPIOA and ADC
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_ADCEN;

    // Set PA0 to analog mode (for LDR input)
    GPIOA->MODER |= GPIO_MODER_MODER0;

    // Configure ADC
    ADC1->CHSELR = ADC_CHSELR_CHSEL0;    // Select channel 0 (PA0)
    ADC1->CFGR1 &= ~ADC_CFGR1_RES;       // Set resolution to 12 bits
    ADC1->CR |= ADC_CR_ADEN;             // Enable ADC

    // Wait for ADC to be ready
    while (!(ADC1->ISR & ADC_ISR_ADRDY));
}

uint16_t Read_ADC(void) {
    // Start ADC conversion
    ADC1->CR |= ADC_CR_ADSTART;

    // Wait for conversion to complete
    while (!(ADC1->ISR & ADC_ISR_EOC));

    // Return ADC value
    return ADC1->DR;
}

void TIM15_PWM_Init(void) {
    // Enable the clock for TIM2 and GPIOA
    RCC->APB2ENR |= RCC_APB2ENR_TIM15EN;
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

    // Set PA1 to alternate function mode for TIM2 CH2 (PWM output)
    GPIOA->MODER &= ~(3 << (2 * 2));   // Clear mode for PA1
    GPIOA->MODER |= (2 << (2 * 2));    // Set PA1 to alternate function mode (10)
    GPIOA->AFR[0] &= ~(0xF << (2 * 4));
    GPIOA->AFR[0] |= (9 << (2 * 4));   // Set PA1 to AF2 (TIM2_CH2)

    // Set up TIM2 for PWM on CH2
    TIM15->PSC = 48 - 1;       // Prescaler to divide 48MHz to 1MHz (1us per tick)
    TIM15->ARR = 1000 - 1;     // Auto-reload value for 1kHz frequency
    TIM15->CCMR1 |= (6 << 4); // PWM mode 1 on TIM2 CH2 (OC2M)
    TIM15->CCMR1 |= (1 << 3); // Enable preload on CH2
    TIM15->CCER |= (1 << 0);   // Enable TIM2 CH2 output
    TIM15->CR1 |= TIM_CR1_CEN; // Enable TIM2
}

void Set_LED_Brightness(uint16_t brightness) {
    // Clamp brightness value between 0 and ARR (999)
    if (brightness > 999) brightness = 999;
    TIM2->CCR2 = brightness;
}

// .-.-.-..-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-


// .-.-.-..-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-
// SETUP-AIR-QUALITY-SENSOR
// Jin Hyun Kim
// .-.-.-..-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-
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

#define ADC_MAX_VALUE 4095
#define SENSOR_VOLTAGE_MAX 10
#define PPM_MAX 100

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
    uint16_t ppm =  ((voltage / SENSOR_VOLTAGE_MAX) * PPM_MAX);

    // Return the estimated PPM value
    return ppm;
}

 
#include "stm32f0xx.h"

int i;  // Variable to store the ADC reading

// Function to initialize GPIOA.6 as an analog input
void GPIO_Init(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN; // Enable clock for GPIOA
    GPIOA->MODER |= (3 << (6 * 2));    // Set PA6 to analog mode
}

// Function to initialize the ADC
void ADC_Init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;    // Enable clock for ADC1
    ADC1->CFGR1 &= ~ADC_CFGR1_RES;         // Set resolution to 12-bit (default)
    ADC1->CHSELR = ADC_CHSELR_CHSEL6;      // Select channel 6 (PA6)
    ADC1->SMPR = ADC_SMPR_SMP;     // Set sampling time to 239.5 cycles

    ADC1->CR |= ADC_CR_ADEN;               // Enable the ADC
    while (!(ADC1->ISR & ADC_ISR_ADRDY));  // Wait until ADC is ready
}

// Function to read the ADC value from channel 6
uint16_t ADC_Read(void) {
    ADC1->CR |= ADC_CR_ADSTART;            // Start ADC conversion
    while (!(ADC1->ISR & ADC_ISR_EOC));    // Wait for the conversion to complete
    return ADC1->DR;                       // Read the ADC data register

}

// .-.-.-..-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-


int main() {
    internal_clock();

    GPIO_Init();
    ADC_Init();

    init_usart5();
    init_usart5_temp_hum();        
    TIM2_Init();        
    DHT22_GPIO_Init();  
  

    enable_tty_interrupt();
    setbuf(stdin,0);
    setbuf(stdout,0);
    setbuf(stderr,0);
    
    uint16_t adc_value;
    uint16_t brightness;

    ADC_Init_LED();       // Initialize ADC for LDR input
    TIM15_PWM_Init();  // Initialize PWM for LED control

    command_shell();

    // while (1) {
    //     // Read ADC value from LDR
    //     adc_value = Read_ADC();

    //     // Map ADC value to brightness (invert if necessary)
    //     // For example, to make the LED brighter in darkness:
    //     brightness = 999 - (adc_value * 999 / 4095);

    //     // Set LED brightness
    //     Set_LED_Brightness(brightness);

    //     // Small delay (adjust as needed)
    //     for (volatile int i = 0; i < 10000; i++);
    // }



    //    printf("Starting DHT22 sensor reading...\n");
    
    // while (1) {

    //     int result = DHT22_Read();
    //     if (result == 0) {
    //         // Successfully read temperature and humidity
    //         printf("Temperature: %d°C, Humidity: %d%%\n", temperature, humidity);
    //     } else if (result == 1) {
    //         // Timeout error
    //         printf("DHT22 Read Timeout.\n");
    //     } else if (result == 2) {
    //         // Checksum error
    //         printf("DHT22 Checksum Error.\n");
    //     } else {
    //         // Unknown error
    //         printf("DHT22 Unknown Error.\n");
    //     }

    //     // Delay of at least 2 seconds before the next read (DHT22 requirement)
    //     for (volatile int i = 0; i < 4000000; i++);  // Simple delay loop (~2 seconds)

    //     // Adjust fan speed
    //     Adjust_Fan_Speed();

    //     // Delay
    //     for (volatile int i = 0; i < 100000; i++); 
    // }
    
    
}