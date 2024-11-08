/**
  ******************************************************************************
  * @file    main.c
  * @author  Weili An, Niraj Menon
  * @date    Feb 3, 2024
  * @brief   ECE 362 Lab 6 Student template
  ******************************************************************************
*/
/*******************************************************************************/

// Fill out your username, otherwise your completion code will have the 
// wrong username!
const char* username = "aamalara";

/*******************************************************************************/ 

#include "stm32f0xx.h"

void set_char_msg(int, char);
void nano_wait(unsigned int);
void game(void);
void internal_clock();
void check_wiring();
void autotest();

//===========================================================================
// Configure GPIOC
//===========================================================================
void enable_ports(void) {
    // Only enable port C for the keypad
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
    GPIOC->MODER &= ~0xffff;
    GPIOC->MODER |= 0x55 << (4*2);
    GPIOC->OTYPER &= ~0xff;
    GPIOC->OTYPER |= 0xf0;
    GPIOC->PUPDR &= ~0xff;
    GPIOC->PUPDR |= 0x55;
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN; // Enable GPIOC clock
    GPIOC->MODER |= GPIO_MODER_MODER8_0; // Set PC8 as output (for LED)

}


uint8_t col; // the column being scanned

void drive_column(int);   // energize one of the column outputs
int  read_rows();         // read the four row inputs
void update_history(int col, int rows); // record the buttons of the driven column
char get_key_event(void); // wait for a button event (press or release)
char get_keypress(void);  // wait for only a button press event.
float getfloat(void);     // read a floating-point number from keypad
void show_keys(void);     // demonstrate get_key_event()

//===========================================================================
// Bit Bang SPI LED Array
//===========================================================================
int msg_index = 0;
uint16_t msg[8] = { 0x0000,0x0100,0x0200,0x0300,0x0400,0x0500,0x0600,0x0700 };
extern const char font[];

//===========================================================================
// Configure PB12 (CS), PB13 (SCK), and PB15 (SDI) for outputs
//===========================================================================
void setup_bb(void) {
    RCC -> AHBENR |= RCC_AHBENR_GPIOBEN;
    GPIOB -> MODER &= ~0xcf000000;
    GPIOB -> MODER |= 0x45000000;
    GPIOB -> BSRR |= GPIO_BSRR_BR_13;
    GPIOB -> BSRR |= GPIO_BSRR_BS_12;  
}

void small_delay(void) {
    nano_wait(50000);
}

//===========================================================================
// Set the MOSI bit, then set the clock high and low.
// Pause between doing these steps with small_delay().
//===========================================================================
void bb_write_bit(int val) {
    // CS (PB12)
    // SCK (PB13)
    // SDI (PB15)
    if(val){ 
        GPIOB -> BSRR = GPIO_BSRR_BS_15;
    }
    else{
        GPIOB -> BSRR = GPIO_BSRR_BR_15;
    }
    small_delay();
    GPIOB->BSRR = GPIO_BSRR_BS_13;
    small_delay();
    GPIOB -> BSRR = GPIO_BSRR_BR_13;
}

//===========================================================================
// Set CS (PB12) low,
// write 16 bits using bb_write_bit,
// then set CS high.
//===========================================================================
void bb_write_halfword(int halfword) {
    GPIOB -> BSRR = GPIO_BSRR_BR_12;
    for(int i = 15; i >= 0; i--){
        bb_write_bit(halfword & (1 << (i))); 
    }
    GPIOB -> BSRR |= GPIO_BSRR_BS_12;
}

//===========================================================================
// Continually bitbang the msg[] array.
//===========================================================================
void drive_bb(void) {
    for(;;)
        for(int d=0; d<8; d++) {
            bb_write_halfword(msg[d]);
            nano_wait(1000000); // wait 1 ms between digits
        }
}

//============================================================================
// Configure Timer 15 for an update rate of 1 kHz.
// Trigger the DMA channel on each update.
// Copy this from lab 4 or lab 5.
//============================================================================
void init_tim15(void) {
    RCC -> APB2ENR |= RCC_APB2ENR_TIM15EN; 
    TIM15 -> PSC = 4800 - 1;
    TIM15 -> ARR = 10 - 1;  
    TIM15 -> DIER |= TIM_DIER_UDE; 
    TIM15 -> CR1 |= TIM_CR1_CEN; 
}

//===========================================================================
// Configure timer 7 to invoke the update interrupt at 1kHz
// Copy from lab 4 or 5.
//===========================================================================
void init_tim7() {
    RCC -> APB1ENR |= RCC_APB1ENR_TIM7EN;
    TIM7 -> PSC = 4800 - 1;
    TIM7 -> ARR = 10 - 1;
    TIM7 -> DIER |= TIM_DIER_UIE;
    TIM7 -> CR1 |= TIM_CR1_CEN;
    NVIC->ISER[0] = 1 << TIM7_IRQn;
}
//===========================================================================
// Copy the Timer 7 ISR from lab 5
//===========================================================================
// TODO To be copied

void TIM7_IRQHandler() {
    TIM7->SR = ~TIM_SR_UIF;
    int rows = read_rows();
    update_history(col, rows);
    col = (col + 1) & 3;
    drive_column(col);
}

//===========================================================================
// Initialize the SPI2 peripheral.
//===========================================================================
void init_spi2(void) {
    RCC -> AHBENR |= RCC_AHBENR_GPIOBEN;
    RCC -> APB1ENR |= RCC_APB1ENR_SPI2EN; 
    GPIOB -> MODER &= ~0xcf000000; 
    GPIOB -> MODER |= 0x8A000000; 
    GPIOB -> AFR[1] &= ~GPIO_AFRH_AFRH7;
    GPIOB -> AFR[1] &= ~GPIO_AFRH_AFRH5;
    GPIOB -> AFR[1] &= ~GPIO_AFRH_AFRH4;
    SPI2 -> CR1 &= ~SPI_CR1_SPE;
    SPI2 -> CR1 |= (SPI_CR1_MSTR | SPI_CR1_BR);
    SPI2 -> CR2 |= SPI_CR2_DS_0 | SPI_CR2_DS_1 | SPI_CR2_DS_2 | SPI_CR2_DS_3; 
    SPI2 -> CR2 |= (SPI_CR2_SSOE | SPI_CR2_NSSP);
    SPI2 -> CR2 |= SPI_CR2_TXDMAEN;
    SPI2 -> CR1 |= SPI_CR1_SPE;
}

//===========================================================================
// Configure the SPI2 peripheral to trigger the DMA channel when the
// transmitter is empty.  Use the code from setup_dma from lab 5.
//===========================================================================
void spi2_setup_dma(void) {
    RCC -> AHBENR |= RCC_AHBENR_DMA1EN;
    DMA1_Channel5 -> CCR &= ~DMA_CCR_EN;
    DMA1_Channel5 -> CPAR = (uint32_t) &(SPI2->DR);
    DMA1_Channel5 -> CMAR = (uint32_t)msg;
    DMA1_Channel5 -> CNDTR = 8;
    DMA1_Channel5 -> CCR|= DMA_CCR_DIR;
    DMA1_Channel5 -> CCR |= DMA_CCR_MINC;
    DMA1_Channel5 -> CCR &= ~DMA_CCR_MSIZE;
    DMA1_Channel5 -> CCR |= DMA_CCR_MSIZE_0;
    DMA1_Channel5 -> CCR &= ~DMA_CCR_PSIZE;
    DMA1_Channel5 -> CCR |= DMA_CCR_PSIZE_0;
    DMA1_Channel5 -> CCR |= DMA_CCR_CIRC;
    SPI2->CR2 |= SPI_CR2_TXDMAEN;
}

//===========================================================================
// Enable the DMA channel.
//===========================================================================
void spi2_enable_dma(void) {
    DMA1_Channel5 -> CCR |= DMA_CCR_EN; // Turn on the enable for the channel   
}

//===========================================================================
// 4.4 SPI OLED Display
//===========================================================================
void init_spi1() {
    RCC -> AHBENR |= RCC_AHBENR_GPIOAEN;
    RCC -> APB2ENR |= RCC_APB2ENR_SPI1EN; 
    GPIOA -> MODER &= ~0xc000fc00; 
    GPIOA-> MODER |= 0x8a00a800; //
    GPIOA -> AFR[1] &= ~GPIO_AFRH_AFRH7;
    GPIOA -> AFR[0] &= ~GPIO_AFRL_AFRL7;
    GPIOA -> AFR[0] &= ~GPIO_AFRL_AFRL6;
    GPIOA -> AFR[0] &= ~GPIO_AFRL_AFRL5;
    SPI1 -> CR1 &= ~SPI_CR1_SPE;
    SPI1 -> CR1 |= (SPI_CR1_BR_0 | SPI_CR1_BR_1 | SPI_CR1_BR_2);
    SPI1 -> CR2 = (SPI_CR2_DS_0 | SPI_CR2_DS_3 | SPI_CR2_SSOE | SPI_CR2_NSSP);
    SPI1 -> CR2 &= ~(SPI_CR2_DS_1 | SPI_CR2_DS_2);
    SPI1 -> CR1 |= SPI_CR1_MSTR;
    SPI1 -> CR2 |= SPI_CR2_TXDMAEN;
    SPI1 -> CR1 |= SPI_CR1_SPE;    
}
void spi_cmd(unsigned int data) {
    while(!(SPI1->SR & SPI_SR_TXE)) {}
    SPI1->DR = data;
}
void spi_data(unsigned int data) {
    spi_cmd(data | 0x200);     
}
void spi1_init_oled() {
    nano_wait(1000000);
    spi_cmd(0x38);
    spi_cmd(0x08);
    spi_cmd(0x01);
    nano_wait(2000000);
    spi_cmd(0x06);
    spi_cmd(0x02);
    spi_cmd(0x0c);    
}
void spi1_display1(const char *string) {
    spi_cmd(0x02);
    while(*string != '\0') {
        spi_data(*string);
        string++;
    }    
}
void spi1_display2(const char *string) {
    spi_cmd(0xc0);
    while(*string != '\0') {
        spi_data(*string);
        string++;
    }    
}

//===========================================================================
// This is the 34-entry buffer to be copied into SPI1.
// Each element is a 16-bit value that is either character data or a command.
// Element 0 is the command to set the cursor to the first position of line 1.
// The next 16 elements are 16 characters.
// Element 17 is the command to set the cursor to the first position of line 2.
//===========================================================================
uint16_t display[34] = {
        0x002, // Command to set the cursor at the first position line 1
        0x200+'E', 0x200+'C', 0x200+'E', 0x200+'3', 0x200+'6', + 0x200+'2', 0x200+' ', 0x200+'i',
        0x200+'s', 0x200+' ', 0x200+'t', 0x200+'h', + 0x200+'e', 0x200+' ', 0x200+' ', 0x200+' ',
        0x0c0, // Command to set the cursor at the first position line 2
        0x200+'c', 0x200+'l', 0x200+'a', 0x200+'s', 0x200+'s', + 0x200+' ', 0x200+'f', 0x200+'o',
        0x200+'r', 0x200+' ', 0x200+'y', 0x200+'o', + 0x200+'u', 0x200+'!', 0x200+' ', 0x200+' ',
};

//===========================================================================
// Configure the proper DMA channel to be triggered by SPI1_TX.
// Set the SPI1 peripheral to trigger a DMA when the transmitter is empty.
//===========================================================================
void spi1_setup_dma(void) {
    RCC -> AHBENR |= RCC_AHBENR_DMA1EN;
    DMA1_Channel3 -> CCR &= ~DMA_CCR_EN;
    DMA1_Channel3 -> CPAR = (uint32_t) & (SPI1 -> DR);
    DMA1_Channel3 -> CMAR = (uint32_t) display;
    DMA1_Channel3 -> CNDTR = 34;
    DMA1_Channel3 -> CCR |= DMA_CCR_DIR;
    DMA1_Channel3 -> CCR |= DMA_CCR_MINC;
    DMA1_Channel3 -> CCR &= ~DMA_CCR_MSIZE;
    DMA1_Channel3 -> CCR |= DMA_CCR_MSIZE_0;
    DMA1_Channel3 -> CCR &= ~DMA_CCR_PSIZE;
    DMA1_Channel3 -> CCR |= DMA_CCR_PSIZE_0;
    DMA1_Channel3 -> CCR |= DMA_CCR_CIRC;
    SPI1 -> CR2 |= SPI_CR2_TXDMAEN;  
}
/*
void init_adc(void) {
    RCC->APB2ENR |= RCC_APB2ENR_ADCEN;   // Enable ADC clock
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;   // Enable GPIOA clock
    GPIOA->MODER |= GPIO_MODER_MODER1;   // Set PA1 to analog mode

    ADC1->CR &= ~ADC_CR_ADEN;            // Ensure the ADC is disabled
    ADC1->CFGR1 &= ~ADC_CFGR1_RES;       // Set to 12-bit resolution (0-4095)
    ADC1->CHSELR = ADC_CHSELR_CHSEL1;    // Select channel 1 (PA1)

    // Calibration step
    ADC1->CR |= ADC_CR_ADCAL;            // Start calibration
    while (ADC1->CR & ADC_CR_ADCAL);     // Wait for calibration to complete

    ADC1->CR |= ADC_CR_ADEN;             // Enable ADC
    while (!(ADC1->ISR & ADC_ISR_ADRDY)); // Wait until ADC is ready
}

void blink_led(uint16_t adc_value) {
    if (adc_value > 2000) {
        GPIOC->BSRR = GPIO_BSRR_BS_8; // Turn on LED on PC8 if ADC > 2000
    } else {
        GPIOC->BSRR = GPIO_BSRR_BR_8; // Turn off LED otherwise
    }
}

float read_temperature(void) {
    ADC1->CR |= ADC_CR_ADSTART;              // Start ADC conversion
    while (!(ADC1->ISR & ADC_ISR_EOC));      // Wait for conversion to complete
    int adc_value = ADC1->DR;                // Read the ADC value

    // Convert ADC reading to temperature (adjust based on your sensor)
    float voltage = adc_value * 3.3 / 4095;  // Calculate voltage (3.3V reference, 12-bit ADC)
    float temperature = (voltage - 0.5) * 100; // Convert to Celsius for TMP36
    return temperature;
}


void int_to_string(int value, char *buffer) {
    int i = 0;
    if (value == 0) {
        buffer[i++] = '0';
    } else {
        while (value > 0) {
            buffer[i++] = (value % 10) + '0';  // Convert digit to character
            value /= 10;
        }
    }
    buffer[i] = '\0';

    // Reverse the string to get the correct order
    int start = 0, end = i - 1;
    while (start < end) {
        char temp = buffer[start];
        buffer[start++] = buffer[end];
        buffer[end--] = temp;
    }
}


void display_temperature(float temperature) {
    char buffer[16];
    int temp_value = (int)temperature; // Convert float to int for display
    int_to_string(temp_value, buffer); // Convert int temperature to string
    spi1_display1("Temp: ");           // Display static "Temp: " prefix
    spi1_display2(buffer);             // Display the converted temperature value
}





void display_test_message(void) {
    spi1_display1("ADC: 1234");       // Directly write "ADC: 1234" to line 1
    spi1_display2("Temp Sensor");     // Static message for line 2
}

*/

void int_to_string(int value, char *buffer) {
    int i = 0;
    if (value == 0) {
        buffer[i++] = '0';
    } else {
        while (value > 0) {
            buffer[i++] = (value % 10) + '0';  // Convert digit to character
            value /= 10;
        }
    }
    buffer[i] = '\0';

    // Reverse the string to get the correct order
    int start = 0, end = i - 1;
    while (start < end) {
        char temp = buffer[start];
        buffer[start++] = buffer[end];
        buffer[end--] = temp;
    }
} 

void TIM6_Init(void) {
    // Enable the TIM6 clock
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
    
    // Set prescaler to make TIM6 tick at 1 MHz (1 tick = 1 microsecond)
    // Assuming SystemCoreClock is 48 MHz, set prescaler to 48 - 1
    TIM6->PSC = 48 - 1;
    
    // Set auto-reload value to maximum (16-bit timer, so 0xFFFF)
    TIM6->ARR = 0xFFFF;
    
    // Start the timer
    TIM6->CR1 |= TIM_CR1_CEN;
}


void delay_us(uint16_t us) {
    // Reset the timer counter
    TIM6->CNT = 0;

    // Wait until the timer counter reaches the desired delay
    while (TIM6->CNT < us);
}

// Configure GPIO for DHT22
void DHT22_Init(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN; // Enable GPIOA clock
    GPIOA->MODER |= GPIO_MODER_MODER1_0; // Set PA1 as output
}

// Start signal to DHT22
void DHT22_Start(void) {
    GPIOA->MODER |= GPIO_MODER_MODER1_0; // Set PA1 as output
    GPIOA->BSRR = GPIO_BSRR_BR_1;        // Pull PA1 low
    delay_us(1000);                      // Wait 1 ms
    GPIOA->BSRR = GPIO_BSRR_BS_1;        // Pull PA1 high
    delay_us(30);                        // Wait 30 us
    GPIOA->MODER &= ~GPIO_MODER_MODER1;  // Set PA1 as input
}

// Read data from DHT22
uint8_t DHT22_Read(void) {
    uint8_t data = 0;
    for (int i = 0; i < 8; i++) {
        int timeout = 1000; // Timeout counter

        // Wait for the pin to go high with timeout
        while (!(GPIOA->IDR & GPIO_IDR_1)) {
            if (--timeout == 0) return 0xFF; // Return an error value if timeout
        }
        
        delay_us(40); // Delay to measure the pulse width
        
        // Check if the pin is high to determine the data bit
        if (GPIOA->IDR & GPIO_IDR_1) {
            data |= (1 << (7 - i)); // Set the bit
        }

        // Wait for the pin to go low, again with timeout
        timeout = 1000;
        while (GPIOA->IDR & GPIO_IDR_1) {
            if (--timeout == 0) return 0xFF; // Return an error value if timeout
        }
    }
    return data;
}


// Get temperature and humidity from DHT22
void DHT22_GetData(int *temperature, int *humidity) {
    DHT22_Start(); // Send start signal

    uint8_t humidity_high = DHT22_Read();
    if (humidity_high == 0xFF) return; // Exit if timeout occurs

    uint8_t humidity_low = DHT22_Read();
    if (humidity_low == 0xFF) return;

    uint8_t temperature_high = DHT22_Read();
    if (temperature_high == 0xFF) return;

    uint8_t temperature_low = DHT22_Read();
    if (temperature_low == 0xFF) return;

    DHT22_Read(); // Read checksum (not used here)

    *humidity = (humidity_high << 8) + humidity_low;
    *temperature = (temperature_high << 8) + temperature_low;
}


void DHT22_Reset(void) {
    // Resetting DHT22 by toggling a GPIO pin (if applicable)
    GPIOA->BSRR = GPIO_BSRR_BR_1;  // Set PA1 (or relevant pin) to low
    delay_us(1000);                // Wait for 1 ms
    GPIOA->BSRR = GPIO_BSRR_BS_1;  // Set PA1 high to power DHT22 back on
    delay_us(1000);                // Wait for DHT22 to reset
}

// Display temperature on OLED
void display_temperature(int temperature) {
    char buffer[16];
    int temp_value = temperature / 10; // Assuming 1 decimal point scaling
    int_to_string(temp_value, buffer); // Convert integer temperature to string
    spi1_display1("Temp: ");
    spi1_display2(buffer);
}

void indicate_temperature(int temperature) {
    int delay_time = 1000000 / (temperature / 10 + 1); // Increase blink rate with temperature
    GPIOC->ODR ^= GPIO_ODR_8; // Toggle LED on PC8
    nano_wait(delay_time);     // Adjust delay based on temperature
}


//===========================================================================
// Enable the DMA channel triggered by SPI1_TX.
//===========================================================================
void spi1_enable_dma(void) {
    DMA1_Channel3->CCR |= DMA_CCR_EN;    
}

//===========================================================================
// Main function
//===========================================================================

int main(void) {
    internal_clock();

    msg[0] |= font['E'];
    msg[1] |= font['C'];
    msg[2] |= font['E'];
    msg[3] |= font[' '];
    msg[4] |= font['3'];
    msg[5] |= font['6'];
    msg[6] |= font['2'];
    msg[7] |= font[' '];

    // GPIO enable
    enable_ports();
    // setup keyboard
    init_tim7();

    // LED array Bit Bang
// #define BIT_BANG
#if defined(BIT_BANG)
    setup_bb();
    drive_bb();
#endif

    // Direct SPI peripheral to drive LED display
//#define SPI_LEDS
#if defined(SPI_LEDS)
    init_spi2();
    spi2_setup_dma();
    spi2_enable_dma();
    init_tim15();
    show_keys();
#endif

    // LED array SPI
//#define SPI_LEDS_DMA
#if defined(SPI_LEDS_DMA)
    init_spi2();
    spi2_setup_dma();
    spi2_enable_dma();
    show_keys();
#endif

    // SPI OLED direct drive
//#define SPI_OLED
#if defined(SPI_OLED)
    init_spi1();
    spi1_init_oled();
    spi1_display1("Hello again,");
    spi1_display2(username);
#endif

    // SPI
//#define SPI_OLED_DMA
#if defined(SPI_OLED_DMA)
    init_spi1();
    spi1_init_oled();
    spi1_setup_dma();
    spi1_enable_dma();
#endif

    // Uncomment when you are ready to generate a code.
    //autotest();

    // Game on!  The goal is to score 100 points.
    internal_clock();
    enable_ports();
    DHT22_Init();
    TIM6_Init();

    int temperature = 0, humidity = 0;
    while (1) {
        DHT22_GetData(&temperature, &humidity); // Attempt to read data from DHT22

        // Toggle LED to show that the loop is continuing
        GPIOC->ODR ^= GPIO_ODR_8;
        nano_wait(1000000); // 1-second delay to make blinking visible
    }
}
