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
const char* username = "htetz";

/*******************************************************************************/ 

#include "stm32f0xx.h"
#include <stdint.h>

void internal_clock();

// Uncomment only one of the following to test each step
// #define STEP1
// #define STEP2
// #define STEP3
// #define STEP4
#define SHELL

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

#ifdef STEP1
int main(void){
    internal_clock();
    init_usart5();
    for(;;) {
        while (!(USART5->ISR & USART_ISR_RXNE)) { }
        char c = USART5->RDR;
        while(!(USART5->ISR & USART_ISR_TXE)) { }
        USART5->TDR = c;
    }
}
#endif

#ifdef STEP2
#include <stdio.h>

// TODO Resolve the echo and carriage-return problem

int __io_putchar(int c) {
    // TODO
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
    while (!(USART5->ISR & USART_ISR_RXNE));
    char c = USART5->RDR;
    if(c == '\r') {
        c = '\n';
    }
    c = __io_putchar(c);
    return c;
}

int main() {
    internal_clock();
    init_usart5();
    setbuf(stdin,0);
    setbuf(stdout,0);
    setbuf(stderr,0);
    printf("Enter your name: ");
    char name[80];
    fgets(name, 80, stdin);
    printf("Your name is %s", name);
    printf("Type any characters.\n");
    for(;;) {
        char c = getchar();
        putchar(c);
    }
}
#endif

#ifdef STEP3
#include <stdio.h>
#include "fifo.h"
#include "tty.h"
int __io_putchar(int c) {
    // TODO
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
    int c = line_buffer_getchar();
    return c;
}

int main() {
    internal_clock();
    init_usart5();
    setbuf(stdin,0);
    setbuf(stdout,0);
    setbuf(stderr,0);
    printf("Enter your name: ");
    char name[80];
    fgets(name, 80, stdin);
    printf("Your name is %s", name);
    printf("Type any characters.\n");
    for(;;) {
        char c = getchar();
        putchar(c);
    }
}
#endif

#ifdef STEP4

#include <stdio.h>
#include "fifo.h"
#include "tty.h"

// TODO DMA data structures
#define FIFOSIZE 16
char serfifo[FIFOSIZE];
int seroffset = 0;

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
// TODO Remember to look up for the proper name of the ISR function

/*



*/




int main() {
    internal_clock();
    init_usart5();
    enable_tty_interrupt();

    setbuf(stdin,0); // These turn off buffering; more efficient, but makes it hard to explain why first 1023 characters not dispalyed
    setbuf(stdout,0);
    setbuf(stderr,0);
    printf("Enter your name: "); // Types name but shouldn't echo the characters; USE CTRL-J to finish
    char name[80];
    fgets(name, 80, stdin);
    printf("Your name is %s", name);
    printf("Type any characters.\n"); // After, will type TWO instead of ONE
    for(;;) {
        char c = getchar();
        putchar(c);
    }
}
#endif





#ifdef SHELL
#include "commands.h"
#include <stdio.h>

#include <stdio.h>
#include "fifo.h"
#include "tty.h"

// TODO DMA data structures
#define FIFOSIZE 16
char serfifo[FIFOSIZE];
int seroffset = 0;

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

int main() {
    internal_clock();
    init_usart5();
    enable_tty_interrupt();
    setbuf(stdin,0);
    setbuf(stdout,0);
    setbuf(stderr,0);
    command_shell();
}
#endif