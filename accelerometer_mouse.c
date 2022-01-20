#include <delay.h>
#include <gpio.h>
#include <stdio.h>
#include <stdlib.h>
#include <stm32.h>
#include <string.h>

/************************* DEBUG *************************/

void debug(char c) {
  if (USART2->SR & USART_SR_TXE) {
    USART2->DR = c;
  }
}

/****************** CONFIG CONSTANTS *********************/

#define VERY_HIGH_IRQ_PRIO 0U
#define HIGH_IRQ_PRIO 1U
#define MIDDLE_IRQ_PRIO 2U
#define LOW_IRQ_PRIO 3U

#define VERY_HIGH_IRQ_SUBPRIO 0U
#define HIGH_IRQ_SUBPRIO 1U
#define MIDDLE_IRQ_SUBPRIO 2U
#define LOW_IRQ_SUBPRIO 3U

#define USART_Mode_Rx_Tx (USART_CR1_RE | USART_CR1_TE)
#define USART_Enable USART_CR1_UE

#define USART_WordLength_8b 0x0000
#define USART_WordLength_9b USART_CR1_M

#define USART_Parity_No 0x0000
#define USART_Parity_Even USART_CR1_PCE
#define USART_Parity_Odd (USART_CR1_PCE | USART_CR1_PS)

#define USART_StopBits_1 0x0000
#define USART_StopBits_0_5 0x1000
#define USART_StopBits_2 0x2000
#define USART_StopBits_1_5 0x3000

#define USART_FlowControl_None 0x0000
#define USART_FlowControl_RTS USART_CR3_RTSE
#define USART_FlowControl_CTS USART_CR3_CTSE

#define HSI_HZ 16000000U
#define PCLK1_HZ HSI_HZ
#define BAUD_RATE 9600U

#define I2C_SPEED_HZ 100000
#define PCLK1_MHZ 16

/*********************** PRIORITIES **********************/

#define PRIO_BITS 2
#define PRIORITY_GROUP (7 - PRIO_BITS)

static inline void IRQsetPriority(IRQn_Type irq, uint32_t prio,
                                  uint32_t subprio) {
  NVIC_SetPriority(
      irq, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), prio, subprio));
}

// DMA exception priorities
#define DMA_IRQ_PRIO VERY_HIGH_IRQ_PRIO
#define DMA_IRQ_SUBPRIO VERY_HIGH_IRQ_SUBPRIO

#define IRQsetDMAPriority(STREAM) \
  IRQsetPriority(STREAM, DMA_IRQ_PRIO, DMA_IRQ_SUBPRIO)

// Timer exception priorities
#define TIMER_IRQ_PRIO VERY_HIGH_IRQ_PRIO
#define TIMER_IRQ_SUBPRIO HIGH_IRQ_SUBPRIO

#define IRQsetTimerPriority(TIMER) \
  IRQsetPriority(TIMER, TIMER_IRQ_PRIO, TIMER_IRQ_SUBPRIO)

/******************** OUTPUT_BUFFER **********************/

#define BUFFER_SIZE 256

typedef struct {
  char* buf[BUFFER_SIZE];
  int start;
  int end;
} OutputBuffer;

OutputBuffer output_buffer;

int output_buffer_empty() { return output_buffer.end == output_buffer.start; }

int output_buffer_full() {
  return (output_buffer.end + 1) % BUFFER_SIZE == output_buffer.start;
}

char* output_buffer_get() {
  if (output_buffer_empty()) {
    return "";
  } else {
    char* res = output_buffer.buf[output_buffer.start];
    output_buffer.start = (output_buffer.start + 1) % BUFFER_SIZE;
    return res;
  }
}

void output_buffer_put(char* word) {
  if (!output_buffer_full()) {
    output_buffer.buf[output_buffer.end] = word;
    output_buffer.end = (output_buffer.end + 1) % BUFFER_SIZE;
  }
}

/******************** TRANSMISSION ***********************/

void continue_transmission() {
  if (output_buffer_empty()) {
    return;
  }

  char* word = output_buffer_get();

  DMA1_Stream6->M0AR = (uint32_t)word;
  DMA1_Stream6->NDTR = strlen(word);
  DMA1_Stream6->CR |= DMA_SxCR_EN;
}

void start_transmission() {
  if ((DMA1_Stream6->CR & DMA_SxCR_EN) == 0 &&
      (DMA1->HISR & DMA_HISR_TCIF6) == 0) {
    continue_transmission();
  }
}

/*********************** PRINTING ************************/

void print(char* word) {
  output_buffer_put(word);
  start_transmission();
}

/******************* ACCELEROMETER ***********************/

#define LIS35DE_ADDR 0x1C  // or 0x1D

#define CTRL_REG1 0x20
#define CTRL_REG1_DEF 0x07
#define CTRL_REG1_PD 0x40  // Active mode

#define OUT_X 0x29
#define OUT_Y 0x2B
#define OUT_Z 0x2D

void write_to_accelerometer_register(uint8_t reg, uint8_t value) {
  I2C1->CR1 |= I2C_CR1_START;

  while (!(I2C1->SR1 & I2C_SR1_SB)) {
  }

  I2C1->DR = LIS35DE_ADDR << 1;

  while (!(I2C1->SR1 & I2C_SR1_ADDR)) {
  }

  I2C1->SR2;

  I2C1->DR = reg;

  while (!(I2C1->SR1 & I2C_SR1_TXE)) {
  }

  I2C1->DR = value;

  while (!(I2C1->SR1 & I2C_SR1_BTF)) {
  }

  I2C1->CR1 |= I2C_CR1_STOP;
}

uint8_t read_accelerometer_register(uint8_t reg) {
  I2C1->CR1 |= I2C_CR1_START;

  while (!(I2C1->SR1 & I2C_SR1_SB)) {
  }

  I2C1->DR = LIS35DE_ADDR << 1;
  // print("b");
  while (!(I2C1->SR1 & I2C_SR1_ADDR)) {
  }

  I2C1->SR2;

  I2C1->DR = reg;
  // print("c");
  while (!(I2C1->SR1 & I2C_SR1_BTF)) {
  }

  I2C1->CR1 |= I2C_CR1_START;
  // print("d");
  while (!(I2C1->SR1 & I2C_SR1_SB)) {
  }

  I2C1->DR = LIS35DE_ADDR << 1 | 1;

  I2C1->CR1 &= ~I2C_CR1_ACK;
  // print("e");
  while (!(I2C1->SR1 & I2C_SR1_ADDR)) {
  }

  I2C1->SR2;

  I2C1->CR1 |= I2C_CR1_STOP;
  // print("f");
  while (!(I2C1->SR1 & I2C_SR1_RXNE)) {
  }
  // print("g");
  uint8_t value = I2C1->DR;

  return value;
}

void activate_accelerometer() {
  write_to_accelerometer_register(CTRL_REG1, CTRL_REG1_DEF | CTRL_REG1_PD);
}

void print_coords() {
  uint8_t x = read_accelerometer_register(OUT_X);
  uint8_t y = read_accelerometer_register(OUT_Y);
  uint8_t z = read_accelerometer_register(OUT_Z);

  char x_s[4];
  char y_s[4];
  char z_s[4];

  itoa(x, x_s, 10);
  itoa(y, y_s, 10);
  itoa(z, z_s, 10);

  output_buffer_put("(");
  output_buffer_put(x_s);
  output_buffer_put(", ");
  output_buffer_put(y_s);
  output_buffer_put(", ");
  output_buffer_put(z_s);
  output_buffer_put(")");
  start_transmission();
}

/********************* DMA_HANDLER ***********************/

void DMA1_Stream6_IRQHandler() {
  uint32_t isr = DMA1->HISR;
  if (isr & DMA_HISR_TCIF6) {
    DMA1->HIFCR = DMA_HIFCR_CTCIF6;

    continue_transmission();
  }
}

/******************** TIMER_HANDLER **********************/

void TIM3_IRQHandler(void) {
  uint32_t it_status = TIM3->SR & TIM3->DIER;
  if (it_status & TIM_SR_UIF) {
    TIM3->SR = ~TIM_SR_UIF;

    print_coords();
  }
  if (it_status & TIM_SR_CC1IF) {
    TIM3->SR = ~TIM_SR_CC1IF;
  }
}

/******************* CONFIGURATION ***********************/

void configure_usart_and_dma() {
  // Enable the clocks
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_DMA1EN;
  RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

  // Configure GPIOA
  GPIOafConfigure(GPIOA, 2, GPIO_OType_PP, GPIO_Fast_Speed, GPIO_PuPd_NOPULL,
                  GPIO_AF_USART2);

  // Configure USART
  USART2->CR1 = USART_CR1_TE | USART_WordLength_8b | USART_Parity_No;
  USART2->CR2 = USART_StopBits_1;
  USART2->CR3 = USART_CR3_DMAT;
  USART2->BRR = (PCLK1_HZ + (BAUD_RATE / 2U)) / BAUD_RATE;

  // Configure DMA
  DMA1_Stream6->CR =
      4U << 25 | DMA_SxCR_PL_1 | DMA_SxCR_MINC | DMA_SxCR_DIR_0 | DMA_SxCR_TCIE;
  DMA1_Stream6->PAR = (uint32_t)&USART2->DR;

  // Clear the DMA exception pointer
  DMA1->HIFCR = DMA_HIFCR_CTCIF6;

  // Set DMA exception priority
  IRQsetDMAPriority(DMA1_Stream6_IRQn);

  // Enable DMA exception
  NVIC_EnableIRQ(DMA1_Stream6_IRQn);

  // Enable USART
  USART2->CR1 |= USART_Enable;
}

void configure_timer() {
  // Enable the clock
  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

  // Configure timer constants
  TIM3->CR1 = TIM_CR1_URS;
  TIM3->PSC = 16000 - 1;  // One tick every millisecond
  TIM3->ARR = 1000;       // Break between exceptions in milliseconds
  TIM3->EGR = TIM_EGR_UG;

  TIM3->CNT = 0;

  // Exceptions
  TIM3->SR = ~(TIM_SR_UIF | TIM_SR_CC1IF);
  TIM3->DIER = TIM_DIER_UIE | TIM_DIER_CC1IE;

  // Set timer exception priority
  IRQsetTimerPriority(TIM3_IRQn);

  // Enable timer exception
  NVIC_EnableIRQ(TIM3_IRQn);

  // Enable timer
  TIM3->CR1 |= TIM_CR1_CEN;
}

void configure_I2C() {
  // Enable the clocks
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
  RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

  // Configure GPIO
  GPIOafConfigure(GPIOB, 8, GPIO_OType_OD, GPIO_Low_Speed, GPIO_PuPd_NOPULL,
                  GPIO_AF_I2C1);
  GPIOafConfigure(GPIOB, 9, GPIO_OType_OD, GPIO_Low_Speed, GPIO_PuPd_NOPULL,
                  GPIO_AF_I2C1);

  // Base version
  I2C1->CR1 = 0;

  // Configure clock frequency
  I2C1->CCR = (PCLK1_MHZ * 1000000) / (I2C_SPEED_HZ << 1);
  I2C1->CR2 = PCLK1_MHZ;
  I2C1->TRISE = PCLK1_MHZ + 1;

  // Enable I2C
  I2C1->CR1 |= I2C_CR1_PE;
}

void configure() {
  // Set priority grouping
  NVIC_SetPriorityGrouping(PRIORITY_GROUP);

  configure_usart_and_dma();
  configure_timer();
  configure_I2C();
  activate_accelerometer();
}

/*********************************************************/

int main() {
  configure();

  for (;;) {
  }
}
