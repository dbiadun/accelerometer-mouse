#include <delay.h>
#include <gpio.h>
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

/********************* DMA_HANDLERS **********************/

void DMA1_Stream6_IRQHandler() {
  uint32_t isr = DMA1->HISR;
  if (isr & DMA_HISR_TCIF6) {
    DMA1->HIFCR = DMA_HIFCR_CTCIF6;

    continue_transmission();
  }
}

/******************* CONFIGURATION ***********************/

void configure() {
  // Enable the clocks
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_DMA1EN;
  RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

  __NOP();

  // Set priority grouping
  NVIC_SetPriorityGrouping(PRIORITY_GROUP);

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

  // Set the DMA exception priority
  IRQsetDMAPriority(DMA1_Stream6_IRQn);

  // Enable the DMA exception
  NVIC_EnableIRQ(DMA1_Stream6_IRQn);

  // Enable USART
  USART2->CR1 |= USART_Enable;
}

/*********************************************************/

int main() {
  configure();

  for (;;) {
  }
}
