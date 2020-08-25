/******************************************************************************/
#if !defined(INCLUDE)
#define __MONITOR__
#include "stm32f1xx_hal.h"

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <stdbool.h>
#include <limits.h>
#include <stdarg.h>

#include "serialio.h"

#include "adc.h"

#ifdef EXT
#undef EXT
#endif

#define EXT
#include "monitor.h"
#endif /* INCLUDE */

#if defined(__MONITOR_INC__)	// <-

#if !defined(EXT)
#define EXT extern
#endif

#include "main.h"
#include "config.h"
#include "dbg.h"

unsigned int millis(void);

void adcRead();
void adcStatus();

typedef struct
{
 union
 {
  struct
  {
   char port;
   char num;
  };
  struct
  {
   uint16_t pinName;
  };
 };
} T_PIN_NAME;

T_PIN_NAME pinName(char *buf, GPIO_TypeDef *port, int pin);
char *gpioStr(char *buf, int size, T_PIN_NAME *pinInfo);
void gpioInfo(GPIO_TypeDef *gpio);
void tmrInfo(TIM_TypeDef *tmr);
void extiInfo(void);
void usartInfo(USART_TypeDef *usart, const char *str);
void i2cInfo(I2C_TypeDef *i2c, const char *str);
void adcInfo(ADC_TypeDef *adc, char n);
void dmaInfo(DMA_TypeDef *dma);
void dmaChannelInfo(DMA_Channel_TypeDef *dmaC, char n);

char portName(GPIO_TypeDef *port);
char timNum(TIM_TypeDef *tmr);

typedef union
{
 struct
 {
  unsigned b0:1;
  unsigned b1:1;
  unsigned b2:1;
  unsigned b3:1;
  unsigned b4:1;
  unsigned b5:1;
  unsigned b6:1;
  unsigned b7:1;
  unsigned b8:1;
  unsigned b9:1;
  unsigned b10:1;
  unsigned b11:1;
  unsigned b12:1;
  unsigned b13:1;
  unsigned b14:1;
  unsigned b15:1;
  unsigned b16:1;
  unsigned b17:1;
  unsigned b18:1;
  unsigned b19:1;
  unsigned b20:1;
  unsigned b21:1;
  unsigned b22:1;
  unsigned b23:1;
  unsigned b24:1;
  unsigned b25:1;
  unsigned b26:1;
  unsigned b27:1;
  unsigned b28:1;
  unsigned b29:1;
  unsigned b30:1;
  unsigned b31:1;
 };
 struct
 {
  int w;
 };
} BITWORD;

#endif	// ->
#ifdef __MONITOR__

extern uint32_t uwTick;

unsigned int millis(void)
{
 return((unsigned int) uwTick);
}

uint32_t buf[8];

void adcRead()
{
 HAL_StatusTypeDef status;
 
 adcInfo(ADC1, 1);
 adcInfo(ADC2, 2);
 dmaInfo(DMA1);
 dmaChannelInfo(DMA1_Channel1, 1);
 
 uint32_t tmp = ADC1->SQR3;
 printf("0 - %d 1 - %d\n", (int) (tmp & 0x1f), (int) ((tmp >> 5) & 0x1f));
 // printf("SQR1 %08x SQR2 %08x SQR3 %08x\n",
 //	(unsigned int) ADC1->SQR1, (unsigned int) ADC1->SQR2,
 //	(unsigned int) ADC1->SQR3);

#if 0
 ADC2->CR2 &= ~ADC_CR2_EXTSEL_Msk;
 ADC2->CR2 |= ADC_CR2_EXTSEL_2 | ADC_CR2_EXTSEL_1 | ADC_CR2_EXTSEL_0 | ADC_CR2_ADON;

 adcInfo(ADC2, 2);

 status = HAL_ADC_Start(&hadc1);
 printf("start status %d\n", status);
 status = HAL_ADC_PollForConversion(&hadc1, 100);
 printf("poll status %d\n", status);
#else
 memset(buf, 0, sizeof(buf));
 uint32_t *p;
 unsigned int count;
 p = buf;
 count = sizeof(buf) / sizeof(uint32_t);
 while (1)
 {
  printf("%08x ", (unsigned int) *p++);
  count -= 1;
  if (count == 0)
  {
   printf("\n");
   break;
  }
 }
 status = HAL_ADCEx_MultiModeStart_DMA(&hadc1, buf, 2);
 printf("dma status %d\n", status);
 p = buf;
 count = sizeof(buf) / sizeof(uint32_t);
 while (1)
 {
  printf("%08x ", (unsigned int) *p++);
  count -= 1;
  if (count == 0)
  {
   printf("\n");
   break;
  }
 }
 printf("buf %8x %8x\n", (unsigned int) buf[0], (unsigned int) buf[1]);
#endif

 adcInfo(ADC1, 1);
 adcInfo(ADC2, 2);
 dmaInfo(DMA1);
 dmaChannelInfo(DMA1_Channel1, 1);
}

void adcStatus()
{
 printf("%8x buf %8x %8x\n",
	(unsigned int) buf, (unsigned int) buf[0], (unsigned int) buf[1]);
 newline();
 adcInfo(ADC1, 1);
 newline();
 adcInfo(ADC2, 2);
 newline();
 newline();
 dmaInfo(DMA1);
 newline();
 dmaChannelInfo(DMA1_Channel1, 1);
}

typedef struct
{
 GPIO_TypeDef *port;
 char name;
} T_GPIO, *P_GPIO;

T_GPIO gpio[] = 
{
 {GPIOA, 'A'},
 {GPIOB, 'B'},
 {GPIOC, 'C'},
 {GPIOD, 'D'},
 {GPIOE, 'E'},
};

char portName(GPIO_TypeDef *port)
{
 for (unsigned int j = 0; j < sizeof(gpio) / sizeof(T_GPIO); j++)
 {
  if (port == gpio[j].port)
  {
   return(gpio[j].name);
  }
 }
 return('*');
}

T_PIN_NAME pinName(char *buf, GPIO_TypeDef *port, int pin)
{
 char pName = portName(port);
 T_PIN_NAME val;
 val.port = pName;
 int pinNum = 0;
 while (pin != 0)
 {
  if (pin & 1)
   break;
  pin >>= 1;
  pinNum++;
 }
 sprintf(buf, "P%c%d", pName, pinNum);
 val.num = pinNum;
 return(val);
}

const char modeInfo[] = {'I', 'O', 'F', 'A'};
const char *typeInfo[] = {"PP", "OD", "  "};
const char *speedInfo[] = {"LS", "MS", "HS", "VH", "  "};
const char *pupdInfo[] = {"  ", "PU", "PD", "**"};

char *gpioStr(char *buf, int size, T_PIN_NAME *pinInfo)
{
 buf[0] = 0;
 for (unsigned int j = 0; j < sizeof(gpio) / sizeof(T_GPIO); j++)
 {
  if (gpio[j].name == pinInfo->port)
  {
#if 0
   GPIO_TypeDef *port = gpio[j].port;
   int pin = pinInfo->num;
//   printf("port  %08x %2d %c %2d\n", (unsigned int) port, pin,
//	  pinInfo->port, pinInfo->num);

   int mode = (port->MODER >> (pin << 1)) & 3;
//   printf("mode  %08x %d\n", (unsigned int) port->MODER, mode);

   int outType = (port->OTYPER >> pin) & 1;
//   printf("type  %08x %d\n", (unsigned int) port->OTYPER, outType);

   int outSpeed = (port->OSPEEDR >> (pin << 1)) & 3;
//   printf("speed %08x %d\n", (unsigned int) port->OSPEEDR, outSpeed);

   int pupd = (port->PUPDR >> (pin << 1)) & 3;
//   printf("pupd  %08x %d\n", (unsigned int) port->PUPDR, pupd);

   int afr = (port->AFR[pin >> 3] >> ((pin << 2) & 0x1f)) & 0xf;
   char interrupt = ' ';
   if (mode == GPIO_MODE_INPUT)
   {
    outType = (sizeof(typeInfo) / sizeof(char *)) - 1;
    outSpeed = (sizeof(speedInfo) / sizeof(char *)) - 1;

    if ((EXTI->IMR >> pin) & 1)
    {
     int exti = (SYSCFG->EXTICR[pin >> 2] >> ((pin << 2) & 0xf)) & 0xf;
     if ((pinInfo->port - 'A') == exti)
      interrupt = 'I';
//     printf("exti %2d pinInfo->port - 'A' %d pin >> 2 %d pin << 2 %d\n",
//	    exti, pinInfo->port - 'A', pin >> 2, pin << 2);
    }
   }

//   printf("afr   %08x %d (pin >> 3) %d ((pin << 2) & 0x1f) %2d\n",
//	  (unsigned int) port->AFR[pin >> 3], afr,
//	  (pin >> 3), ((pin << 2) & 0x1f));
//   flushBuf();

   snprintf(buf, size, "%c %c %2s %2s %2s %2d",
	    interrupt, modeInfo[mode], typeInfo[outType],
	    speedInfo[outSpeed], pupdInfo[pupd], afr);
#else
   snprintf(buf, size, "\n");
#endif
   break;
  }
 }
 return(buf);
}

void gpioInfo(GPIO_TypeDef *gpio)
{
 printf("gpio %x %c\n",(unsigned int) gpio, portName(gpio));
#if 0
 printf("MODER   %8x ",(unsigned int) gpio->MODER);
 printf("OTYPER  %8x\n",(unsigned int) gpio->OTYPER);
 printf("OSPEEDR %8x ",(unsigned int) gpio->OSPEEDR);
 printf("PUPDR   %8x\n",(unsigned int) gpio->PUPDR);
#else
 printf("CRL    %8x ",(unsigned int) gpio->CRL);
 printf("CRH    %8x\n",(unsigned int) gpio->CRH);
#endif
 printf("IDR     %8x ",(unsigned int) gpio->IDR);
 printf("ODR     %8x\n",(unsigned int) gpio->ODR);
 printf("BSRR    %8x ",(unsigned int) gpio->BSRR);
 printf("LCKR    %8x\n",(unsigned int) gpio->LCKR);
#if 0
 printf("AFR[0]  %8x ",(unsigned int) gpio->AFR[0]);
 printf("AFR[1]  %8x\n",(unsigned int) gpio->AFR[1]);
#endif
 int i;
 printf("         ");
 for (i = 0; i < 16; i++)
  printf(" %2d", i);

 int val;
 
#if 0
 printf("\nmoder    ");
 val = gpio->MODER;
 for (i = 0; i < 16; i++)
  printf(" %2d", (val >> (2 * i)) & 0x3);

 printf("\notyper   ");
 val = gpio->OTYPER;
 for (i = 0; i < 16; i++)
  printf(" %2d", (val >> i) & 0x1);

 printf("\nopspeedr ");
 val = gpio->OSPEEDR;
 for (i = 0; i < 16; i++)
  printf(" %2d", (val >> (2 * i)) & 0x3);

 printf("\npupdr    ");
 val = gpio->PUPDR;
 for (i = 0; i < 16; i++)
  printf(" %2d", (val >> (2 * i)) & 0x3);
#else
 printf("\nmode     ");
 val = gpio->CRL;
 for (i = 0; i < 8; i++)
  printf(" %2d", (val >> (4 * i)) & 0x3);

 val = gpio->CRH;
 for (i = 0; i < 8; i++)
  printf(" %2d", (val >> (4 * i)) & 0x3);

 printf("\ncnf      ");
 val = gpio->CRL;
 for (i = 0; i < 8; i++)
  printf(" %2d", (val >> ((4 * i) + 2)) & 0x3);

 val = gpio->CRH;
 for (i = 0; i < 8; i++)
  printf(" %2d", (val >> ((4 * i) + 2)) & 0x3);
#endif

 printf("\nidr      ");
 val = gpio->IDR;
 for (i = 0; i < 16; i++)
  printf(" %2d", (val >> i) & 0x1);

 printf("\nodr      ");
 val = gpio->ODR;
 for (i = 0; i < 16; i++)
  printf(" %2d", (val >> i) & 0x1);

#if 0
 printf("\nafr      ");
 val = gpio->AFR[0];
 for (i = 0; i < 8; i++)
  printf(" %2d", (val >> (4 * i)) & 0xf);
 val = gpio->AFR[1];
 for (i = 0; i < 8; i++)
  printf(" %2d", (val >> (4 * i)) & 0xf);
#endif
 printf("\n");
 flushBuf();
}

typedef struct
{
 TIM_TypeDef *tmr;
 char num;
} T_TIM, *P_TIM;

T_TIM tim[] =
{
 {TIM1,  1},
 {TIM2,  2},
 {TIM3,  3},
 {TIM4,  4},
#if 0
 {TIM5,  5},
#ifdef TIM6
 {TIM6,  6},
#endif
#ifdef TIM7
 {TIM7,  7},
#endif
#ifdef TIM8
 {TIM8,  8},
#endif
 {TIM9,  9},
 {TIM10, 10},
 {TIM11, 11},
#ifdef TIM12
 {TIM12, 12},
#endif
#ifdef TIM13
 {TIM13, 13},
#endif
#ifdef TIM14
 {TIM14, 14},
#endif
#endif
};

char timNum(TIM_TypeDef *tmr)
{
 for (unsigned int j = 0; j < sizeof(tim) / sizeof(T_TIM); j++)
 {
  if (tmr == tim[j].tmr)
  {
   return(tim[j].num);
  }
 }
 return(0);
}

void tmrInfo(TIM_TypeDef *tmr)
{
 printf("tmr %x TIM%d\n",(unsigned int) tmr, timNum(tmr));
 printf("CR1   %8x ",(unsigned int) tmr->CR1);
 printf("CR2   %8x\n",(unsigned int) tmr->CR2);
 printf("SMCR  %8x ",(unsigned int) tmr->SMCR);
 printf("DIER  %8x\n",(unsigned int) tmr->DIER);
 printf("SR    %8x ",(unsigned int) tmr->SR);
 printf("EGR   %8x\n",(unsigned int) tmr->EGR);
 printf("CCMR1 %8x ",(unsigned int) tmr->CCMR1);
 printf("CCMR2 %8x\n",(unsigned int) tmr->CCMR2);
 printf("CCER  %8x ",(unsigned int) tmr->CCER);
 printf("CNT   %8x\n",(unsigned int) tmr->CNT);
 printf("PSC   %8x ",(unsigned int) tmr->PSC);
 printf("ARR   %8x\n",(unsigned int) tmr->ARR);
 printf("RCR   %8x ",(unsigned int) tmr->RCR);
 printf("CCR1  %8x\n",(unsigned int) tmr->CCR1);
 printf("CCR2  %8x ",(unsigned int) tmr->CCR2);
 printf("CCR3  %8x\n",(unsigned int) tmr->CCR3);
 printf("CCR4  %8x ",(unsigned int) tmr->CCR4);
 printf("BDTR  %8x\n",(unsigned int) tmr->BDTR);
 printf("DCR   %8x ",(unsigned int) tmr->DCR);
 printf("OR    %8x\n",(unsigned int) tmr->OR);
 flushBuf();
}

void extiInfo(void)
{
 printf("EXTI %x\n",(unsigned int) EXTI);
 int i;
 printf("      ");
 for (i = 0; i <= 22; i++)
  printf(" %2d", i);

 printf("\nIMR   ");
 int val = EXTI->IMR;
 for (i = 0; i <= 22; i++)
  printf(" %2d", (val >> i) & 0x1);

 printf("\nEMR   ");
 val = EXTI->EMR;
 for (i = 0; i <= 22; i++)
  printf(" %2d", (val >> i) & 0x1);

 printf("\nRTSR  ");
 val = EXTI->RTSR;
 for (i = 0; i <= 22; i++)
  printf(" %2d", (val >> i) & 0x1);

 printf("\nFTSR  ");
 val = EXTI->FTSR;
 for (i = 0; i <= 22; i++)
  printf(" %2d", (val >> i) & 0x1);

 printf("\nSWIER ");
 val = EXTI->SWIER;
 for (i = 0; i <= 22; i++)
  printf(" %2d", (val >> i) & 0x1);

 printf("\nPR    ");
 val = EXTI->PR;
 for (i = 0; i <= 22; i++)
  printf(" %2d", (val >> i) & 0x1);

#if 0
 printf("\nSYSCFG %x\n",(unsigned int) SYSCFG);
 printf("      ");
 for (i = 0; i < 16; i++)
  printf(" %2d", i);
#endif

#if 0
 printf("\nEXTICR");
 int mask = EXTI->IMR;
 for (i = 0; i < 4; i++)
 {
  val = SYSCFG->EXTICR[i];
  int j;
  for (j = 0; j < 4; j++)
  {
   printf("  %c", (mask & 1) ? 'a' + ((val >> (4 * j)) & 0xf) : ' ');
   mask >>= 1;
  }
 }
#endif
 printf("\n");
 flushBuf();
}

void usartInfo(USART_TypeDef *usart, const char *str)
{
 printf("usart %x %s\n",(unsigned int) usart, str);
 printf("SR   %8x ",(unsigned int) usart->SR);
 printf("DR   %8x\n",(unsigned int) usart->DR);
 printf("BRR  %8x ",(unsigned int) usart->BRR);
 printf("CR1  %8x\n",(unsigned int) usart->CR1);
 printf("CR2  %8x ",(unsigned int) usart->CR2);
 printf("CR3  %8x\n",(unsigned int) usart->CR3);
 printf("GTPR %8x\n",(unsigned int) usart->GTPR);
 flushBuf();
}

void i2cInfo(I2C_TypeDef *i2c, const char *str)
{
 printf("i2c %x %s\n",(unsigned int) i2c, str);
 printf("CR1   %8x ",(unsigned int) i2c->CR1);
 printf("CR2   %8x\n",(unsigned int) i2c->CR2);
 printf("OAR1  %8x ",(unsigned int) i2c->OAR1);
 printf("OAR2  %8x\n",(unsigned int) i2c->OAR2);
 printf("SR1   %8x ",(unsigned int) i2c->SR1);
 printf("SR2   %8x\n",(unsigned int) i2c->SR2);
 printf("DR    %8x ",(unsigned int) i2c->DR);
 printf("CCR   %8x\n",(unsigned int) i2c->CCR);
 printf("TRISE %8x\n",(unsigned int) i2c->TRISE);
 flushBuf();
}

void adcInfo(ADC_TypeDef *adc, char n)
{
 printf("ADC%d %x  DR %0x8\n",
	n, (unsigned int) adc, (unsigned int) &adc->DR);
 printf("SR    %8x\n",(unsigned int) adc->SR);
 printf("CR1   %8x ",(unsigned int) adc->CR1);
 printf("CR2   %8x\n",(unsigned int) adc->CR2);
 printf("HTR   %8x ",(unsigned int) adc->HTR);
 printf("LTR   %8x\n",(unsigned int) adc->LTR);
 printf("L     %8x ",(unsigned int) ((adc->SQR1 >> 20) & 0xf));
 printf("DR    %8x\n",(unsigned int) adc->DR);
 int i;
 printf("     ");
 for (i = 0; i < 16; i++)
  printf(" %2d", i);
 printf("\n");

 printf("SMPR ");
 int32_t tmp = adc->SMPR2;
 for (i = 0; i < 10; i++)
 {
  printf(" %2u", (unsigned int) (tmp & 7));
  tmp >>= 3;
 }
 tmp = adc->SMPR1;
 for (i = 0; i < 6; i++)
 {
  printf(" %2u", (unsigned int) (tmp & 7));
  tmp >>= 3;
 }
 printf("\n");

 printf("SQR  ");
 tmp = adc->SQR3;
 for (i = 0; i < 6; i++)
 {
  printf(" %2u", (unsigned int) (tmp & 7));
  tmp >>= 5;
 }
 tmp = adc->SQR2;
 for (i = 0; i < 6; i++)
 {
  printf(" %2u", (unsigned int) (tmp & 7));
  tmp >>= 5;
 }
 tmp = adc->SQR1;
 for (i = 0; i < 4; i++)
 {
  printf(" %2u", (unsigned int) (tmp & 7));
  tmp >>= 5;
 }
 printf("\n");
 flushBuf();
}

void dmaInfo(DMA_TypeDef *dma)
{
 printf("DMA1 %08x\n", (unsigned int) dma);
 printf("ISR   %8x ",(unsigned int) dma->ISR);
 printf("IFCR  %8x\n",(unsigned int) dma->IFCR);
 flushBuf();
}

void dmaChannelInfo(DMA_Channel_TypeDef *dmaC, char n)
{
 printf("DMA_Channel%d %08x\n", n, (unsigned int) dmaC);
 printf("CCR   %8x ",(unsigned int) dmaC->CCR);
 printf("CNDTR %8x\n",(unsigned int) dmaC->CNDTR);
 printf("CPAR  %8x ",(unsigned int) dmaC->CPAR);
 printf("CMAR  %8x\n",(unsigned int) dmaC->CMAR);
 flushBuf();
}

#endif
