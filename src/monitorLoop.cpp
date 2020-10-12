//******************************************************************************
#if defined(STM32F1)
#include "stm32f1xx_hal.h"
#endif
#if defined(STM32F3)
#include "stm32f3xx_hal.h"
#endif
#if defined(STM32F4)
#include "stm32f4xx_hal.h"
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <limits.h>

#include "config.h"
#include "adc.h"
#include "serialio.h"
#include "lclcmd.h"
#include "remcmd.h"

#define EXT extern
#include "current.h"

void monitorLoopSetup(void);
extern "C" int16_t monitorLoop(void);
extern "C" void hard_fault_handler_c (unsigned int * hardfault_args);

uint32_t runTime;
#define RUN_TIMEOUT 5000

#if PIN_DISPLAY
void pinDisplay(void);

typedef struct
{
 char *name;
 GPIO_TypeDef *port;
 int pin;
} T_PINDEF, *P_PINDEF;

#define PIN(name, pin) {#name, pin ## _GPIO_Port, pin ## _Pin}
#define DBGPIN(name) {#name, name ## _GPIO_Port, name ## _Pin}

T_PINDEF pinDef[] =
{
#include "dbgPin.h"
};

#endif

#define DATA_SIZE 0

#if DATA_SIZE

extern "C" unsigned int getSP(void);

extern char __bss_start__;
extern char __bss_end__;
extern char __data_start__;
extern char __data_end__;
extern char __stack;
extern char __Main_Stack_Limit;
#endif

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

void monitorLoopSetup(void)
{
 flushBuf();

 /* initialize timer 11 as index timer */

#if 0

 unsigned int tmrClkFreq = HAL_RCC_GetPCLK2Freq() * 2;

 indexTmrMax(65536);

 #ifdef USEC_SHARED_INDEX
 indexTmrScl((tmrClkFreq / 1000000U) - 1); /* load scaler */
 idxFreq = 1000000U;
 #else
 indexTmrScl(0);
 idxFreq = tmrClkFreq;
 #endif

 idxTrkFreq = idxFreq * 6;
 indexTmrClrIF();
 indexTmrSetIE();
 indexTmrStart();
#endif
}

#define LED_DELAY 500

#define PWR_INTERVAL (500)

int16_t monitorLoop(void)
{
 uint32_t ledUpdTime;
 uint32_t pwrUpdTime;
 unsigned char ch;
 uint32_t extInt[] =
 {
  EXTI0_IRQn,
  EXTI1_IRQn,
//  EXTI2_IRQn,
  EXTI3_IRQn,
  EXTI4_IRQn,
  EXTI9_5_IRQn,
  EXTI15_10_IRQn
 };

 putstr("polled output starting\n");

#if 0
 DBGMCU->APB1FZ = DBGMCU_APB1_FZ_DBG_IWDG_STOP; /* stop wd on debug */
#endif

 uint32_t *p = extInt;		/* external interrupt list */
 int i = sizeof(extInt) / sizeof(uint32_t); /* sizeof list */
 while (--i >= 0)		/* while not at end of list */
  HAL_NVIC_DisableIRQ((IRQn_Type) *p++);	/* disable external interrupt */

#if 0
#if REM_ISR
 initRem();
#else
 HAL_NVIC_DisableIRQ(REMOTE_IRQn);
#endif
#endif

 initCharBuf();

 putstr("start monitor loop\n");
#if defined(REMPORT)
 putstr1("start remcmd\n");
#endif

 monitorLoopSetup();
 
 #if DATA_SIZE
 unsigned int bss = (unsigned int) (&__bss_end__ - &__bss_start__);
 unsigned int data = (unsigned int) (&__data_end__ - &__data_start__);
 printf("data %u bss %u total %u\n", data, bss, data + bss);
 printf("stack %08x stackLimit %08x sp %08x\n",
	(unsigned int) &__stack, (unsigned int) &__Main_Stack_Limit,
	getSP());
 #endif

 printf("DWT_CTRL %x\n", (unsigned int) DWT->CTRL);
 resetCnt();
 startCnt();
 stopCnt();
 printf("cycles %u\n", getCycles());

 clockFreq = HAL_RCC_GetHCLKFreq();
 tmrFreq = HAL_RCC_GetPCLK2Freq();
 printf("clock frequency %u FCY %u\n",
	(unsigned int) clockFreq, (unsigned int) tmrFreq);
 printf("sysTick load %d\n", (int) SysTick->LOAD);

 uint32_t counter = clockFreq / (CYCLES_SEC * SAMPLES_CYCLE * CHAN_PAIRS);
 uint16_t psc = 1;
 uint32_t ctr;
 while (1)
 {
  ctr = counter / psc;
  if (ctr < 65536)
   break;
  psc += 1;
 }
 printf("tmr1 psc %u ctr %u\n", (unsigned int) psc, (unsigned int) ctr);
 psc -= 1;
 adcTmrScl(psc);
 adcTmrMax(ctr);
#if 1
 adcTmrCCR(ctr / 2);
#endif

#if PIN_DISPLAY
 pinDisplay();
#endif

#define HAL 1

#if HAL
#if defined(STM32F1)
 HAL_StatusTypeDef status = HAL_ADCEx_Calibration_Start(&hadc1);
#endif
#if defined(STM32F3)
 HAL_StatusTypeDef status =
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
 #endif
#if defined(STM32F1)
 printf("calibration status %d\n", status);
 status = HAL_ADCEx_Calibration_Start(&hadc2);
#endif
#if defined(STM32F3)
 status = HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
#endif
 printf("calibration status %d\n", status);
 #else

 printf("calibration 1\n");
 flushBuf();
 LL_ADC_Enable(ADC1);
 ADC1->CR2 |= ADC_CR2_RSTCAL;
 while ((ADC1->CR2 & ADC_CR2_RSTCAL) != 0)
 {
  putx('r');
 }

 ADC1->CR2 |= ADC_CR2_CAL;
 while ((ADC1->CR2 & ADC_CR2_CAL) != 0)
 {
  putx('1');
 }

 printf("calibration 2\n");
 flushBuf();
 LL_ADC_Enable(ADC2);
 ADC2->CR2 |= ADC_CR2_RSTCAL;
 while ((ADC2->CR2 & ADC_CR2_RSTCAL) != 0)
 {
  putx('r');
 }

 ADC2->CR2 |= ADC_CR2_CAL;
 while ((ADC2->CR2 & ADC_CR2_CAL) != 0)
 {
  putx('2');
 }
 
 printf("calibration done\n");
 LL_ADC_Disable(ADC1);
 LL_ADC_Disable(ADC2);

 #endif	 /* HAL */

 rmsCfgInit();
 pwrUpdTime = millis() - PWR_INTERVAL;

 ledUpdTime = millis();
 ledSet();
 while (1)			/* main loop */
 {
  newline();
  while (1)			/* input background loop */
  {

   uint32_t t = millis();
   if ((t - ledUpdTime) > LED_DELAY) /* if time to flash led */
   {
    ledUpdTime = t;
    if (led())
     ledClr();
    else
     ledSet();
   }

   if (pwrActive)
   {
    if ((t - pwrUpdTime) >= PWR_INTERVAL)
    {
     pwrUpdTime = t;
     for (i = 0; i < maxChan; i++)
     {
      chanCfg[i].pwr->update = true;
     }
    }
   
    for (i = 0; i < maxChan; i++)
    {
     P_CHANCFG chan = &chanCfg[i];
     if (chan->type == POWER_CHAN)
     {
      updatePower(chan->pwr);
     }
     else if (chan->type == CURRENT_CHAN)
     {
      updateCurrent(chan->cur);
     }
#if 0
     if (pwr->done)
     {
      printf("offset %d vRms %d v %5.3f\n",
	     pwr->v.offset, pwr->vRms,
	     ((pwr->vRms * (float) 3300) / 4095) / 1000);
      printf("offset %d cRms %d c %5.3f\n",
	     pwr->c.offset, pwr->cRms,
	     ((pwr->cRms * (float) 3300) / 4095) / 1000);
      pwr->done = false;
     }
#endif
    }
   }

   pollBufChar();		/* check for data to output */
   if (dbgRxReady())		/* if character available */
   {
    ch = dbgRxRead();		/* return it */
    putBufChar(ch);		/* echo input */
    break;
   }
  }

  flushBuf();
  lclcmd(ch);			/* local commands */
  flushBuf();
 }
}

#if PIN_DISPLAY

#define CON_SIZE (sizeof(conDef) / sizeof(T_CONDEF))
#define CON_PINS (CON_SIZE / 2)

void pinDisplay()
{
 P_PINDEF pin = pinDef;
 for (unsigned int i = 0; i < (sizeof(pinDef) / sizeof(T_PINDEF)); i++)
 {
  char t0[8];
  T_PIN_NAME val;
  val = pinName(t0, pin->port, pin->pin);
//  printf("port %08x pin %08x gpio %c pin %2d\n",
//	 (unsigned int) pin->port, (unsigned int) pin->pin,
//	 val.port, val.num);
   
  P_CONDEF con = conDef;
  char *connector = "";
  unsigned int j;
  for (j = 0; j < CON_SIZE; j++)
  {
   if ((val.port == con->port)
   &&  (val.num == con->pin))
   {
    connector = j < CON_PINS ? "L" : "R";
    if (j > CON_PINS)
     j -= CON_PINS;
    j += 1;
    break;
   }
   con++;
  }

  char t1[6];
  char t2[40];
  sprintf(t1, "%s %d", connector, j);
  printf("%-9s %-5s %-4s %s\n",
	 pin->name, t0, t1, gpioStr(t2, sizeof(t2), &val));
  pin++;
 }
}

#endif

extern "C" void hard_fault_handler_c (unsigned int * hardfault_args)
{
 unsigned int stacked_r0;
 unsigned int stacked_r1;
 unsigned int stacked_r2;
 unsigned int stacked_r3;
 unsigned int stacked_r12;
 unsigned int stacked_lr;
 unsigned int stacked_pc;
 unsigned int stacked_psr;
 
 stacked_r0 = ((unsigned long) hardfault_args[0]);
 stacked_r1 = ((unsigned long) hardfault_args[1]);
 stacked_r2 = ((unsigned long) hardfault_args[2]);
 stacked_r3 = ((unsigned long) hardfault_args[3]);
 
 stacked_r12 = ((unsigned long) hardfault_args[4]);
 stacked_lr = ((unsigned long) hardfault_args[5]);
 stacked_pc = ((unsigned long) hardfault_args[6]);
 stacked_psr = ((unsigned long) hardfault_args[7]);
 
 dbgBuffer = 0;
 
 printf("\n\n[Hard fault handler - all numbers in hex]\n");
 printf("R0 = %x\n", stacked_r0);
 printf("R1 = %x\n", stacked_r1);
 printf("R2 = %x\n", stacked_r2);
 printf("R3 = %x\n", stacked_r3);
 printf("R12 = %x\n", stacked_r12);
 printf("LR [R14] = %x  subroutine call return address\n", stacked_lr);
 printf("PC [R15] = %x  program counter\n", stacked_pc);
 printf("PSR = %x\n", stacked_psr);
 printf("BFAR = %x\n", (*((volatile unsigned int *) (0xE000ED38))));
 printf("CFSR = %x\n", (*((volatile unsigned int *) (0xE000ED28))));
 printf("HFSR = %x\n", (*((volatile unsigned int *) (0xE000ED2C))));
 printf("DFSR = %x\n", (*((volatile unsigned int *) (0xE000ED30))));
 printf("AFSR = %x\n", (*((volatile unsigned int *) (0xE000ED3C))));
 printf("SCB_SHCSR = %x\n", (unsigned int) SCB->SHCSR);
  
 while (1);
}

#if 0
 flushBuf();
 P_DBG d = dbgDef;
 for (unsigned int i = 0; i < (sizeof(dbgDef) / sizeof(T_DBG)); i++)
 {
  P_GPIO g = gpio;
  for (unsigned int j = 0; j < (sizeof(gpio) / sizeof(T_GPIO)); j++)
  {
   if (d->port == g->port)
   {
    int mask = 1;
    int k;
    for (k = 0; k < 15; k++)
    {
     if (mask == d->pin)
     {
      printf("dbg%d p%s%d\n", i, g->name, k);
      break;
     }
     mask <<= 1;
    }
    break;
   }
   g++;
  }
  d++;
 }
#endif
