#if !defined(INCLUDE)
#define __LCLCMD__
#if defined(STM32F1)
#include "stm32f1xx_hal.h"
#endif

#if defined(STM32F3)
#include "stm32f3xx_hal.h"
#endif

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#define EXT extern
#include "config.h"
#include "serialio.h"
#include "current.h"
#include "cyclectr.h"

#include "i2c.h"

#include "i2cx.h"
#include "lcd.h"
#include "spix.h"
#include "max31856.h"
#include "stm32Info.h"

#ifdef EXT
#undef EXT
#endif

#define EXT
#include "lclcmd.h"
#endif

#if defined(__LCLCMD_INC__)	// <-

#if !defined(EXT)
#define EXT extern
#endif

void lclcmd(int ch);

#if 0
void afioInfo(void);
void bkpInfo(void);
void rtcInfo(void);
#endif

#endif	// ->
#ifdef __LCLCMD__

//int lastFlags;

#if PIN_DISPLAY
void pinDisplay();
#endif

typedef struct
{
 char ch;
 GPIO_TypeDef *port;
} T_PORT_LIST, *P_PORT_LIST;

T_PORT_LIST portList[] =
{
 {'a', GPIOA},
 {'b', GPIOB},
 {'c', GPIOC},
 {'d', GPIOD},
 {'e', GPIOE},
};

void delayMSec(volatile uint32_t mSec)
{
 uint32_t start = millis();
 
 while ((millis() - start) < mSec)
  ;
}

void lclcmd(int ch)
{
 if (ch == 'i')			/* init character buffer */
 {
  initCharBuf();
 }
#if 0
 else if (ch == 'J')
 {
  newline();
  startCnt();
  unsigned int timeout = 500 * (HAL_RCC_GetHCLKFreq() / 1000000);
  for (uint8_t address = 8; address < 120; address++)
  {
   if (i2c_start(I2C1, address))
   {
    printf("found %02x\n", (unsigned int) address);
   }
   i2c_stop(I2C1);
   unsigned int start = getCycles();
   while ((I2C1->SR2 & I2C_SR2_BUSY) != 0)
   {
    if ((getCycles() - start) > timeout)
     break;
   }
   printf("sr1 %8x sr2 %8x\n",
	  (unsigned int) I2C1->SR1, (unsigned int) I2C1->SR2);
   flushBuf();
  }
 }
#endif
 else if (ch == 'L')
 {
  newline();
  lcdInit();
 }
 else if (ch == 'R')
 {
  printf("\nbkp reset\n");
  RCC->BDCR |= RCC_BDCR_BDRST;
  RCC->BDCR &= ~RCC_BDCR_BDRST;
 }
 else if (ch == 'W')
 {
  newline();
  setBacklight(1);
  setCursor(0, 0);
  char buf[2];
  buf[1] = 0;
  for (int row = 0; row < 4; row++)
  {
   setCursorBuf(0, row);
   buf[0] = '0' + row;
   lcdString(buf);
   lcdString(" Test");
   i2cSend();
   while (i2cCtl.state != I_IDLE)
    i2cControl();
  }
 }
 else if (ch == 'H')
 {
  newline();
  MX_I2C1_Init();
  i2cInfo(I2C1, "I2C1");
  rccInfo();
 }
#if 0
 else if (ch == 'I')
 {
  newline();
  i2cInfo(I2C1, "I2C1");
  rccInfo();
  i2cWrite(0x55);
 }
 #endif 
 else if (ch == 'b')
 {
  printf("\ncharOverflow %d\n", charOverflow);
 }
 else if (ch == 'C')
  currentCmds();
#if 0
 else if (ch == 'e')
 {
  adcTmrStop();
  adcTmrClrIE();
  printf("\ntimer stopped\n");
 }
 else if (ch == 'a')
 {
  newline();
  adcRead1();
 }
 else if (ch == 'r')
 {
  newline();
  adcRun();
 }
 else if (ch == 's')
 {
  newline();
  adcStatus();
 }
 else if (ch == 't')
 {
  newline();
  rmsTestInit();
  rmsTest();
 }
 else if (ch == 'c')
 {
  testIndex = 0;
 }
 else if (ch == 'd')
 {
  newline();
  int count = 2 * SAMPLES_CYCLE;
  int16_t *p = testData;
  int col = 0;
  int offset = testOffset >> SAMPLE_SHIFT;
  while (1)
  {
   uint16_t val = *p++;

   printf("%5d ", (int) (val - offset));
   count -= 1;
   col++;
   if (col == 8)
   {
    col = 0;
    printf("\n");
   }
   if (count == 0)
   {
    if (col != 0)
     printf("\n");
    break;
   }
  }
 }
#endif

#endif

#if PIN_DISPLAY
 else if (ch == '>')
 {
  newline();
  pinDisplay();
 }
#endif

 else if (ch == '*')
 {
  unsigned int reg = 0;
  unsigned int mask = 0;
  unsigned int invert = 0;

  while (1)
  {
   if (query(&getnum, "\nreg "))
    reg = val;
   else
    break;

   if (query(&getnum, "mask "))
    mask = val;

   if (query(&getnum, "invert "))
    invert = val != 0;

   int set = (((reg & mask) != 0) ^ invert);
   int clr = (((reg & mask) == 0) ^ invert);

   printf("\nreg %02x mask %02x invert %02x set %d clr %d",
	  reg, mask, invert, set, clr);
  }
 }
 else if (ch == 'F')
 {
  if (query(&getnum, "IRQn: "))
  {
   HAL_NVIC_EnableIRQ((IRQn_Type) val);
  }
 }
 else if (ch == 'I')
 {
  ch = query("\nPort: ");
  unsigned int i;
  P_PORT_LIST p = portList;
  GPIO_TypeDef *port = 0;
  for (i = 0; i < sizeof(portList) / sizeof(T_PORT_LIST); i++)
  {
   if (ch == p->ch)
   {
    port = p->port;
    putBufChar(ch);
    break;
   }
   p++;
  }
  if (port != 0)
  {
   if (query(&gethex, "\nmask: "))
   {
    port->ODR = val;
    printf("\n");
    gpioInfo(port);
    while (1)
    {
     printf("done: ");
     flushBuf();
     while (dbgRxReady() == 0)	/* while no character */
      ;
     ch = dbgRxRead();
     putBufChar(ch);
     newline();
     flushBuf();
     if (ch == 'y')
      break;
    }
   }
  }
 }
 else if (ch == 'Q')
  info();
 
#if 0 // DBGMSG
 else if (ch == 'D')		/* dump dbg buffer */
 {
  newline();
  int empty = dbgemp;
  int i;
  for (i = 0; i < dbgcnt; i++)
  {
   P_DBGMSG p = &dbgdata[empty];
   float t = (float) p->time / 1000;
#if DBGMSG == 2
   printf("%8.3f %7s %6d\n", t, dMessageList[(int) p->dbg], (int) p->val);
#else
   printf("%8.3f %8s %6d\n", t, p->str, (int) p->val);
#endif
   empty++;
   if (empty >= MAXDBGMSG)
    empty = 0;
  }
  printf("z %d x %d\n", zLoc, xLoc);
 }
 else if (ch == 'E')		/* clear debug buffer */
 {
  memset(&dbgdata, 0, sizeof(dbgdata));
  dbgcnt = 0;
  dbgfil = 0;
  dbgemp = 0;
 }
#endif /* DBGMSG */

 else if (ch == 'd')		/* dump memory */
 {
  putBufChar(' ');
  if (gethex())
  {
   unsigned char *p;

   p = (unsigned char *) (int) val;
   if (gethex())
   {
    newline();
    prtbuf(p, val);
   }
  }
 }
 else if (ch == 'r')		/* read memory */
 {
  putBufChar(' ');
  if (gethex())
  {
   printf("%x", *(int16_t *) (int) val);
  }
 }
 else if (ch == 'w')
 {
  putBufChar(' ');
  if (gethex())
  {
   int16_t *p;
   p = (int16_t *) (int) val;
   printf(" %x ", *p);
   if (gethex())
   {
    *p = val;
   }
  }
 }
#if 0
 else if (ch == 'p')
 {
  if (query(&getnm, ' '))
  {
   print = val;
  }
 }
#endif

 else if (ch == 'T')		/* thermocouple commands */
  max56Cmds();

#if DBGTRK
 else if (ch == 'T')		/* print track buffer */
 {
  int16_t idx = trkidx;

  printf("\n");
  dbgTrk = false;
  if constexpr (DBGTRK1W)
  {
   int16_t i = sizeof(trkbuf) / sizeof(int16_t);
   while (--i >= 0)
   {
    int16_t tmp = (int16_t) (*(int16_t *) &trkbuf[idx]);
    printf("%4d %6d %4x\n", idx, tmp, (uint16_t) tmp);
    flushBuf();
    idx += 1;
    idx &= (TRKBUFSIZE - 1);
   }
  }
  if constexpr (DBGTRK1L)
  {
   int16_t i = sizeof(trkbuf) / sizeof(int);
   while (--i >= 0)
   {
    int *p = (int *) &trkbuf[idx];
    int tmp0 = *p;
    printf("%4d %10d\n", idx, tmp0);
    flushBuf();
    idx += 2;
    idx &= (TRKBUFSIZE - 1);
   }
  }
  if constexpr (DBGTRK2L)
  {
   int16_t i = sizeof(trkbuf) / (2 * sizeof(int));
   while (--i >= 0)
   {
    int *p = (int *) &trkbuf[idx];
    int tmp0 = *p;
    int tmp1 = *(p + 1);
    printf("%4d %10d %10d\n", idx, tmp0, tmp1);
    flushBuf();
    idx += 4;
    idx &= (TRKBUFSIZE - 1);
   }
  }
  if constexpr (DBGTRK2WL)
  {
   int16_t i = sizeof(trkbuf) / (2 * sizeof(int16_t) + sizeof(int));
   while (--i >= 0)
   {
    int16_t *p = (int16_t *) &trkbuf[idx];
    int16_t tmp0 = *p++;
    int16_t tmp1 = *p++;
    int tmp2 = *((int32_t *) p);
    printf("%4d %4d %5u %10d\n", idx, tmp0, (uint16_t) tmp1, tmp2);
    flushBuf();
    idx += 4;
    idx &= (TRKBUFSIZE - 1);
   }
  }
  dbgTrk = true;
 }
 else if (ch == 'B')		/* clear track buffer */
 {
  dbgTrk = false;
  memset(trkbuf, 0, sizeof(trkbuf)); /* clear track buffer */
  trkidx = 0;			/* reset index */
  dbgTrk = true;		/* start saving tracking info */
 }
#endif	/* DBGTRK */

 else if (ch == 'v')
 {
  tmrInfo(TIM3);
  tmrInfo(TIM4);
 }
}

#if 0
void afioInfo(void)
{
#if defined(STM32F1)
 printf("AFIO %x\n", (unsigned int) AFIO);
 printf("EVCR      %8x ",  (unsigned int) AFIO->EVCR);
 printf("MAPR      %8x\n", (unsigned int) AFIO->MAPR);
 printf("EXTICR[0] %8x ",  (unsigned int) AFIO->EXTICR[0]);
 printf("EXTICR[1] %8x\n", (unsigned int) AFIO->EXTICR[1]);
 printf("EXTICR[2] %8x ",  (unsigned int) AFIO->EXTICR[2]);
 printf("EXTICR[3] %8x\n", (unsigned int) AFIO->EXTICR[3]);
 printf("MAPR2     %8x\n", (unsigned int) AFIO->MAPR2);
#endif
}

void bkpInfo(void)
{
#if defined(STM32F1)
 printf("BKP %x\n", (unsigned int) BKP);
 printf("RTCCR     %8x ",  (unsigned int) BKP->RTCCR);
 printf("CR        %8x\n", (unsigned int) BKP->CR);
 printf("CSR       %8x\n", (unsigned int) BKP->CSR);
#endif
}

void rtcInfo(void)
{
#if defined(STM32F1)
 printf("RTC %x\n", (unsigned int) RTC);
 printf("CRH       %8x ",  (unsigned int) RTC->CRH);
 printf("CRL       %8x\n", (unsigned int) RTC->CRL);
 printf("PRLH      %8x ",  (unsigned int) RTC->PRLH);
 printf("PRLL      %8x\n", (unsigned int) RTC->PRLL);
 printf("DIVH      %8x ",  (unsigned int) RTC->DIVH);
 printf("DIVL      %8x\n", (unsigned int) RTC->DIVL);
 printf("CNTH      %8x ",  (unsigned int) RTC->CNTH);
 printf("CNTL      %8x\n", (unsigned int) RTC->CNTL);
 printf("ALRH      %8x ",  (unsigned int) RTC->ALRH);
 printf("ALRL      %8x\n", (unsigned int) RTC->ALRL);
#endif
}
#endif
