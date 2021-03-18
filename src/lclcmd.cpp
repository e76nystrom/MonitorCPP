#if !defined(INCLUDE)
#define __LCLCMD__
#if defined(STM32F1)
#include "stm32f1xx_hal.h"
#endif	/* STM32F1 */

#if defined(STM32F3)
#include "stm32f3xx_hal.h"
#endif	/* STM32F3 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#define EXT extern
#include "serialio.h"
#include "current.h"
#include "cyclectr.h"

#include "i2c.h"

#include "i2cx.h"
#include "lcd.h"
#include "spix.h"
#include "max31856.h"
#include "max31865.h"
#include "stm32Info.h"

#ifdef EXT
#undef EXT
#endif	/* EXT */

#define EXT
#include "lclcmd.h"
#endif	/* INCLUDE */

#if defined(__LCLCMD_INC__)	// <-

#if !defined(EXT)
#define EXT extern
#endif	/* EXT */

void lclcmd(int ch);

#if 0
void afioInfo(void);
void bkpInfo(void);
void rtcInfo(void);
#endif	/* 0 */

#endif	// ->
#ifdef __LCLCMD__

extern "C" void RTC_IRQHandler(void);
uint32_t rtcMillis;
uint32_t rtcSec;
uint32_t rtcOverflow;

//int lastFlags;

#if PIN_DISPLAY
void pinDisplay();
#endif	/* PIN_DISPLAY */

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

void switchRTCInfo()
{
 newline();
 rccInfo();
 newline();
 pwrInfo();
 newline();
 bkpInfo();
 newline();
 __disable_irq();
 unsigned int tSec = rtcSec;
 unsigned int cntl = RTC->CNTL;
 unsigned int tMillis = millis() - rtcMillis;
 __enable_irq();
 printf("millis %u rtcSec %u CNTL %u rtcOverflow %u\n\n",
	(unsigned int) tMillis,	(unsigned int) tSec,
	(unsigned int) cntl, (unsigned int) rtcOverflow);
 rtcInfo();
}

bool setRtcCnt(int val)
{
 RTC->CRL |= RTC_CRL_CNF;
 RTC->PRLH = 0;
 RTC->PRLL = val;
 RTC->CNTH = 0;
 RTC->CNTL = 0;
 RTC->CNTH = 0;
 RTC->CNTL = 0;
 RTC->CRL &= ~RTC_CRL_CNF;
 uint32_t t0 = millis();
 while ((RTC->CRL & RTC_CRL_RTOFF) == 0)
 {
  if ((millis() - t0) > 1000U)
  {
   return(true);
  }
 }
 return(false);
}

void switchRTC(void)
{
 newline();
 bool err = false;
 uint32_t t0;
 bitState("RCC_APB1ENR_PWREN", &RCC->APB1ENR, RCC_APB1ENR_PWREN);
 flushBuf();
 if ((RCC->APB1ENR & RCC_APB1ENR_PWREN) == 0)
 {
  printf("enable pwr\n");
  RCC->APB1ENR |= RCC_APB1ENR_PWREN;
  volatile uint32_t tmpreg = RCC->APB1ENR & RCC_APB1ENR_PWREN;
  (void) tmpreg;
 }

 bitState("PWR_CR_DBP", &PWR->CR, PWR_CR_DBP);
 flushBuf();
 if ((PWR->CR & PWR_CR_DBP) == 0)
 {
  printf("setting PWR_CR_DBP\n");
  PWR->CR |= PWR_CR_DBP;
  t0 = millis();
  while ((PWR->CR & PWR_CR_DBP) == 0)
  {
   if ((millis() - t0) > 100U)
   {
    err = true;
    printf("PWR_CR_DBP == 0\n");
    break;
   }
  }
 }

 if (!err)
 {
  printf("BDCR %08x\n", (unsigned int) RCC->BDCR);
  flushBuf();
  uint32_t tmp = RCC->BDCR & ~RCC_BDCR_RTCSEL_Msk;
  tmp |= RCC_BDCR_RTCSEL_LSE;
  __HAL_RCC_BACKUPRESET_FORCE();
  __HAL_RCC_BACKUPRESET_RELEASE();
  RCC->BDCR = tmp;
  printf("BDCR %08x\n", (unsigned int) RCC->BDCR);

  bitState("RCC_BDCR_LSEON", &RCC->BDCR, RCC_BDCR_LSEON);
  flushBuf();
  RCC->BDCR |= RCC_BDCR_LSEON;
  t0 = millis();
  while ((RCC->BDCR & RCC_BDCR_LSERDY) == 0)
  {
   if ((millis() - t0) > 10000U)
   {
    err = true;
    printf("RCC_BDCR_LSERDY == 0\n");
    flushBuf();
    break;
   }
  }
  if (!err)
  {
   t0 = millis() - t0;
   printf("start time %u\n", (unsigned int) t0);
   flushBuf();
  }
 }

 if (!err)
 {
  t0 = millis();
  while ((RTC->CRL & RTC_CRL_RTOFF) == 0)
  {
   if ((millis() - t0) > 1000U)
   {
    err = true;
    break;
   }
  }
 }

 if (!err)
 {
  err = setRtcCnt(32768-1);
 }

 if (!err)
 {
  RTC->CRL &= ~RTC_CRL_RSF;
  t0 = millis();
  while ((RTC->CRL & RTC_CRL_RSF) == 0)
  {
   if ((millis() - t0) > 100U)
   {
    err = true;
    break;
   }
  }
 }
}

void rtcEnableIRQ(void)
{
 rtcMillis = millis();
 rtcSec = 0;
 RTC->CRH |= RTC_CRH_OWIE | RTC_CRH_SECIE;
 newline();
 rtcInfo();
}

#if defined(TestOut_Pin)

void setRtcGPIO(void)
{
 GPIO_InitTypeDef GPIO_InitStruct;

 RCC->BDCR &= ~RCC_BDCR_LSEON;

 GPIO_InitStruct.Pin = TestOut_Pin;
 GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
 GPIO_InitStruct.Pull = GPIO_NOPULL;
 GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
 HAL_GPIO_Init(TestOut_GPIO_Port, &GPIO_InitStruct);

 GPIO_InitStruct.Pin = TestIn_Pin;
 GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
 GPIO_InitStruct.Pull = GPIO_NOPULL;
 HAL_GPIO_Init(TestIn_GPIO_Port, &GPIO_InitStruct);
 newline();
 gpioInfo(GPIOC);
 newline();
 switchRTCInfo();
}

inline void testOutIni() {}
inline void testOutSet() {TestOut_GPIO_Port->BSRR = TestOut_Pin;}
inline void testOutClr() {TestOut_GPIO_Port->BSRR = (TestOut_Pin << 16);}
inline bool testOut() {return((TestOut_GPIO_Port->ODR & TestOut_Pin) != 0);}
inline void testOutToggle()
{
 if (testOut())
  testOutClr();
 else
  testOutSet();
}

void toggleRtcGPIO(void)
{
 printf("\ntoggling TestOut_Pin: ");
 flushBuf();
 while (1)
 {
  testOutToggle();
  if (dbgRxReady() != 0)
  {
   char ch = dbgRxRead();
   if (ch == 3)
   {
    newline();
    break;
   }
  }
 }
}

#endif	/* TestOut_Pin */

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
#endif	/* 0 */
 else if (ch == 'L')
 {
  newline();
  lcdInit();
 }
 else if (ch == 'R')
 {
  newline();
  bkpInfo();
  printf("\nbkp reset\n\n");
  RCC->BDCR |= RCC_BDCR_BDRST;
  RCC->BDCR &= ~RCC_BDCR_BDRST;
  RCC->BDCR = RCC_BDCR_RTCEN | RCC_BDCR_RTCSEL_HSE;
  setRtcCnt(62500-1);
  bkpInfo();
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
 #endif	 /* 0 */
 else if (ch == 'b')
 {
  printf("\ncharOverflow %d\n", charOverflow);
 }
 else if (ch == 'C')
 {
  currentCmds();
 }
 else if (ch == 'p')
 {
  switchRTCInfo();
 }
 else if (ch == 'q')
 {
  switchRTC();
 }
 else if (ch == 'j')
 {
  rtcEnableIRQ();
 }
#if defined(TestOut_Pin)
 else if (ch == 'o')
 {
  setRtcGPIO();
 }
 else if (ch == 't')
 {
  toggleRtcGPIO();
 }
#endif	/* TestOut_Pin */
 
#if PIN_DISPLAY
 else if (ch == '>')
 {
  newline();
  pinDisplay();
 }
#endif	/* PIN_DISPLAY */

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
   typedef struct sMask
   {
    union
    {
     struct
     {
      uint16_t mask;
      uint16_t flag;
     };
     struct
     {
      uint32_t val;
     };
    };
   } T_MASK;
   T_MASK andMask;
   T_MASK orMask;
   andMask.val = 0;
   orMask.val = 0;
   if (query(&gethex, "\nand mask: "))
   {
    andMask.mask = val;
    andMask.flag = 1;
   }
   if (query(&gethex, "\nor mask: "))
   {
    orMask.mask = val;
    orMask.flag = 1;
   }
   if (andMask.flag)
    port->ODR &= andMask.mask;
   if (orMask.flag)
    port->ODR |= orMask.mask;
   if (andMask.flag || orMask.flag)
   {
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
#endif	/* DBGMSG == 2 */
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
  if (query(&getnum, ' '))
  {
   print = val;
  }
 }
#endif	/* 0 */

 else if (ch == 'T')		/* thermocouple commands */
  max56Cmds();
 else if (ch == 'U')		/* rtd commands */
  max65Cmds();

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
#define __HAL_RCC_PWR_IS_CLK_DISABLED() \
 ((RCC->APB1ENR & (RCC_APB1ENR_PWREN)) == RESET)

#define __HAL_RCC_PWR_CLK_ENABLE() \
 do {							\
  __IO uint32_t tmpreg;					\
  SET_BIT(RCC->APB1ENR, RCC_APB1ENR_PWREN);		\
  /* Delay after an RCC peripheral clock enabling */	\
  tmpreg = READ_BIT(RCC->APB1ENR, RCC_APB1ENR_PWREN);	\
  UNUSED(tmpreg);					\
 } while(0U)

void tmp(void)
{
 /*------------------------------ LSE Configuration -------------------------*/
 if (((RCC_OscInitStruct->OscillatorType) & RCC_OSCILLATORTYPE_LSE) == RCC_OSCILLATORTYPE_LSE)
 {
  FlagStatus       pwrclkchanged = RESET;

  /* Check the parameters */
  assert_param(IS_RCC_LSE(RCC_OscInitStruct->LSEState));

  /* Update LSE configuration in Backup Domain control register    */
  /* Requires to enable write access to Backup Domain of necessary */
  if (__HAL_RCC_PWR_IS_CLK_DISABLED())
  {
   __HAL_RCC_PWR_CLK_ENABLE();
   pwrclkchanged = SET;
  }

// if ((PWR->CR & PWR_CR_DBP) != 0)
  if (HAL_IS_BIT_CLR(PWR->CR, PWR_CR_DBP))
  {
   /* Enable write access to Backup domain */
   SET_BIT(PWR->CR, PWR_CR_DBP);

   /* Wait for Backup domain Write protection disable */
   tickstart = HAL_GetTick();

   while (HAL_IS_BIT_CLR(PWR->CR, PWR_CR_DBP))
   {
    if ((HAL_GetTick() - tickstart) > RCC_DBP_TIMEOUT_VALUE)
    {
     return HAL_TIMEOUT;
    }
   }
  }

  /* Set the new LSE configuration -----------------------------------------*/
  __HAL_RCC_LSE_CONFIG(RCC_OscInitStruct->LSEState);
  /* Check the LSE State */
  if (RCC_OscInitStruct->LSEState != RCC_LSE_OFF)
  {
   /* Get Start Tick */
   tickstart = HAL_GetTick();

   /* Wait till LSE is ready */
   while (__HAL_RCC_GET_FLAG(RCC_FLAG_LSERDY) == RESET)
   {
    if ((HAL_GetTick() - tickstart) > RCC_LSE_TIMEOUT_VALUE)
    {
     return HAL_TIMEOUT;
    }
   }
  }
  else
  {
   /* Get Start Tick */
   tickstart = HAL_GetTick();

   /* Wait till LSE is disabled */
   while (__HAL_RCC_GET_FLAG(RCC_FLAG_LSERDY) != RESET)
   {
    if ((HAL_GetTick() - tickstart) > RCC_LSE_TIMEOUT_VALUE)
    {
     return HAL_TIMEOUT;
    }
   }
  }

  /* Require to disable power clock if necessary */
  if (pwrclkchanged == SET)
  {
   __HAL_RCC_PWR_CLK_DISABLE();
  }
 }
}
#endif	/* 0 */

#endif	/* __LCLCMD__ */
#if 0

/*
After init with RTC enabled through cubemx

RCC 40021000
CR        3035783 CFGR       1d840a
APB2RSTR        0 APB1RSTR        0
APB2ENR      4e3d APB1ENR  18244000
CIR             0 AHBENR         14
BDCR         8103 CSR      1c000000

PWR 40007000
CR             100 CSR              0

BKP 40006c00
RTCCR            0 CR               0
CSR              0

RTC 40002800
CRH              0 CRL             29
PRLH             0 PRLL          7fff
DIVH             0 DIVL          677f
CNTH             0 CNTL            15
ALRH          ffff ALRL          ffff

gpio 40011000 C
CRL     44444444 CRH     44244444
IDR            0 ODR            0
BSRR           0 LCKR           0
           0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15
mode       0  0  0  0  0  0  0  0  0  0  0  0  0  2  0  0
cnf        1  1  1  1  1  1  1  1  1  1  1  1  1  0  1  1
idr        0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0
odr        0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0
*/

/*
after settings form lclcmd 'q'

RCC 40021000
CR        3035783 CFGR       1d840a
APB2RSTR        0 APB1RSTR        0
APB2ENR      4e3d APB1ENR  18244000
CIR             0 AHBENR         14
BDCR         8103 CSR      1c000000

PWR 40007000
CR             100 CSR              0

BKP 40006c00
RTCCR            0 CR               0
CSR              0

RTC 40002800
CRH              0 CRL             29
PRLH             0 PRLL          7fff
DIVH             0 DIVL          22fb
CNTH             0 CNTL            1c
ALRH          ffff ALRL          ffff
*/

#endif

void RTC_IRQHandler(void)
{
 if (RTC->CRL & RTC_CRL_OWF)
 {
  RTC->CRL &= ~RTC_CRL_OWF;
  rtcOverflow += 1;
 }
 if (RTC->CRL & RTC_CRL_SECF)
 {
  dbg5Toggle();
  RTC->CRL &= ~RTC_CRL_SECF;
  rtcSec += 1;
 }
}
