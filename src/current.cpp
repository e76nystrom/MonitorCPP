//******************************************************************************
#define __CURRENT__
#if defined(STM32F1)
#include "stm32f1xx_hal.h"
#include "stm32f1xx_ll_adc.h"
#include "stm32f1xx_ll_dma.h"
#endif	/* STM32F1 */
#if defined(STM32F4)
#include "stm32f4xx_hal.h"
#include "stm32f4xx_ll_adc.h"
#include "stm32f4xx_ll_dma.h"
#endif	/* STM32F4 */

#include "main.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "math.h"
//#include <cmath>
#include <stdbool.h>
#include <limits.h>
#include <stdarg.h>

#if defined(ARDUINO_ARCH_STM32)
#include <Arduino.h>
#else
#include "serialio.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#endif	/* ARDUINO_ARCH_STM32 */

#ifdef EXT
#undef EXT
#endif

#define EXT
#if defined(ARDUINO_ARCH_STM32)
#include "monitor.h"
#endif	/* ARDUINO_ARCH_STM32 */
#include "current.h"

#if defined(STM32MON)

#define DMA 0
#define SIMULTANEOUS 0

#if 0
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
#endif
#if DMA
extern DMA_HandleTypeDef hdma_adc1;
#endif	/* DMA */

#include "dbg.h"
#endif	/* STM32MON */

#if defined(__CURRENT_INC__)	// <-

#if !defined(EXT)
#define EXT extern
#endif	/* EXT */

#define PWR_SIZE 32
#define INITIAL_COUNT 10000
#define CYCLE_COUNT 60
#define SAMPLE_SHIFT 8

#define CYCLES_SEC 60
#define SAMPLES_CYCLE 16
#define CHAN_PAIRS 2
#define ADC_BITS 12

#define INITIAL_SAMPLES 10000
#define RMS_INITIAL int(INITIAL_SAMPLES / SAMPLES_CYCLE) * SAMPLES_CYCLE

enum pwrState {initAvg, waitZero, avgData, cycleDone};

typedef struct s_adcData
{
 union
 {
  struct
  {
   uint16_t voltage;
   uint16_t current;
  };
  struct
  {
   uint32_t data;
  };
 };
} T_ADC_DATA, *P_ADC_DATA;

typedef struct s_rms
{
 int sample;			/* current sample */
 int value;			/* value after offset operation */
 int offset;			/* filtered offset */
 int sum;			/* sum of squares */
} T_RMS, *P_RMS;

#if 0
typedef struct s_buffer
{
 int filPtr;			/* fill pointer */
 int empPtr;			/* empty pointer */
 int count;			/* number in buffer */
 T_ADC_DATA buf[PWR_SIZE];	/* buffer */
} T_BUFFER, *P_BUFFER;
#endif

typedef struct s_pwrData
{
 uint32_t time;			/* time of reading */
 int vSum;			/* voltage sum of squares */
 int cSum;			/* current sum of squares */
 int pwrSum;			/* sum of voltage time current */
 int samples;			/* samples */
} T_PWR_DATA, *P_PWR_DATA;

typedef struct s_pwrBuf
{
 int filPtr;			/* fill pointer */
 int empPtr;			/* empty pointer */
 int count;			/* number in buffer */
 T_PWR_DATA buf[PWR_SIZE];	/* buffer */
} T_PWR_BUF, *P_PWR_BUF;

#define DISPLAY_INTERVAL 12

typedef struct s_rmsPwr
{
 pwrState state;		/* curent state */
 pwrState lastState;		/* last state */
 bool lastBelow;		/* last sample below voltage offset */
 int cycleCount;
 T_RMS v;			/* voltage */
 T_RMS c;			/* current */
 int samples;			/* sample counter */
 int pwrSum;			/* power sum */
 int sampleCount;		/* sample count for last reading */
 int displayCount;		/* counter for results display */
 int pwrAccum;			/* accumulator for power */
 int vRms;			/* rms voltage */
 int cRms;			/* rms current */
 int realPwr;			/* real power */
 int aprntPwr;			/* apparent power */
 int pwrFactor;			/* power factor */
 int pwrDir;			/* power direction */
 bool update;			/* time to update */
 bool done;			/* update done */
 struct s_chanCfg *cfg;		/* channel configuration */
// T_BUFFER b;			/* buffer */
 T_PWR_BUF pwrBuf;
} T_RMSPWR, *P_RMSPWR;

typedef struct s_chanCfg
{
 int count;			/* number of readings to take */
 int vltChan;			/* voltage channel */
 int curChan;			/* current channel */
 P_RMSPWR pwr;			/* rms power data */
// P_BUFFER buf;			/* buffer */
} T_CHANCFG, *P_CHANCFG;

#define MAX_CHAN 2

EXT uint32_t clockFreq;
EXT uint32_t tmrFreq;
EXT T_RMSPWR rmsPower[MAX_CHAN];

EXT bool pwrActive;
EXT int maxChan;
EXT int curChan;
EXT T_CHANCFG chanCfg[MAX_CHAN];
#if 0
EXT uint16_t *adc1Buf;
EXT uint16_t *adc2Buf;
#else
EXT P_RMS adc1Rms;
EXT P_RMS adc2Rms;
#endif

void rmsTestInit(void);
void rmsTest(void);

void rmsCfgInit(void);
#if 0
void rmsUpdate(int sample, P_RMS rms);
#endif
void updatePower(P_RMSPWR pwr);

void adcRead(void);
void adcRun(void);
void adcRead1(void);
void adcStatus(void);
void adcTmrTest(void);

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

inline uint32_t cpuCycles(void) { return(SysTick->VAL); }
inline uint32_t interval(uint32_t start, uint32_t end)
{
 if (end > start)
  return(start + 0x01000000 - end);
 else
  return(start - end);
}

#if 1
#define cycleCtr 1
#define DWT_CTRL_CycCntEna DWT_CTRL_CYCCNTENA_Msk
inline void resetCnt()
{
 DWT->CTRL &= ~DWT_CTRL_CycCntEna; // disable the counter    
 DWT->CYCCNT = 0;		// reset the counter
}

inline void startCnt()
{
 DWT->CTRL |= DWT_CTRL_CycCntEna; // enable the counter
}

inline void stopCnt()
{
 DWT->CTRL &= ~DWT_CTRL_CycCntEna; // disable the counter    
}

inline unsigned int getCycles()
{
 return DWT->CYCCNT;
}

inline void getCycles(uint32_t *val)
{
 *val = DWT->CYCCNT;
}
#else
#define cycleCtr 0
inline void resetCnt() {}
inline void startCnt() {}
inline void stopCnt() {}
inline unsigned int getCycles() {return(0);}
inline void getCycles(uint32_t *val) {};
#endif

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

inline void adcTmrInit() { \
	__HAL_RCC_TIM1_CLK_ENABLE(); \
	TIM1->CR1 |= TIM_CR1_DIR; \
	TIM1->CR1 &= ~TIM_CR1_CEN;}

inline void adcTmrBDTR() {TIM1->BDTR |= TIM_BDTR_MOE;}

inline void     adcTmrClrIE()         {TIM1->DIER &= ~TIM_DIER_UIE;}
inline void     adcTmrSetIE()         {TIM1->DIER |= TIM_DIER_UIE;}
inline uint16_t adcTmrTstIE()         \
	{return((TIM1->DIER & TIM_IT_UPDATE) != 0);}
inline uint16_t adcTmrIF()            \
	{return((TIM1->SR & TIM_FLAG_UPDATE) != 0);}
inline void     adcTmrClrIF()         {TIM1->SR = ~TIM_SR_UIF;}
inline void     adcTmrStart()         {TIM1->CR1 |= TIM_CR1_CEN;}
inline void     adcTmrPulse()         \
	{TIM1->CR1 |= (TIM_CR1_OPM | TIM_CR1_CEN);}
inline void     adcTmrStop()          \
	{TIM1->CR1 &= ~(TIM_CR1_OPM | TIM_CR1_CEN);}
inline void     adcTmrScl(uint16_t y) {TIM1->PSC = (y);}
inline uint16_t adcTmrRead()          {return(TIM1->CNT);}
inline void     adcTmrCntClr()        {TIM1->CNT = 0;}
inline void     adcTmrCnt(uint16_t x) {TIM1->CNT = (x);}
inline void     adcTmrMax(uint16_t x) {TIM1->ARR = ((x) - 1);}
inline void     adcTmrSet(uint16_t x) {TIM1->ARR = (x);}
inline uint16_t adcTmrMaxRead()       {return(TIM1->ARR);}

inline void     adcTmrCCR(uint16_t x) {TIM1->CCR1 = (x);}
inline void     adcTmrPWMMode()       \
	{TIM1->CCMR1 = (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1);}
inline void     adcTmrPWMEna()        \
	{adcTmrBDTR(); TIM1->CCER |= TIM_CCER_CC1E;}
inline void     adcTmrPWMDis()        {TIM1->CCER &= ~TIM_CCER_CC1E;}
inline uint16_t adcTmrReadCCR()       {return(TIM1->CCR1);}
inline uint16_t adcTmrReadCCMR()      {return(TIM1->CCMR1);}
inline void     adcTmrCC1ClrIF()      {TIM1->SR = ~TIM_SR_CC1IF;}
inline void     adcTmrCC1ClrIE()      {TIM1->DIER &= ~TIM_DIER_CC1IE;}
inline void     adcTmrCC1SetIE()      {TIM1->DIER |= TIM_DIER_CC1IE;}

#if !defined(ARDUINO_ARCH_STM32)
unsigned int millis(void);
#endif	/* ARDUINO_ARCH_AVR */
 
#endif /* __CURRENT_INC__ */	// ->
#ifdef __CURRENT__

unsigned int dmaInts;
unsigned int adc1Ints;
unsigned int adc2Ints;
unsigned int timUpInts;
unsigned int timCCInts;

unsigned int adc1DR;
unsigned int adc2DR;

#if defined(ARDUINO_ARCH_STM32)

#define newline newLine
#define flushBuf flush

#endif	/* ARDUINO_ARCH_STM32 */

extern uint32_t uwTick;

#if !defined(ARDUINO_ARCH_STM32)
unsigned int millis(void)
{
 return((unsigned int) uwTick);
}
#endif	/* ARDUINO_ARCH_AVR */

#define SAMPLES 16
uint16_t buf[2 * (SAMPLES + 8)];
uint16_t *adc1Ptr;
uint16_t *adc2Ptr;

int adcOffset;
int adcScale;
double angle;
double angleInc;
double pfAngle;
int rmsCount;

void rmsTestInit(void)
{
 memset((void *) &rmsPower, 0, sizeof(rmsPower));
 adcOffset = (1 << ADC_BITS) / 2;
 adcScale = (int) (((1 << ADC_BITS) - 1) / 2.5);
 angle = 0.0;
 angleInc = (2 * M_PI) / SAMPLES_CYCLE;
 pfAngle = ((double) M_PI) / 20.0;
 printf("angleInc %7.4f pfAngle %7.4f\n", angleInc, pfAngle);
 rmsCount = 0;
}

bool adcTest = false;
T_ADC_DATA adcData;

void rmsTest(void)
{
 flushBuf();
 rmsCount = 0;
 P_RMSPWR pwr = &rmsPower[0];
 pwr->state = initAvg;
 adcTest = true;
 pwr->samples = 0;
 pwrActive = true;
 adcRead();
#if 0
 T_ADC_DATA adcData;
 while (!pwr->done)
 {
  angle = fmod((double) rmsCount * angleInc, (double) (2 * M_PI));
  adcData.voltage = rint(adcScale * sin(angle) + adcOffset);
  adcData.current = rint(adcScale * sin(angle + pfAngle) + adcOffset);
  if (pwr->b.count < PWR_SIZE)
  {
   int ptr = pwr->b.filPtr;
   pwr->b.buf[ptr] = adcData;
   ptr += 1;
   if (ptr >= PWR_SIZE)
    ptr = 0;
   pwr->b.filPtr = ptr;
   pwr->b.count += 1;
   updatePower(pwr);
   rmsCount += 1;
  }
 }
 adcTest = false;
 pwrActive = false;
 printf("adcOffset %d adcScale %d rms %d\n",
	adcOffset, adcScale, (int) (adcScale / sqrt(2)));
 printf("offset %d vRms %d v %5.3f\n",
	pwr->v.offset, pwr->vRms, ((pwr->vRms * (float) 3300) / 4095) / 1000);
 printf("offset %d cRms %d c %5.3f\n",
	pwr->c.offset, pwr->cRms, ((pwr->cRms * (float) 3300) / 4095) / 1000);
#endif
}

void rmsCfgInit(void)
{
 maxChan = 1;
 curChan = 0;

 P_RMSPWR pwr = &rmsPower[0];
 memset((void *) pwr, 0, sizeof(T_RMSPWR));

 P_CHANCFG cfg = &chanCfg[0];
 pwr->cfg = cfg;
 cfg->pwr = pwr;
 cfg->count = 2;
 cfg->vltChan = ADC1_0;
 cfg->curChan = ADC2_0;
// cfg->buf = &pwr->b;
}

#if 0
void rmsUpdate(int sample, P_RMS rms)
{
 sample <<= SAMPLE_SHIFT;
 rms->sample = sample;
 int offset = rms->offset;
 offset = offset + ((sample - offset) >> 10);
 rms->offset = offset;
 sample -= offset;
 sample >>= SAMPLE_SHIFT;
 rms->value = sample;
 rms->sum += sample * sample;
}
#endif

uint32_t iSqrt(uint32_t a_nInput)
{
 uint32_t op  = a_nInput;
 uint32_t res = 0;
 uint32_t one = 1uL << 30;
// The second-to-top bit is set: use 1u << 14 for uint16_t type;
// use 1uL<<30 for uint32_t type
// "one" starts at the highest power of four <= than the argument.

 while (one > op)
 {
  one >>= 2;
 }
 while (one != 0)
 {
  if (op >= res + one)
  {
   op = op - (res + one);
   res = res +  2 * one;
  }
  res >>= 1;
  one >>= 2;
 }
 return(res);
}

#if 0
uint32_t start1;
uint32_t end1;
uint32_t start2;
uint32_t end2;
uint32_t start3;
uint32_t end3;

void updatePower(P_RMSPWR pwr)
{
 if (pwr->b.count != 0)
 {
  int ptr = pwr->b.empPtr;
  T_ADC_DATA adcData = pwr->b.buf[ptr];
  pwr->b.count -= 1;
  ptr++;
  if (ptr >= PWR_SIZE)
   ptr = 0;
  pwr->b.empPtr = ptr;

  dbg0Set();
  rmsUpdate(adcData.voltage, &pwr->v);
  rmsUpdate(adcData.current, &pwr->c);
  pwr->pwrSum += pwr->v.value * pwr->c.value;
  pwr->samples += 1;
  dbg0Clr();

  switch (pwr->state)
  {
  case initAvg:			/* sample to get zero offset */
   if (pwr->samples >= INITIAL_COUNT)
   {
    pwr->state = waitZero;
    pwr->lastBelow = false;
   }
   break;

  case waitZero:		/* wait for cycle start */
   if (pwr->update)		/* if update enabled */
   {
    if (pwr->v.sample >= pwr->v.offset)
    {
     if (pwr->lastBelow)
     {
      pwr->v.sum = 0;
      pwr->c.sum = 0;
      pwr->pwrSum = 0;
      pwr->samples = 0;
      pwr->state = avgData;
      pwr->update = false;
      pwr->cycleCount = CYCLE_COUNT;
     }
     pwr->lastBelow = false;
    }
    else
     pwr->lastBelow = true;
   }
   break;

  case avgData:			/* sample for cycles */
   if (pwr->v.sample >= pwr->v.offset)
   {
    if (pwr->lastBelow)
    {
     pwr->cycleCount -= 1;
     if (pwr->cycleCount <= 0)
     {
      dbg1Set();
      // uint32_t start = cpuCycles();
      resetCnt();
      startCnt();
      int samples = pwr->samples;
      pwr->sampleCount = samples;

#if 1
      pwr->vRms = iSqrt(pwr->v.sum / samples);
      pwr->cRms = iSqrt(pwr->c.sum / samples);
#else
#if 0
      uint32_t start = cpuCycles();
      uint32_t overhead = cpuCycles();
      overhead = interval(start, overhead);

      start = cpuCycles();
      pwr->vRms = iSqrt(pwr->v.sum / samples);
      pwr->cRms = iSqrt(pwr->c.sum / samples);
      uint32_t total = cpuCycles();
      total = interval(start, total);

      start = cpuCycles();
      pwr->vRms = (int) sqrt((double) pwr->v.sum / samples);
      pwr->cRms = (int) sqrt((double) pwr->c.sum / samples);
      uint32_t total1 = cpuCycles();
      total1 = interval(start, total1);
#else
      start1 = cpuCycles();
      end1 = cpuCycles();
      uint32_t overhead = interval(start1, end1);

      start2 = cpuCycles();
      pwr->vRms = iSqrt(pwr->v.sum / samples);
      pwr->cRms = iSqrt(pwr->c.sum / samples);
      end2 = cpuCycles();
      uint32_t total = interval(start2, end2);

      start3 = cpuCycles();
      pwr->vRms = (int) sqrt((double) pwr->v.sum / samples);
      pwr->cRms = (int) sqrt((double) pwr->c.sum / samples);
      end3 = cpuCycles();
      uint32_t total1  = interval(start3, end3);
#endif
      printf("overhead %u iSqrt %u sqrt %u\n",
	     (unsigned int) overhead, (unsigned int) total,
	     (unsigned int) total1);
#endif
      pwr->realPwr = pwr->pwrSum / samples;
      pwr->aprntPwr = pwr->vRms * pwr->cRms;
      pwr->pwrFactor = (100 * pwr->realPwr) / pwr->aprntPwr;
      // uint32_t total = cpuCycles();
      // total = interval(start, total);
      stopCnt();
      uint32_t total= getCycles();
      dbg1Clr();
      printf("cycles %u usec %u\n",
	     (unsigned int) total, (unsigned int) total / 72);
      printf("sample %d vSum %d vRms %d cSum %d cRms %d\n"
	     "pwrSum %d, realPwr %d aprntPwr %d pwrFactor %d\n",
	     samples, pwr->v.sum, pwr->vRms, pwr->c.sum, pwr->cRms,
	     pwr->pwrSum, pwr->realPwr, pwr->aprntPwr, pwr->pwrFactor);
      pwr->v.sum = 0;
      pwr->c.sum = 0;
      pwr->pwrSum = 0;
      pwr->samples = 0;
      pwr->done = true;
      pwr->state = cycleDone;
     }
    }
    pwr->lastBelow = false;
   }
   else
    pwr->lastBelow = true;
   break;

  case cycleDone:
   if (!pwr->done)
   {
    pwr->lastBelow = false;
    pwr->state = waitZero;
   }
   break;

  default:
   pwr->state = initAvg;
  }
  if (pwr->state != pwr->lastState)
  {
   printf("state %d lastState %d count %d\n",
	  pwr->state, pwr->lastState, pwr->samples);
   pwr->lastState = pwr->state;
  }
 }
}
#else
void updatePower(P_RMSPWR pwr)
{
 if (pwr->pwrBuf.count != 0)
 {
  dbg0Set();
  int p = pwr->pwrBuf.empPtr;
  P_PWR_DATA buf = &pwr->pwrBuf.buf[p];
  p += 1;
  if (p >= PWR_SIZE)
   p = 0;
  pwr->pwrBuf.empPtr = p;
  int samples = buf->samples;
  pwr->vRms = iSqrt(buf->vSum / samples);
  pwr->cRms = iSqrt(buf->cSum / samples);
  pwr->realPwr = buf->pwrSum / samples;
  pwr->pwrAccum =+ pwr->realPwr;
  pwr->aprntPwr = pwr->vRms * pwr->cRms;
  pwr->pwrFactor = (100 * pwr->realPwr) / pwr->aprntPwr;
  __disable_irq();
  pwr->pwrBuf.count -= 1;
  __enable_irq();
  dbg0Clr();
  pwr->displayCount -= 1;
  putx('+');
  if (pwr->displayCount <= 0)
  {
   pwr->displayCount = DISPLAY_INTERVAL;
   newline();
   printf("sample %d vSum %d vRms %d cSum %d cRms %d\n"
	  "pwrSum %d, realPwr %d aprntPwr %d pwrFactor %d\n",
	  samples, buf->vSum, pwr->vRms, buf->cSum, pwr->cRms,
	  buf->pwrSum, pwr->realPwr, pwr->aprntPwr, pwr->pwrFactor);
  }
 }
}
#endif

void printBuf(void)
{
 int count = sizeof(buf) / sizeof(uint16_t);
 uint16_t *p = (uint16_t *) buf;
 int col = 0;
 while (1)
 {
  val = *p++;
  val = (val * 3300) / 4095;
  printf("%4d ", (int) val);
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

#define HAL 1
#define TMR_TRIG 0
#define TRIG1 0

void cfgInfo(void)
{
 printf("\n**********\n");
 adcInfo(ADC1, 1);
 newline();
 adcInfo(ADC2, 2);
 newline();
 if (DMA)
 {
  dmaInfo(DMA1);
  newline();
  dmaChannelInfo(DMA1_Channel1, 1);
  newline();
 }
 flushBuf();
}

int adcCounter = 0;
int dbgCounter = 0;

void delayMillis(unsigned int t)
{
 unsigned int time = millis();
 while ((millis() - time) < t)
 {}
}

void adcRead1(void)
{
//#if HAL
// HAL_StatusTypeDef status;
//#endif	 /* HAL */
 
 // int32_t tmp = ADC1->SQR3;
 // printf("0 - %d 1 - %d\n", (int) (tmp & 0x1f), (int) ((tmp >> 5) & 0x1f));
 // printf("SQR1 %08x SQR2 %08x SQR3 %08x\n",
 //	(unsigned int) ADC1->SQR1, (unsigned int) ADC1->SQR2,
 //	(unsigned int) ADC1->SQR3);

 memset(buf, 0, sizeof(buf));
 pwrActive = false;

 dbg0Clr();
 dbg1Clr();
 dbg2Clr();
 dbg3Clr();

 adcTmrInit();
 adcTmrClrIF();
 adcTmrSetIE();
 adcTmrCC1ClrIF();
 adcTmrCC1SetIE();
 adcTmrStart();

#if HAL == 0
 if (HAL == 0)
 {
  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PDATAALIGN_WORD);
  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MDATAALIGN_WORD);
  LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1);
  LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_1, (uint32_t) &ADC1->DR,
			 (uint32_t) buf, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, (uint32_t) SAMPLES);

  // ADC1->SR = (ADC_SR_STRT | ADC_SR_JSTRT | ADC_SR_JEOC | ADC_SR_EOC | ADC_SR_AWD);
  ADC1->SR = 0;
  ADC2->SR = 0;

  LL_ADC_Enable(ADC1);
  LL_ADC_Enable(ADC2);
  ADC2->CR2 |= ADC_CR2_EXTTRIG;
  LL_ADC_REG_SetTriggerSource(ADC2, LL_ADC_REG_TRIG_SOFTWARE);

  LL_ADC_REG_SetTriggerSource(ADC1, LL_ADC_REG_TRIG_SOFTWARE);

  //ADC1->SR = 0;
  //ADC2->SR = 0;
  //ADC1->CR1 |= ADC_CR1_EOCIE;
 }
#endif /* HAL */

#define SAMPLING_TIME LL_ADC_SAMPLINGTIME_41CYCLES_5
 LL_ADC_SetChannelSamplingTime(ADC1, ADC1_0, SAMPLING_TIME);
 LL_ADC_SetChannelSamplingTime(ADC1, ADC1_1, SAMPLING_TIME);
 LL_ADC_SetChannelSamplingTime(ADC2, ADC2_0, SAMPLING_TIME);
 LL_ADC_SetChannelSamplingTime(ADC2, ADC2_1, SAMPLING_TIME);

 cfgInfo();

#if HAL
 if (HAL)
 {
  ADC2->CR2 &= ~ADC_CR2_EXTSEL_Msk;
  ADC2->CR2 |= (ADC_CR2_EXTSEL_2 | ADC_CR2_EXTSEL_1 | ADC_CR2_EXTSEL_0 |
		ADC_CR2_ADON);

  //hdma_adc1.State = HAL_DMA_STATE_READY;

  //DMA1_Channel1->CNDTR = SAMPLES;

#if TMR_TRIG
  if (TMR_TRIG == 1)
  {
   status = HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t *) buf, SAMPLES);
   printf("dma status %d\n", status);

   status = HAL_ADC_Start(&hadc1);
   printf("start status adc1 %d\n", status);
   status = HAL_ADC_Start(&hadc2);
   printf("start status adc2 %d\n", status);
  }
#else /* TMR_TRIG == 0 */
  if (TMR_TRIG == 0)
  {
#if 0
   if (0)			/* repeat calls here */
   {
    for (int i = 0; i < SAMPLES; i++)
    {
     status = HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t *) buf, SAMPLES);
     printf("dma status %d\n", status);
    }
   }
   else				/* use timer interrupt to repeat */
#else
   {
    adc1Ints = 0;
    adc2Ints = 0;
    dmaInts = 0;
    if (1)			/* setup with low level */
    {
     printf("starting adc and dma\n");
     dbg0Set();
#if SIMULTANEOUS
     if (SIMULTANEOUS)
     {
      LL_ADC_SetMultimode(ADC12_COMMON, LL_ADC_MULTI_INDEPENDENT);
     }
     else
#endif
     {
      adc1Ptr = &buf[0];
      adc2Ptr = &buf[1];
     }
     ADC1->CR1 &= ~ADC_CR1_EOCIE;
     LL_ADC_Enable(ADC1);
     delayMillis(2);
     adc1DR = ADC1->DR;

     ADC2->CR1 &= ~ADC_CR1_EOCIE;
     LL_ADC_Enable(ADC2);
     delayMillis(2);
     adc2DR = ADC2->DR;
     
#if DMA
     if (DMA)
     {
      DMA1_Channel1->CCR &= ~(DMA_CCR_EN);
      ADC1->CR2 |= ADC_CR2_DMA;
      LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_1, (uint32_t) &ADC1->DR,
			     (uint32_t) buf, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
      LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, (uint32_t) SAMPLES);
      DMA1_Channel1->CCR |= (DMA_CCR_TCIE | DMA_CCR_EN);
     }
#endif /* DMA */

     ADC1->CR1 |= ADC_CR1_EOCIE;
     ADC2->CR1 |= ADC_CR1_EOCIE;
#if (TRIG1)
     if (TRIG1)
     {
      LL_ADC_REG_SetTriggerSource(ADC1, LL_ADC_REG_TRIG_EXT_TIM1_CH1);
      LL_ADC_REG_StartConversionExtTrig(ADC1, ADC_CR2_EXTTRIG);
      LL_ADC_REG_SetTriggerSource(ADC2, LL_ADC_REG_TRIG_EXT_TIM1_CH1);
      LL_ADC_REG_StartConversionExtTrig(ADC2, ADC_CR2_EXTTRIG);
     }
#endif
#if SIMULTANEOUS
     if (SIMULTANEOUS)
     {
      LL_ADC_SetMultimode(ADC12_COMMON, LL_ADC_MULTI_DUAL_REG_SIMULT);
     }
#endif
     dbg0Clr();
    }
#if 0
    else			/* use hal to start */
    {
     status = HAL_ADCEx_MultiModeStop_DMA(&hadc1);
     printf("HAL_ADCEx_MultiModeStop_DMA %d\n", status);
     LL_ADC_SetMultimode(ADC12_COMMON, LL_ADC_MULTI_DUAL_REG_SIMULT);
     adc1DR = ADC1->DR;
     adc2DR = ADC2->DR;
     if (SIMULTANEOUS)
     {
      status = HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t *) buf, SAMPLES);
      printf("HAL_ADCEx_MultiModeStart_DMA %d\n", status);
     }
    }
#endif
    cfgInfo();
    adcCounter = SAMPLES;
   }
  }
 #endif
#endif /* TMR_TRIG */
 }
#else  /* HAL == 0 */
 else
 {
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);

#if TMR_TRIG
  if (TMR_TRIG)
  {
   LL_ADC_REG_SetTriggerSource(ADC1, LL_ADC_REG_TRIG_EXT_TIM1_CH1);
   LL_ADC_REG_StartConversionExtTrig(ADC1, ADC_CR2_EXTTRIG);
  }
#else
  else
  {
   printf("DMA1 ch1 CNDTR %d\n\n",
	  (int) LL_DMA_GetDataLength(DMA1, LL_DMA_CHANNEL_1));
   uint32_t time;
   for (int i = 0; i < SAMPLES; i++)
   {
    printf("DMA1 ch1 CNDTR %d\n",
	   (int) LL_DMA_GetDataLength(DMA1, LL_DMA_CHANNEL_1));
    LL_ADC_REG_StartConversionSWStart(ADC1);
    time = millis();
    while ((millis() - time) < 10)
    {}
   }
   newline();
  }
#endif /* TMR_TRIG */
 }
#endif /* HAL */

 cfgInfo();
 printBuf();

#if HAL == 0
 if (HAL == 0)
 {
 LL_ADC_Disable(ADC1);
 LL_ADC_Disable(ADC2);
 LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_1);
 ADC1->CR2 &= ~ADC_CR2_EXTTRIG;
 }
#endif /* HAL */
}

void adcRun(void)
{
 P_RMSPWR pwr = &rmsPower[0];
 pwr->state = initAvg;
 pwr->lastState = initAvg;
 pwrActive = true;
 adcRead();
}

void adcRead(void)
{
 if (!pwrActive)
 {
  memset(buf, 0, sizeof(buf));
  adc1Ptr = &buf[0];
  adc2Ptr = &buf[1];
 }

 dbg0Clr();
 dbg1Clr();
 dbg2Clr();
 dbg3Clr();

 adcTmrInit();
 adcTmrClrIF();
 adcTmrSetIE();
 adcTmrCC1ClrIF();
 adcTmrCC1SetIE();
 adcTmrStart();

#define SAMPLING_TIME LL_ADC_SAMPLINGTIME_41CYCLES_5
 LL_ADC_SetChannelSamplingTime(ADC1, ADC1_0, SAMPLING_TIME);
 LL_ADC_SetChannelSamplingTime(ADC1, ADC1_1, SAMPLING_TIME);
 LL_ADC_SetChannelSamplingTime(ADC2, ADC2_0, SAMPLING_TIME);
 LL_ADC_SetChannelSamplingTime(ADC2, ADC2_1, SAMPLING_TIME);

 adc1Ints = 0;
 adc2Ints = 0;
 dmaInts = 0;

 printf("starting adc and dma\n");
 dbg0Set();

 ADC1->CR1 &= ~ADC_CR1_EOCIE;
 LL_ADC_Enable(ADC1);
 delayMillis(2);
 adc1DR = ADC1->DR;

 ADC2->CR1 &= ~ADC_CR1_EOCIE;
 LL_ADC_Enable(ADC2);
 delayMillis(2);
 adc2DR = ADC2->DR;

 ADC1->CR1 |= ADC_CR1_EOCIE;
 ADC2->CR1 |= ADC_CR1_EOCIE;
 dbg0Clr();

 if (!pwrActive)
  adcCounter = SAMPLES;
}

void adcStatus(void)
{
 printf("dmaInts %u adc1Ints %u adc2Ints %u timUpInts %u timCCInts %u\n",
	dmaInts, adc1Ints, adc2Ints, timUpInts, timCCInts);
 newline();
 tmrInfo(TIM1);
 newline();
 adcInfo(ADC1, 1);
 newline();
 adcInfo(ADC2, 2);
 newline();
#if DMA
 dmaInfo(DMA1);
 newline();
 dmaChannelInfo(DMA1_Channel1, 1);
 newline();
#endif
 printBuf();
}

void adcTmrTest(void)
{
 adcTmrInit();
 adcTmrClrIF();
 adcTmrSetIE();
 adcTmrCC1ClrIF();
 adcTmrCC1SetIE();
 adcTmrStart();
 newline();
 tmrInfo(TIM1);
}

void adcTmrTestStop(void)
{
 adcTmrStop();
}

#if 0
extern "C" void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
#if defined(ARDUINO_ARCH_STM32)
 DBGPORT.write('*');
#else
 putx('*');
#endif 
}
#endif

//extern DMA_HandleTypeDef hdma_adc1;

#if DMA
extern "C" void DMA1_Channel1_IRQHandler(void)
{
//#if HAL
 dbg4Set();
 if (HAL)
 {
 HAL_DMA_IRQHandler(&hdma_adc1);
// DMA1->IFCR &= ~(DMA_CCR_TCIE | DMA_CCR_TEIE);/* DMA_IFCR_CGIF1 */
// LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_1);
// LL_ADC_Disable(ADC1);
// LL_ADC_Disable(ADC2);
 }
//#else
 else
 {
 LL_DMA_ClearFlag_TC1(DMA1);
 LL_DMA_ClearFlag_GI1(DMA1);
 }
//#endif /* HAL */
 dmaInts += 1;
 dbg4Clr();
}
#endif	/* DMA */

#define SINGLE 1

extern "C" void TIM1_UP_IRQHandler(void)
{
 adcTmrClrIF();
 timUpInts += 1;

 if (pwrActive)
 {
  P_CHANCFG chan = &chanCfg[curChan];
  if (chan->count != 0)
  {
   dbg1Set();
   LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, chan->vltChan);
   LL_ADC_REG_SetSequencerRanks(ADC2, LL_ADC_REG_RANK_1, chan->curChan);
 
   if (adcTest)
   {
    angle = fmod((double) rmsCount * angleInc, (double) (2 * M_PI));
    adcData.voltage = rint(adcScale * sin(angle) + adcOffset);
    adcData.current = rint(adcScale * sin(angle + pfAngle) + adcOffset);
    rmsCount += 1;
   }
   
#if 1
   P_RMSPWR pwr = chan->pwr;
   adc1Rms = &pwr->v;
   adc2Rms = &pwr->c;
   pwr->samples += 1;
   switch (pwr->state)
   {
   case initAvg:		/* sample to get zero offset */
    if (pwr->samples >= INITIAL_COUNT)
    {
     pwr->state = waitZero;
     pwr->lastBelow = false;
    }
    break;

   case waitZero:		/* wait for cycle start */
    if (pwr->v.sample >= pwr->v.offset)
    {
     if (pwr->lastBelow)
     {
      pwr->v.sum = 0;
      pwr->c.sum = 0;
      pwr->pwrSum = 0;
      pwr->samples = 0;
      pwr->state = avgData;
      pwr->cycleCount = CYCLE_COUNT;
     }
     pwr->lastBelow = false;
    }
    else
     pwr->lastBelow = true;
    break;
 
   case avgData:			/* sample for cycles */
    pwr->pwrSum += pwr->v.value * pwr->c.value;
    if (pwr->v.sample >= pwr->v.offset)
    {
     if (pwr->lastBelow)
     {
      pwr->cycleCount -= 1;
      if (pwr->cycleCount <= 0)
      {
       int samples = pwr->samples;
       if (pwr->pwrBuf.count < PWR_SIZE)
       {
	int ptr = pwr->pwrBuf.filPtr;
	P_PWR_DATA pwrData = &pwr->pwrBuf.buf[ptr];
	ptr += 1;
	if (ptr >= PWR_SIZE)
	 ptr = 0;
	pwr->pwrBuf.filPtr = ptr;
	pwrData->time = millis();
	pwrData->samples = samples;
	pwrData->vSum = pwr->v.sum;
	pwrData->cSum = pwr->c.sum;
	pwrData->pwrSum = pwr->pwrSum;
	pwr->pwrBuf.count += 1;
	putx('.');
       }
       pwr->v.sum = 0;
       pwr->c.sum = 0;
       pwr->pwrSum = 0;
       pwr->samples = 0;
       pwr->cycleCount = CYCLE_COUNT;
      }
     }
     pwr->lastBelow = false;
    }
    else
     pwr->lastBelow = true;
    break;

   default:
    pwr->samples = 0;
    pwr->state = avgData;
    break;
   }

   ADC2->CR2 |= (ADC_CR2_SWSTART | ADC_CR2_EXTTRIG);
   ADC1->CR2 |= (ADC_CR2_SWSTART | ADC_CR2_EXTTRIG);

   curChan += 1;
   if (curChan >= maxChan)
    curChan = 0;
#else
   P_BUFFER adcBuf = chan->buf;
   if (adcBuf->count < PWR_SIZE)
   {
    int ptr = adcBuf->filPtr;
    P_ADC_DATA data = &adcBuf->buf[ptr];
    adc1Buf = &data->voltage;
    adc2Buf = &data->current;
    ptr += 1;
    if (ptr > PWR_SIZE)
     ptr = 0;
    adcBuf->filPtr = ptr;
    adcBuf->count += 1;
 
    ADC2->CR2 |= (ADC_CR2_SWSTART | ADC_CR2_EXTTRIG);
    ADC1->CR2 |= (ADC_CR2_SWSTART | ADC_CR2_EXTTRIG);

    curChan += 1;
    if (curChan >= maxChan)
     curChan = 0;
   }
#endif
   dbg1Clr();
  }
 }
//#if 0
 else
 {
  if (adcCounter != 0)
  {
   dbg1Set();
#if 0
   switch (dbgCounter)
   {
   case 0:
    dbg3Clr();
    dbg0Set();
    break;
   case 1:
    dbg0Clr();
    dbg1Set();
    break;
   case 2:
    dbg1Clr();
    dbg2Set();
    break;
   case 3:
    dbg2Clr();
    dbg3Set();
    break;
   default:
    dbgCounter = 0;
    break;
   }
   dbgCounter += 1;
   if (dbgCounter > 3)
    dbgCounter = 0;
#endif
   adcCounter -= 1;
   if (0)
   {
    ADC1->CR1 |= ADC_CR1_EOCIE;
    ADC2->CR1 |= ADC_CR1_EOCIE;
   }
   if (TRIG1 == 0)
   {
    if (SINGLE)
    {
     if ((adcCounter & 1) == 1)
     {
      LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, ADC1_0);
      LL_ADC_REG_SetSequencerRanks(ADC2, LL_ADC_REG_RANK_1, ADC1_1);
     }
     else
     {
      LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, ADC2_0);
      LL_ADC_REG_SetSequencerRanks(ADC2, LL_ADC_REG_RANK_1, ADC2_1);
     }
    }
    ADC2->CR2 |= (ADC_CR2_SWSTART | ADC_CR2_EXTTRIG);
    ADC1->CR2 |= (ADC_CR2_SWSTART | ADC_CR2_EXTTRIG);
    dbg1Clr();
   }
  }
 }
//#endif
}

extern "C" void TIM1_CC_IRQHandler(void)
{
 adcTmrCC1ClrIF();
 timCCInts += 1;
}

extern "C" void ADC1_2_IRQHandler(void)
{
 if (ADC1->SR & ADC_SR_EOC)
 {
  dbg2Set();
  adc1Ints += 1;

  if (pwrActive)
  {
#if 0
   uint16_t *p = adc1Buf;
   if (p != 0)
    *p = ADC1->DR;
#else
   P_RMS rms = adc1Rms;
   int sample = ADC1->DR;
   if (adcTest)
    sample = adcData.voltage;
   sample <<= SAMPLE_SHIFT;
   rms->sample = sample;
   int offset = rms->offset;
   offset = offset + ((sample - offset) >> 10);
   rms->offset = offset;
   sample -= offset;
   sample >>= SAMPLE_SHIFT;
   rms->value = sample;
   rms->sum += sample * sample;
#endif
  }
//#if 0
  else
  {
   if (SIMULTANEOUS == 0)
   {
    *adc1Ptr = ADC1->DR;
    adc1Ptr += 2;
    if (TRIG1)
    {
     if ((adcCounter & 1) == 1)
      LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, ADC1_0);
     else
      LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, ADC1_1);
    }
   }
   else
   {
    if (ADC1->DR)
    {
    }
   }
  }
//#endif

  ADC1->SR &= ~ADC_SR_STRT;
  dbg2Clr();
 }
 if (ADC2->SR & ADC_SR_EOC)
 {
  dbg3Set();
  adc2Ints += 1;

  if (pwrActive)
  {
#if 0
   uint16_t *p = adc2Buf;
   if (p != 0)
    *p = ADC2->DR;
#else
   P_RMS rms = adc2Rms;
   int sample = ADC2->DR;
   if (adcTest)
    sample = adcData.current;
   sample <<= SAMPLE_SHIFT;
   rms->sample = sample;
   int offset = rms->offset;
   offset = offset + ((sample - offset) >> 10);
   rms->offset = offset;
   sample -= offset;
   sample >>= SAMPLE_SHIFT;
   rms->value = sample;
   rms->sum += sample * sample;
#endif
  }
//#if 0
  else
  {
   if (SIMULTANEOUS == 0)
   {
    *adc2Ptr = ADC2->DR;
    adc2Ptr += 2;
    if (TRIG1)
    {
     if ((adcCounter & 1) == 1)
      LL_ADC_REG_SetSequencerRanks(ADC2, LL_ADC_REG_RANK_1, ADC2_0);
     else
      LL_ADC_REG_SetSequencerRanks(ADC2, LL_ADC_REG_RANK_1, ADC2_1);
    }
   }
   else
   {
    if (ADC2->DR)
    {
    }
   }
  }
//#endif

  ADC2->SR &= ~ADC_SR_STRT;
  dbg3Clr();
 }
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
 printf("CRL     %8x ",(unsigned int) gpio->CRL);
 printf("CRH     %8x\n",(unsigned int) gpio->CRH);
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
 printf("ADC%d %08x  DR %08x\n",
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

#if defined(ARDUINO_ARCH_STM32)

extern unsigned char getNum();
extern int val;
int lastFlags;

char query(unsigned char (*get)(), const char *format, ...)
{
 va_list args;
 va_start(args, format);
 vprintf(format, args);
 va_end(args);
 flushBuf();
 char ch = get();
 newline();
 return(ch);
}

void info()
{
 if (query(&getNum, " flag [0x%x]: ", lastFlags) == 0)
 {
  val = lastFlags;
 }
 else
 {
  lastFlags = val;
 }
 printf("\n");
 flushBuf();
 if (val & 0x01)
  tmrInfo(TIM1);
#ifdef TIM2
 if (val & 0x02)
  tmrInfo(TIM2);
#endif
#ifdef TIM3
 if (val & 0x04)
  tmrInfo(TIM3);
#endif
#ifdef TIM4
 if (val & 0x08)
  tmrInfo(TIM4);
#endif
#ifdef TIM5
 if (val & 0x10)
  tmrInfo(TIM5);
#endif

 if (val & 0x20)
 {
#ifdef TIM6
  tmrInfo(TIM6);
#endif
#ifdef TIM7
  tmrInfo(TIM7);
#endif
 }

#ifdef TIM8
 if (val & 0x40)
  tmrInfo(TIM8);
#endif

 if (val & 0x80)
 {
#ifdef TIM9
  tmrInfo(TIM9);
#endif
#ifdef TIM15
  tmrInfo(TIM15);
#endif
 }

 if (val & 0x100)
 {
#ifdef TIM10
  tmrInfo(TIM10);
#endif
#ifdef TIM16
  tmrInfo(TIM16);
#endif
 }
  
 if (val & 0x200)
 {
#ifdef TIM11
  tmrInfo(TIM11);
#endif
#ifdef TIM17
  tmrInfo(TIM17);
#endif
 }
  
#ifdef TIM12
 if (val & 0x400)
  tmrInfo(TIM12);
#endif

 if (val & 0x800)		/* exti */
  extiInfo();

 if (val & 0x01000)
  gpioInfo(GPIOA);
 if (val & 0x02000)
  gpioInfo(GPIOB);
 if (val & 0x04000)
  gpioInfo(GPIOC);
#ifdef GPIOD
 if (val & 0x08000)
  gpioInfo(GPIOD);
#endif
#ifdef GPIOE
 if (val & 0x10000)
  gpioInfo(GPIOE);
#endif
#ifdef GPIOF
 if (val & 0x20000)
  gpioInfo(GPIOF);
#endif
#ifdef GPIOG
 if (val & 0x40000)
  gpioInfo(GPIOH);
#endif

 if (val & 0x100000)
  usartInfo(USART1, "DBG");
 if (val & 0x200000)
  usartInfo(USART3, "WIFI");

#ifdef I2C1
 if (val & 0x400000)
  i2cInfo(I2C1, "I2C1");
#endif
}
#endif	/* ARDUINO_ARCH_STM32 */

#endif	/* __CURRENT__ */
