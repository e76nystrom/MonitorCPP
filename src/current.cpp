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
#else  /* ARDUINO_ARCH_STM32 */
#include "adc.h"
#include "config.h"
#include "serialio.h"
//#include "dma.h"
#include "tim.h"
#endif	/* ARDUINO_ARCH_STM32 */

#include "stm32Info.h"

#ifdef EXT
#undef EXT
#endif	/* EXT */

#define EXT
#if defined(ARDUINO_ARCH_STM32)
#include "monitor.h"
#endif	/* ARDUINO_ARCH_STM32 */

#include "current.h"
#include "cyclectr.h"

#define DMA 0
#define SIMULTANEOUS 0

#if defined(STM32MON)
#if 0
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
#endif	/* 0 */
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
#define CUR_SIZE 32
#define INITIAL_COUNT 10000
#define CYCLE_COUNT 60
#define SAMPLE_SHIFT 8

#define CYCLES_SEC 60
#define SAMPLES_CYCLE 16
#define CHAN_PAIRS 2
#define ADC_BITS 12

#define ADC_MAX ((1 << ADC_BITS) - 1)
#define VREF_1000 3300

#define INITIAL_SAMPLES 10000
#define RMS_INITIAL int(INITIAL_SAMPLES / SAMPLES_CYCLE) * SAMPLES_CYCLE

enum pwrState {initAvg, waitZero, avgData, cycleDone};
enum curState {initCur, avgCur, curDone};
enum CHAN_TYPE {POWER_CHAN, CURRENT_CHAN};

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
 uint64_t sum;			/* sum of squares */
 int min;
 int max;
} T_RMS, *P_RMS;

#if 0
typedef struct s_buffer
{
 int filPtr;			/* fill pointer */
 int empPtr;			/* empty pointer */
 int count;			/* number in buffer */
 T_ADC_DATA buf[PWR_SIZE];	/* buffer */
} T_BUFFER, *P_BUFFER;
#endif	/* 0 */

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

#define DISPLAY_INTERVAL (12 * 1000)
#define MEASURE_INTERVAL (60 * 1000)

typedef struct s_rmsPwr
{
 pwrState state;		/* curent state */
 pwrState lastState;		/* last state */
 bool lastBelow;		/* last sample below voltage offset */
 int cycleCount;		/* cycle counter */
 T_RMS v;			/* voltage */
 T_RMS c;			/* current */
 int samples;			/* sample counter */
 int pwrSum;			/* power sum */
 int sampleCount;		/* sample count for last reading */
 int displayTime;		/* time for last display */
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
 T_PWR_BUF pwrBuf;
} T_RMSPWR, *P_RMSPWR;

typedef struct s_curData
{
 uint32_t time;			/* time of reading */
 uint64_t sum;			/* current sum of squares */
 int samples;			/* samples */
 int offset;
 int min;
 int max;
} T_CUR_DATA, *P_CUR_DATA;

typedef struct s_curBuf
{
 int filPtr;			/* fill pointer */
 int empPtr;			/* empty pointer */
 int count;			/* number in buffer */
 T_CUR_DATA buf[PWR_SIZE];	/* buffer */
} T_CUR_BUF, *P_CUR_BUF;

typedef struct s_rmsCur
{
 curState state;		/* curent measurement state */
 curState lastState;		/* last curent measurement state */
 T_RMS c;			/* current */
 int samples;			/* sample counter */
 int rms;			/* rms current */
 uint64_t rmsSum;		/* sum for rms calculation */
 int measureTime;		/* time of last measurement */
 int rmsSamples;		/* samples for rms calculation */
 int minuteRms;			/* rms value for one minute */
 int displayTime;		/* time of last display */
 int minuteCount;
 struct s_chanCfg *cfg;		/* channel configuration */
 T_CUR_BUF curBuf;
} T_RMSCUR, *P_RMSCUR;

typedef struct s_chanCfg
{
 CHAN_TYPE type;		/* channel type */
 int curChan;			/* current channel */
 int vltChan;			/* voltage channel */
 int curScale;			/* current scale */
 int voltScale;			/* voltage scale */
 union
 {
  P_RMSPWR pwr;			/* rms power data */
  P_RMSCUR cur;			/* rms current data */
 };
} T_CHANCFG, *P_CHANCFG;

#define MAX_CHAN 4
#define MAX_CHAN_POWER 2
#define MAX_CHAN_CURRENT 4

EXT uint32_t clockFreq;
EXT uint32_t tmrFreq;
EXT T_RMSPWR rmsPower[MAX_CHAN_POWER];
EXT T_RMSCUR rmsCurrent[MAX_CHAN_CURRENT];

EXT bool pwrActive;
EXT int maxChan;
EXT int curChan;
EXT T_CHANCFG chanCfg[MAX_CHAN];
EXT P_RMS adc1Rms;
EXT P_RMS adc2Rms;

EXT int testIndex;
EXT int testChan;
EXT int16_t testData[2 * SAMPLES_CYCLE];
EXT int testOffset;

void rmsTestInit(void);
void rmsTest(void);

void rmsCfgInit(void);
//#define POLL_UPDATE_POWER
#if defined(POLL_UPDATE_POWER)
void rmsUpdate(int sample, P_RMS rms);
#else
void currentUpdate();
void updatePower(P_CHANCFG chan);
void updateCurrent(P_CHANCFG chan);
#endif	/* POLL_UPDATE_POWER */

void adcRead(void);
void adcRun(void);
void adcRead1(void);
void adcStatus(void);
void adcTmrTest(void);

inline uint32_t cpuCycles(void) { return(SysTick->VAL); }
inline uint32_t interval(uint32_t start, uint32_t end)
{
 if (end > start)
  return(start + 0x01000000 - end);
 else
  return(start - end);
}

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

bool extTrig;
bool updChannel;

#if defined(ARDUINO_ARCH_STM32)
//#define putDbg(ch) DBGPORT.write(ch);
#define putDbg(ch)
#else  /* ARDUINO_ARCH_STM32 */
#define putDbg(ch) putx(ch)
#endif /* ARDUINO_ARCH_STM32 */

#if defined(STM32F1)
#define SAMPLING_TIME LL_ADC_SAMPLINGTIME_41CYCLES_5
#endif	/* STM32F1 */
#if defined(STM32F3)
#define SAMPLING_TIME LL_ADC_SAMPLINGTIME_61CYCLES_5
#endif	/* STM32F3 */

#if defined(ARDUINO_ARCH_STM32)

#define newline newLine
#define flushBuf flush

#endif	/* ARDUINO_ARCH_STM32 */

#if !defined(ARDUINO_ARCH_STM32)
extern volatile uint32_t uwTick;

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

#if defined(POLL_UPDATE_POWER)
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
#endif	/* POLL_UPDATE_POWER */
}

void rmsCfgInit(void)
{
 maxChan = 2;			/* maximum channel */
 curChan = 0;			/* current channel */

 P_CHANCFG cfg = &chanCfg[0];
 memset((void *) cfg, 0, sizeof(chanCfg));

 P_RMSPWR pwr = &rmsPower[0];
 memset((void *) pwr, 0, sizeof(rmsPower));

 cfg->type = POWER_CHAN;
 cfg->curScale = 20.0;
 cfg->voltScale = 1.0;
 pwr->cfg = cfg;
 pwr->state = initAvg;
 pwr->lastState = initAvg;
 cfg->pwr = pwr;
 cfg->vltChan = ADC1_0;
 cfg->curChan = ADC2_0;

 cfg++;

 P_RMSCUR cur = &rmsCurrent[0];
 memset((void *) cur, 0, sizeof(rmsCurrent));
 
 cfg->type = CURRENT_CHAN;
 cfg->curScale = 20.0;
 cur->cfg = cfg;
 cur->state = initCur;
 cur->lastState = initCur;
 cfg->cur = cur;
 cfg->curChan = ADC1_1;
}

#if defined(POLL_UPDATE_POWER)
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
#endif	/* POLL_UPDATE_POWER */

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

#if defined(POLL_UPDATE_POWER)
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
#define USE_ISQRT
#if defined(USE_ISQRT)
      pwr->vRms = iSqrt(pwr->v.sum / samples);
      pwr->cRms = iSqrt(pwr->c.sum / samples);
#else  /* USE_ISQRT */
#define TIME_ISQRT
#if TIME_ISQRT
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
#else  /* TIME_ISQRT */
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
#endif	/* TIME_ISQRT */
      printf("overhead %u iSqrt %u sqrt %u\n",
	     (unsigned int) overhead, (unsigned int) total,
	     (unsigned int) total1);
#endif	/* USE_ISQRT */
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

#else /* PoLL_UPDATE_POWER */

void currentUpdate()
{
 if (pwrActive)
 {
  for (int i = 0; i < maxChan; i++)
  {
   P_CHANCFG chan = &chanCfg[i];
   if (chan->type == POWER_CHAN)
   {
    updatePower(chan);
   }
   else if (chan->type == CURRENT_CHAN)
   {
    updateCurrent(chan);
   }
  }
 }
}

char *i64toa(uint32_t val, char *buf)
{
 char tmp[32];
 int count = 0;
 char *p = tmp;
 while (val != 0)
 {
  int dig = (int) (val % 10);
  *p++ = (char) (dig + '0');
  count += 1;
  val /= 10;
 }
 char *p1 = buf;
 if (count == 0)
  *p1++ = '0';
 else
 {
  while (--count >= 0)
   *p1++ = *--p;
 }
 *p1++ = 0;
 return(buf);
}

void updatePower(P_CHANCFG chan)
{
 P_RMSPWR pwr = chan->pwr;
 while (pwr->pwrBuf.count != 0)
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
  putDbg('P');
  uint32_t t = millis();
  if ((t - pwr->displayTime) >= DISPLAY_INTERVAL)
  {
   pwr->displayTime += DISPLAY_INTERVAL;
   newline();
   printf("sample %d vSum %d vRms %d cSum %d cRms %d\n"
	  "pwrSum %d, realPwr %d aprntPwr %d pwrFactor %d\n",
	  samples, buf->vSum, pwr->vRms, buf->cSum, pwr->cRms,
	  buf->pwrSum, pwr->realPwr, pwr->aprntPwr, pwr->pwrFactor);
  }
 }
}

void updateCurrent(P_CHANCFG chan)
{
 P_RMSCUR cur = chan->cur;
 while (cur->curBuf.count != 0)
 {
  dbg0Set();
  int p = cur->curBuf.empPtr;
  P_CUR_DATA buf = &cur->curBuf.buf[p];
  p += 1;
  if (p >= CUR_SIZE)
   p = 0;
  cur->curBuf.empPtr = p;
  int samples = buf->samples;
  cur->rms = iSqrt(buf->sum / samples);

  cur->rmsSamples += samples;
  cur->rmsSum += buf->sum;
  cur->minuteCount += 1;

  __disable_irq();
  cur->curBuf.count -= 1;
  __enable_irq();
  dbg0Clr();
  putDbg('C');

  uint32_t t = millis();
  if ((t - cur->measureTime) >= MEASURE_INTERVAL)
  {
   char convBuf[32];
   cur->measureTime += MEASURE_INTERVAL;
   cur->minuteRms = iSqrt(int(cur->rmsSum / cur->rmsSamples));
   int maRms = ((cur->minuteRms * VREF_1000) / ADC_MAX) * chan->curScale;
   int amps = maRms / 1000;
   int mAmps = maRms - (amps * 1000);
#if !defined(ARDUINO_ARCH_STM32)
   newline();
#endif	/* ARDUINO_ARCH_STM32 */
   printf("minute rms count %d samples %5d sum %10s rms %4d %4d %2d.%03d\n",
	  cur->minuteCount, cur->rmsSamples, i64toa(cur->rmsSum, convBuf),
	  cur->minuteRms, maRms, amps, mAmps);
   cur->minuteCount = 0;
   cur->rmsSamples = 0;
   cur->rmsSum = 0;

#if defined(ARDUINO_ARCH_STM32)
#if defined(WIFI_ENA)
   char buf[128];
   char *p;
   p = cpyStr(buf, F0("node=" EMONCMS_NODE "&csv="));
   sprintf(p, "%d.%03d", amps, mAmps);
   emonData(buf);
#endif	/* WIFI_ENA */
#endif	/* ARDUINO_ARCH_STM32 */
  }
  
  if ((t - cur->displayTime) >= DISPLAY_INTERVAL)
  {
   char convBuf[32];
   cur->displayTime += DISPLAY_INTERVAL;
   int offset = buf->offset >> SAMPLE_SHIFT;
#if !defined(ARDUINO_ARCH_STM32)
   newline();
#endif	/* ARDUINO_ARCH_STM32 */
   printf("sample %3d min %4d %4d max %4d %4d offset %4d sum %10s rms %4d\n",
	  samples, buf->min, offset - buf->min, buf->max, buf->max - offset,
	  offset, i64toa(buf->sum, convBuf), (cur->rms * VREF_1000) / ADC_MAX);
  }
 }
}

#endif	/* POLL_UPDATE_POWER */

void printBufC(void)
{
 int count = sizeof(buf) / sizeof(uint16_t);
 uint16_t *p = (uint16_t *) buf;
 int col = 0;
 while (1)
 {
  uint16_t val = *p++;
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
 tmrInfo(TIM1);
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
int adc1Counter;
int adc2Counter;
int adc1Chan;
int adc2Chan;
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

 memset(buf, 0, sizeof(buf));
 pwrActive = false;

 extTrig = true;
 updChannel = true;

 dbg0Clr();
 dbg1Clr();
 dbg2Clr();
 dbg3Clr();

#if defined(STM32F3)
 if (extTrig)
 {
  printf("extTrig gpio\n");
  LL_GPIO_SetPinMode(TIM1_CH1_GPIO_Port, TIM1_CH1_Pin, LL_GPIO_MODE_ALTERNATE);
  if (TIM1_CH1_Pin < 8)
   LL_GPIO_SetAFPin_0_7(TIM1_CH1_GPIO_Port, TIM1_CH1_Pin, GPIO_AF2_TIM1);
  else
   LL_GPIO_SetAFPin_8_15(TIM1_CH1_GPIO_Port, TIM1_CH1_Pin, GPIO_AF2_TIM1);
 }
#endif	/* STM323F3 */

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

 LL_ADC_SetChannelSamplingTime(ADC1, ADC1_0, SAMPLING_TIME);
 LL_ADC_SetChannelSamplingTime(ADC1, ADC1_1, SAMPLING_TIME);
 LL_ADC_SetChannelSamplingTime(ADC2, ADC2_0, SAMPLING_TIME);
 LL_ADC_SetChannelSamplingTime(ADC2, ADC2_1, SAMPLING_TIME);

 cfgInfo();

#if HAL
 if (HAL)
 {
#if defined(STM32F1)
  ADC2->CR2 &= ~ADC_CR2_EXTSEL_Msk;
  ADC2->CR2 |= (ADC_CR2_EXTSEL_2 | ADC_CR2_EXTSEL_1 | ADC_CR2_EXTSEL_0 |
		ADC_CR2_ADON);
#endif	/* STM32F1 */
#if defined(STM32F3)
  if (!extTrig)
  {
   printf("software trigger adc\n");
   LL_ADC_REG_SetTriggerSource(ADC1, LL_ADC_REG_TRIG_SOFTWARE);
   LL_ADC_REG_SetTriggerSource(ADC2, LL_ADC_REG_TRIG_SOFTWARE);
  }
  else
  {
   printf("timer trigger adc\n");
   LL_ADC_REG_SetTriggerSource(ADC1, LL_ADC_REG_TRIG_EXT_TIM1_CH1_ADC12);
   LL_ADC_REG_SetTriggerSource(ADC2, LL_ADC_REG_TRIG_EXT_TIM1_CH1_ADC12);
  }
#endif	/* STM32F3 */
  dbg0Set();

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
// #define REPEAT_CALLS
#if defined(REPEAT_CALLS)
   if (0)			/* repeat calls here */
   {
    for (int i = 0; i < SAMPLES; i++)
    {
     status = HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t *) buf, SAMPLES);
     printf("dma status %d\n", status);
    }
   }
   else				/* use timer interrupt to repeat */
#else  /* REPEAT_CALLS */
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
#endif	/* SIMULTANEOUS */
     {
      adc1Ptr = &buf[0];
      adc2Ptr = &buf[1];
      adc1Counter = SAMPLES;
      adc2Counter = SAMPLES;
      adc1Chan = 0;
      adc2Chan = 0;
      if (updChannel)
      {
       LL_ADC_REG_SetSequencerDiscont(ADC1, LL_ADC_REG_SEQ_DISCONT_DISABLE);
       LL_ADC_REG_SetSequencerLength(ADC1, LL_ADC_REG_SEQ_SCAN_DISABLE);
       LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, ADC1_0);
       LL_ADC_REG_SetSequencerDiscont(ADC2, LL_ADC_REG_SEQ_DISCONT_DISABLE);
       LL_ADC_REG_SetSequencerLength(ADC2, LL_ADC_REG_SEQ_SCAN_DISABLE);
       LL_ADC_REG_SetSequencerRanks(ADC2, LL_ADC_REG_RANK_1, ADC2_0);
      }
      else
      {
       LL_ADC_REG_SetSequencerDiscont(ADC1, LL_ADC_REG_SEQ_DISCONT_1RANK);
       LL_ADC_REG_SetSequencerLength(ADC1, LL_ADC_REG_SEQ_SCAN_ENABLE_2RANKS);
       LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, ADC1_0);
       LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_2, ADC1_1);
       LL_ADC_REG_SetSequencerDiscont(ADC2, LL_ADC_REG_SEQ_DISCONT_1RANK);
       LL_ADC_REG_SetSequencerLength(ADC2, LL_ADC_REG_SEQ_SCAN_ENABLE_2RANKS);
       LL_ADC_REG_SetSequencerRanks(ADC2, LL_ADC_REG_RANK_1, ADC2_0);
       LL_ADC_REG_SetSequencerRanks(ADC2, LL_ADC_REG_RANK_2, ADC2_1);
      }
     }
#if defined(STM32F1)
     ADC1->CR1 &= ~ADC_CR1_EOCIE;
#endif	/* STM32F1 */
#if defined(STM32F3)
     ADC1->IER &= ~(ADC_IER_ADRDYIE | ADC_IER_EOCIE |
		    ADC_IER_EOSIE | ADC_IER_OVRIE);
#endif	/* STM32F3 */
     LL_ADC_Enable(ADC1);
     delayMillis(2);
     adc1DR = ADC1->DR;

#if defined(STM32F1)
     ADC2->CR1 &= ~ADC_CR1_EOCIE;
#endif	/* STM32F1 */
#if defined(STM32F3)
     ADC2->IER &= ~(ADC_IER_ADRDYIE | ADC_IER_EOCIE |
		    ADC_IER_EOSIE | ADC_IER_OVRIE);
#endif	/* STM32F3 */
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

#if defined(STM32F1)
     ADC1->CR1 |= ADC_CR1_EOCIE;
     ADC2->CR1 |= ADC_CR1_EOCIE;
#endif	/* STM32F1 */
#if defined(STM32F3)
     LL_ADC_EnableIT_EOC(ADC1);
     LL_ADC_EnableIT_EOC(ADC2);
     if (extTrig)
     {
      LL_ADC_REG_StartConversion(ADC1);
      LL_ADC_REG_StartConversion(ADC2);
     }
#endif	/* STM32F3 */
#if (TRIG1)
     if (TRIG1)
     {
      LL_ADC_REG_SetTriggerSource(ADC1, LL_ADC_REG_TRIG_EXT_TIM1_CH1);
      LL_ADC_REG_StartConversionExtTrig(ADC1, ADC_CR2_EXTTRIG);
      LL_ADC_REG_SetTriggerSource(ADC2, LL_ADC_REG_TRIG_EXT_TIM1_CH1);
      LL_ADC_REG_StartConversionExtTrig(ADC2, ADC_CR2_EXTTRIG);
     }
#endif	/* TRIG1 */
#if SIMULTANEOUS
     if (SIMULTANEOUS)
     {
      LL_ADC_SetMultimode(ADC12_COMMON, LL_ADC_MULTI_DUAL_REG_SIMULT);
     }
#endif /* SIMULTANEOUS */
     dbg0Clr();
    }
//#define HAL_TO_START
#if defined(HAL_TO_START)
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
#endif	/* HAL_TO_START */
    cfgInfo();
    adcCounter = SAMPLES;
   }
  }
#endif	/* REPEAT_CALLS */
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
  else
#else  /* TMR_TRIG */
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
 printBufC();

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
 rmsCfgInit();		/* initialize configuration */
 adcTest = false;
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

 uint32_t count = tmrFreq / (CYCLES_SEC * SAMPLES_CYCLE * maxChan);
 uint16_t psc = 1;
 while (true)
 {
  if ((count / psc) < 65536)
   break;
  psc += 1;
 }
 printf("timer 1 preScaler %u count %u\n",
	(unsigned int) psc, (unsigned int) count);
 adcTmrScl(psc - 1);
 adcTmrMax(count);
  
 adcTmrInit();
 adcTmrClrIF();
 adcTmrSetIE();
 adcTmrCC1ClrIF();
 adcTmrCC1SetIE();

 LL_ADC_SetChannelSamplingTime(ADC1, ADC1_0, SAMPLING_TIME);
 LL_ADC_SetChannelSamplingTime(ADC1, ADC1_1, SAMPLING_TIME);
 LL_ADC_SetChannelSamplingTime(ADC2, ADC2_0, SAMPLING_TIME);
 LL_ADC_SetChannelSamplingTime(ADC2, ADC2_1, SAMPLING_TIME);

 adc1Ints = 0;
 adc2Ints = 0;
 dmaInts = 0;

 printf("starting adc and dma\n");
 dbg0Set();

#if defined(STM32F1)
 ADC1->CR1 &= ~ADC_CR1_EOCIE;
#endif	/* STM32F1 */
#if defined(STM32F3)
 ADC1->IER &= ~(ADC_IER_ADRDYIE | ADC_IER_EOCIE |
		ADC_IER_EOSIE | ADC_IER_OVRIE);
#endif	/* STM32F3 */
 LL_ADC_Enable(ADC1);
 delayMillis(2);
 adc1DR = ADC1->DR;

#if defined(STM32F1)
 ADC2->CR1 &= ~ADC_CR1_EOCIE;
#endif	/* STM32F1 */
#if defined(STM32F3)
 ADC1->IER &= ~(ADC_IER_ADRDYIE | ADC_IER_EOCIE |
		ADC_IER_EOSIE | ADC_IER_OVRIE);
#endif	/* STM32F3 */
 LL_ADC_Enable(ADC2);
 delayMillis(2);
 adc2DR = ADC2->DR;

#if defined(STM32F1)
 ADC1->CR1 |= ADC_CR1_EOCIE;
 ADC2->CR1 |= ADC_CR1_EOCIE;
#endif	/* STM32F1 */
#if defined(STM32F3)
 LL_ADC_EnableIT_EOC(ADC1);
 LL_ADC_EnableIT_EOC(ADC2);
#endif	/* STM32F3 */
 dbg0Clr();

 adcInfo(ADC1, 1);
 newline();
 adcInfo(ADC2, 2);
 newline();
 flushBuf();
 if (!pwrActive)
  adcCounter = SAMPLES;

 adcTmrStart();			/* start timer */
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
#endif	/* DMA */
 printBufC();
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
 putDbg('*');
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
//#else /* HAL */
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

#if defined(STM32F1)
extern "C" void TIM1_UP_IRQHandler(void)
#endif	/* STM32F1 */
#if defined(STM32F3)
extern "C" void TIM1_UP_TIM16_IRQHandler(void)
#endif	/* STM32F3 */
{
 adcTmrClrIF();
 timUpInts += 1;

 if (pwrActive)
 {
  P_CHANCFG chan = &chanCfg[curChan];
  dbg1Set();
 
  if (adcTest)
  {
   angle = fmod((double) rmsCount * angleInc, (double) (2 * M_PI));
   adcData.voltage = rint(adcScale * sin(angle) + adcOffset);
   adcData.current = rint(adcScale * sin(angle + pfAngle) + adcOffset);
   rmsCount += 1;
  }
   
  if (chan->type == POWER_CHAN)
  {
   testChan = chan->vltChan;
   LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, chan->vltChan);
   LL_ADC_REG_SetSequencerRanks(ADC2, LL_ADC_REG_RANK_1, chan->curChan);
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
 
   case avgData:		/* sample for cycles */
    pwr->pwrSum += pwr->v.value * pwr->c.value;
    if (pwr->v.sample >= pwr->v.offset)
    {
     if (pwr->lastBelow)
     {
#if defined(Dbg5_Pin)
      if ((Dbg5_GPIO_Port->ODR & Dbg5_Pin) != 0)
       dbg5Clr();
      else
       dbg5Set();
#endif	/* Dbg5_Pin */
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
	putDbg('p');
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

#if defined(STM32F1)
   ADC2->CR2 |= (ADC_CR2_SWSTART | ADC_CR2_EXTTRIG);
   ADC1->CR2 |= (ADC_CR2_SWSTART | ADC_CR2_EXTTRIG);
#endif	/* STM32F1 */
#if defined(STM32F3)
   LL_ADC_REG_StartConversion(ADC2);
   LL_ADC_REG_StartConversion(ADC1);
#endif	/* STM32F3 */
  }
  else if (chan->type == CURRENT_CHAN)
  {
   testChan = chan->curChan;
   LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, chan->curChan);
   P_RMSCUR cur = chan->cur;
   adc1Rms = &cur->c;
   cur->samples += 1;
   switch (cur->state)
   {
   case initCur:
    if (cur->samples >= INITIAL_COUNT)
    {
     cur->state = avgCur;
     cur->samples = 0;
     uint32_t t = millis();
     cur->displayTime = t;
     cur->measureTime = t;
     cur->minuteCount = 0;
     cur->rmsSamples = 0;
     cur->rmsSum = 0;
    }
    break;

   case avgCur:
   {
    int samples = cur->samples;
    if (samples >= (CYCLES_SEC * SAMPLES_CYCLE))
    {
#if defined(Dbg4_Pin)
     if ((Dbg4_GPIO_Port->ODR &  Dbg4_Pin) != 0)
      dbg4Clr();
     else
      dbg4Set();
#endif	/* Dbg4_Pin */
     if (cur->curBuf.count < CUR_SIZE)
     {
      int ptr = cur->curBuf.filPtr;
      P_CUR_DATA curData = &cur->curBuf.buf[ptr];
      ptr += 1;
      if (ptr >= CUR_SIZE)
       ptr = 0;
      cur->curBuf.filPtr = ptr;
      curData->time = millis();
      curData->samples = samples;
      P_RMS c = &cur->c;
      curData->sum = c->sum;
      curData->offset = c->offset;
      curData->min = c->min;
      curData->max = c->max;
      c->max = 0;
      c->min = 1 << ADC_BITS;
      cur->curBuf.count += 1;
      putDbg('c');
     }
     cur->c.sum = 0;
     cur->samples = 0;
    }
    break;
   }

   case curDone:
    break;
     
   default:
    cur->state = initCur;
    break;
   }

#if defined(STM32F1)
   ADC1->CR2 |= (ADC_CR2_SWSTART | ADC_CR2_EXTTRIG);
#endif	/* STM32F1 */
#if defined(STM32F3)
   LL_ADC_REG_StartConversion(ADC1);
#endif	/* STM32F3 */
  }

  curChan += 1;
  if (curChan >= maxChan)
   curChan = 0;
  dbg1Clr();
 } /* pwrActive */
 else				/* test mode */
 {
  if (adc1Counter > 0)
  {
   adc1Counter -= 1;
   if (extTrig == 0)
   {
#if defined(STM32F1)
    ADC2->CR2 |= (ADC_CR2_SWSTART | ADC_CR2_EXTTRIG);
    ADC1->CR2 |= (ADC_CR2_SWSTART | ADC_CR2_EXTTRIG);
#endif	/* STM32F1 */
#if defined(STM32F3)
    LL_ADC_REG_StartConversion(ADC2);
    LL_ADC_REG_StartConversion(ADC1);
#endif	/* STM32F3 */
    dbg1Clr();
   }
  } /* adc1Counter > 0 */
 } /* pwrActive */
}

extern "C" void TIM1_CC_IRQHandler(void)
{
 adcTmrCC1ClrIF();
 timCCInts += 1;
}

extern "C" void ADC1_2_IRQHandler(void)
{
 if (				/* if adc1 interrupt active */
#if defined(STM32F1)
  ADC1->SR & ADC_SR_EOC
#endif	/* STM32F1 */
#if defined(STM32F3)
  LL_ADC_IsActiveFlag_EOC(ADC1)
#endif	/* STM32F3 */
  )
 {
  dbg2Set();
  adc1Ints += 1;

  if (pwrActive)		/* if measuring power */
  {
   P_RMS rms = adc1Rms;
   int sample = ADC1->DR;
   if (adcTest)
    sample = adcData.voltage;
   if (sample > rms->max)
    rms->max = sample;
   if (sample < rms->min)
    rms->min = sample;
   int offset = rms->offset;
   if (testChan == ADC1_1)
   {
    if (testIndex < (2 * SAMPLES_CYCLE))
    {
     testData[testIndex] = sample;
     testIndex += 1;
     testOffset = offset;
    }
   }
   sample <<= SAMPLE_SHIFT;
   rms->sample = sample;
   offset = offset + ((sample - offset) >> 10);
   rms->offset = offset;
   sample -= offset;
   sample >>= SAMPLE_SHIFT;
   rms->value = sample;
   rms->sum += sample * sample;
  }
  else				/* if test mode */
  {
   *adc1Ptr = ADC1->DR;
   adc1Ptr += 2;
   if (updChannel)
   {
    adc1Chan += 1;
    if ((adc1Chan & 1) == 1)
     LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, ADC1_1);
    else
     LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, ADC1_0);
   }
  } /* pwrActive */

#if defined(STM32F1)
  ADC1->SR &= ~ADC_SR_STRT;
#endif	/* STM32F1 */
#if defined(STM32F3)
  ADC1->ISR = ADC_ISR_ADRDY | ADC_ISR_EOSMP | ADC_ISR_EOC | ADC_ISR_EOS;
#endif	/* STM32F3 */
  dbg2Clr();
 } /* adc1 interrupt active */
 
 if (				/* if adc2 interrrupt active */
#if defined(STM32F1)
  ADC2->SR & ADC_SR_EOC
#endif	/* STM32F1 */
#if defined(STM32F3)
  LL_ADC_IsActiveFlag_EOC(ADC2)
#endif	/* STM32F3 */
  )
 {
  dbg3Set();
  adc2Ints += 1;

  if (pwrActive)		/* measuring power */
  {
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
  }
  else				/* test mode */
  {
   *adc2Ptr = ADC2->DR;
   adc2Ptr += 2;
   if (updChannel)
   {
    adc2Chan += 1;
    if ((adc2Chan & 1) == 1)
     LL_ADC_REG_SetSequencerRanks(ADC2, LL_ADC_REG_RANK_1, ADC2_1);
    else
     LL_ADC_REG_SetSequencerRanks(ADC2, LL_ADC_REG_RANK_1, ADC2_0);
   }
  } /* pwrActive */

#if defined(STM32F1)
  ADC2->SR &= ~ADC_SR_STRT;
#endif	/* STM32F1 */
#if defined(STM32F3)
  ADC2->ISR = ADC_ISR_ADRDY | ADC_ISR_EOSMP | ADC_ISR_EOC | ADC_ISR_EOS;
#endif	/* STM32F3 */
  dbg3Clr();
 } /* adc2 interrupt active */
}

#endif	/* __CURRENT__ */
