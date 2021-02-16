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

#if MAX_CHAN_POWER > 0
T_RMSPWR rmsPower[MAX_CHAN_POWER];
#endif	/* MAX_CHAN_POWER */
#if MAX_CHAN_RMS > 0
T_RMSCHAN rmsData[MAX_CHAN_RMS];
#endif	/* MAX_CHAN_RMS */

T_CHANCFG chanCfg[MAX_CHAN] =
{
#if 1
 {POWER_CHAN, 'p', {ADC1, ADC1_0}, {ADC2, ADC2_0}, 20, 0.15119 * 10,
  (P_RMSPWR) &rmsPower[0]},
 {RMS_CHAN, 'c', {ADC1, ADC1_0}, {0, 0}, 20, 0,
  (P_RMSPWR) &rmsData[0]},
 {RMS_CHAN, 'v', {ADC2, ADC2_0} , {0, 0}, 0.15119 * 10, 0,
  (P_RMSPWR) &rmsData[1]},
#else
 {RMS_CHAN, 'v', {ADC2, ADC2_0} , {0, 0}, 0.15119 * 10, 0,
  (P_RMSPWR) &rmsData[0]},
#endif	/* 0 */
};

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

#if defined(STM32F1)
#include "stm32f1xx_ll_adc.h"
#endif	/* STM32F1 */

#if defined(STM32F4)
#include "stm32f4xx_ll_adc.h"
#endif

#if !defined(EXT)
#define EXT extern
#endif	/* EXT */

#define PWR_SIZE 32
#define RMS_SIZE 32
#define INITIAL_COUNT 10000
#define CYCLE_COUNT 60
#define SAMPLE_SHIFT 8

#define CYCLES_SEC 60
#define SAMPLES_CYCLE 16
#define CHAN_PAIRS 2
#define ADC_BITS 12

#define ADC_MAX ((1 << ADC_BITS) - 1)
#define VREF_1000 3300
inline int scaleAdc(int val) {return((val * VREF_1000) / ADC_MAX);}

#define INITIAL_SAMPLES 10000
#define RMS_INITIAL int(INITIAL_SAMPLES / SAMPLES_CYCLE) * SAMPLES_CYCLE

enum pwrState {initAvg, waitZero, avgData, cycleDone};
enum chanState {initRms, avgRms, rmsDone};
enum CHAN_TYPE {POWER_CHAN, RMS_CHAN};
enum ADC_NUM {ADC_1, ADC_2, ADC_X};

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
 char label;			/* channel label */
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
 int samples;			/* samples */
 uint64_t sum;			/* current sum of squares */
 int offset;
 int min;
 int max;
} T_CHAN_DATA, *P_CHAN_DATA;

typedef struct s_curBuf
{
 int filPtr;			/* fill pointer */
 int empPtr;			/* empty pointer */
 int count;			/* number in buffer */
 T_CHAN_DATA buf[RMS_SIZE];	/* buffer */
} T_CHAN_BUF, *P_CHAN_BUF;

typedef struct s_rmsChan
{
 chanState state;		/* curent measurement state */
 chanState lastState;		/* last curent measurement state */
 char label;			/* channel label */
 ADC_TypeDef *adc;		/* pointer to adc hardware */
 P_RMS *adcRms;			/* pointer to isr pointer */
 T_RMS rmsAccum;			/* current */
 int samples;			/* sample counter */
 int rms;			/* rms current */
 uint64_t rmsSum;		/* sum for rms calculation */
 int measureTime;		/* time of last measurement */
 int rmsSamples;		/* samples for rms calculation */
 int minuteRms;			/* rms value for one minute */
 int displayTime;		/* time of last display */
 int minuteCount;
 struct s_chanCfg *cfg;		/* channel configuration */
 T_CHAN_BUF chanBuf;
} T_RMSCHAN, *P_RMSCHAN;

typedef struct s_adcChan
{
 ADC_TypeDef *adc;
 long unsigned int chan;
} T_ADCCHAN;
 
typedef struct s_chanCfg
{
 CHAN_TYPE type;		/* channel type */
 char label;			/* channel label */
 T_ADCCHAN rmsAdc;		/* rms adc adc */
 T_ADCCHAN voltAdc;		/* voltage adc */
 union
 {
  float curScale;		/* current scale */
  float rmsScale;		/* rms scale */
 };
 float voltScale;		/* voltage scale */
 union
 {
  P_RMSPWR pwr;			/* rms power data */
  P_RMSCHAN rms;		/* single channal rms data */
 };
} T_CHANCFG, *P_CHANCFG;

EXT uint32_t clockFreq;
EXT uint32_t tmrFreq;

EXT bool pwrActive;
EXT int maxChan;
EXT int curChan;

#if !defined(__CURRENT__)
EXT T_CHANCFG chanCfg[MAX_CHAN];
#endif	/* __CURRENT__ */
 
EXT P_RMS adc1Rms;
EXT P_RMS adc2Rms;

void rmsTestInit(void);
void rmsTest(void);

//void rmsCfgInit(void);
void rmsCfgInit(P_CHANCFG cfg, int count);

void currentUpdate();
void updatePower(P_CHANCFG chan);
void updateRms(P_CHANCFG chan);

void adcRead(void);
void adcRun(void);
void adcRead1(void);
void adcStatus(void);
void adcTmrTest(void);

#if defined(__CURRENT__)

#if defined(STM32F1)
inline bool adcIntFlag(ADC_TypeDef *adc)
{
 return((adc->SR & ADC_SR_EOC) != 0);
}

inline void adcStart(ADC_TypeDef *adc)
{
 adc->CR2 |= (ADC_CR2_SWSTART | ADC_CR2_EXTTRIG);
}

inline void adcClrInt(ADC_TypeDef *adc)
{
 adc->SR &= ~ADC_SR_STRT;
}
#endif	/* STM32F1 */

#if defined(STM32F3)
inline bool adcIntFlag(ADC_TypeDef *adc)
{
 return(LL_ADC_IsActiveFlag_EOC(adc));
}

inline void adcStart(ADC_TypeDef *adc)
{
 LL_ADC_REG_StartConversion(adc);
}

inline void adcClrInt(ADC_TypeDef *adc)
{
 adc->ISR = ADC_ISR_ADRDY | ADC_ISR_EOSMP | ADC_ISR_EOC | ADC_ISR_EOS;
}
#endif	/* STM32F3 */

#endif	/* __CURRENT__ */

void currentCmds(void);

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
  unsigned b0:1;  unsigned b1:1;  unsigned b2:1;  unsigned b3:1;
  unsigned b4:1;  unsigned b5:1;  unsigned b6:1;  unsigned b7:1;
  unsigned b8:1;  unsigned b9:1;  unsigned b10:1; unsigned b11:1;
  unsigned b12:1; unsigned b13:1; unsigned b14:1; unsigned b15:1;
  unsigned b16:1; unsigned b17:1; unsigned b18:1; unsigned b19:1;
  unsigned b20:1; unsigned b21:1; unsigned b22:1; unsigned b23:1;
  unsigned b24:1; unsigned b25:1; unsigned b26:1; unsigned b27:1;
  unsigned b28:1; unsigned b29:1; unsigned b30:1; unsigned b31:1;
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
uint16_t *testPtr;
int testCount = 0;
int testSave = 0;
 
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
#if MAX_CHAN_POWER > 0
 memset((void *) &rmsPower, 0, sizeof(rmsPower));
 adcOffset = (1 << ADC_BITS) / 2;
 adcScale = (int) (((1 << ADC_BITS) - 1) / 2.5);
 angle = 0.0;
 angleInc = (2 * M_PI) / SAMPLES_CYCLE;
 pfAngle = ((double) M_PI) / 20.0;
 printf("angleInc %7.4f pfAngle %7.4f\n", angleInc, pfAngle);
 rmsCount = 0;
#endif
}

bool adcTest = false;
T_ADC_DATA adcData;

void rmsTest(void)
{
#if MAX_CHAN_POWER > 0
 flushBuf();
 rmsCount = 0;
 P_RMSPWR pwr = &rmsPower[0];
 pwr->state = initAvg;
 adcTest = true;
 pwr->samples = 0;
 pwrActive = true;
 adcRead();
#endif
}

#if 0
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
 cfg->voltAdc.chan = ADC1_0;
 cfg->rmsAdc.chan = ADC2_0;

 cfg++;

 P_RMSCUR cur = &rmsCurrent[0];
 memset((void *) cur, 0, sizeof(rmsCurrent));
 
 cfg->type = CURRENT_CHAN;
 cfg->curScale = 20.0;
 cur->cfg = cfg;
 cur->state = initCur;
 cur->lastState = initCur;
 cfg->cur = cur;
 cfg->rmsAdc.chan = ADC1_1;
}
#endif

void adcChanInit(T_ADCCHAN adc)
{
 printf("adcChanInit adc %08x chan %08x\n",
	(unsigned int) adc.adc, (unsigned int) adc.chan);
 LL_ADC_SetChannelSamplingTime(adc.adc, adc.chan, SAMPLING_TIME);
}

void rmsCfgInit(P_CHANCFG cfg, int count)
{
 printf("rmsCfgInit %08x n %d\n", (unsigned int) cfg, count);
 printf("adc1Rms %08x adc2Rms %08x\n",
	(unsigned int) &adc1Rms, (unsigned int) &adc2Rms);

 maxChan = count;		/* maximum channel */
 curChan = 0;			/* current channel */

 for (int i = 0; i < count; i++)
 {
  adcChanInit(cfg->rmsAdc);
  switch (cfg->type)
  {
  case POWER_CHAN:
  {
#if MAX_CHAN_POWER > 0
   adcChanInit(cfg->voltAdc);
   P_RMSPWR pwr = cfg->pwr;
   memset((void *) pwr, 0, sizeof(T_RMSPWR));
   pwr->state = initAvg;
   pwr->lastState = initAvg;
   pwr->label = cfg->label;
#endif	/* MAX_CHAN_POWER */
  }
  break;

  case RMS_CHAN:
  {
#if MAX_CHAN_RMS > 0
   P_RMSCHAN rms = cfg->rms;
   memset((void *) rms, 0, sizeof(T_RMSCHAN));
   rms->adc = cfg->rmsAdc.adc;
   if (rms->adc == ADC1)
    rms->adcRms = &adc1Rms;
   else if (rms->adc == ADC2)
    rms->adcRms = &adc2Rms;
   rms->state = initRms;
   rms->lastState = initRms;
   rms->label = cfg->label;
   printf("%d %c adcData %08x\n", i, rms->label, (unsigned int) rms->adcRms);
#endif	/* MAX_CHAN_RMS */
  }
  break;

  }
  cfg++;
 }
}

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
   else if (chan->type == RMS_CHAN)
   {
    updateRms(chan);
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
   printf("p sample %d vSum %d vRms %d cSum %d cRms %d\n"
	  "p pwrSum %d, realPwr %d aprntPwr %d pwrFactor %d\n",
	  samples, buf->vSum, pwr->vRms, buf->cSum, pwr->cRms,
	  buf->pwrSum, pwr->realPwr, pwr->aprntPwr, pwr->pwrFactor);
  }
 }
}

void updateRms(P_CHANCFG chan)
{
 P_RMSCHAN rms = chan->rms;
 while (rms->chanBuf.count != 0)
 {
  dbg0Set();
  int p = rms->chanBuf.empPtr;
  P_CHAN_DATA buf = &rms->chanBuf.buf[p];
  p += 1;
  if (p >= RMS_SIZE)
   p = 0;
  rms->chanBuf.empPtr = p;
  int samples = buf->samples;
  rms->rms = iSqrt(buf->sum / samples);

  rms->rmsSamples += samples;
  rms->rmsSum += buf->sum;
  rms->minuteCount += 1;

  __disable_irq();
  rms->chanBuf.count -= 1;
  __enable_irq();
  dbg0Clr();
  putDbg('*');

  uint32_t t = millis();
  if ((t - rms->measureTime) >= MEASURE_INTERVAL)
  {
   char convBuf[32];
   rms->measureTime += MEASURE_INTERVAL;
   rms->minuteRms = iSqrt(int(rms->rmsSum / rms->rmsSamples));
#if !defined(ARDUINO_ARCH_STM32)
   newline();
#endif	/* ARDUINO_ARCH_STM32 */
   if (rms->label == 'c')
   {
    int mRms = (int) (scaleAdc(rms->minuteRms) * chan->rmsScale);
    int val = mRms / 1000;
    int mval = mRms - (val * 1000);
    printf("c minute rms count %d samples %5d sum %10s rms %4d %4d %2d.%03d\n",
	   rms->minuteCount, rms->rmsSamples,
	   i64toa(rms->rmsSum, convBuf), rms->minuteRms, mRms, val, mval);
   }
   else
   {
    int Rms = (int) ((scaleAdc(rms->minuteRms)) * chan->rmsScale);
    int val = Rms / 10;
    int mval = Rms - (val * 10);
    printf("v minute rms count %d samples %5d sum %10s rms %4d %4d %3d.%01d\n",
	   rms->minuteCount, rms->rmsSamples,
	   i64toa(rms->rmsSum, convBuf), rms->minuteRms, Rms, val, mval);
   }
   rms->minuteCount = 0;
   rms->rmsSamples = 0;
   rms->rmsSum = 0;

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
  
  if ((t - rms->displayTime) >= DISPLAY_INTERVAL)
  {
   char convBuf[32];
   rms->displayTime += DISPLAY_INTERVAL;
   int offset = buf->offset >> SAMPLE_SHIFT;
#if !defined(ARDUINO_ARCH_STM32)
   newline();
#endif	/* ARDUINO_ARCH_STM32 */
   printf("%c sample %3d min %4d %4d max %4d %4d delta %4d "
	  "offset %4d sum %10s rms %4d %4d\n",
	  rms->label, samples, buf->min, offset - buf->min, buf->max,
	  buf->max - offset, scaleAdc(buf->max - buf->min), offset,
	  i64toa(buf->sum, convBuf), rms->rms, scaleAdc(rms->rms));
  }
 }
}

void printBufC(bool scale)
{
 int count = sizeof(buf) / sizeof(uint16_t);
 uint16_t *p = (uint16_t *) buf;
 int col = 0;
 while (1)
 {
  uint16_t val = *p++;
  if (scale)
   val = scaleAdc(val);
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
 printBufC(1);

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
 rmsCfgInit(&chanCfg[0], sizeof(chanCfg) / sizeof(T_CHANCFG)); /* init cfg */
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
 printBufC(1);
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
  dbg1Set();
  P_CHANCFG chan = &chanCfg[curChan];
 
  if (adcTest)
  {
   angle = fmod((double) rmsCount * angleInc, (double) (2 * M_PI));
   adcData.voltage = rint(adcScale * sin(angle) + adcOffset);
   adcData.current = rint(adcScale * sin(angle + pfAngle) + adcOffset);
   rmsCount += 1;
  }
   
  if (chan->type == POWER_CHAN)
  {
   LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, chan->voltAdc.chan);
   LL_ADC_REG_SetSequencerRanks(ADC2, LL_ADC_REG_RANK_1, chan->rmsAdc.chan);
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
      dbg5Toggle();
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

   adcStart(ADC2);
   adcStart(ADC1);
  }
  else if (chan->type == RMS_CHAN)
  {
   P_RMSCHAN rms = chan->rms;
   *(rms->adcRms) = &rms->rmsAccum;
   LL_ADC_REG_SetSequencerRanks(rms->adc, LL_ADC_REG_RANK_1,
				chan->rmsAdc.chan);

   rms->samples += 1;
   switch (rms->state)
   {
   case initRms:
    if (rms->samples >= INITIAL_COUNT)
    {
     rms->state = avgRms;
     rms->samples = 0;
     uint32_t t = millis();
     rms->displayTime = t;
     rms->measureTime = t;
     rms->minuteCount = 0;
     rms->rmsSamples = 0;
     rms->rmsSum = 0;
    }
    break;

   case avgRms:
   {
    int samples = rms->samples;
    if (samples >= (CYCLES_SEC * SAMPLES_CYCLE))
    {
     dbg4Toggle();
     if (rms->chanBuf.count < RMS_SIZE)
     {
      rms->chanBuf.count += 1;
      int ptr = rms->chanBuf.filPtr;
      P_CHAN_DATA chanData = &rms->chanBuf.buf[ptr];
      ptr += 1;
      if (ptr >= RMS_SIZE)
       ptr = 0;
      rms->chanBuf.filPtr = ptr;

      chanData->time = millis();
      chanData->samples = samples;
      P_RMS accum = &rms->rmsAccum;
      chanData->sum = accum->sum;
      chanData->offset = accum->offset;
      chanData->min = accum->min;
      chanData->max = accum->max;

      accum->sum= 0;
      accum->max = 0;
      accum->min = 1 << ADC_BITS;
      rms->samples = 0;
      putDbg(rms->label);
     }
    }
    break;
   }

   case rmsDone:
    break;
     
   default:
    rms->state = initRms;
    break;
   }

   adcStart(rms->adc);
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
    adcStart(ADC2);
    adcStart(ADC1);
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

inline void saveData(int sample)
{
 if (testSave)
 {
  if (--testCount < 0)
  {
   testCount = sizeof(buf) / sizeof(uint16_t);
   testPtr = buf;
  }
  *testPtr++ = sample;
 }
}

extern "C" void ADC1_2_IRQHandler(void)
{
 if (adcIntFlag(ADC1))		/* if adc1 interrupt active */
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

   rms->sample = sample;
   sample <<= SAMPLE_SHIFT;

   int offset = rms->offset;
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

  adcClrInt(ADC1);
  dbg2Clr();
 } /* adc1 interrupt active */
 
 if (adcIntFlag(ADC2))		/* if adc2 interrupt active */
 {
  dbg3Set();
  adc2Ints += 1;

  if (pwrActive)		/* measuring power */
  {
   P_RMS rms = adc2Rms;
   int sample = ADC2->DR;

   saveData(sample);

   if (adcTest)
    sample = adcData.current;

   if (sample > rms->max)
    rms->max = sample;
   if (sample < rms->min)
    rms->min = sample;

   rms->sample = sample;
   sample <<= SAMPLE_SHIFT;

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

  adcClrInt(ADC1);
  dbg3Clr();
 } /* adc2 interrupt active */
}

#if !defined(ARDUINO_ARCH_STM32)
void currentCmds(void)
{
 while (1)
 {
  printf("\nC: ");
  flushBuf();
  while (dbgRxReady() == 0)	/* while no character */
  {
   pollBufChar();		/* check for data to output */
  }
  char ch = dbgRxRead();
  putBufChar(ch);
  newline();
  if (ch == 'e')
  {
   adcTmrStop();
   adcTmrClrIE();
   printf("\ntimer stopped\n");
  }
  else if (ch == 'p')
  {
   newline();
   printBufC(0);
  }
  else if (ch == 'a')
  {
   newline();
   adcRead1();
  }
  else if (ch == 'r')
  {
   newline();
   testCount = 0;
   testSave = true;
   adcRun();
   break;
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
#if 0
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
  else if (ch == 'x')
  {
   break;
  }
 }
}
#endif	/* ARDUINO_ARCH_STM32 */

#endif	/* __CURRENT__ */
