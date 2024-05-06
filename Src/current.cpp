/******************************************************************************/
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
#include <math.h>

#include <stdbool.h>
#include <limits.h>
#include <stdarg.h>

#if defined(ARDUINO_ARCH_STM32)

#include "monitorSTM32.h"
#include "serial.h"

#else  /* ARDUINO_ARCH_STM32 */

#include "adc.h"
#include "serialio.h"
//#include "dma.h"
#include "tim.h"

#if defined(EXT)
#undef EXT
#endif	/* EXT */
#define EXT

#include "config.h"
#include "current.h"

#undef EXT
#endif	/* ARDUINO_ARCH_STM32 */

#include "stm32Info.h"

#if defined(EXT)
#undef EXT
#endif	/* EXT */

#include "cyclectr.h"

#define EXT
#if defined(ARDUINO_ARCH_STM32)

#include "current.h"
#include "monitor.h"
#include "currentSTM32.h"

#else

#if MAX_CHAN_POWER > 0
T_RMSPWR rmsPower[MAX_CHAN_POWER];
#endif	/* MAX_CHAN_POWER */

#if MAX_CHAN_RMS > 0
T_RMSCHAN rmsData[MAX_CHAN_RMS];
#endif	/* MAX_CHAN_RMS */

#define NO_SCALE

/* {type, label, rmsAdc {adc, chan, voltAdc {adc, chan}, rmsAdc, voltAdc, 
   [curScale | rmsScale], voltScale, [pwr | rms]} */

T_CHANCFG chanCfg[MAX_CHAN] =
{
#if TEST_POWER
 
#if defined(NO_SCALE)

/* type,      label,	{cAdc, cChan},	{vAdc, vChan},	cScale, vScale, */
 {POWER_CHAN, 'p',	{ADC1, ADC1_0}, {ADC2, ADC2_0}, 1,	1,
  /* pwrData */
  (P_RMSPWR) &rmsPower[0]},

/* type,    label,  {cAdc, cChan},  {0, 0}, cScale, 0, cRmsData */
 {RMS_CHAN, 'c',    {ADC1, ADC1_0}, {0, 0}, 1,	     0, (P_RMSPWR) &rmsData[0]},

/* type,    label, {vAdc, vChan},   {0, 0}, vScale,  0, vRmsData */
 {RMS_CHAN, 'v',   {ADC2, ADC2_0} , {0, 0}, 1,	     0, (P_RMSPWR) &rmsData[1]},
 
#else  /* NO_SCALE */

/* type,      label,	{cAdc, cChan},	{vAdc, vChan},	cScale, vScale, */
 {POWER_CHAN, 'p',	{ADC1, ADC1_0}, {ADC2, ADC2_0}, 30,	0.15119 * VOLT_SCALE,
  /* pwrData */
  (P_RMSPWR) &rmsPower[0]},

/* type,    label,  {cAdc, cChan},  {0, 0}, cScale, 0, cRmsData */
 {RMS_CHAN, 'c',    {ADC1, ADC1_0}, {0, 0}, 30,	    0, (P_RMSPWR) &rmsData[0]},

/* type,    label, {vAdc, vChan},   {0, 0}, vScale,		  0, vRmsData */
 {RMS_CHAN, 'v',   {ADC2, ADC2_0} , {0, 0}, 0.15119 * VOLT_SCALE, 0, (P_RMSPWR) &rmsData[1]},
 
#endif	/* NO_SCALE*/
 
#else  /* TEST_POWER */
 
/* type, label, {cAdc, cChan}, {0, 0}, cScale, 0, cRmsData */
 {RMS_CHAN, 'c', {ADC1, ADC1_0}, {0, 0}, 30, 0, (P_RMSPWR) &rmsData[0]},

#endif	/* TEST_POWER */
};

#endif	/* ARDUINO_ARCH_STM32 */

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
//#include "dbg.h"
#endif	/* STM32MON */

#if defined(__CURRENT_INC__)	// <-
#if !defined(__CURRENT_INC__)
#define __CURRENT_INC__

#if defined(STM32F1)
#include "stm32f1xx_ll_adc.h"
#endif	/* STM32F1 */

#if defined(STM32F4)
#include "stm32f4xx_ll_adc.h"
#endif

#if !defined(EXT)
#define EXT extern
#endif	/* EXT */

#define PWR_SIZE 16		/* power buffers */
#define RMS_SIZE 16		/* rms buffers */
#define INITIAL_SAMPLES 10000	/* initial samples for offset */
#define CYCLES_SEC 60		/* line frequency */
#define SECONDS_BUFFER 6	/* cycles per buffer */
#define CYCLE_COUNT (CYCLES_SEC * SECONDS_BUFFER) /* cycles in buffer */
#define SAMPLE_SHIFT 8

#define SAMPLES_CYCLE 16	/* samples per wave */
#define CHAN_PAIRS 2
#define ADC_BITS 12		/* number of adc bits */

#define ADC_MAX_VAL ((1 << ADC_BITS) - 1) /* max adc count */

#define CURRENT_SCALE 1000	/* current scale factor */
#define VOLT_SCALE 10		/* voltage scale factor */

#define VREF_1000 3300		/* ref voltage times current scale factor */
#define VREF_10 33		/* ref voltage time volt scale factor */

#define PWR_INTERVAL 500	/* power update interval */

#define DISPLAY_INTERVAL (12 * 1000) /* buffer display interval */
#define MEASURE_INTERVAL (60 * 1000) /* one min measurement interval */

#define BUFFERS_1M (60 / SECONDS_BUFFER)
#define BUFFERS_15M (15 * BUFFERS_1M)
#define BUFFERS_60M (60 * BUFFERS_1M)

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

#define RMS_DATA_SIZE (4 * SAMPLES_CYCLE) /* amount to save for debug */

typedef struct s_rms
{
 int sample;			/* current sample */
 int value;			/* value after offset operation */
 int offset;			/* filtered offset */
 int64_t sum;			/* sum of squares */
 int min;			/* min value */
 int max;			/* max value */
 bool save;			/* save data */
 int count;			/* data count */
 uint16_t *dataP;		/* data pointer */
 uint16_t data[RMS_DATA_SIZE];	/* data buffer */
} T_RMS, *P_RMS;

typedef struct s_pwrData
{
 uint32_t time;			/* time of reading */
 int samples;			/* samples */
 int64_t vSum;			/* voltage sum of squares */
 int64_t cSum;			/* current sum of squares */
 int64_t pwrSum;		/* sum of voltage times current */
 int64_t absPwrSum;		/* sum abs val of voltage times current */
 int vDelta;			/* voltage adc delta value */
 int cDelta;			/* current adc delta value */
} T_PWR_DATA, *P_PWR_DATA;

typedef struct s_pwrSave
{
 uint32_t time;			/* time of reading */
 int samples;			/* samples */
 int64_t vSum;			/* voltage sum of squares */
 int64_t cSum;			/* current sum of squares */
 int64_t pwrSum;		/* sum of voltage times current */
 int64_t absPwrSum;		/* sum abs val of voltage times current */
} T_PWR_SAVE, *P_PWR_SAVE;

typedef struct s_pwrAccum
{
 uint32_t time;			/* time of reading */
 int buffers;			/* buffers added */
 int samples;			/* samples */
 int64_t vSum;			/* voltage sum of squares */
 int64_t cSum;			/* current sum of squares */
 int64_t pwrSum;		/* sum of voltage times current */
 int64_t absPwrSum;		/* sum abs val of voltage times current */
} T_PWR_ACCUM, *P_PWR_ACCUM;

typedef struct s_pwrTotal
{
 T_PWR_ACCUM p;			/* power save data */
 const char *label;		/* interval label */
 int vRms;			/* rms voltage */
 int cRms;			/* rms current */
 int realPwr;			/* real power */
 int absRealPwr;		/* abs val real power */
 int aprntPwr;			/* apparent power */
 int pwrFactor;			/* power factor */
 char pwrStr[16];		/* power string */
 char vStr[16];			/* current string */
 char cStr[16];			/* voltage string */
} T_PWR_TOTAL, *P_PWR_TOTAL;

typedef struct s_pwrBuf
{
 int filPtr;			/* fill pointer */
 int empPtr;			/* empty pointer */
 int count;			/* number in buffer */
 T_PWR_DATA buf[PWR_SIZE];	/* buffer */
} T_PWR_BUF, *P_PWR_BUF;

typedef struct s_rmsPwr
{
 pwrState state;		/* curent state */
 pwrState lastState;		/* last state */
 uint32_t bufTime;		/* time of current buffer */
 int bufCount;			/* number of buffers processed */
 uint32_t lastBufTime;		/* last buffer time */
 uint32_t lastTime;		/* last update time */
 char label;			/* channel label */
 bool lastBelow;		/* last sample below voltage offset */
 /* scale factors */
 float curScale;		/* adc count to current */
 float voltScale;		/* adc count to voltage */
 double pwrScale;		/* power scaling factor */
 /* accumulators for interrupt routine */
 int cycleCount;		/* interrupt cycle counter */
 int samples;			/* interrupt sample counter */
 T_RMS c;			/* interrupt current accumulator */
 T_RMS v;			/* interrupt voltage accumulator */
 int64_t pwrSum;		/* interrupt power sum */
 int64_t absPwrSum;		/* interrupt abs val power sum */
 /* saved values for various intervals */
 T_PWR_TOTAL pwr1M;		/* one minute power */
 T_PWR_TOTAL pwr15M;		/* 15 mounte power*/
 T_PWR_TOTAL pwr60M;		/* 60 minute power */
 /* one minute calculated values */
 int vRms;			/* rms voltage */
 int cRms;			/* rms current */
 int realPwr;			/* real power */
 int aprntPwr;			/* apparent power */
 int pwrFactor;			/* power factor */
 int pwrDir;			/* power direction */
 /* timing variables */
 int displayTime;		/* time for last display */
 int measureTime;		/* time for measurement */
 struct s_chanCfg *cfg;		/* channel configuration */
 T_PWR_BUF pwrBuf;		/* power buffers */
} T_RMSPWR, *P_RMSPWR;

typedef struct s_chanData
{
 uint32_t time;			/* time of reading */
 int samples;			/* samples */
 uint64_t sum;			/* current sum of squares */
 int offset;
 int min;
 int max;
} T_CHAN_DATA, *P_CHAN_DATA;

typedef struct s_chanBuf
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
 uint32_t lastTime;		/* last update time */
 char label;			/* channel label */
 ADC_TypeDef *adc;		/* pointer to adc hardware */
 P_RMS *adcRms;			/* pointer to isr pointer */
 T_RMS rmsAccum;		/* rms accumulator */
 int samples;			/* sample counter */
 int rms;			/* rms current */
 int64_t rmsSum;		/* sum for rms calculation */
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

EXT unsigned int pwrUpdTime;
 
EXT uint32_t clockFreq;
EXT uint32_t tmrFreq;

EXT bool pwrActive;
EXT int maxChan;
EXT int curChan;

#define PWR_SAVE_BUFFERS 60

EXT T_PWR_SAVE pwrSaveBuf[PWR_SAVE_BUFFERS];
EXT bool pwrSaveActive;
EXT int pwrSaveCount;
EXT P_PWR_SAVE pwrSavePtr;

#if !defined(__CURRENT__)
EXT T_CHANCFG chanCfg[MAX_CHAN];
#endif	/* __CURRENT__ */
 
EXT P_RMS adc1Rms;
EXT P_RMS adc2Rms;

EXT bool cmdActive;
EXT int pwrDbg;

#define DBG_PWR_DISPLAY 0x001
#define DBG_PWR_1M	0x002
#define DBG_PWR_15M	0x004
#define DBG_PWR_60M	0x008
#define DBG_PWR_SEND	0x010
#define DBG_PWR_SUMMARY 0x020
#define DBG_RMS_DISPLAY 0x040
#define DBG_RMS_MEASURE 0x080
#define DBG_RMS_SEND	0x100
#define DBG_BUF_RAW	0x200
#define DBG_BUF_CALC	0x400
#define DBG_PWR_SAVE	0x800

inline int scaleAdc(int val) {return((val * VREF_1000) / ADC_MAX_VAL);}

inline int scaleAdc(int val, float scale)
{
 return((int) (scale * ((val * VREF_1000) / ADC_MAX_VAL)));
}

#define I64BUF_SIZE 22
char *i64toa(int64_t val, char *buf);
char *fmtVal(char *buf, int size, int val, int scale);
char *fmtScaled(char *buf, int size, int value, int scale);

void rmsTestInit(void);
void rmsTest(void);

void rmsCfgInit(P_CHANCFG cfg, int count);

void powerUpdate();
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

inline void	adcTmrClrIE()	      {TIM1->DIER &= ~TIM_DIER_UIE;}
inline void	adcTmrSetIE()	      {TIM1->DIER |= TIM_DIER_UIE;}
inline uint16_t adcTmrTstIE()	      \
	{return((TIM1->DIER & TIM_IT_UPDATE) != 0);}
inline uint16_t adcTmrIF()	      \
	{return((TIM1->SR & TIM_FLAG_UPDATE) != 0);}
inline void	adcTmrClrIF()	      {TIM1->SR = ~TIM_SR_UIF;}
inline void	adcTmrStart()	      {TIM1->CR1 |= TIM_CR1_CEN;}
inline void	adcTmrPulse()	      \
	{TIM1->CR1 |= (TIM_CR1_OPM | TIM_CR1_CEN);}
inline void	adcTmrStop()	      \
	{TIM1->CR1 &= ~(TIM_CR1_OPM | TIM_CR1_CEN);}
inline void	adcTmrScl(uint16_t y) {TIM1->PSC = (y);}
inline uint16_t adcTmrRead()	      {return(TIM1->CNT);}
inline void	adcTmrCntClr()	      {TIM1->CNT = 0;}
inline void	adcTmrCnt(uint16_t x) {TIM1->CNT = (x);}
inline void	adcTmrMax(uint16_t x) {TIM1->ARR = ((x) - 1);}
inline void	adcTmrSet(uint16_t x) {TIM1->ARR = (x);}
inline uint16_t adcTmrMaxRead()	      {return(TIM1->ARR);}

inline void	adcTmrCCR(uint16_t x) {TIM1->CCR1 = (x);}
inline void	adcTmrPWMMode()	      \
	{TIM1->CCMR1 = (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1);}
inline void	adcTmrPWMEna()	      \
	{adcTmrBDTR(); TIM1->CCER |= TIM_CCER_CC1E;}
inline void	adcTmrPWMDis()	      {TIM1->CCER &= ~TIM_CCER_CC1E;}
inline uint16_t adcTmrReadCCR()	      {return(TIM1->CCR1);}
inline uint16_t adcTmrReadCCMR()      {return(TIM1->CCMR1);}
inline void	adcTmrCC1ClrIF()      {TIM1->SR = ~TIM_SR_CC1IF;}
inline void	adcTmrCC1ClrIE()      {TIM1->DIER &= ~TIM_DIER_CC1IE;}
inline void	adcTmrCC1SetIE()      {TIM1->DIER |= TIM_DIER_CC1IE;}

#if !defined(ARDUINO_ARCH_STM32)
unsigned int millis(void);
#endif	/* ARDUINO_ARCH_AVR */
 
#endif	/* __CURRENT_INC__ */
#endif /* __CURRENT_INC__ */	// ->
#ifdef __CURRENT__

#if defined(ARDUINO_ARCH_STM32)
unsigned char getNum();
extern int numVal;

void putx(char ch);

char get()
{
 while (DBGPORT.available() == 0)
  ;
 char ch = DBGPORT.read();
 return(ch);
}

char query(const char *format, ...);
//char query(unsigned char (*get)(), const char *format, ...);

char query(const char *format, ...)
{
 va_list args;
 va_start(args, format);
 vprintf(format, args);
 va_end(args);
 fflush(stdout);
 char ch = get();
 putx(ch);
 newLine();
 return(ch);
}

#if 0
char query(unsigned char (*get)(), const char *format, ...)
{
 va_list args;
 va_start(args, format);
 vprintf(format, args);
 va_end(args);
 fflush(stdout);
 char ch = get();
 newLine();
 return(ch);
}
#endif

#endif	/* ARDUINO_ARCH_STM32 */

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
//#define putDbg(ch) putBufChar(ch)
#define putDbg(ch)
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
uint16_t adcBuf[2 * (SAMPLES + 8)];
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

void adcChanInit(T_ADCCHAN adc)
{
 printf("adcChanInit adc %08x chan %d\n",
	(unsigned int) adc.adc, (int) (adc.chan & ADC_CR1_AWDCH));
 LL_ADC_SetChannelSamplingTime(adc.adc, adc.chan, SAMPLING_TIME);
}

void rmsCfgInit(P_CHANCFG cfg, int count)
{
 printf("rmsCfgInit %08x channels %d\n", (unsigned int) cfg, count);
 printf("adc1Rms %08x adc2Rms %08x\n",
	(unsigned int) &adc1Rms, (unsigned int) &adc2Rms);
 char convBuf[I64BUF_SIZE];
 i64toa(0x7fffffffffffffff, convBuf);
 printf("i64toa %d %s\n", strnlen(convBuf, I64BUF_SIZE), convBuf);

 char buf[34];
 fmtScaled(buf, sizeof(buf), 1234, 1000000);
 printf("%s\n", buf);
 fmtScaled(buf, sizeof(buf), 1234, 100000);
 printf("%s\n", buf);
 fmtScaled(buf, sizeof(buf), 1234, 10000);
 printf("%s\n", buf);
 fmtScaled(buf, sizeof(buf), 1234, 1000);
 printf("%s\n", buf);
 fmtScaled(buf, sizeof(buf), 1234, 100);
 printf("%s\n", buf);
 fmtScaled(buf, sizeof(buf), 1234, 10);
 printf("%s\n", buf);
 fmtScaled(buf, sizeof(buf), 1234, 1);
 printf("%s\n", buf);

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
   pwr->curScale = (VREF_1000 * cfg->curScale) / (float) ADC_MAX_VAL;
   pwr->voltScale = (VREF_1000 * cfg->voltScale) / (float) ADC_MAX_VAL;
   printf("%d curScale %7.3f voltScale %7.3f\n",
	  i, pwr->curScale, pwr->voltScale);
   double pwrScale = (double) (VREF_1000 * VREF_1000);
   pwrScale /= (double) (ADC_MAX_VAL * ADC_MAX_VAL);
   pwrScale *= (double) (cfg->curScale * cfg->voltScale / VOLT_SCALE);
   pwr->pwrScale = pwrScale;
   pwr->pwr1M.label = "1M ";
   pwr->pwr15M.label = "15M";
   pwr->pwr60M.label = "60M";
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
   res = res +	2 * one;
  }
  res >>= 1;
  one >>= 2;
 }
 return(res);
}

bool newLineFlag;

#if defined(ARDUINO_ARCH_STM32)
void checkNewLine(void) {}
#else
void checkNewLine(void)
{
 if (newLineFlag)
 {
  newLineFlag = false;
  newline();
 }
}
#endif	/* ARDUINO_ARCH_STM32 */

void powerUpdate()
{
 if (pwrActive)
 {
  unsigned int t = millis();
  if ((t - pwrUpdTime) >= PWR_INTERVAL)
  {
   pwrUpdTime += PWR_INTERVAL;
   newLineFlag = true;

   int pwrSave = pwrDbg;
   if (cmdActive)
    pwrDbg = 0;

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

   pwrDbg = pwrSave;
  }
 }
}

char *i64toa(int64_t val, char *buf)
{
 char tmp[I64BUF_SIZE];
 int count = 0;
 char *p = tmp;
 char *p1 = buf;

 if (val < 0)
 {
  *p1++ = '-';
  val = -val;
 }

 while (val != 0)
 {
  *p++ = ((char) (val % 10) + '0');
  count += 1;
  val /= 10;
 }

 if (count > 0)
 {
  while (--count >= 0)
   *p1++ = *--p;
 }
 else
  *p1++ = '0';

 *p1++ = 0;
 return(buf);
}

void savePwrData(P_PWR_DATA buf)
{
 if (pwrSaveActive)
 {
  P_PWR_SAVE save = pwrSavePtr;
  save->time = buf->time;
  save->samples = buf->samples;
  save->vSum = buf->vSum;
  save->cSum = buf->cSum;
  save->pwrSum = buf->pwrSum;
  save->absPwrSum = buf->absPwrSum;
  pwrSaveCount += 1;
  pwrSavePtr += 1;
  if (pwrSaveCount >= PWR_SAVE_BUFFERS)
  {
   pwrSavePtr = pwrSaveBuf;
   pwrSaveCount = 0;
  }
 }
}

void pwrAdd(P_PWR_ACCUM dst, P_PWR_ACCUM src)
{
 dst->samples += src->samples;
 dst->buffers += src->buffers;
 dst->vSum += src->vSum;
 dst->cSum += src->cSum;
 dst->pwrSum += src->pwrSum;
 dst->absPwrSum += src->absPwrSum;
}

void pwrClr(uint32_t *p)
{
 int count = sizeof(T_PWR_ACCUM) / sizeof(int32_t);
 while (--count >= 0)
 {
  *p++ = 0;
 }
}

char *fmtVal(char *buf, int size, int val, int scale)
{
 const char *neg = "";
 if (val < 0)
 {
  val = -val;
  neg = "-";
 }
 
 int width = 0;
 if (scale == 1000)
  width = 3;
 else if (scale == 10)
  width = 1;

 int pVal = val / scale;
 int fracVal = val - pVal * scale;

 snprintf(buf, size, "%s%d.%0*d", neg, pVal, width, fracVal);
 return(buf);
}

char *fmtScaled(char *buf, int size, int value, int scale)
{
 char tmp[34];
 int count = 0;
 char *p = tmp;
 char *p1 = buf;

 int fracDig = 0;
 while (scale >= 10)
 {
  scale /= 10;
  fracDig += 1;
 }

 int len = 1;
 if (value < 0)
 {
  *p1++ = '-';
  value = -value;
  len += 1;
 }

 while (value != 0)
 {
  *p++ = ((char) (value % 10) + '0');
  value /= 10;
  count += 1;
 }

 // printf("count %d fracDig %d\n", count, fracDig);

 len += (count > fracDig) ? (count + 1) : (fracDig + 2);
 
 if (len <= size)
 {
  if (fracDig >= count)
  {
   *p1++ = '0';
   *p1++ = '.';
   while (--fracDig >= count)
    *p1++ = '0';
   while(--count >= 0)
    *p1++ = *--p;
  }
  else
  {
   while (count >= 0)
   {
    if (fracDig == count)
    {
     if (fracDig != 0)
      *p1++ = '.';
    }
    *p1++ = *--p;
    count -= 1;
   }
  }
 }
 *p1++ = 0;
// printf("len %d actual %d\n", len, (int) (p1 - buf));
 return(buf);
}

void pwrCalc(P_RMSPWR pwr, P_PWR_TOTAL p, int dbg)
{
 int samples = p->p.samples;
 /* voltage in tenths of a volt */
 p->vRms = (int) (iSqrt(p->p.vSum / samples) * pwr->voltScale);
 /* current in milliamps */
 p->cRms = (int) (iSqrt(p->p.cSum / samples) * pwr->curScale);
 /* power in milliamp * volts */
 p->realPwr = (int) ((p->p.pwrSum * pwr->pwrScale) / samples);
 p->absRealPwr = (int) ((p->p.absPwrSum * pwr->pwrScale) / samples);
 /* power in milliamp * volts */
 p->aprntPwr = (p->vRms * p->cRms) / VOLT_SCALE;
 p->pwrFactor = (100 * p->absRealPwr) / p->aprntPwr;

#if 0
 int pWatts = p->realPwr / CURRENT_SCALE;
 int fracWatts = p->realPwr - pWatts * CURRENT_SCALE;
 sprintf(p->pwrStr, "%d.%03d", pWatts, fracWatts);
#else
 fmtVal(p->pwrStr, sizeof(p->pwrStr), p->realPwr, CURRENT_SCALE);
 fmtVal(p->vStr, sizeof(p->vStr), p->vRms, VOLT_SCALE);
 fmtVal(p->cStr, sizeof(p->cStr), p->cRms, CURRENT_SCALE);
#endif
	 
 if (pwrDbg & dbg)
 {
  char convBuf[I64BUF_SIZE];
  printf("p%s0 %5u buf %3d %5d samples %4d vSum %16s ",
	 p->label, (unsigned int) (pwr->bufTime - p->p.time),
	 p->p.buffers, pwr->bufCount-1,
	 samples, i64toa(p->p.vSum, convBuf));
  printf("cSum %16s ", i64toa(p->p.cSum, convBuf));
  printf("pwrSum %16s ", i64toa(p->p.pwrSum, convBuf));
  printf("absPwrSum %16s\n", i64toa(p->p.absPwrSum, convBuf));
 }
 
 if (pwrDbg & dbg)
 {
  printf("p%s1 vRms %5d cRms %5d aprntPwr %8d realPwr %8d pwrFactor %3d "
	 "watts %s\n",
	 p->label, p->vRms, p->cRms, p->aprntPwr, p->realPwr, p->pwrFactor,
	 p->pwrStr);
 }
}

/*
b    18, 5999,5759,6084377237, 387033291,1461745416,1472785442
*/

void updatePower(P_CHANCFG chan)
{
 P_RMSPWR pwr = chan->pwr;
 while (pwr->pwrBuf.count != 0)
 {
  dbg0Set();
  int p = pwr->pwrBuf.empPtr;
  int ep = p;
  P_PWR_DATA buf = &pwr->pwrBuf.buf[p];
  p += 1;
  if (p >= PWR_SIZE)
   p = 0;
  pwr->pwrBuf.empPtr = p;

  int samples = buf->samples;
  pwr->bufTime = buf->time;
  pwr->pwr1M.p.buffers += 1;
  pwr->pwr1M.p.samples += samples;
  pwr->pwr1M.p.vSum += buf->vSum;
  pwr->pwr1M.p.cSum += buf->cSum;
  pwr->pwr1M.p.pwrSum += buf->pwrSum;
  pwr->pwr1M.p.absPwrSum += buf->absPwrSum;

  if (pwrDbg & DBG_BUF_RAW)
  {
   char convBuf[I64BUF_SIZE];
   printf("b %2d, %5d, %5u, %4d, %10s, ",
	  ep, pwr->bufCount, (unsigned int) (buf->time - pwr->lastBufTime),
	  samples, i64toa(buf->vSum, convBuf));
   printf("%10s, ", i64toa(buf->cSum, convBuf));
   printf("%10s, ", i64toa(buf->pwrSum, convBuf));
   printf("%10s\n", i64toa(buf->absPwrSum, convBuf));
  }

  if (pwrDbg & DBG_BUF_CALC)
  {
   int vRms = (int) (iSqrt(buf->vSum / samples) * pwr->voltScale);
   int cRms = (int) (iSqrt(buf->cSum / samples) * pwr->curScale);
   int realPwr = (int) ((buf->pwrSum * pwr->pwrScale) / samples);
   int absRealPwr = (int) ((buf->absPwrSum * pwr->pwrScale) / samples);
   int aprntPwr = (vRms * cRms) / VOLT_SCALE;
   int pwrFactor = (100 * absRealPwr) / aprntPwr;
   printf("b %2d, %5d, %5u, %4d, ",
	  ep, pwr->bufCount, (unsigned int) (buf->time - pwr->lastBufTime),
	  samples);
   char buf[16];
   printf("%5s, ", fmtScaled(buf, sizeof(buf), vRms, VOLT_SCALE));
   printf("%6s, ", fmtScaled(buf, sizeof(buf), cRms, CURRENT_SCALE));
   printf("%9s, ", fmtScaled(buf, sizeof(buf), realPwr, CURRENT_SCALE));
   printf("%8s, ", fmtScaled(buf, sizeof(buf), aprntPwr, CURRENT_SCALE));
   printf("%2d\n", pwrFactor);
  }

  if (pwrDbg & DBG_PWR_SAVE)
   savePwrData(buf);

  if (pwrDbg & DBG_PWR_DISPLAY)
  {
   uint32_t t = millis();
   if ((t - pwr->displayTime) >= DISPLAY_INTERVAL)
   {
    pwr->displayTime += DISPLAY_INTERVAL;
    checkNewLine();

    /* voltage in tenths of a volt */
    int vRms = (int) (iSqrt(buf->vSum / samples) * pwr->voltScale);
    /* current in milliamps */
    int cRms = (int) (iSqrt(buf->cSum / samples) * pwr->curScale);
    /* power in milliamp * volts */
    int realPwr = (int) ((buf->pwrSum * pwr->pwrScale) / samples);
    int absRealPwr = (int) ((buf->absPwrSum * pwr->pwrScale) / samples);
    /* power in milliamp * volts */
    int aprntPwr = (vRms * cRms) / VOLT_SCALE;
    int pwrFactor = (100 * absRealPwr) / aprntPwr;

    printf("pi0 %2d %5u ", ep, (unsigned int) (buf->time - pwr->lastTime));
    char convBuf[I64BUF_SIZE];
    printf("samples %4d vSum %12s ",
	   samples, i64toa(buf->vSum, convBuf));
    printf("vDelta %4d cSum %12s ",
	   buf->vDelta, i64toa(buf->cSum, convBuf));
    printf("cDelta %4d pwrSum %12s ",
	   buf->cDelta, i64toa(buf->pwrSum, convBuf));
    printf("absPwrSum %12s\n",
	   i64toa(buf->absPwrSum, convBuf));

    printf("pi1 vRms %5d cRms %5d aprntPwr %8d realPwr %8d pwrFactor %3d\n",
	   vRms, cRms, aprntPwr, realPwr, pwrFactor);
    pwr->lastTime = buf->time;
   }
  }

  pwr->bufCount += 1;
  pwr->lastBufTime = buf->time;

  __disable_irq();
  pwr->pwrBuf.count -= 1;
  __enable_irq();

  dbg0Clr();
  putDbg('P');

  if (pwr->pwr1M.p.buffers >= BUFFERS_1M)
  {
   pwrCalc(pwr, &pwr->pwr1M, DBG_PWR_1M);
   pwrAdd(&pwr->pwr15M.p, &pwr->pwr1M.p);

   if (pwr->pwr15M.p.buffers >= BUFFERS_15M)
   {
    pwrCalc(pwr, &pwr->pwr15M, DBG_PWR_15M);
    pwrAdd(&pwr->pwr60M.p, &pwr->pwr15M.p);

    if (pwr->pwr60M.p.buffers >= BUFFERS_60M)
    {
     pwrCalc(pwr, &pwr->pwr60M, DBG_PWR_60M);

     pwrClr((uint32_t *) &pwr->pwr60M.p);
     pwr->pwr60M.p.time = pwr->bufTime;
    }

    pwrClr((uint32_t *) &pwr->pwr15M.p);
    pwr->pwr15M.p.time = pwr->bufTime;
   }

#if defined(ARDUINO_ARCH_STM32) && defined(WIFI_ENA) && EMON_POWER
   char buf[128];
   snprintf(buf, sizeof(buf), "node=" EMONCMS_NODE "&csv=%s,%s,%s",
	    pwr->pwr1M.pwrStr, pwr->pwr1M.vStr, pwr->pwr1M.cStr);
   emonData(buf);
#endif	/* ARDUINO_ARCH_STM32 && WIFI_ENA */

   if (pwrDbg & DBG_PWR_SEND)
    printf("pwr %s v %s c %s\n",
	   pwr->pwr1M.pwrStr, pwr->pwr1M.vStr, pwr->pwr1M.cStr);

   pwrClr((uint32_t *) &pwr->pwr1M.p);
   pwr->pwr1M.p.time = pwr->bufTime;
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
  int ep = p;
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
  if (pwrDbg & DBG_RMS_DISPLAY)
  {
   if ((t - rms->displayTime) >= DISPLAY_INTERVAL)
   {
    rms->displayTime += DISPLAY_INTERVAL;
    int offset = buf->offset >> SAMPLE_SHIFT;
    checkNewLine();
    char convBuf[I64BUF_SIZE];
    printf("r %c0 %2d %5u ",
	   rms->label, ep, (unsigned int) (buf->time - rms->lastTime));
    printf("sample %3d min %4d %4d max %4d %4d delta %4d "
	   "offset %4d sum %10s rms %4d %4d\n",
	   samples, buf->min, offset - buf->min, buf->max,
	   buf->max - offset, scaleAdc(buf->max - buf->min), offset,
	   i64toa(buf->sum, convBuf), rms->rms,
	   scaleAdc(rms->rms, chan->rmsScale));
    rms->lastTime = buf->time;
   } /* DISPLAY_INTERVAL */
  }
  
  if ((t - rms->measureTime) >= MEASURE_INTERVAL)
  {
   rms->measureTime += MEASURE_INTERVAL;
   rms->minuteRms = iSqrt(int(rms->rmsSum / rms->rmsSamples));
   int mRms = scaleAdc(rms->minuteRms, chan->rmsScale);

   if (pwrDbg & DBG_RMS_MEASURE)
   {
    char convBuf[I64BUF_SIZE];
    checkNewLine();
    printf("%c1 minute rms count %d samples %5d sum %10s rms %5d %5d ",
	   rms->label, rms->minuteCount, rms->rmsSamples,
	   i64toa(rms->rmsSum, convBuf), rms->minuteRms, mRms);

    if (rms->label == 'c')
    {
     int val = mRms / CURRENT_SCALE;
     int mval = mRms - (val * CURRENT_SCALE);
     printf("%2d.%03d\n", val, mval);
    }
    else
    {
     int val = mRms / VOLT_SCALE;
     int mval = mRms - (val * VOLT_SCALE);
     printf("%3d.%01d\n", val, mval);
    }
   }

   rms->minuteCount = 0;
   rms->rmsSamples = 0;
   rms->rmsSum = 0;

#if defined(ARDUINO_ARCH_STM32) && defined(WIFI_ENA) && EMON_RMS
   char buf0[128];
   char buf1[16];
   snprintf(buf0, sizeof(buf0), "node=" EMONCMS_NODE "&csv=%s",
	    fmtScaled(buf1, sizeof(buf1),
		      mRms, (rms->label == 'c') ? CURRENT_SCALE : VOLT_SCALE));
   emonData(buf0);
   if (pwrDbg & DBG_RMS_SEND)
    printf("rms %c %s\n", rms->label, buf1);
#endif	/* ARDUINO_ARCH_STM32 && WIFI_ENA*/
  } /* MEASURE_INTEVAL */
 }
}

void printBufC(bool scale)
{
 int count = sizeof(adcBuf) / sizeof(uint16_t);
 uint16_t *p = (uint16_t *) adcBuf;
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

 memset(adcBuf, 0, sizeof(adcBuf));
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
      adc1Ptr = &adcBuf[0];
      adc2Ptr = &adcBuf[1];
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

void adcTmrConfig()
{
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
 newline();
 tmrInfo(TIM1);
}

typedef struct sDbgCfg
{
 GPIO_TypeDef *gpio;
 int pin;
} T_DBG_CFG, *P_DBG_CFG;

T_DBG_CFG dbgCfg[] =
{
#if defined(Dbg0_Pin)
 {Dbg0_GPIO_Port, Dbg0_Pin},
#endif
#if defined(Dbg4_Pin)
 {Dbg1_GPIO_Port, Dbg1_Pin},
#endif
#if defined(Dbg4_Pin)
 {Dbg2_GPIO_Port, Dbg2_Pin},
#endif
#if defined(Dbg4_Pin)
 {Dbg3_GPIO_Port, Dbg3_Pin},
#endif
#if defined(Dbg4_Pin)
 {Dbg4_GPIO_Port, Dbg4_Pin},
#endif
#if defined(Dbg5_Pin)
 {Dbg5_GPIO_Port, Dbg5_Pin},
#endif
#if defined(Dbg6_Pin)
 {Dbg6_GPIO_Port, Dbg6_Pin},
#endif
};

void dbgInit()
{
 GPIO_InitTypeDef GPIO_InitStruct;

 P_DBG_CFG dbg = dbgCfg;
 GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
 GPIO_InitStruct.Pull = GPIO_NOPULL;
 for (unsigned int i = 0; i < (sizeof(dbgCfg) / sizeof(T_DBG_CFG)); i++)
 {
  GPIO_InitStruct.Pin = dbg->pin;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  
  HAL_GPIO_Init(dbg->gpio, &GPIO_InitStruct);
  dbg++;
 }
 newline();
 gpioInfo(GPIOA);
 newline();
 gpioInfo(GPIOB);
 newline();
}

void adcRun(void)
{
 dbgInit();
#if EMON_POWER
 pwrDbg = DBG_BUF_CALC | DBG_PWR_SEND;
#endif	/* EMON_POWER */
#if EMON_RMS
 pwrDbg = DBG_RMS_DISPLAY | DBG_RMS_MEASURE | DBG_RMS_SEND;
#endif	/* EMON_RMS */
 adcTmrConfig();
 newline();
 rmsCfgInit(&chanCfg[0], sizeof(chanCfg) / sizeof(T_CHANCFG)); /* init cfg */
 adcTest = false;
 pwrActive = true;
 adcRead();
 pwrUpdTime = millis();
}

void adcRead(void)
{
 if (!pwrActive)
 {
  memset(adcBuf, 0, sizeof(adcBuf));
  adc1Ptr = &adcBuf[0];
  adc2Ptr = &adcBuf[1];
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

 newline();
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
#define timerIRQ TIM1_UP_IRQHandler
#endif	/* STM32F1 */
#if defined(STM32F3)
#define timerIRQ TIM1_UP_TIM16_IRQHandler
#endif	/* STM32F3 */

extern "C" void timerIRQ(void)
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
   LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, chan->rmsAdc.chan);
   LL_ADC_REG_SetSequencerRanks(ADC2, LL_ADC_REG_RANK_1, chan->voltAdc.chan);
   P_RMSPWR pwr = chan->pwr;
   adc1Rms = &pwr->c;
   adc2Rms = &pwr->v;
   pwr->samples += 1;
   switch (pwr->state)
   {
   case initAvg:		/* sample to get zero offset */
    if (pwr->samples >= INITIAL_SAMPLES)
    {
     pwrClr((uint32_t *) &pwr->pwr1M.p);
     pwrClr((uint32_t *) &pwr->pwr15M.p);
     pwrClr((uint32_t *) &pwr->pwr60M.p);

     uint32_t t = millis();
     pwr->displayTime = t;
     pwr->measureTime = t;

     pwr->pwr1M.p.time = t;
     pwr->pwr15M.p.time = t;
     pwr->pwr60M.p.time = t;

     pwr->lastBelow = false;
     pwr->state = waitZero;
    }
    break;

   case waitZero:		/* wait for cycle start */
    if (pwr->v.sample >= pwr->v.offset)
    {
     if (pwr->lastBelow)
     {
      pwr->v.sum = 0;
      pwr->v.max = 0;
      pwr->v.min = ADC_MAX_VAL;
      pwr->c.sum = 0;
      pwr->c.max = 0;
      pwr->c.min = ADC_MAX_VAL;
      pwr->pwrSum = 0;
      pwr->absPwrSum = 0;
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
   {
    int instPwr = pwr->v.value * pwr->c.value;
    pwr->pwrSum += instPwr;
    
    if (instPwr >= 0)
     pwr->absPwrSum += instPwr;
    else
     pwr->absPwrSum -= instPwr;

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
	pwrData->vDelta = pwr->v.max - pwr->v.min;
	pwrData->cSum = pwr->c.sum;
	pwrData->cDelta = pwr->c.max - pwr->c.min;
	pwrData->pwrSum = pwr->pwrSum;
	pwrData->absPwrSum = pwr->absPwrSum;

	pwr->pwrBuf.count += 1;
	putDbg('p');
       }
       pwr->v.sum = 0;
       pwr->v.max = 0;
       pwr->v.min = ADC_MAX_VAL;
       pwr->c.sum = 0;
       pwr->c.max = 0;
       pwr->c.min = ADC_MAX_VAL;
       pwr->pwrSum = 0;
       pwr->absPwrSum = 0;
       pwr->samples = 0;
       pwr->cycleCount = CYCLE_COUNT;
      }
     }
     pwr->lastBelow = false;
    }
    else
     pwr->lastBelow = true;
   }
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
    if (rms->samples >= INITIAL_SAMPLES)
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
   testCount = sizeof(adcBuf) / sizeof(uint16_t);
   testPtr = adcBuf;
  }
  *testPtr++ = sample;
 }
}

void saveRmsData(P_RMS rms, uint16_t sample)
{
 if (rms->save)
 {
  if (--rms->count < 0)
  {
   rms->count = RMS_DATA_SIZE;
   rms->dataP = rms->data;
  }
  *rms->dataP++ = sample;
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
   P_RMS rms = adc1Rms;		/* get pointer to channel data */
   int sample = ADC1->DR;	/* read adc */

   if (adcTest)
    sample = adcData.current;

   saveRmsData(rms, sample);

   if (sample > rms->max)	/* set max and min */
    rms->max = sample;
   if (sample < rms->min)
    rms->min = sample;

   sample <<= SAMPLE_SHIFT;	/* scale sample */
   rms->sample = sample;	/* save scaled sample */

   int offset = rms->offset;	/* get offset in local variable */
   offset += ((sample - offset) >> 10); /* update offset */
   rms->offset = offset;	/* update offset */
   sample -= offset;		/* offset sample */
   sample >>= SAMPLE_SHIFT;	/* remove scale factor */
   rms->value = sample;		/* save sample value */
   rms->sum += sample * sample; /* update squared sum */
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

   saveRmsData(rms, sample);

   if (adcTest)
    sample = adcData.voltage;

   if (sample > rms->max)
    rms->max = sample;
   if (sample < rms->min)
    rms->min = sample;

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

  adcClrInt(ADC1);
  dbg3Clr();
 } /* adc2 interrupt active */
}

void printRmsBuf(P_RMS rms)
{
 int count = RMS_DATA_SIZE;
 uint16_t *p = rms->data;

 int col = 0;
 while (1)
 {
  uint16_t val = *p++;
  printf("%4d ", (int) val);
  count -= 1;
  col++;
  if (col == 8)
  {
   col = 0;
   printf("\n");
   flushBuf();
  }
  if (count == 0)
  {
   if (col != 0)
    printf("\n");
   break;
  }
 }
}

#if defined(ARDUINO_ARCH_STM32)
typedef struct
{
 char ch;
 GPIO_TypeDef *port;
} T_PORT_LIST, *P_PORT_LIST;

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

inline void putBufChar(char c)
{
 DBGPORT.write(c);
}

inline int dbgRxReady()
{
 return(DBGPORT.available());
}

inline char getx()
{
 return(DBGPORT.read());
}

inline char dbgRxRead()
{
 return(DBGPORT.read());
}

unsigned char gethex(void)
{
 char ch;
 int count;

 numVal = 0;
 count = 0;
 while (count <= 8)
 {
  ch = getx();
  if ((ch >= '0')
  &&  (ch <= '9'))
  {
   putBufChar(ch);
   ch -= '0';
   count++;
  }
  else if ((ch >= 'a')
  &&	   (ch <= 'f'))
  {
   putBufChar(ch);
   ch -= 'a' - 10;
   count++;
  }
  else if ((ch == 8)
       ||  (ch == 127))
  {
   if (count > 0)
   {
    --count;
    numVal >>= 4;
    putBufChar(8);
    putBufChar(' ');
    putBufChar(8);
   }
  }
  else if (ch == ' ')
  {
   putBufChar(ch);
   break;
  }
  else if (ch == '\r')
   break;
  else
   continue;
  numVal <<= 4;
  numVal += ch;
 }
 return(count != 0);
}

#endif	/* ARDUINO_ARCH_STM32 */

void currentCmds(void)		/* C in lclcmd for current commands */
{
 cmdActive = true;
 newline();
 printf("current commands\n");
 flushBuf();
 while (1)
 {
  char ch;
  newline();
  printf("C: ");
  flushBuf();

#if defined(ARDUINO_ARCH_STM32)
  while (1)
  {
   if (DBGPORT.available())
   {
    ch = DBGPORT.read();
    DBGPORT.write(ch);
    newLine();
    break;
   }
  }
#else
  while (dbgRxReady() == 0)	/* while no character */
  {
   pollBufChar();		/* check for data to output */
  }

  ch = dbgRxRead();
  putBufChar(ch);
  newline();
#endif	/* ARDUINO_ARCH_STM32 */

  if (ch == '?')
  {
   printf("r - run data collection \n");
   printf("e - end data collection\n");
   printf("a - read adc data\n");
   printf("p - print adc data\n");
   flushBuf();
   printf("S - set save flags\n");
   printf("P - clear save and print\n");
   printf("c - clear adc counters\n");
   printf("d - set debug flags \n");
   printf("Q - stm32Info \n");
   flushBuf();
   printf("R - run power save\n");
   printf("D - dump power save\n");
   printf("s - adc status\n");
   printf("t - test init and run\n");
   flushBuf();
   printf("f - test scaling \n");
   printf("x - exit current commands \n");
   flushBuf();
  }

  else if (ch == 'r')		/* *** start here */
  {
   newline();
   testCount = 0;
   testSave = true;
   adcRun();
   break;			/* exit loop to allow console output */
  }

  else if (ch == 'e')		/* stop timer to stop data collection */
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

  else if (ch == 'p')
  {
   newline();
   printBufC(0);
  }

  else if (ch == 's')
  {
   newline();
   adcStatus();
  }

  else if (ch == 'c')
  {
   __disable_irq();
   dmaInts = 0;
   adc1Ints = 0;
   adc2Ints = 0;
   timUpInts = 0;
   timCCInts = 0;
   __enable_irq();
  }

  else if (ch == 'd')
  {
   newline();
   printf("0x001 PWR_DISP\n"\
	  "0x002 PWR_1M\n"\
	  "0x004 PWR_15M\n"\
	  "0x008 PWR_30M\n"\
	  "0x010 PWR_SEND\n"\
	  "0x020 PWR_SUMMARY\n"\
	  "0x040 RMS_DISP\n"\
	  "0x080 RMS_MEASURE\n"\
	  "0x100 RMS_SEND\n"\
	  "0x200 BUF_RAW\n"\
	  "0x400 BUF_CALC\n"
	  "0x800 PWR_SAVE\n"
    );
   ch = query(&gethex, "dbg flag [0x%04x]: ", pwrDbg);
   if (ch)
   {
    pwrDbg = numVal;
   }
   printf("pwrDbg 0x%03x\n", pwrDbg);
  }

#if defined(ARDUINO_ARCH_STM32)
  else if (ch == 'Q')
  {
   info();
  }
#endif	/* ARDUINO_ARCH_STM32 */

  else if (ch == 'R')
  {
   newline();
   memset((void *) pwrSaveBuf, 0, sizeof(pwrSaveBuf));
   pwrSaveCount = 0;
   pwrSavePtr = pwrSaveBuf;
   pwrDbg |= DBG_PWR_SAVE;
   pwrSaveActive = true;
   printf("saving power data %u\n", (unsigned int) sizeof(pwrSaveBuf));
  }

  else if (ch == 'D')
  {
   if (pwrSaveActive)
   {
    newline();
    pwrSaveActive = false;
    P_PWR_SAVE buf = pwrSavePtr;
    uint32_t lastTime = buf->time;
    char convBuf[I64BUF_SIZE];
    int count = pwrSaveCount;
    for (int i = 0; i < PWR_SAVE_BUFFERS; i++)
    {
     printf("%2d, %2d, %7u, %4d, %10s, ",
	    i, count, (unsigned int) (buf->time - lastTime), buf->samples,
	    i64toa(buf->vSum, convBuf));
     printf("%10s, ", i64toa(buf->cSum, convBuf));
     printf("%10s, ", i64toa(buf->pwrSum, convBuf));
     printf("%10s\n", i64toa(buf->absPwrSum, convBuf));
     flushBuf();
     lastTime = buf->time;
     count += 1;
     if (count >= PWR_SAVE_BUFFERS)
     {
      buf = pwrSaveBuf;
      count = 0;
     }
     else
      buf += 1;
    }
   }
   else
    printf("power save not active\n");
  }

  else if (ch == 't')
  {
   newline();
   rmsTestInit();
   rmsTest();
  }

  else if (ch == 'S')
  {
   P_CHANCFG cfg = chanCfg;
   for (int i = 0; i < MAX_CHAN; i++)
   {
    switch(cfg->type)
    {
    case POWER_CHAN:
    {
     P_RMSPWR pwr = cfg->pwr;
     pwr->c.save = true;
     pwr->v.save = true;
    }
    break;

    case RMS_CHAN:
    {
     P_RMSCHAN rms = cfg->rms;
     rms->rmsAccum.save = true;
    }
    break;
    }
    cfg++;
   }
  }

  else if (ch == 'P')
  {
   P_CHANCFG cfg = chanCfg;
   for (int i = 0; i < MAX_CHAN; i++)
   {
    switch(cfg->type)
    {
    case POWER_CHAN:
    {
     P_RMSPWR pwr = cfg->pwr;
     pwr->c.save = false;
     pwr->v.save = false;
    }
    break;

    case RMS_CHAN:
    {
     P_RMSCHAN rms = cfg->rms;
     rms->rmsAccum.save = false;
    }
    break;
    }
    cfg++;
   }
   cfg = chanCfg;
   for (int i = 0; i < MAX_CHAN; i++)
   {
    switch(cfg->type)
    {
    case POWER_CHAN:
    {
     P_RMSPWR pwr = cfg->pwr;
     printf("pwr c\n");
     printRmsBuf(&pwr->c);
     printf("pwr v\n");
     printRmsBuf(&pwr->v);
    }
    break;

    case RMS_CHAN:
    {
     P_RMSCHAN rms = cfg->rms;
     printf("rms %c\n", rms->label);
     printRmsBuf(&rms->rmsAccum);
    }
    break;
    }
    cfg++;
   }
  }

#if defined(ARDUINO_ARCH_STM32)
  else if (ch == 'I')
  {
   static T_PORT_LIST portList[] =
    {
     {'a', GPIOA},
     {'b', GPIOB},
     {'c', GPIOC},
//     {'d', GPIOD},
//     {'e', GPIOE},
    };

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
    T_MASK andMask;
    T_MASK orMask;
    andMask.val = 0;
    orMask.val = 0;
    if (query(&gethex, "\nand mask: "))
    {
     andMask.mask = numVal;
     andMask.flag = 1;
    }

    if (query(&gethex, "\nor mask: "))
    {
     orMask.mask = numVal;
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
      while (dbgRxReady() == 0) /* while no character */
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
#endif	/* ARDUINO_ARCH_STM32 */

  else if (ch == 'f')
  {
   int value = 0;
   int scale = 0;
   char buf[32];
   while (query(&getNum, "val [%d]: ", value))
   {
    value = numVal;
    if (query(&getNum, "scale [%d]: ", scale))
     scale = numVal;
    printf("value %d scale %d fmt %s\n", value, scale,
	   fmtScaled(buf, sizeof(buf), value, scale));
   }
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
 cmdActive = false;
}

#endif	/* __CURRENT__ */
