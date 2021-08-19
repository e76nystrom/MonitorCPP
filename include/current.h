#if 1	// <-
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
#define VREF_10 33		/* rev voltage time volt scale factor */

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

#define DBG_PWR_DISPLAY	0x001
#define DBG_PWR_1M      0x002
#define DBG_PWR_15M	0x004
#define DBG_PWR_60M	0x008
#define DBG_PWR_SEND	0x010
#define DBG_PWR_SUMMARY 0x020
#define DBG_RMS_DISPLAY 0x040
#define DBG_RMS_MEASURE 0x080
#define DBG_RMS_SEND    0x100
#define DBG_BUF_RAW	0x200
#define DBG_BUF_CALC	0x400

inline int scaleAdc(int val) {return((val * VREF_1000) / ADC_MAX_VAL);}

inline int scaleAdc(int val, float scale)
{
 return((int) (scale * ((val * VREF_1000) / ADC_MAX_VAL)));
}

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
 
#endif	/* __CURRENT_INC__ */
#endif /* __CURRENT_INC__ */	// ->
