#if 1	// <-

#if defined(STM32F1)
#include "stm32f1xx_ll_adc.h"
#endif	/* STM32F1 */

#if defined(STM32F4)
#include "stm32f4xx_ll_adc.h"
#endif

#if !defined(EXT)
#define EXT extern
#endif	/* EXT */

#define PWR_SIZE 16
#define RMS_SIZE 16
#define INITIAL_COUNT 10000
#define CYCLE_COUNT 60
#define SAMPLE_SHIFT 8

#define CYCLES_SEC 60
#define SAMPLES_CYCLE 16
#define CHAN_PAIRS 2
#define ADC_BITS 12

#define ADC_MAX ((1 << ADC_BITS) - 1)
#define VREF_1000 3300
#define VREF_10 33

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

#define RMS_DATA_SIZE 64

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
 int vDelta;			/* voltage adc delta value */
 int cDelta;			/* current adc delta value */
 int64_t pwrSum;		/* sum of voltage times current */
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
 uint32_t lastTime;		/* last update time */
 char label;			/* channel label */
 bool lastBelow;		/* last sample below voltage offset */
 int cycleCount;		/* cycle counter */
 T_RMS c;			/* current */
 T_RMS v;			/* voltage */
 float curScale;		/* adc count to current */
 float voltScale;		/* adc count to voltage */
 double pwrScale;
 int64_t pwrScaleNum;		/* adc count power numerator */
 int pwrScaleDenom;		/* adc count power denominator */
 int samples;			/* sample counter */
 int64_t pwrSum;		/* power sum */
 int sampleCount;		/* sample count for last reading */
 int displayTime;		/* time for last display */
 int vRms;			/* rms voltage */
 int cRms;			/* rms current */
 int realPwr;			/* real power */
 int realPwrTotal;		/* total real power */
 int aprntPwr;			/* apparent power */
 int pwrFactor;			/* power factor */
 int pwrDir;			/* power direction */
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

inline int scaleAdc(int val) {return((val * VREF_1000) / ADC_MAX);}

inline int scaleAdc(int val, float scale)
{
 return((int) (scale * ((val * VREF_1000) / ADC_MAX)));
}

void rmsTestInit(void);
void rmsTest(void);

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
