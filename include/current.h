#if 1	// <-

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

#define CYCLE_CTR
#if defined(CYCLE_CTR)
#define cycleCtr 0
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
#else  /* CYCLE_CTR */
#define cycleCtr 0
inline void resetCnt() {}
inline void startCnt() {}
inline void stopCnt() {}
inline unsigned int getCycles() {return(0);}
inline void getCycles(uint32_t *val) {};
#endif	/* CYCLE_CTR */

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
