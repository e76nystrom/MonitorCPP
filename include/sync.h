#if 1	// <-

#if !defined(EXT)
#define EXT extern
#endif

#include "main.h"
#include "config.h"
#include "dbg.h"

unsigned int millis(void);

#if 0

#define DBG_CMP 1		/* debug capture timer */
#define DBG_CMP_TIME 1		/* debug capture interrupt timing */
#define DBG_INT 1		/* debug internal timer */
#define DBG_COUNT 1		/* debug count input and output pulses */

#define ARRAY 1			/* use array for cycle length */
#define ARRAY_LEN 2048		/* size of array */
#define START_DELAY 2		/* delay internal start */

#define CALC_STEP_WIDTH(x) ((uint16_t) ((cfgFcy * x) / 1000000l))

typedef uint8_t boolean;

#include "dbgtrk.h"

#define DIR_POS      1           /* 0x01 positive direction */
#define DIR_NEG      -1          /* 0x-1 negative direction */

#define MAX_TIMEOUT UINT_MAX
#define REMCMD_TIMEOUT 1000U
#define INDEX_TIMEOUT 1500U

EXT unsigned int remcmdUpdateTime;
EXT unsigned int remcmdTimeout;

EXT uint32_t cfgFcy;
EXT uint64_t clocksMin;		/* timer clocks per minute */

EXT int16_t encTmo;		/* encoder pulse timeout */

typedef struct
{
 uint32_t encCount;		/* encoder counts */
 uint32_t intCount;		/* internal counts */
 uint32_t cycleCount;		/* encoder cycle counter */
 int missedStart;		/* start flag missed */
 
 int16_t encCycLen;		/* encoder cycle length */
 int16_t encPulse;		/* encoder pulse number */
 uint16_t lastEnc;		/* last encoder capture */
 uint32_t encClocks;		/* clocks in current encoder cycle */
 uint32_t cycleClocks;		/* estimated clocks in cycle */

 uint16_t preScale;		/* counter pre scaler */

 int intCycLen;			/* internal cycle length */
 int intPulse;			/* internal pulse number */
 uint32_t intClocks;		/* clocks in current internal cycle */

 boolean startInt;		/* start internal timer */
 boolean measure;		/* measure flag */
 boolean stop;			/* stop flag */

#if START_DELAY
 int16_t startDelay;		/* initial delay */
#endif

#if ARRAY
 uint16_t delta[ARRAY_LEN];	/* saved delta values */
#endif
} T_CMP_TMR, *P_CMP_TMR;

EXT T_CMP_TMR cmpTmr;

void encoderSetup(void);
void encoderMeasure(void);
void encoderCalculate(void);
void encoderStart(void);
void encoderStop(void);

#if ENCODER_TEST

EXT uint32_t testPulseMin;
EXT uint16_t testEncPreScaler;
EXT uint16_t testEncCount;
EXT int rpm;
EXT char encRun;
EXT int encRunCount;
EXT int encCounter;
EXT int encLines;
EXT int encPulse;
EXT int encRevCounter;
EXT char encState;
EXT char encRev;

void testEncoderStart(void);
void testEncoderStop(void);

#endif

#endif

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

char portName(GPIO_TypeDef *port);
char timNum(TIM_TypeDef *tmr);

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

#endif	// ->
