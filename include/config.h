#include "main.h"
#define WD_ENA 1

#define PIN_DISPLAY 0
#define ENCODER_TEST 0
#define USEC_MIN (60L * 1000000L)

#define DBGPORT USART1
#define REMPORT USART3

#define ADC1_0 LL_ADC_CHANNEL_2
#define ADC1_1 LL_ADC_CHANNEL_0
#define ADC2_0 LL_ADC_CHANNEL_1
#define ADC2_1 LL_ADC_CHANNEL_3

#define remoteISR(x) USART3_IRQHandler(x)
#define REMOTE_IRQn USART3_IRQn

#define encoderISR(x) EXTI15_10_IRQHandler(x)

#include "pinDef.h"
