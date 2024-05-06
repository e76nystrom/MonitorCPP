#if !defined(__CONFIG_H__)
#define __CONFIG_H__
#include "main.h"
#define WD_ENA 1

#define PIN_DISPLAY 0
#define USEC_MIN (60L * 1000000L)

#define DBGPORT USART1

#define ADC1_0 LL_ADC_CHANNEL_0
#define ADC1_1 LL_ADC_CHANNEL_2
#define ADC2_0 LL_ADC_CHANNEL_1
#define ADC2_1 LL_ADC_CHANNEL_3

#define TEST_POWER 1
#if TEST_POWER

#define MAX_CHAN_POWER 1
#define MAX_CHAN_RMS 2
#define MAX_CHAN (MAX_CHAN_POWER + MAX_CHAN_RMS)

#else

#define MAX_CHAN_POWER 0
#define MAX_CHAN_RMS 1
#define MAX_CHAN (MAX_CHAN_POWER + MAX_CHAN_RMS)

#endif	/* TEST_POWER */

#include "pinDef.h"
#endif	/* __CONFIG_H__ */
