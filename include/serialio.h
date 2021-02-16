#if 1	// <-

#include "config.h"
#include "dbg.h"

#if !defined(EXT)
#define EXT extern
#endif

#define DBGMSG 0

void wdUpdate(void);

enum RTN_VALUES
{
 NO_VAL,
 INT_VAL,
 FLOAT_VAL,
};

void newline(void);

#define MAXDIG 10		/* maximum input digits */

/* debug port routines */

void putx(char c);
void putstr(const char *p);
void sndhex(unsigned char *p, int size);
char getx(void);
unsigned char gethex(void);
char getstr(char *buf, int bufLen);
unsigned char getnum(void);
unsigned char getfloat(void);

char query(const char *format, ...);
char query(unsigned char (*get)(), const char *format, ...);

void prtbuf(unsigned char *p, int size);
void prtibuf(int16_t *p, int size);

/* polled remote port routines */

void putx1(char c);
void putstr1(const char *p);
void sndhex1(unsigned char *p, int size);
char gethex1(void);
char getstr1(char *buf, int bufLen);
unsigned char getnum1(void);

/* interrupt remote port routines */

extern "C" void remoteISR(void);
void initRem(void);
void putRem(char ch);
void putstrRem(const char *p);
void sndhexRem(unsigned char *p, int size);
int getRem(void);
char gethexRem(void);
char getstrRem(char *buf, int bufLen);
unsigned char getnumRem(void);

/* debug message routines */

#if DBGMSG
#if DBGMSG == 2
void dbgmsg(char dbg, int32_t val);
#else
void dbgmsg(const char *str, int32_t val);
void dbgmsgx(const char *str, char reg, int32_t val);
#endif
void clrDbgBuf(void);
#else
#define dbgmsg(a, b)
#define dbgmsgx(a, b, c)
#endif	/* DBGMSG */

/* debug port buffered character routines */

void initCharBuf(void);
void putBufChar(char ch);
void putBufCharIsr(char ch);
void putBufCharX(char ch);
void putBufStr(const char *s);
void putBufStrX(const char *s);
void putBufStrIsr(const char *s);
int pollBufChar(void);
void flushBuf(void);

/* printf output */

int __attribute__((__weak__, __section__(".libc")))
write (int handle, char *buffer, int len);
extern "C" ssize_t _write (int fd, const char* buf, size_t nbyte);

EXT unsigned char *p;
EXT int32_t val;
EXT float fVal;

EXT int32_t valRem;
EXT float fValRem;

EXT char dbgBuffer;

EXT char lineStart;
EXT char eolFlag;

#if defined(STM32F1)

inline uint32_t dbgRxReady()
{
 return(DBGPORT->SR & USART_SR_RXNE);
}
inline uint32_t dbgRxRead()
{
 return(DBGPORT->DR);
}
inline uint32_t dbgRxOverrun()
{
 return(DBGPORT->SR & USART_SR_ORE);
}
inline uint32_t dbgTxEmpty()
{
 return(DBGPORT->SR & USART_SR_TXE);
}
inline void dbgTxSend(char ch)
{
 DBGPORT->DR = ch;
}
inline void dbgTxIntEna()
{
 DBGPORT->CR1 |= USART_CR1_TXEIE; /* enable transmit interrupt */
}
inline void dbgTxIntDis()
{
 DBGPORT->CR1 &= ~USART_CR1_TXEIE; /* disable transmit interrupt */
}

#if defined(REMPORT)
inline uint32_t remRxReady()
{
 return(REMPORT->SR & USART_SR_RXNE);
}
inline char remRxRead()
{
 return((char) REMPORT->DR);
}
inline void remRxIntEna()
{
 REMPORT->CR1 |= USART_CR1_RXNEIE;
}
inline uint32_t remRxOverrun()
{
 return(REMPORT->SR & USART_SR_ORE);
}
inline uint32_t remTxEmpty()
{
 return(REMPORT->SR & USART_SR_TXE);
}
inline void remTxSend(char ch)
{
 REMPORT->DR = ch;
}
inline void remTxIntEna()
{
 REMPORT->CR1 |= USART_CR1_TXEIE; /* enable transmit interrupt */
}
inline void remTxIntDis()
{
 REMPORT->CR1 &= ~USART_CR1_TXEIE; /* disable transmit interrupt */
}
#endif	/* REMPORT */
#endif	/* STM32F1 */

#if defined(STM32F3)
inline uint32_t dbgRxReady()
{
 return(DBGPORT->ISR & USART_ISR_RXNE);
}
inline uint32_t dbgRxRead()
{
 return(DBGPORT->RDR);
}
inline uint32_t dbgRxOverrun()
{
 return(DBGPORT->ISR & USART_ISR_ORE);
}
inline uint32_t dbgTxEmpty()
{
 return(DBGPORT->ISR & USART_ISR_TXE);
}
inline void dbgTxSend(char ch)
{
 DBGPORT->TDR = ch;
}
inline void dbgTxIntEna()
{
 DBGPORT->CR1 |= USART_CR1_TXEIE; /* enable transmit interrupt */
}
inline void dbgTxIntDis()
{
 DBGPORT->CR1 &= ~USART_CR1_TXEIE; /* disable transmit interrupt */
}
#endif

/* debug port macros */

#define PUTX(c) while ((DBGPORT->SR & USART_SR_TXE) == 0); DBGPORT->DR = c
#define SNDHEX(val) sndhex((unsigned char *) &val, sizeof(val))

#if DBGMSG

#define MAXDBGMSG 200

#if DBGMSG == 2

typedef struct
{
 char dbg;
 int32_t val;
 int32_t time;
} T_DBGMSG, *P_DBGMSG;

#else

typedef struct
{
 char str[12];
 int32_t val;
 int32_t time;
} T_DBGMSG, *P_DBGMSG;

#endif

EXT T_DBGMSG dbgdata[MAXDBGMSG];

EXT int16_t dbgcnt;
EXT uint16_t dbgfil;
EXT uint16_t dbgemp;

#endif	/* DBGMSG */

/* remote port interrupt driven routines */

#define TX_BUF_SIZE 140
#define RX_BUF_SIZE 80

typedef struct
{
 int tx_fil;
 int tx_emp;
 int tx_count;
 char tx_buffer[TX_BUF_SIZE];
 int rx_fil;
 int rx_emp;
 int rx_count;
 char rx_buffer[RX_BUF_SIZE];
 int state;
} T_REMCTL, *P_REMCTL;

EXT T_REMCTL remCtl;

/* debug output buffer routines */

#define CHAR_BUF_SIZE 1024
EXT int charFil;
EXT int charEmp;
EXT int charCount;
EXT unsigned int charOverflow;
EXT char charBuf[CHAR_BUF_SIZE];

#define ISR_BUF_SIZE 64
EXT int isrFil;
EXT int isrEmp;
EXT int isrCount;
EXT unsigned int isrOverflow;
EXT char isrBuf[ISR_BUF_SIZE];

#endif	// ->
