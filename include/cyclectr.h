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
