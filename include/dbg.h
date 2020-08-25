#if defined(__STM32F1xx_HAL_H) | defined(STM32F3)
#if !defined(__DBG_H)
#define __DBG_H

constexpr auto DBG0 = 1;
constexpr auto DBG1 = 1;
constexpr auto DBG2 = 1;
constexpr auto DBG3 = 1;
constexpr auto DBG4 = 1;
constexpr auto DBG5 = 1;

#ifdef Dbg0_Pin
inline void dbg0Ini() {}
inline void dbg0Set() {Dbg0_GPIO_Port->BSRR = Dbg0_Pin;}
inline void dbg0Clr() {Dbg0_GPIO_Port->BSRR = (Dbg0_Pin << 16);}
#else
inline void dbg0Ini() {}
inline void dbg0Set() {}
inline void dbg0Clr() {}
#endif

#ifdef Dbg1_Pin
inline void dbg1Ini() {}
inline void dbg1Set() {Dbg1_GPIO_Port->BSRR = Dbg1_Pin;}
inline void dbg1Clr() {Dbg1_GPIO_Port->BSRR = (Dbg1_Pin << 16);}
#else
inline void dbg1Ini() {}
inline void dbg1Set() {}
inline void dbg1Clr() {}
#endif

#ifdef Dbg2_Pin
inline void dbg2Ini() {}
inline void dbg2Set() {Dbg2_GPIO_Port->BSRR = Dbg2_Pin;}
inline void dbg2Clr() {Dbg2_GPIO_Port->BSRR = (Dbg2_Pin << 16);}
#else
inline void dbg2Ini() {}
inline void dbg2Set() {}
inline void dbg2Clr() {}
#endif

#ifdef Dbg3_Pin
inline void dbg3Ini() {}
inline void dbg3Set() {Dbg3_GPIO_Port->BSRR = Dbg3_Pin;}
inline void dbg3Clr() {Dbg3_GPIO_Port->BSRR = (Dbg3_Pin << 16);}
#else
inline void dbg3Ini() {}
inline void dbg3Set() {}
inline void dbg3Clr() {}
#endif

#ifdef Dbg4_Pin
inline void dbg4Ini() {}
inline void dbg4Set() {Dbg4_GPIO_Port->BSRR = Dbg4_Pin;}
inline void dbg4Clr() {Dbg4_GPIO_Port->BSRR = (Dbg4_Pin << 16);}
#else
inline void dbg4Ini() {}
inline void dbg4Set() {}
inline void dbg4Clr() {}
#endif

#ifdef Dbg5_Pin
inline void dbg5Ini() {}
inline void dbg5Set() {Dbg5_GPIO_Port->BSRR = Dbg5_Pin;}
inline void dbg5Clr() {Dbg5_GPIO_Port->BSRR = (Dbg5_Pin << 16);}
#else
inline void dbg5Ini() {}
inline void dbg5Set() {}
inline void dbg5Clr() {}
#endif

/* toggle on capture input */
#ifdef Dbg0_Pin
inline void dbgIntCSet() {Dbg0_GPIO_Port->BSRR = Dbg0_Pin;}
inline void dbgIntCClr() {Dbg0_GPIO_Port->BSRR = (Dbg0_Pin << 16);}
#else
inline void dbgIntCSet() {}
inline void dbgIntCClr() {}
#endif

/* toggle on output pulse */
#ifdef Dbg1_Pin
inline void dbgIntPSet() {Dbg1_GPIO_Port->BSRR = Dbg1_Pin;}
inline void dbgIntPClr() {Dbg1_GPIO_Port->BSRR = (Dbg1_Pin << 16);}
#else
inline void dbgIntPSet() {}
inline void dbgIntPClr() {}
#endif

/* toggle on input cycle */
#ifdef Dbg2_Pin
inline void dbgCycleSet() {Dbg2_GPIO_Port->BSRR = Dbg2_Pin;}
inline void dbgCycleClr() {Dbg2_GPIO_Port->BSRR = (Dbg2_Pin << 16);}
#else
inline void dbgCycleSet() {}
inline void dbgCycleClr() {}
#endif

/* end of internal pulse cycle */
#ifdef Dbg3_Pin
inline void dbgCycEndSet() {Dbg3_GPIO_Port->BSRR = Dbg3_Pin;}
inline void dbgCycEndClr() {Dbg3_GPIO_Port->BSRR = (Dbg3_Pin << 16);}
#else
inline void dbgCycEndSet() {}
inline void dbgCycEndClr() {}
#endif

/* length of capture isr */
#ifdef Dbg4_Pin
inline void dbgCapIsrSet() {Dbg4_GPIO_Port->BSRR = Dbg4_Pin;}
inline void dbgCapIsrClr() {Dbg4_GPIO_Port->BSRR = (Dbg4_Pin << 16);}
#else
inline void dbgCapIsrSet() {}
inline void dbgCapIsrClr() {}
#endif

/* length of internal isr */
#ifdef Dbg5_Pin
inline void dbgIntIsrSet() {Dbg5_GPIO_Port->BSRR = Dbg5_Pin;}
inline void dbgIntIsrClr() {Dbg5_GPIO_Port->BSRR = (Dbg5_Pin << 16);}
#else
inline void dbgIntIsrSet() {}
inline void dbgIntIsrClr() {}
#endif

#endif /* __DBG_H */
#endif /* __STM32F1xx_HAL_H */
