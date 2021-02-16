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
inline bool dbg4() {return((Dbg4_GPIO_Port->ODR & Dbg4_Pin) != 0);}
inline void dbg4Toggle()
{
 if (dbg4())
  dbg4Clr();
 else
  dbg4Set();
}
#else
inline void dbg4Ini() {}
inline void dbg4Set() {}
inline void dbg4Clr() {}
inline bool dbg4() {return(0);}
inline void dbg4Toggle() {}
#endif

#ifdef Dbg5_Pin
inline void dbg5Ini() {}
inline void dbg5Set() {Dbg5_GPIO_Port->BSRR = Dbg5_Pin;}
inline void dbg5Clr() {Dbg5_GPIO_Port->BSRR = (Dbg5_Pin << 16);}
inline bool dbg5() {return((Dbg5_GPIO_Port->ODR & Dbg5_Pin) != 0);}
inline void dbg5Toggle()
{
 if (dbg5())
  dbg5Clr();
 else
  dbg5Set();
}
#else
inline void dbg5Ini() {}
inline void dbg5Set() {}
inline void dbg5Clr() {}
inline bool dbg5() {return(0);}
inline void dbg5Toggle() {}
#endif

#endif /* __DBG_H */
#endif /* __STM32F1xx_HAL_H */
