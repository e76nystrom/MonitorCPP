#if defined( __STM32F1xx_HAL_H) | defined( __STM32F3xx_HAL_H)
#if !defined(__PINDEF_H)
#define __PINDEF_H

/* encoder pulse output */
#ifdef Encoder_Pin
inline void encoderSet() {Encoder_GPIO_Port->BSRR = Encoder_Pin;}
inline void encoderClr() {Encoder_GPIO_Port->BSRR = (Encoder_Pin << 16);}
inline uint16_t encoder() {return((Encoder_GPIO_Port->ODR & Encoder_Pin) != 0);}
#else
inline void encoderSet() {}
inline void encoderClr() {}
inline uint16_t encoder() {return(0);}
#endif

/* sync signal output */
#ifdef SyncOut_Pin
inline void syncOutSet() {SyncOut_GPIO_Port->BSRR = SyncOut_Pin;}
inline void syncOutClr() {SyncOut_GPIO_Port->BSRR = (SyncOut_Pin << 16);}
inline uint16_t syncOut() {return((SyncOut_GPIO_Port->ODR & SyncOut_Pin) != 0);}
#else
inline void syncOutSet() {}
inline void syncOutClr() {}
inline uint16_t syncOut() {return(0);}
#endif

/* start encoder */
#ifdef ZFlag_Pin
inline uint16_t zFlag() {return((ZFlag_GPIO_Port->IDR & ZFlag_Pin) != 0);}
inline uint16_t zFlagIsSet() {return((ZFlag_GPIO_Port->IDR & ZFlag_Pin) != 0);}
inline uint16_t zFlagIsClr() {return((ZFlag_GPIO_Port->IDR & ZFlag_Pin) == 0);}
#else
inline uint16_t zFlagSet() {return(0);}
inline uint16_t zFlagClr() {return(0);}
#endif

/* encoder ready */
#ifdef XFlag_Pin
inline void xFlagSet() {XFlag_GPIO_Port->BSRR = XFlag_Pin;}
inline void xFlagClr() {XFlag_GPIO_Port->BSRR = (XFlag_Pin << 16);}
inline uint16_t xFlag() {return((XFlag_GPIO_Port->ODR & XFlag_Pin) != 0);}
#else
inline void xFlagSet() {}
inline void xFlagClr() {}
inline uint16_t xFlag() {return(0);}
#endif

/* start encoder */
#ifdef ZFlag_Pin
inline uint16_t start() {return((ZFlag_GPIO_Port->IDR & ZFlag_Pin) != 0);}
inline uint16_t startIsSet() {return((ZFlag_GPIO_Port->IDR & ZFlag_Pin) != 0);}
inline uint16_t startIsClr() {return((ZFlag_GPIO_Port->IDR & ZFlag_Pin) == 0);}
#else
inline uint16_t startSet() {return(0);}
inline uint16_t startClr() {return(0);}
#endif

/* encoder ready */
#ifdef XFlag_Pin
inline void readySet() {XFlag_GPIO_Port->BSRR = XFlag_Pin;}
inline void readyClr() {XFlag_GPIO_Port->BSRR = (XFlag_Pin << 16);}
inline uint16_t ready() {return((XFlag_GPIO_Port->ODR & XFlag_Pin) != 0);}
#else
inline void readySet() {}
inline void readyClr() {}
inline uint16_t ready() {return(0);}
#endif

/* led output */
#ifdef Led_Pin
inline void ledSet() {Led_GPIO_Port->BSRR = Led_Pin;}
inline void ledClr() {Led_GPIO_Port->BSRR = (Led_Pin << 16);}
inline uint16_t led() {return((Led_GPIO_Port->ODR & Led_Pin) != 0);}
#else
inline void ledSet() {}
inline void ledClr() {}
inline uint16_t led() {return(0);}
#endif

/* a encoder input */
#ifdef A_Pin
inline uint16_t a() {return((A_GPIO_Port->IDR & A_Pin) != 0);}
inline uint16_t aIsSet() {return((A_GPIO_Port->IDR & A_Pin) != 0);}
inline uint16_t aIsClr() {return((A_GPIO_Port->IDR & A_Pin) == 0);}
#else
inline uint16_t aSet() {return(0);}
inline uint16_t aClr() {return(0);}
#endif

/* b encoder input */
#ifdef B_Pin
inline uint16_t b() {return((B_GPIO_Port->IDR & B_Pin) != 0);}
inline uint16_t bIsSet() {return((B_GPIO_Port->IDR & B_Pin) != 0);}
inline uint16_t bIsClr() {return((B_GPIO_Port->IDR & B_Pin) == 0);}
#else
inline uint16_t bSet() {return(0);}
inline uint16_t bClr() {return(0);}
#endif

/* a test signal */
#ifdef ATest_Pin
inline void aTestSet() {ATest_GPIO_Port->BSRR = ATest_Pin;}
inline void aTestClr() {ATest_GPIO_Port->BSRR = (ATest_Pin << 16);}
inline uint16_t aTest() {return((ATest_GPIO_Port->ODR & ATest_Pin) != 0);}
#else
inline void aTestSet() {}
inline void aTestClr() {}
inline uint16_t aTest() {return(0);}
#endif

/* b test signal */
#ifdef BTest_Pin
inline void bTestSet() {BTest_GPIO_Port->BSRR = BTest_Pin;}
inline void bTestClr() {BTest_GPIO_Port->BSRR = (BTest_Pin << 16);}
inline uint16_t bTest() {return((BTest_GPIO_Port->ODR & BTest_Pin) != 0);}
#else
inline void bTestSet() {}
inline void bTestClr() {}
inline uint16_t bTest() {return(0);}
#endif

/* sync test signal */
#ifdef IndexTest_Pin
inline void indexTestSet() {IndexTest_GPIO_Port->BSRR = IndexTest_Pin;}
inline void indexTestClr() {IndexTest_GPIO_Port->BSRR = (IndexTest_Pin << 16);}
inline uint16_t indexTest() {return((IndexTest_GPIO_Port->ODR & IndexTest_Pin) != 0);}
#else
inline void indexTestSet() {}
inline void indexTestClr() {}
inline uint16_t indexTest() {return(0);}
#endif

#endif /* __PINDEF_H */
#endif /* __STM32F1xx_HAL_H */
