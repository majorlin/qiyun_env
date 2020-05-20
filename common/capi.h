#ifndef __CAPI_H_
#define ADDR32(x)                (*((volatile uint32_t*)(x)))
#define RC32(x, y)               do { if ((x) != (y)) {GPIOB->PTOR = 0x3;printf("E");} } while(0)
#define WCE32(x, y, z)           do { (x) = y; if ((x) != (z)) {GPIOB->PTOR = 0x3;printf("E");} } while(0)
#define WO32(x, y)               do { (x) = (y); } while(0)
#endif
