#include "common.h"
#define MRCC_UNLOCK_KEY                         (0x5b000000)
#include <stdio.h>
extern void env_sysinit(void);
extern void flash_test();
int main(){
    env_sysinit();
    MRCC0->CTRL[MRCC_GPIO] = MRCC_UNLOCK_KEY;
    MRCC0->CTRL[MRCC_GPIO] = MRCC_CTRL_RSTB_MASK|MRCC_CTRL_PR_MASK|MRCC_CTRL_CM_MASK;
    MRCC0->CTRL[MRCC_PORTB] = MRCC_UNLOCK_KEY;
    MRCC0->CTRL[MRCC_PORTB] = MRCC_CTRL_RSTB_MASK|MRCC_CTRL_PR_MASK|MRCC_CTRL_CM_MASK;

    PORTB->PCR[0] = PORT_PCR_MUX(1);
    PORTB->PCR[1] = PORT_PCR_MUX(1);
    GPIOB->PDDR |= 0x3;
    //flash_test();
    return 0;
}
