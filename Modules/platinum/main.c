#include "common.h"
#define MRCC_UNLOCK_KEY                         (0x5b000000)
// #include <stdio.h>
extern void env_sysinit(void);
int main(){
    env_sysinit();
    //printf("Hello, world!\r\n");
    MRCC0->CTRL[MRCC_GPIO] = MRCC_UNLOCK_KEY;
    MRCC0->CTRL[MRCC_GPIO] = MRCC_CTRL_RSTB_MASK|MRCC_CTRL_PR_MASK|MRCC_CTRL_CM_MASK;
    MRCC0->CTRL[MRCC_PORTB] = MRCC_UNLOCK_KEY;
    MRCC0->CTRL[MRCC_PORTB] = MRCC_CTRL_RSTB_MASK|MRCC_CTRL_PR_MASK|MRCC_CTRL_CM_MASK;

    PORTB->PCR[0] = PORT_PCR_MUX(1);
    PORTB->PCR[1] = PORT_PCR_MUX(1);
    GPIOB->PDDR |= 0x3;
    // toggle GPIO
    while(1){
        for(int i =  0; i < 0x10; i++){
            asm("NOP");
        }
        GPIOB->PTOR = 0x3;
    }
    return 0;
}
