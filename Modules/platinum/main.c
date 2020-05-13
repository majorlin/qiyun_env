#include "common.h"
int main(){
    // toggle GPIO
    *(volatile uint32_t *)(0x40065000 + 124) = 0x5b000000;
    *(volatile uint32_t *)(0x40065000 + 124) = 0x00c00001;

    *(volatile uint32_t *)(0x40065000 + 132) = 0x5b000000;
    *(volatile uint32_t *)(0x40065000 + 132) = 0x00c00001;
    PORTB->PCR[0] = PORT_PCR_MUX(1);
    PORTB->PCR[1] = PORT_PCR_MUX(1);
    GPIOB->PDDR |= 0x3;
    // volatile uint32_t i = 0;
    while(1){
        for(int i =  0; i < 0x10; i++){
            asm("NOP");
        }
        GPIOB->PTOR = 0x3;
    }
    return 0;
}
