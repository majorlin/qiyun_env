#include "common.h"
int main(){
    // toggle GPIO
    for(int i = 0; i < 32; i++){
        PORTA->PCR[i] = PORT_PCR_MUX(1);
    }
    GPIOA->PDDR = 0xFFFFFFFF;
    while(1){
        GPIOA->PTOR = 0xFFFFFFFF;
    }
    return 0;
}
