#include "common.h"
#include "testcase.h"
#include <stdio.h>
extern void env_sysinit(void);
int main(){
    env_sysinit();
    printf("Hello, world!\r\n");
    GPIOB->PDDR |= 0x3;
    flash_erase();
    flash_program();
    flexcan_print_regs(CAN0);
    flexcan_print_regs(CAN1);
    while(1){
    }
    return 0;
}
