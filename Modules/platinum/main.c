#include "common.h"
#include "testcase.h"
#include <stdio.h>
extern void env_sysinit(void);
int main(){
    env_sysinit();
    printf("Hello, world!\r\n");
    flexcan_print_regs(CAN0);
    flexcan_print_regs(CAN1);
    while(1){
    }
    return 0;
}
