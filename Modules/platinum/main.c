#include "common.h"
#include "testcase.h"
#include <stdio.h>
extern void env_sysinit(void);
extern void flexcan_test(void);
int main(){
    env_sysinit();
    flexcan_test();
    while(1){
    }
    return 0;
}
