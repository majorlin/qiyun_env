#include "common.h"
#define MRCC_UNLOCK_KEY                         (0x5b000000)
#include <stdio.h>
#define __ram_func __attribute__( ( long_call, section(".data#") ) )
extern void env_sysinit(void);
__ram_func uint32_t FMU_check_for_fstat_errors(void)
{
    uint32_t ret_val;

    ret_val = FMU0->FSTAT;
    ret_val &= (FMU_FSTAT_PVIOL_MASK | FMU_FSTAT_ACCERR_MASK |
            FMU_FSTAT_CMDABT_MASK | FMU_FSTAT_FAIL_MASK); //RDCOL

    return ret_val;
}
/*Erase Flash Sector.  Erases all 4K byte addresses in a flash sector.
*/
__ram_func uint32_t FMU_ERSSCR(uint32_t* pAddr)
{
    uint32_t ret_val;

    FMU0->FSTAT = FMU_FSTAT_PVIOL_MASK | FMU_FSTAT_ACCERR_MASK | FMU_FSTAT_CMDABT_MASK;
    //write the command into the FMU module's CCOB registers.
    FMU0->FCCOB[0] = 0x42U;//FMU_ERSSCR_CMD;
    //Run command
    FMU0->FSTAT = FMU_FSTAT_CCIF_MASK;  //w1c to launch a flash command.

    FMU0->FSTAT |= FMU_FSTAT_PEWEN(1); //1-Writes are enabled for one flash or IFR Phrase
    for (int i = 0; i < 32; i++)
    {
        (*(pAddr + i)) = i << 24 | i << 16 | i << 8 | i;

    }

    //clears FSTAT[PEWEN] field
    FMU0->FSTAT |= FMU_FSTAT_PEWEN(0);

    //wait for PERDY to set and clear it
    while (( FMU0->FSTAT & FMU_FSTAT_PERDY_MASK) == 0);
    FMU0->FSTAT = FMU_FSTAT_PERDY_MASK;  //w1c to clear.

    //wait for command to complete
    while (( FMU0->FSTAT & FMU_FSTAT_CCIF_MASK) == 0);

    ret_val = FMU_check_for_fstat_errors();
    //returns 0 for pass or fstat error bits for fail.
    return ret_val;
}
__ram_func uint32_t FMU_PGM_PAGE(uint32_t* pAddr)
{
    uint32_t ret_val;

    //Clear any previous errors.   Some errors prevent a new command from running.
    FMU0->FSTAT = FMU_FSTAT_PVIOL_MASK | FMU_FSTAT_ACCERR_MASK | FMU_FSTAT_CMDABT_MASK;
    //write the command into the FMU module's CCOB registers.
    FMU0->FCCOB[0] = 0x23; //FMU_PGM_PAGE_CMD

    //Run command
    FMU0->FSTAT = FMU_FSTAT_CCIF_MASK;  //w1c to launch a flash command.


    FMU0->FSTAT |= FMU_FSTAT_PEWEN(1); //1-Writes are enabled for one flash or IFR Phrase
    for (int i = 0; i < 32; i++)
    {
        (*(pAddr + i)) = i << 24 | i << 16 | i << 8 | i;

    }

    //wait for PERDY to set and clear it
    while (( FMU0->FSTAT & FMU_FSTAT_PERDY_MASK) == 0);
    FMU0->FSTAT = FMU_FSTAT_PERDY_MASK;  //w1c to clear.

    //wait for command to complete
    while (( FMU0->FSTAT & FMU_FSTAT_CCIF_MASK) == 0);

    ret_val = FMU_check_for_fstat_errors();
    //returns 0 for pass or fstat error bits for fail.
    return ret_val;
}

__ram_func void flash_test(){
    //program 16 bytes
    // INFO("######","--------program main mem--------------------------");
    // WCE32(ADDR32(0x40020010),0x00000024,0x00000024); //cmd
    FMU0->FCCOB[0] = 0x00000024;
    // WO32(ADDR32(0x40020000),0x00000080);
    FMU0->FSTAT = 0x00000080;

    // while(!(ADDR32(0x40020000)&0x80));
    while(!(FMU0->FSTAT & 0x80));
    // RC32(ADDR32(0x40020000),0x00000280);
    // if (FMU0->FSTAT != 0x00000280){
    //     printf("E: FSTAT = 0x%lx\r\n", FMU0->FSTAT);
    // }

    uint32_t* flash_array = (uint32_t*) 0x2000;
    // RC32(ADDR32(0x00002000),0x12345678);
    // RC32(ADDR32(0x00002004),0x87654321);
    // RC32(ADDR32(0x00002008),0x55AA33CC);
    // RC32(ADDR32(0x0000200C),0xFF00FF00);
    for (int i = 0; i < 4; i++){
        flash_array[i] = i << 24 | i << 16 | i << 8 | i;
    }

    // // sector erase
    // // INFO("######","--------erase main sector--------------------------");
    // // WCE32(ADDR32(0x40020010),0x00000042,0x00000042); //cmd
    // FMU0->FCCOB[0] = 0x00000042;
    // // WO32(ADDR32(0x40020000),0x00000080);
    // FMU0->FSTAT = 0x00000080;
    // // while(!(ADDR32(0x40020000)&0x80));
    // while(!(FMU0->FSTAT & 0x80));
    // // RC32(ADDR32(0x40020000),0x00000280);
    // if (FMU0->FSTAT != 0x00000280){
    //     printf("E: FSTAT = 0x%x\n", FMU0->FSTAT);
    // }
}
void print_flash(uint32_t* pAddr, int len){
    for (int i =  0; i < len; i++){
        printf("[0x%08lx] = 0x%08lx\r\n", (uint32_t)(pAddr + i), pAddr[i]);
    } 
}
int main(){
    env_sysinit();
    // printf("Hello, world!\r\n");
    MRCC0->CTRL[MRCC_GPIO] = MRCC_UNLOCK_KEY;
    MRCC0->CTRL[MRCC_GPIO] = MRCC_CTRL_RSTB_MASK|MRCC_CTRL_PR_MASK|MRCC_CTRL_CM_MASK;
    MRCC0->CTRL[MRCC_PORTB] = MRCC_UNLOCK_KEY;
    MRCC0->CTRL[MRCC_PORTB] = MRCC_CTRL_RSTB_MASK|MRCC_CTRL_PR_MASK|MRCC_CTRL_CM_MASK;

    PORTB->PCR[0] = PORT_PCR_MUX(1);
    PORTB->PCR[1] = PORT_PCR_MUX(1);
    GPIOB->PDDR |= 0x3;
    // uint32_t* flash_array = (uint32_t*) 0x2000;
    // // printf("Before Program\r\n");
    // print_flash(flash_array, 2);
    // printf("FSTAT = 0x%lx\n", FMU0->FSTAT);
    // // // FMU_ERSSCR(flash_array);
    // //FMU_PGM_PAGE(flash_array);
    // flash_test();
    // // printf("After Program\r\n");
    // print_flash(flash_array, 4);
    // // // toggle GPIO
    while(1){
        for(int i =  0; i < 0x10; i++){
            asm("NOP");
        }
        printf("Hello World!\r\n");
        GPIOB->PTOR = 0x3;
    }
    return 0;
}
