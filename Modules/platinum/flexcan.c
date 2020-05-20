/*
 * Copyright (c) 2020 NXP
 * ******************************************************************************
 * File: flexcan.c
 * Created Date: Wednesday, May 20th 2020, 1:56:30 pm
 * Author: Major Lin
 * -----
 * Last Modified: Wed May 20 2020
 * Modified By: Major Lin
 * -----
 * 
 * -----
 * HISTORY:
 * Date      	By           	Comments
 * ----------	-------------	----------------------------------------------------------
 * ******************************************************************************
 */

#include "common.h"
#include <stdio.h>

void flexcan_print_regs(CAN_Type *base){
    printf("MCR\t=0x%08lx\r\n", base->MCR);
    printf("CTRL1\t=0x%08lx\r\n", base->CTRL1);
    printf("TIMER\t=0x%08lx\r\n", base->TIMER);
    printf("RXMGMASK\t=0x%08lx\r\n", base->RXMGMASK);
    printf("RX14MASK\t=0x%08lx\r\n", base->RX14MASK);
    printf("RX15MASK\t=0x%08lx\r\n", base->RX15MASK);
    printf("ECR\t=0x%08lx\r\n", base->ECR);
    printf("ESR1\t=0x%08lx\r\n", base->ESR1);
    printf("IMASK1\t=0x%08lx\r\n", base->IMASK1);
    printf("IFLAG1\t=0x%08lx\r\n", base->IFLAG1);
    printf("CTRL2\t=0x%08lx\r\n", base->CTRL2);
    printf("ESR2\t=0x%08lx\r\n", base->ESR2);
    printf("CRCR\t=0x%08lx\r\n", base->CRCR);
    printf("RXFGMASK\t=0x%08lx\r\n", base->RXFGMASK);
    printf("RXFIR\t=0x%08lx\r\n", base->RXFIR);
    printf("CBT\t=0x%08lx\r\n", base->CBT);
}
