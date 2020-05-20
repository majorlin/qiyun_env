/*
 * Copyright (c) 2020 NXP
 * ******************************************************************************
 * File: flash.c
 * Created Date: Wednesday, May 20th 2020, 1:56:44 pm
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

void flash_program(){
    FMC->FCTRL = 0x01;
    FMC->FADDR = 0x4000;
    FMC->FDATA[0] = 0x12345678;
    FMC->FDATA[1] = 0x12345678;
    FMC->FDATA[2] = 0x12345678;
    FMC->FDATA[3] = 0x12345678;
    FMC->FCMD = 0x24;
    FMC->FSTAT = 0x80;
}
void flash_erase(){
    FMC->FADDR = 0x2000;
    FMC->FCMD = 0x42;
    FMC->FSTAT = 0x80;
}

