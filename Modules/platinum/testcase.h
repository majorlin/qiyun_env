/*
 * Copyright (c) 2020 NXP
 * ******************************************************************************
 * File: testcase.h
 * Created Date: Wednesday, May 20th 2020, 1:57:26 pm
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


#ifndef __TESTCASE_H__
#define __TESTCASE_H__
void flash_program();
void flash_erase();
void flexcan_print_regs(CAN_Type *base);
#endif /* __TESTCASE_H__ */