#include "common.h"
void env_uart_init(){

    MRCC0->CTRL[MRCC_UART1] = MRCC_UNLOCK_KEY;
    MRCC0->CTRL[MRCC_UART1] = MRCC_CTRL_RSTB_MASK|MRCC_CTRL_PR_MASK|MRCC_CTRL_CM_MASK;

    MRCC0->CTRL[MRCC_PORTD] = MRCC_UNLOCK_KEY;
    MRCC0->CTRL[MRCC_PORTD] = MRCC_CTRL_RSTB_MASK|MRCC_CTRL_PR_MASK|MRCC_CTRL_CM_MASK;

    PORTD->PCR[13] = PORT_PCR_MUX(3);
    PORTD->PCR[14] = PORT_PCR_MUX(3);

    UART1->LCR |= UART_LCR_DLAB_EN;
    UART1->DLL = 1; //(uint32_t) divl;
	UART1->DLH = 0; //(uint32_t) divh;
    UART1->LCR &= ~UART_LCR_DLAB_EN;
    // 8N1
    UART1->LCR = (UART_LCR_WLEN8 | UART_LCR_SBS_1BIT);
    // FIFO
    UART1->FCR = (UART_FCR_TRG_LEV3|UART_FCR_FIFO_EN);
}
void env_put_char(char c)
{
    // while((UART_IIR_INTID_MASK & UART1->IIR) != UART_IIR_INTID_THRE);
    while((UART_LSR_THRE & UART1->LSR) == 0);
	UART1->THR = (uint32_t) c;
}

char env_get_char()
{
    while(UART_LSR_RDR & UART1->LSR);
    return UART1->RBR;
}
void disable_wdog(void)
{
  *(unsigned int*)(0x40052000+0x14) = 0xB0D9A1C4;   //unlock key1
  *(unsigned int*)(0x40052000+0x14) = 0x1A1E3B0F;  //unlock key2
  *(unsigned int*)(0x40052000+0x08) = 0x00000000;  //disable wdog
}
void env_sysinit(void)
{
    disable_wdog();
    env_uart_init();
}

void HardFault_Handler(void)
{
    while(1);
}
