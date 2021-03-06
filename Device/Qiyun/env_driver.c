#include "common.h"
void env_uart_init(){

    UART1->LCR |= UART_LCR_DLAB_EN;
    UART1->DLL = 7; //(uint32_t) divl;
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

void disable_wdog(){
    *(uint32_t*)(0x40052000 + 0x14) = 0xB0D9A1C4;
    *(uint32_t*)(0x40052000 + 0x14) = 0x1A1E3B0F;
    *(uint32_t*)(0x40052000 + 0x08) = 0x00000000;
}
void env_board_init(){
    // GPIO Module Clock
    MRCC0->CTRL[MRCC_GPIO] = MRCC_UNLOCK_KEY;
    MRCC0->CTRL[MRCC_GPIO] = MRCC_CTRL_RSTB_MASK|MRCC_CTRL_PR_MASK|MRCC_CTRL_CM_MASK;
    // POARTB Module Clock
    MRCC0->CTRL[MRCC_PORTB] = MRCC_UNLOCK_KEY;
    MRCC0->CTRL[MRCC_PORTB] = MRCC_CTRL_RSTB_MASK|MRCC_CTRL_PR_MASK|MRCC_CTRL_CM_MASK;
    // PORTD Module Clock
    MRCC0->CTRL[MRCC_PORTD] = MRCC_UNLOCK_KEY;
    MRCC0->CTRL[MRCC_PORTD] = MRCC_CTRL_RSTB_MASK|MRCC_CTRL_PR_MASK|MRCC_CTRL_CM_MASK;
    // PORTC Module Clock
    MRCC0->CTRL[MRCC_PORTC] = MRCC_UNLOCK_KEY;
    MRCC0->CTRL[MRCC_PORTC] = MRCC_CTRL_RSTB_MASK|MRCC_CTRL_PR_MASK|MRCC_CTRL_CM_MASK;
    // UART Module Clock
    MRCC0->CTRL[MRCC_UART1] = MRCC_UNLOCK_KEY;
    MRCC0->CTRL[MRCC_UART1] = MRCC_CTRL_RSTB_MASK|MRCC_CTRL_PR_MASK|MRCC_CTRL_CM_MASK;
    // CAN0 Module Clock
    MRCC0->CTRL[MRCC_CAN0] = MRCC_UNLOCK_KEY;
    MRCC0->CTRL[MRCC_CAN0] = MRCC_CTRL_RSTB_MASK|MRCC_CTRL_PR_MASK|MRCC_CTRL_CM_MASK;
    // CAN1 Module Clock
    MRCC0->CTRL[MRCC_CAN1] = MRCC_UNLOCK_KEY;
    MRCC0->CTRL[MRCC_CAN1] = MRCC_CTRL_RSTB_MASK|MRCC_CTRL_PR_MASK|MRCC_CTRL_CM_MASK;
    // UART PIN
    PORTD->PCR[13] = PORT_PCR_MUX(3);
    PORTD->PCR[14] = PORT_PCR_MUX(3);
    // LED 0 LED1
    PORTB->PCR[0] = PORT_PCR_MUX(1);
    PORTB->PCR[1] = PORT_PCR_MUX(1);
    // CAN0 PIN
    PORTC->PCR[3] = PORT_PCR_MUX(3); // TX
    PORTC->PCR[2] = PORT_PCR_MUX(3); // RX
    // CAN1 PIN
    PORTC->PCR[7] = PORT_PCR_MUX(3); // TX
    PORTC->PCR[6] = PORT_PCR_MUX(3); // RX
}
void env_sysinit(void)
{
    disable_wdog();
    env_board_init();
    env_uart_init();
    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(stdin, NULL, _IONBF, 0);
}

/******************************************************************************
* default_isr(void)
*
* Default ISR definition.
*
* In:  n/a
* Out: n/a
******************************************************************************/
// The register frame pushed onto the stack during exceptions
typedef struct
{
    uint32_t r0;
    uint32_t r1;
    uint32_t r2;
    uint32_t r3;
    uint32_t r12;
    uint32_t lr;
    void *pc;
    uint32_t psr;
} hw_stackframe_t;
void HardFault_Handler(void)
{
    // set up arguments and call _hardfault_isr
    asm("mov    r0, lr\n"  // arg 0
        "mrs    r1, psp\n" // arg 1
        "mrs    r2, msp\n" // arg 2
        "b      _default_isr\n");
}
static void _default_isr(uint32_t lr, void *psp, void *msp) __attribute__((used));
static void _default_isr(uint32_t lr, void *psp, void *msp)
{
#define VECTORNUM (*(volatile uint8_t *)(0xE000ED04))
    hw_stackframe_t *frame;

    // Find the active stack pointer (MSP or PSP)
    if (lr & 0x4)
        frame = psp;
    else
        frame = msp;

    printf("\r\n** HARD FAULT **\r\n\tpc=0x%p\r\n\tmsp=0x%p\r\n\tpsp=0x%p\r\n",
        frame->pc, msp, psp);
    printf("\r\n****default_isr entered on vector %d*****\r\n", VECTORNUM);
    asm("WFI");
}
