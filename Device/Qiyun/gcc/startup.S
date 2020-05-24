/*****************************************************************************/
/* Version: GCC for ARM Embedded Processors                                  */
/*****************************************************************************/
    .syntax unified
    .arch armv6-m

    .section .isr_vector, "a"
    .align 2
    .globl __isr_vector
__isr_vector:
    .long   __StackTop                                      /* Top of Stack */
    .long   Reset_Handler                                   /* Reset Handler */
    .long   NMI_Handler                                     /* NMI Handler*/
    .long   HardFault_Handler                               /* Hard Fault Handler*/
    .long   MemManage_Handler                                               /* Reserved*/
    .long   BusFault_Handler                                               /* Reserved*/
    .long   UsageFault_Handler                                               /* Reserved*/
    .long   0                                               /* Reserved*/
    .long   0                                               /* Reserved*/
    .long   0                                               /* Reserved*/
    .long   0                                               /* Reserved*/
    .long   SVC_Handler                                     /* SVCall Handler*/
    .long   DebugMon_Handler                                               /* Reserved*/
    .long   0                                               /* Reserved*/
    .long   PendSV_Handler                                  /* PendSV Handler*/
    .long   SysTick_Handler                                 /* SysTick Handler*/

    .long   DMA_CH0_IRQHandler
    .long   DMA_CH1_IRQHandler
    .long   DMA_ERR_IRQHandler
    .long   FLASH_IRQHandler
    .long   PMU_IRQHandler
    .long   I2C0_IRQHandler
    .long   I2C1_IRQHandler
    .long   SPI0_IRQHandler
    .long   SPI1_IRQHandler
    .long   SPI2_IRQHandler
    .long   UART0_IRQHandler
    .long   UART1_IRQHandler
    .long   UART2_IRQHandler
    .long   UART3_IRQHandler
    .long   ADC0_IRQHandler
    .long   ADC1_IRQHandler
    .long   CMP0_IRQHandler
    .long   TIM0_IRQHandler
    .long   TIM1_IRQHandler
    .long   TIM2_IRQHandler
    .long   CAN0_IRQHandler
    .long   CAN1_IRQHandler
    .long   RTC_IRQHandler
    .long   RESERVED0_IRQHandler
    .long   RESERVED1_IRQHandler
    .long   SCC_IRQHandler
    .long   WDOG_IRQHandler
    .long   EWDT_IRQHandler
    .long   LPTIM0_IRQHandler
    .long   SRMC_IRQHandler
                                                            /* External Interrupts*/

    .size    __isr_vector, . - __isr_vector


    .text
    .thumb

/* Reset Handler */

    .thumb_func
    .align 2
    .globl   Reset_Handler
    .weak    Reset_Handler
    .type    Reset_Handler, %function
Reset_Handler:
    cpsid   i               /* Mask interrupts */
    .equ    VTOR, 0xE000ED08
    ldr     r0, =VTOR
    ldr     r1, =__isr_vector
    str     r1, [r0]
    ldr     r2, [r1]
    msr     msp, r2
#ifndef __NO_SYSTEM_INIT
    ldr   r0,=SystemInit
    blx   r0
#endif
/*     Loop to copy data from read only memory to RAM. The ranges
 *      of copy from/to are specified by following symbols evaluated in
 *      linker script.
 *      __etext: End of code section, i.e., begin of data sections to copy from.
 *      __data_start__/__data_end__: RAM address range that data should be
 *      copied to. Both must be aligned to 4 bytes boundary.  */

    ldr    r1, =__etext
    ldr    r2, =__data_start__
    ldr    r3, =__data_end__

    subs    r3, r2
    ble     .LC0

.LC1:
    subs    r3, 4
    ldr    r0, [r1,r3]
    str    r0, [r2,r3]
    bgt    .LC1
.LC0:

#ifdef __STARTUP_CLEAR_BSS
/*     This part of work usually is done in C library startup code. Otherwise,
 *     define this macro to enable it in this startup.
 *
 *     Loop to zero out BSS section, which uses following symbols
 *     in linker script:
 *      __bss_start__: start of BSS section. Must align to 4
 *      __bss_end__: end of BSS section. Must align to 4
 */
    ldr r1, =__bss_start__
    ldr r2, =__bss_end__

    subs    r2, r1
    ble .LC3

    movs    r0, 0
.LC2:
    subs    r2, 4
    str r0, [r1, r2]
    bge .LC2
.LC3:
#endif
    cpsie   i               /* Unmask interrupts */
#ifndef __START
#define __START _start
#endif
#ifndef __ATOLLIC__
    ldr   r0,=__START
    blx   r0
#else
    # ldr   r0,=__libc_init_array
    # blx   r0
    ldr   r0,=main
    bx    r0
#endif
    .pool
    .size Reset_Handler, . - Reset_Handler

    .align  1
    .thumb_func
    .weak DefaultISR
    .type DefaultISR, %function
DefaultISR:
    ldr r0, =DefaultISR
    bx r0
    .size DefaultISR, . - DefaultISR

    .align 1
    .thumb_func
    .weak NMI_Handler
    .type NMI_Handler, %function
NMI_Handler:
    ldr   r0,=NMI_Handler
    bx    r0
    .size NMI_Handler, . - NMI_Handler

    .align 1
    .thumb_func
    .weak HardFault_Handler
    .type HardFault_Handler, %function
HardFault_Handler:
    ldr   r0,=HardFault_Handler
    bx    r0
    .size HardFault_Handler, . - HardFault_Handler

    .align 1
    .thumb_func
    .weak SVC_Handler
    .type SVC_Handler, %function
SVC_Handler:
    ldr   r0,=SVC_Handler
    bx    r0
    .size SVC_Handler, . - SVC_Handler

    .align 1
    .thumb_func
    .weak PendSV_Handler
    .type PendSV_Handler, %function
PendSV_Handler:
    ldr   r0,=PendSV_Handler
    bx    r0
    .size PendSV_Handler, . - PendSV_Handler

    .align 1
    .thumb_func
    .weak SysTick_Handler
    .type SysTick_Handler, %function
SysTick_Handler:
    ldr   r0,=SysTick_Handler
    bx    r0
    .size SysTick_Handler, . - SysTick_Handler

    .align 1
    .thumb_func
    .weak DMA_CH0_IRQHandler
    .type DMA_CH0_IRQHandler, %function
DMA_CH0_IRQHandler:
    ldr   r0,=DMA_CH0_DriverIRQHandler
    bx    r0
    .size DMA_CH0_IRQHandler, . - DMA_CH0_IRQHandler

    .align 1
    .thumb_func
    .weak DMA_CH1_IRQHandler
    .type DMA_CH1_IRQHandler, %function
DMA_CH1_IRQHandler:
    ldr   r0,=DMA_CH1_DriverIRQHandler
    bx    r0
    .size DMA_CH1_IRQHandler, . - DMA_CH1_IRQHandler

    .align 1
    .thumb_func
    .weak DMA_ERR_IRQHandler
    .type DMA_ERR_IRQHandler, %function
DMA_ERR_IRQHandler:
    ldr   r0,=DMA_ERR_DriverIRQHandler
    bx    r0
    .size DMA_ERR_IRQHandler, . - DMA_ERR_IRQHandler

    .align 1
    .thumb_func
    .weak FLASH_IRQHandler
    .type FLASH_IRQHandler, %function
FLASH_IRQHandler:
    ldr   r0,=FLASH_DriverIRQHandler
    bx    r0
    .size FLASH_IRQHandler, . - FLASH_IRQHandler

    .align 1
    .thumb_func
    .weak PMU_IRQHandler
    .type PMU_IRQHandler, %function
PMU_IRQHandler:
    ldr   r0,=PMU_DriverIRQHandler
    bx    r0
    .size PMU_IRQHandler, . - PMU_IRQHandler

    .align 1
    .thumb_func
    .weak I2C0_IRQHandler
    .type I2C0_IRQHandler, %function
I2C0_IRQHandler:
    ldr   r0,=I2C0_DriverIRQHandler
    bx    r0
    .size I2C0_IRQHandler, . - PMU_IRQHandler

    .align 1
    .thumb_func
    .weak I2C1_IRQHandler
    .type I2C1_IRQHandler, %function
I2C1_IRQHandler:
    ldr   r0,=I2C1_DriverIRQHandler
    bx    r0
    .size I2C1_IRQHandler, . - PMU_IRQHandler

    .align 1
    .thumb_func
    .weak SPI0_IRQHandler
    .type SPI0_IRQHandler, %function
SPI0_IRQHandler:
    ldr   r0,=SPI0_DriverIRQHandler
    bx    r0
    .size SPI0_IRQHandler, . - PMU_IRQHandler

    .align 1
    .thumb_func
    .weak SPI1_IRQHandler
    .type SPI1_IRQHandler, %function
SPI1_IRQHandler:
    ldr   r0,=SPI1_DriverIRQHandler
    bx    r0
    .size SPI1_IRQHandler, . - PMU_IRQHandler

    .align 1
    .thumb_func
    .weak SPI2_IRQHandler
    .type SPI2_IRQHandler, %function
SPI2_IRQHandler:
    ldr   r0,=SPI2_DriverIRQHandler
    bx    r0
    .size SPI2_IRQHandler, . - PMU_IRQHandler

    .align 1
    .thumb_func
    .weak UART0_IRQHandler
    .type UART0_IRQHandler, %function
UART0_IRQHandler:
    ldr   r0,=UART0_DriverIRQHandler
    bx    r0
    .size UART0_IRQHandler, . - UART0_IRQHandler

    .align 1
    .thumb_func
    .weak UART1_IRQHandler
    .type UART1_IRQHandler, %function
UART1_IRQHandler:
    ldr   r0,=UART1_DriverIRQHandler
    bx    r0
    .size UART1_IRQHandler, . - UART1_IRQHandler

    .align 1
    .thumb_func
    .weak UART2_IRQHandler
    .type UART2_IRQHandler, %function
UART2_IRQHandler:
    ldr   r0,=UART2_DriverIRQHandler
    bx    r0
    .size UART2_IRQHandler, . - UART2_IRQHandler

    .align 1
    .thumb_func
    .weak UART3_IRQHandler
    .type UART3_IRQHandler, %function
UART3_IRQHandler:
    ldr   r0,=UART3_DriverIRQHandler
    bx    r0
    .size UART3_IRQHandler, . - UART3_IRQHandler

    .align 1
    .thumb_func
    .weak ADC0_IRQHandler
    .type ADC0_IRQHandler, %function
ADC0_IRQHandler:
    ldr   r0,=ADC0_DriverIRQHandler
    bx    r0
    .size ADC0_IRQHandler, . - ADC0_IRQHandler

    .align 1
    .thumb_func
    .weak ADC1_IRQHandler
    .type ADC1_IRQHandler, %function
ADC1_IRQHandler:
    ldr   r0,=ADC1_DriverIRQHandler
    bx    r0
    .size ADC1_IRQHandler, . - ADC1_IRQHandler

    .align 1
    .thumb_func
    .weak CMP0_IRQHandler
    .type CMP0_IRQHandler, %function
CMP0_IRQHandler:
    ldr   r0,=CMP0_DriverIRQHandler
    bx    r0
    .size CMP0_IRQHandler, . - CMP0_IRQHandler

    .align 1
    .thumb_func
    .weak TIM0_IRQHandler
    .type TIM0_IRQHandler, %function
TIM0_IRQHandler:
    ldr   r0,=TIM0_DriverIRQHandler
    bx    r0
    .size TIM0_IRQHandler, . - TIM0_IRQHandler

    .align 1
    .thumb_func
    .weak TIM1_IRQHandler
    .type TIM1_IRQHandler, %function
TIM1_IRQHandler:
    ldr   r0,=TIM1_DriverIRQHandler
    bx    r0
    .size TIM1_IRQHandler, . - TIM1_IRQHandler

    .align 1
    .thumb_func
    .weak TIM2_IRQHandler
    .type TIM2_IRQHandler, %function
TIM2_IRQHandler:
    ldr   r0,=TIM2_DriverIRQHandler
    bx    r0
    .size TIM2_IRQHandler, . - TIM2_IRQHandler

    .align 1
    .thumb_func
    .weak CAN0_IRQHandler
    .type CAN0_IRQHandler, %function
CAN0_IRQHandler:
    ldr   r0,=CAN0_DriverIRQHandler
    bx    r0
    .size CAN0_IRQHandler, . - CAN0_IRQHandler

    .align 1
    .thumb_func
    .weak CAN1_IRQHandler
    .type CAN1_IRQHandler, %function
CAN1_IRQHandler:
    ldr   r0,=CAN1_DriverIRQHandler
    bx    r0
    .size CAN1_IRQHandler, . - CAN1_IRQHandler

    .align 1
    .thumb_func
    .weak RTC_IRQHandler
    .type RTC_IRQHandler, %function
RTC_IRQHandler:
    ldr   r0,=RTC_DriverIRQHandler
    bx    r0
    .size RTC_IRQHandler, . - RTC_IRQHandler

    .align 1
    .thumb_func
    .weak RESERVED0_IRQHandler
    .type RESERVED0_IRQHandler, %function
RESERVED0_IRQHandler:
    ldr   r0,=RESERVED0_DriverIRQHandler
    bx    r0
    .size RESERVED0_IRQHandler, . - RESERVED0_IRQHandler

    .align 1
    .thumb_func
    .weak RESERVED1_IRQHandler
    .type RESERVED1_IRQHandler, %function
RESERVED1_IRQHandler:
    ldr   r0,=RESERVED1_DriverIRQHandler
    bx    r0
    .size RESERVED1_IRQHandler, . - RESERVED1_IRQHandler

    .align 1
    .thumb_func
    .weak SCC_IRQHandler
    .type SCC_IRQHandler, %function
SCC_IRQHandler:
    ldr   r0,=SCC_DriverIRQHandler
    bx    r0
    .size SCC_IRQHandler, . - SCC_IRQHandler

    .align 1
    .thumb_func
    .weak WDOG_IRQHandler
    .type WDOG_IRQHandler, %function
WDOG_IRQHandler:
    ldr   r0,=WDOG_DriverIRQHandler
    bx    r0
    .size WDOG_IRQHandler, . - WDOG_IRQHandler

    .align 1
    .thumb_func
    .weak EWDT_IRQHandler
    .type EWDT_IRQHandler, %function
EWDT_IRQHandler:
    ldr   r0,=EWDT_DriverIRQHandler
    bx    r0
    .size EWDT_IRQHandler, . - EWDT_IRQHandler

    .align 1
    .thumb_func
    .weak LPTIM0_IRQHandler
    .type LPTIM0_IRQHandler, %function
LPTIM0_IRQHandler:
    ldr   r0,=LPTIM0_DriverIRQHandler
    bx    r0
    .size LPTIM0_IRQHandler, . - LPTIM0_IRQHandler

    .align 1
    .thumb_func
    .weak SRMC_IRQHandler
    .type SRMC_IRQHandler, %function
SRMC_IRQHandler:
    ldr   r0,=SRMC_DriverIRQHandler
    bx    r0
    .size SRMC_IRQHandler, . - SRMC_IRQHandler

/*    Macro to define default handlers. Default handler
 *    will be weak symbol and just dead loops. They can be
 *    overwritten by other handlers */
    .macro def_irq_handler  handler_name
    .weak \handler_name
    .set  \handler_name, DefaultISR
    .endm

/* Exception Handlers */
    def_irq_handler    MemManage_Handler
    def_irq_handler    BusFault_Handler
    def_irq_handler    UsageFault_Handler
    def_irq_handler    DebugMon_Handler
    def_irq_handler    DMA_CH0_DriverIRQHandler
    def_irq_handler    DMA_CH1_DriverIRQHandler
    def_irq_handler    DMA_ERR_DriverIRQHandler
    def_irq_handler    FLASH_DriverIRQHandler
    def_irq_handler    PMU_DriverIRQHandler
    def_irq_handler    I2C0_DriverIRQHandler
    def_irq_handler    I2C1_DriverIRQHandler
    def_irq_handler    SPI0_DriverIRQHandler
    def_irq_handler    SPI1_DriverIRQHandler
    def_irq_handler    SPI2_DriverIRQHandler
    def_irq_handler    UART0_DriverIRQHandler
    def_irq_handler    UART1_DriverIRQHandler
    def_irq_handler    UART2_DriverIRQHandler
    def_irq_handler    UART3_DriverIRQHandler
    def_irq_handler    ADC0_DriverIRQHandler
    def_irq_handler    ADC1_DriverIRQHandler
    def_irq_handler    CMP0_DriverIRQHandler
    def_irq_handler    TIM0_DriverIRQHandler
    def_irq_handler    TIM1_DriverIRQHandler
    def_irq_handler    TIM2_DriverIRQHandler
    def_irq_handler    CAN0_DriverIRQHandler
    def_irq_handler    CAN1_DriverIRQHandler
    def_irq_handler    RTC_DriverIRQHandler
    def_irq_handler    RESERVED0_DriverIRQHandler
    def_irq_handler    RESERVED1_DriverIRQHandler
    def_irq_handler    SCC_DriverIRQHandler
    def_irq_handler    WDOG_DriverIRQHandler
    def_irq_handler    EWDT_DriverIRQHandler
    def_irq_handler    LPTIM0_DriverIRQHandler
    def_irq_handler    SRMC_DriverIRQHandler

    .end
