#ifndef __CPU_HEADER_H_
#define __CPU_HEADER_H_

/* ----------------------------------------------------------------------------
   -- Interrupt vector numbers
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup Interrupt_vector_numbers Interrupt vector numbers
 * @{
 */

/** Interrupt Number Definitions */
#define NUMBER_OF_INT_VECTORS 48                 /**< Number of interrupts in the Vector table */

typedef enum IRQn {
  /* Auxiliary constants */
  NotAvail_IRQn                = -128,             /**< Not available device specific interrupt */

  /* Core interrupts */
  NonMaskableInt_IRQn          = -14,              /**< Non Maskable Interrupt */
  HardFault_IRQn               = -13,              /**< Cortex-M0 SV Hard Fault Interrupt */
  SVCall_IRQn                  = -5,               /**< Cortex-M0 SV Call Interrupt */
  PendSV_IRQn                  = -2,               /**< Cortex-M0 Pend SV Interrupt */
  SysTick_IRQn                 = -1,               /**< Cortex-M0 System Tick Interrupt */

  /* Device specific interrupts */
} IRQn_Type;

/*!
 * @}
 */ /* end of group Interrupt_vector_numbers */
/* ----------------------------------------------------------------------------
   -- Cortex M0 Core Configuration
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup Cortex_Core_Configuration Cortex M0 Core Configuration
 * @{
 */

#define __CM0PLUS_REV                  0x0000    /**< Core revision r0p0 */
#define __MPU_PRESENT                  0         /**< Defines if an MPU is present or not */
#define __VTOR_PRESENT                 1         /**< Defines if VTOR is present or not */
#define __NVIC_PRIO_BITS               2         /**< Number of priority bits implemented in the NVIC */
#define __Vendor_SysTickConfig         0         /**< Vendor specific implementation of SysTickConfig is defined */

#include "core_cm0plus.h"              /* Core Peripheral Access Layer */

/*!
 * @}
 */ /* end of group Cortex_Core_Configuration */

/* ----------------------------------------------------------------------------
   -- PORT Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PORT_Peripheral_Access_Layer PORT Peripheral Access Layer
 * @{
 */

/** PORT - Register Layout Typedef */
typedef struct {
  __IO uint32_t VERID;                             /**< offset: 0x0 */
  __IO uint32_t PARAM;                             /**< offset: 0x4 */
  __IO uint32_t GPCLR;                             /**< offset: 0x8 */
  __IO uint32_t GPCHR;                             /**< offset: 0xc */
  __IO uint32_t PCR[32];                           /**< offset: 0x10, step 0x04 */
  __IO uint32_t ISF;                               /**< offset: 0x90 */
       uint8_t RESERVED_0[44];
  __IO uint32_t DFER;                              /**< offset: 0xC0 */
  __IO uint32_t DFCR;                              /**< offset: 0xC4 */
  __IO uint32_t DFWR;                              /**< Digital Filter Width Register, offset: 0xC8 */
} PORT_Type;

/* ----------------------------------------------------------------------------
   -- PORT Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PORT_Register_Masks PORT Register Masks
 * @{
 */

/*! @name PCR - Pin Control Register n */
/*! @{ */
#define PORT_PCR_PS_MASK                         (0x1U)
#define PORT_PCR_PS_SHIFT                        (0U)
/*! PS - Pull Select
 *  0b0..Internal pulldown resistor is enabled on the corresponding pin, if the corresponding PE field is set.
 *  0b1..Internal pullup resistor is enabled on the corresponding pin, if the corresponding PE field is set.
 */
#define PORT_PCR_PS(x)                           (((uint32_t)(((uint32_t)(x)) << PORT_PCR_PS_SHIFT)) & PORT_PCR_PS_MASK)
#define PORT_PCR_PE_MASK                         (0x2U)
#define PORT_PCR_PE_SHIFT                        (1U)
/*! PE - Pull Enable
 *  0b0..Internal pullup or pulldown resistor is not enabled on the corresponding pin.
 *  0b1..Internal pullup or pulldown resistor is enabled on the corresponding pin, if the pin is configured as a digital input.
 */
#define PORT_PCR_PE(x)                           (((uint32_t)(((uint32_t)(x)) << PORT_PCR_PE_SHIFT)) & PORT_PCR_PE_MASK)
#define PORT_PCR_SRE_MASK                        (0x4U)
#define PORT_PCR_SRE_SHIFT                       (2U)
/*! SRE - Slew Rate Enable
 *  0b0..Fast slew rate is configured on the corresponding pin, if the pin is configured as a digital output.
 *  0b1..Slow slew rate is configured on the corresponding pin, if the pin is configured as a digital output.
 */
#define PORT_PCR_SRE(x)                          (((uint32_t)(((uint32_t)(x)) << PORT_PCR_SRE_SHIFT)) & PORT_PCR_SRE_MASK)
#define PORT_PCR_ODE_MASK                        (0x20U)
#define PORT_PCR_ODE_SHIFT                       (5U)
/*! ODE - Open Drain Enable
 *  0b0..Open drain output is disabled on the corresponding pin.
 *  0b1..Open drain output is enabled on the corresponding pin, if the pin is configured as a digital output.
 */
#define PORT_PCR_ODE(x)                          (((uint32_t)(((uint32_t)(x)) << PORT_PCR_ODE_SHIFT)) & PORT_PCR_ODE_MASK)
#define PORT_PCR_MUX_MASK                        (0x700U)
#define PORT_PCR_MUX_SHIFT                       (8U)
#define PORT_PCR_MUX(x)                          (((uint32_t)(((uint32_t)(x)) << PORT_PCR_MUX_SHIFT)) & PORT_PCR_MUX_MASK)
#define PORT_PCR_LK_MASK                         (0x8000U)
#define PORT_PCR_LK_SHIFT                        (15U)
/*! LK - Lock Register
 *  0b0..Pin Control Register fields [15:0] are not locked.
 *  0b1..Pin Control Register fields [15:0] are locked and cannot be updated until the next system reset.
 */
#define PORT_PCR_LK(x)                           (((uint32_t)(((uint32_t)(x)) << PORT_PCR_LK_SHIFT)) & PORT_PCR_LK_MASK)
#define PORT_PCR_IRQC_MASK                       (0xF0000U)
#define PORT_PCR_IRQC_SHIFT                      (16U)
#define PORT_PCR_IRQC(x)                         (((uint32_t)(((uint32_t)(x)) << PORT_PCR_IRQC_SHIFT)) & PORT_PCR_IRQC_MASK)
#define PORT_PCR_ISF_MASK                        (0x1000000U)
#define PORT_PCR_ISF_SHIFT                       (24U)
#define PORT_PCR_ISF(x)                          (((uint32_t)(((uint32_t)(x)) << PORT_PCR_ISF_SHIFT)) & PORT_PCR_ISF_MASK)
/*! @} */

/*!
 * @}
 */ /* end of group PORT_Register_Masks */

/* PORT - Peripheral instance base addresses */
/** Peripheral PORTA base address */
#define PORTA_BASE                               (0x40049000u)
/** Peripheral PORTA base pointer */
#define PORTA                                    ((PORT_Type *)PORTA_BASE)
/** Peripheral PORTB base address */
#define PORTB_BASE                               (0x4004A000u)
/** Peripheral PORTB base pointer */
#define PORTB                                    ((PORT_Type *)PORTB_BASE)
/** Peripheral PORTC base address */
#define PORTC_BASE                               (0x4004B000u)
/** Peripheral PORTC base pointer */
#define PORTC                                    ((PORT_Type *)PORTC_BASE)
/** Peripheral PORTD base address */
#define PORTD_BASE                               (0x4004C000u)
/** Peripheral PORTD base pointer */
#define PORTD                                    ((PORT_Type *)PORTD_BASE)
/** Peripheral PORTE base address */
#define PORTE_BASE                               (0x4004D000u)
/** Peripheral PORTE base pointer */
#define PORTE                                    ((PORT_Type *)PORTE_BASE)


/* ----------------------------------------------------------------------------
   -- GPIO Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup FGPIO_Peripheral_Access_Layer FGPIO Peripheral Access Layer
 * @{
 */

/** FGPIO - Register Layout Typedef */
typedef struct {
  __IO uint32_t PDOR;                              /**< Port Data Output Register, offset: 0x0 */
  __O  uint32_t PSOR;                              /**< Port Set Output Register, offset: 0x4 */
  __O  uint32_t PCOR;                              /**< Port Clear Output Register, offset: 0x8 */
  __O  uint32_t PTOR;                              /**< Port Toggle Output Register, offset: 0xC */
  __I  uint32_t PDIR;                              /**< Port Data Input Register, offset: 0x10 */
  __IO uint32_t PDDR;                              /**< Port Data Direction Register, offset: 0x14 */
  __IO uint32_t GACR;                              /**< GPIO Attribute Checker Register, offset: 0x18 */
} GPIO_Type;
/** Peripheral GPIOA base address */
#define GPIOA_BASE                               (0x400FF000u)
/** Peripheral GPIOA base pointer */
#define GPIOA                                    ((GPIO_Type *)GPIOA_BASE)
/** Peripheral GPIOB base address */
#define GPIOB_BASE                               (0x400FF040u)
/** Peripheral GPIOB base pointer */
#define GPIOB                                    ((GPIO_Type *)GPIOB_BASE)
/** Peripheral GPIOC base address */
#define GPIOC_BASE                               (0x400FF080u)
/** Peripheral GPIOC base pointer */
#define GPIOC                                    ((GPIO_Type *)GPIOC_BASE)
/** Peripheral GPIOD base address */
#define GPIOD_BASE                               (0x400FF0c0u)
/** Peripheral GPIOD base pointer */
#define GPIOD                                    ((GPIO_Type *)GPIOD_BASE)
/** Peripheral GPIOE base address */
#define GPIOE_BASE                               (0x400FF100u)
/** Peripheral GPIOE base pointer */
#define GPIOE                                    ((GPIO_Type *)GPIOE_BASE)


/* ----------------------------------------------------------------------------
   -- FMU Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup FMU_Peripheral_Access_Layer FMU Peripheral Access Layer
 * @{
 */

/** FMU - Register Layout Typedef */
typedef struct {
  __IO uint32_t FSTAT;                             /**< Flash Status Register, offset: 0x0 */
  __IO uint32_t FCNFG;                             /**< Flash Configuration Register, offset: 0x4 */
  __IO uint32_t FCTRL;                             /**< Flash Control Register, offset: 0x8 */
  __I  uint32_t FTEST;                             /**< Flash Test Register, offset: 0xC */
  __IO uint32_t FCCOB[8];                          /**< Flash Common Command Object Registers, array offset: 0x10, array step: 0x4 */
} FMU_Type;

/* ----------------------------------------------------------------------------
   -- FMU Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup FMU_Register_Masks FMU Register Masks
 * @{
 */

/*! @name FSTAT - Flash Status Register */
/*! @{ */
#define FMU_FSTAT_FAIL_MASK                      (0x1U)
#define FMU_FSTAT_FAIL_SHIFT                     (0U)
/*! FAIL - Command Fail Flag
 *  0b0..Error not detected
 *  0b1..Error detected
 */
#define FMU_FSTAT_FAIL(x)                        (((uint32_t)(((uint32_t)(x)) << FMU_FSTAT_FAIL_SHIFT)) & FMU_FSTAT_FAIL_MASK)
#define FMU_FSTAT_CMDABT_MASK                    (0x4U)
#define FMU_FSTAT_CMDABT_SHIFT                   (2U)
/*! CMDABT - Command Abort Flag
 *  0b0..No command abort detected
 *  0b1..Command abort detected
 */
#define FMU_FSTAT_CMDABT(x)                      (((uint32_t)(((uint32_t)(x)) << FMU_FSTAT_CMDABT_SHIFT)) & FMU_FSTAT_CMDABT_MASK)
#define FMU_FSTAT_PVIOL_MASK                     (0x10U)
#define FMU_FSTAT_PVIOL_SHIFT                    (4U)
/*! PVIOL - Command Protection Violation Flag
 *  0b0..No protection violation detected
 *  0b1..Protection violation detected
 */
#define FMU_FSTAT_PVIOL(x)                       (((uint32_t)(((uint32_t)(x)) << FMU_FSTAT_PVIOL_SHIFT)) & FMU_FSTAT_PVIOL_MASK)
#define FMU_FSTAT_ACCERR_MASK                    (0x20U)
#define FMU_FSTAT_ACCERR_SHIFT                   (5U)
/*! ACCERR - Command Access Error Flag
 *  0b0..No access error detected
 *  0b1..Access error detected
 */
#define FMU_FSTAT_ACCERR(x)                      (((uint32_t)(((uint32_t)(x)) << FMU_FSTAT_ACCERR_SHIFT)) & FMU_FSTAT_ACCERR_MASK)
#define FMU_FSTAT_CWSABT_MASK                    (0x40U)
#define FMU_FSTAT_CWSABT_SHIFT                   (6U)
/*! CWSABT - Command Write Sequence Abort Flag
 *  0b0..Command write sequence not aborted
 *  0b1..Command write sequence aborted
 */
#define FMU_FSTAT_CWSABT(x)                      (((uint32_t)(((uint32_t)(x)) << FMU_FSTAT_CWSABT_SHIFT)) & FMU_FSTAT_CWSABT_MASK)
#define FMU_FSTAT_CCIF_MASK                      (0x80U)
#define FMU_FSTAT_CCIF_SHIFT                     (7U)
/*! CCIF - Command Complete Interrupt Flag
 *  0b0..Flash command, initialization, or power mode recovery in progress
 *  0b1..Flash command, initialization, or power mode recovery has completed
 */
#define FMU_FSTAT_CCIF(x)                        (((uint32_t)(((uint32_t)(x)) << FMU_FSTAT_CCIF_SHIFT)) & FMU_FSTAT_CCIF_MASK)
#define FMU_FSTAT_CMDPRT_MASK                    (0x300U)
#define FMU_FSTAT_CMDPRT_SHIFT                   (8U)
/*! CMDPRT - Command protection level
 *  0b00..Secure, normal access
 *  0b01..Secure, privileged access
 *  0b10..Nonsecure, normal access
 *  0b11..Nonsecure, privileged access
 */
#define FMU_FSTAT_CMDPRT(x)                      (((uint32_t)(((uint32_t)(x)) << FMU_FSTAT_CMDPRT_SHIFT)) & FMU_FSTAT_CMDPRT_MASK)
#define FMU_FSTAT_CMDP_MASK                      (0x800U)
#define FMU_FSTAT_CMDP_SHIFT                     (11U)
/*! CMDP - Command protection status flag
 *  0b0..Command protection level and domain ID are stale
 *  0b1..Command protection level (CMDPRT) and domain ID (CMDDID) are set
 */
#define FMU_FSTAT_CMDP(x)                        (((uint32_t)(((uint32_t)(x)) << FMU_FSTAT_CMDP_SHIFT)) & FMU_FSTAT_CMDP_MASK)
#define FMU_FSTAT_CMDDID_MASK                    (0xF000U)
#define FMU_FSTAT_CMDDID_SHIFT                   (12U)
#define FMU_FSTAT_CMDDID(x)                      (((uint32_t)(((uint32_t)(x)) << FMU_FSTAT_CMDDID_SHIFT)) & FMU_FSTAT_CMDDID_MASK)
#define FMU_FSTAT_DFDIF_MASK                     (0x10000U)
#define FMU_FSTAT_DFDIF_SHIFT                    (16U)
/*! DFDIF - Double Bit Fault Detect Interrupt Flag
 *  0b0..Double bit fault not detected during a valid flash read access
 *  0b1..Double bit fault detected (or FCTRL[FDFD] is set) during a valid flash read access
 */
#define FMU_FSTAT_DFDIF(x)                       (((uint32_t)(((uint32_t)(x)) << FMU_FSTAT_DFDIF_SHIFT)) & FMU_FSTAT_DFDIF_MASK)
#define FMU_FSTAT_PEWEN_MASK                     (0x3000000U)
#define FMU_FSTAT_PEWEN_SHIFT                    (24U)
/*! PEWEN - Program-Erase Write Enable Control
 *  0b00..Writes are not enabled
 *  0b01..Writes are enabled for one flash or IFR phrase
 *  0b10..Writes are enabled for one flash or IFR page
 *  0b11..Reserved
 */
#define FMU_FSTAT_PEWEN(x)                       (((uint32_t)(((uint32_t)(x)) << FMU_FSTAT_PEWEN_SHIFT)) & FMU_FSTAT_PEWEN_MASK)
#define FMU_FSTAT_PERDY_MASK                     (0x80000000U)
#define FMU_FSTAT_PERDY_SHIFT                    (31U)
/*! PERDY - Program-Erase Ready Status Flag
 *  0b0..Program or sector erase command operation not stalled
 *  0b1..Program or sector erase command operation ready to execute
 */
#define FMU_FSTAT_PERDY(x)                       (((uint32_t)(((uint32_t)(x)) << FMU_FSTAT_PERDY_SHIFT)) & FMU_FSTAT_PERDY_MASK)
/*! @} */

/*! @name FCNFG - Flash Configuration Register */
/*! @{ */
#define FMU_FCNFG_CCIE_MASK                      (0x80U)
#define FMU_FCNFG_CCIE_SHIFT                     (7U)
/*! CCIE - Command Complete Interrupt Enable
 *  0b0..Command complete interrupt disabled
 *  0b1..Command complete interrupt enabled
 */
#define FMU_FCNFG_CCIE(x)                        (((uint32_t)(((uint32_t)(x)) << FMU_FCNFG_CCIE_SHIFT)) & FMU_FCNFG_CCIE_MASK)
#define FMU_FCNFG_ERSREQ_MASK                    (0x100U)
#define FMU_FCNFG_ERSREQ_SHIFT                   (8U)
/*! ERSREQ - Erase All Request
 *  0b0..No request or request complete
 *  0b1..Request to run the Erase All operation
 */
#define FMU_FCNFG_ERSREQ(x)                      (((uint32_t)(((uint32_t)(x)) << FMU_FCNFG_ERSREQ_SHIFT)) & FMU_FCNFG_ERSREQ_MASK)
#define FMU_FCNFG_DFDIE_MASK                     (0x10000U)
#define FMU_FCNFG_DFDIE_SHIFT                    (16U)
/*! DFDIE - Double Bit Fault Detect Interrupt Enable
 *  0b0..Double bit fault detect interrupt disabled
 *  0b1..Double bit fault detect interrupt enabled
 */
#define FMU_FCNFG_DFDIE(x)                       (((uint32_t)(((uint32_t)(x)) << FMU_FCNFG_DFDIE_SHIFT)) & FMU_FCNFG_DFDIE_MASK)
/*! @} */

/*! @name FCTRL - Flash Control Register */
/*! @{ */
#define FMU_FCTRL_RWSC_MASK                      (0xFU)
#define FMU_FCTRL_RWSC_SHIFT                     (0U)
#define FMU_FCTRL_RWSC(x)                        (((uint32_t)(((uint32_t)(x)) << FMU_FCTRL_RWSC_SHIFT)) & FMU_FCTRL_RWSC_MASK)
#define FMU_FCTRL_LSACTIVE_MASK                  (0x100U)
#define FMU_FCTRL_LSACTIVE_SHIFT                 (8U)
/*! LSACTIVE - Low speed active mode
 *  0b0..Full speed active mode requested
 *  0b1..Low speed active mode requested
 */
#define FMU_FCTRL_LSACTIVE(x)                    (((uint32_t)(((uint32_t)(x)) << FMU_FCTRL_LSACTIVE_SHIFT)) & FMU_FCTRL_LSACTIVE_MASK)
#define FMU_FCTRL_FDFD_MASK                      (0x10000U)
#define FMU_FCTRL_FDFD_SHIFT                     (16U)
/*! FDFD - Force Double Bit Fault Detect
 *  0b0..FSTAT[DFDIF] sets only if a double bit fault is detected during a valid flash read access from the platform flash controller
 *  0b1..FSTAT[DFDIF] sets during any valid flash read access from the platform flash controller. An interrupt
 *       request is generated if the DFDIE bit is set.
 */
#define FMU_FCTRL_FDFD(x)                        (((uint32_t)(((uint32_t)(x)) << FMU_FCTRL_FDFD_SHIFT)) & FMU_FCTRL_FDFD_MASK)
#define FMU_FCTRL_ABTREQ_MASK                    (0x1000000U)
#define FMU_FCTRL_ABTREQ_SHIFT                   (24U)
/*! ABTREQ - Abort Request
 *  0b0..No request to abort a command write sequence
 *  0b1..Request to abort a command write sequence
 */
#define FMU_FCTRL_ABTREQ(x)                      (((uint32_t)(((uint32_t)(x)) << FMU_FCTRL_ABTREQ_SHIFT)) & FMU_FCTRL_ABTREQ_MASK)
/*! @} */

/*! @name FTEST - Flash Test Register */
/*! @{ */
#define FMU_FTEST_TMECTL_MASK                    (0x1U)
#define FMU_FTEST_TMECTL_SHIFT                   (0U)
/*! TMECTL - Test Mode Entry Control
 *  0b0..FTEST register always reads 0 and writes to FTEST are ignored
 *  0b1..FTEST register is readable and can be written to enable writability of TME
 */
#define FMU_FTEST_TMECTL(x)                      (((uint32_t)(((uint32_t)(x)) << FMU_FTEST_TMECTL_SHIFT)) & FMU_FTEST_TMECTL_MASK)
#define FMU_FTEST_TMEWR_MASK                     (0x2U)
#define FMU_FTEST_TMEWR_SHIFT                    (1U)
/*! TMEWR - Test Mode Entry Writable
 *  0b0..TME bit is not writable
 *  0b1..TME bit is writable
 */
#define FMU_FTEST_TMEWR(x)                       (((uint32_t)(((uint32_t)(x)) << FMU_FTEST_TMEWR_SHIFT)) & FMU_FTEST_TMEWR_MASK)
#define FMU_FTEST_TME_MASK                       (0x4U)
#define FMU_FTEST_TME_SHIFT                      (2U)
/*! TME - Test Mode Entry
 *  0b0..Test mode entry not requested
 *  0b1..Test mode entry requested
 */
#define FMU_FTEST_TME(x)                         (((uint32_t)(((uint32_t)(x)) << FMU_FTEST_TME_SHIFT)) & FMU_FTEST_TME_MASK)
#define FMU_FTEST_TMODE_MASK                     (0x8U)
#define FMU_FTEST_TMODE_SHIFT                    (3U)
/*! TMODE - Test Mode Status
 *  0b0..Test mode not active
 *  0b1..Test mode active
 */
#define FMU_FTEST_TMODE(x)                       (((uint32_t)(((uint32_t)(x)) << FMU_FTEST_TMODE_SHIFT)) & FMU_FTEST_TMODE_MASK)
#define FMU_FTEST_TMELOCK_MASK                   (0x10U)
#define FMU_FTEST_TMELOCK_SHIFT                  (4U)
/*! TMELOCK - Test Mode Entry Lock
 *  0b0..FTEST register not locked from accepting writes
 *  0b1..FTEST register locked from accepting writes
 */
#define FMU_FTEST_TMELOCK(x)                     (((uint32_t)(((uint32_t)(x)) << FMU_FTEST_TMELOCK_SHIFT)) & FMU_FTEST_TMELOCK_MASK)
/*! @} */

/*! @name FCCOB - Flash Common Command Object Registers */
/*! @{ */
#define FMU_FCCOB_CCOBn_MASK                     (0xFFFFFFFFU)
#define FMU_FCCOB_CCOBn_SHIFT                    (0U)
#define FMU_FCCOB_CCOBn(x)                       (((uint32_t)(((uint32_t)(x)) << FMU_FCCOB_CCOBn_SHIFT)) & FMU_FCCOB_CCOBn_MASK)
/*! @} */

/* The count of FMU_FCCOB */
#define FMU_FCCOB_COUNT                          (8U)


/*!
 * @}
 */ /* end of group FMU_Register_Masks */


/* FMU - Peripheral instance base addresses */
/** Peripheral FMU0 base address */
#define FMU0_BASE                                (0x40020000u)
/** Peripheral FMU0 base pointer */
#define FMU0                                     ((FMU_Type *)FMU0_BASE)
/** Array initializer of FMU peripheral base addresses */
#define FMU_BASE_ADDRS                           { FMU0_BASE }
/** Array initializer of FMU peripheral base pointers */
#define FMU_BASE_PTRS                            { FMU0 }

/*!
 * @}
 */ /* end of group FMU_Peripheral_Access_Layer */


#endif /* __CPU_HEADER_H_ */
