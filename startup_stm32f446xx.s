/*
 * Startup file for STM32F446RE — bare-metal
 * Adapted from STMicroelectronics CMSIS device startup (startup_stm32f446xx.s)
 * for the Arm GNU Toolchain (hard float, cortex-m4).
 *
 * Provides:
 *   - Initial stack pointer and vector table
 *   - Reset_Handler: set SP, SystemInit, copy .data, clear .bss, call main
 *   - Weak interrupt handlers (overridable in application)
 */

.syntax unified
.cpu    cortex-m4
.fpu    fpv4-sp-d16
.thumb

.global  g_pfnVectors
.global  Reset_Handler

.extern  _estack
.extern  _sidata
.extern  _sdata
.extern  _edata
.extern  _sbss
.extern  _ebss
.extern  SystemInit
.extern  main

/* Vector table — located at start of Flash (0x08010000 via linker script) */
.section  .isr_vector
.align  2
.type   g_pfnVectors, %object
g_pfnVectors:
    .word  _estack                          /* Initial SP */
    .word  Reset_Handler                    /* Reset */

    .word  NMI_Handler                      /* NMI */
    .word  HardFault_Handler                /* Hard fault */
    .word  MemManage_Handler                /* MPU fault */
    .word  BusFault_Handler                 /* Bus fault */
    .word  UsageFault_Handler               /* Usage fault */
    .word  0                                /* Reserved */
    .word  0                                /* Reserved */
    .word  0                                /* Reserved */
    .word  0                                /* Reserved */
    .word  SVC_Handler                      /* SVCall */
    .word  DebugMon_Handler                 /* Debug monitor */
    .word  0                                /* Reserved */
    .word  PendSV_Handler                   /* PendSV */
    .word  SysTick_Handler                  /* SysTick */

    /* External interrupts (STM32F446RE) */
    .word  WWDG_IRQHandler                  /* 0  Window WatchDog */
    .word  PVD_IRQHandler                   /* 1  PVD through EXTI */
    .word  TAMP_STAMP_IRQHandler            /* 2  Tamper/TimeStamp */
    .word  RTC_WKUP_IRQHandler              /* 3  RTC Wakeup */
    .word  FLASH_IRQHandler                 /* 4  Flash */
    .word  RCC_IRQHandler                   /* 5  RCC */
    .word  EXTI0_IRQHandler                 /* 6  EXTI0 */
    .word  EXTI1_IRQHandler                 /* 7  EXTI1 */
    .word  EXTI2_IRQHandler                 /* 8  EXTI2 (QEI index) */
    .word  EXTI3_IRQHandler                 /* 9  EXTI3 */
    .word  EXTI4_IRQHandler                 /* 10 EXTI4 (SPI1 NSS rising edge) */
    .word  DMA1_Stream0_IRQHandler          /* 11 DMA1 Stream 0 */
    .word  DMA1_Stream1_IRQHandler          /* 12 DMA1 Stream 1 */
    .word  DMA1_Stream2_IRQHandler          /* 13 DMA1 Stream 2 */
    .word  DMA1_Stream3_IRQHandler          /* 14 DMA1 Stream 3 */
    .word  DMA1_Stream4_IRQHandler          /* 15 DMA1 Stream 4 */
    .word  DMA1_Stream5_IRQHandler          /* 16 DMA1 Stream 5 */
    .word  DMA1_Stream6_IRQHandler          /* 17 DMA1 Stream 6 */
    .word  ADC_IRQHandler                   /* 18 ADC1/2/3 */
    .word  CAN1_TX_IRQHandler               /* 19 CAN1 TX */
    .word  CAN1_RX0_IRQHandler              /* 20 CAN1 RX0 */
    .word  CAN1_RX1_IRQHandler              /* 21 CAN1 RX1 */
    .word  CAN1_SCE_IRQHandler              /* 22 CAN1 SCE */
    .word  EXTI9_5_IRQHandler               /* 23 EXTI[9:5] */
    .word  TIM1_BRK_TIM9_IRQHandler         /* 24 TIM1 Break / TIM9 (Base thread) */
    .word  TIM1_UP_TIM10_IRQHandler         /* 25 TIM1 Update / TIM10 (Servo thread) */
    .word  TIM1_TRG_COM_TIM11_IRQHandler    /* 26 TIM1 Trigger/Commutation / TIM11 (Comms thread) */
    .word  TIM1_CC_IRQHandler               /* 27 TIM1 Capture Compare */
    .word  TIM2_IRQHandler                  /* 28 TIM2 (QEI encoder) */
    .word  TIM3_IRQHandler                  /* 29 TIM3 */
    .word  TIM4_IRQHandler                  /* 30 TIM4 */
    .word  I2C1_EV_IRQHandler               /* 31 I2C1 Event */
    .word  I2C1_ER_IRQHandler               /* 32 I2C1 Error */
    .word  I2C2_EV_IRQHandler               /* 33 I2C2 Event */
    .word  I2C2_ER_IRQHandler               /* 34 I2C2 Error */
    .word  SPI1_IRQHandler                  /* 35 SPI1 */
    .word  SPI2_IRQHandler                  /* 36 SPI2 */
    .word  USART1_IRQHandler                /* 37 USART1 (stdio) */
    .word  USART2_IRQHandler                /* 38 USART2 */
    .word  USART3_IRQHandler                /* 39 USART3 */
    .word  EXTI15_10_IRQHandler             /* 40 EXTI[15:10] */
    .word  RTC_Alarm_IRQHandler             /* 41 RTC Alarm */
    .word  OTG_FS_WKUP_IRQHandler           /* 42 OTG FS Wakeup */
    .word  TIM8_BRK_TIM12_IRQHandler        /* 43 TIM8 Break / TIM12 */
    .word  TIM8_UP_TIM13_IRQHandler         /* 44 TIM8 Update / TIM13 */
    .word  TIM8_TRG_COM_TIM14_IRQHandler    /* 45 TIM8 Trigger/Commutation / TIM14 */
    .word  TIM8_CC_IRQHandler               /* 46 TIM8 Capture Compare */
    .word  DMA1_Stream7_IRQHandler          /* 47 DMA1 Stream 7 */
    .word  0                                /* 48 Reserved (FSMC not on F446) */
    .word  SDIO_IRQHandler                  /* 49 SDIO */
    .word  TIM5_IRQHandler                  /* 50 TIM5 */
    .word  SPI3_IRQHandler                  /* 51 SPI3 */
    .word  UART4_IRQHandler                 /* 52 UART4 */
    .word  UART5_IRQHandler                 /* 53 UART5 */
    .word  TIM6_DAC_IRQHandler              /* 54 TIM6 / DAC */
    .word  TIM7_IRQHandler                  /* 55 TIM7 */
    .word  DMA2_Stream0_IRQHandler          /* 56 DMA2 Stream 0 (SPI1 RX) */
    .word  DMA2_Stream1_IRQHandler          /* 57 DMA2 Stream 1 */
    .word  DMA2_Stream2_IRQHandler          /* 58 DMA2 Stream 2 */
    .word  DMA2_Stream3_IRQHandler          /* 59 DMA2 Stream 3 (SPI1 TX) */
    .word  DMA2_Stream4_IRQHandler          /* 60 DMA2 Stream 4 */
    .word  0                                /* 61 Reserved (ETH not on F446RE) */
    .word  0                                /* 62 Reserved */
    .word  CAN2_TX_IRQHandler               /* 63 CAN2 TX */
    .word  CAN2_RX0_IRQHandler              /* 64 CAN2 RX0 */
    .word  CAN2_RX1_IRQHandler              /* 65 CAN2 RX1 */
    .word  CAN2_SCE_IRQHandler              /* 66 CAN2 SCE */
    .word  OTG_FS_IRQHandler                /* 67 OTG FS */
    .word  DMA2_Stream5_IRQHandler          /* 68 DMA2 Stream 5 */
    .word  DMA2_Stream6_IRQHandler          /* 69 DMA2 Stream 6 */
    .word  DMA2_Stream7_IRQHandler          /* 70 DMA2 Stream 7 */
    .word  USART6_IRQHandler                /* 71 USART6 */
    .word  I2C3_EV_IRQHandler               /* 72 I2C3 Event */
    .word  I2C3_ER_IRQHandler               /* 73 I2C3 Error */
    .word  0                                /* 74 Reserved (OTG HS not on F446RE) */
    .word  0                                /* 75 Reserved */
    .word  0                                /* 76 Reserved */
    .word  0                                /* 77 Reserved */
    .word  0                                /* 78 Reserved (DCMI not on F446RE) */
    .word  0                                /* 79 Reserved */
    .word  0                                /* 80 Reserved (RNG not on F446RE) */
    .word  FPU_IRQHandler                   /* 81 FPU */

.size   g_pfnVectors, .-g_pfnVectors

/* Reset_Handler — CMSIS order: SP → SystemInit → .data → .bss → main */
.section  .text.Reset_Handler, "ax"
.align  2
.weak   Reset_Handler
.type   Reset_Handler, %function
Reset_Handler:
    /* Set the initial stack pointer */
    ldr   sp, =_estack

    /* Call the clock system initialization function (PLL, bus dividers) */
    bl    SystemInit

    /* Copy the data segment initializers from Flash to SRAM */
    ldr   r2, =_sidata
    ldr   r3, =_sdata
    ldr   r4, =_edata
    cmp   r3, r4
    bge   copy_done
copy_loop:
    ldr   r5, [r2], #4
    str   r5, [r3], #4
    cmp   r3, r4
    blt   copy_loop
copy_done:

    /* Zero fill the .bss segment in SRAM */
    ldr   r3, =_sbss
    ldr   r4, =_ebss
    mov   r5, #0
    cmp   r3, r4
    bge   clear_done
clear_loop:
    str   r5, [r3], #4
    cmp   r3, r4
    blt   clear_loop
clear_done:

    /* Call the application's entry point */
    bl    main

    /* If main returns, loop forever (safe for embedded) */
hang:
    b     hang

.size   Reset_Handler, .-Reset_Handler

/* Weak default interrupt handlers — branch to themselves (overridable) */
.macro  weak_handler name
    .weak   \name
    .type   \name, %function
\name:
    b       \name
    .size   \name, .-\name
.endm

weak_handler NMI_Handler
weak_handler HardFault_Handler
weak_handler MemManage_Handler
weak_handler BusFault_Handler
weak_handler UsageFault_Handler
weak_handler SVC_Handler
weak_handler DebugMon_Handler
weak_handler PendSV_Handler
weak_handler SysTick_Handler
weak_handler WWDG_IRQHandler
weak_handler PVD_IRQHandler
weak_handler TAMP_STAMP_IRQHandler
weak_handler RTC_WKUP_IRQHandler
weak_handler FLASH_IRQHandler
weak_handler RCC_IRQHandler
weak_handler EXTI0_IRQHandler
weak_handler EXTI1_IRQHandler
weak_handler EXTI2_IRQHandler
weak_handler EXTI3_IRQHandler
weak_handler EXTI4_IRQHandler
weak_handler DMA1_Stream0_IRQHandler
weak_handler DMA1_Stream1_IRQHandler
weak_handler DMA1_Stream2_IRQHandler
weak_handler DMA1_Stream3_IRQHandler
weak_handler DMA1_Stream4_IRQHandler
weak_handler DMA1_Stream5_IRQHandler
weak_handler DMA1_Stream6_IRQHandler
weak_handler ADC_IRQHandler
weak_handler CAN1_TX_IRQHandler
weak_handler CAN1_RX0_IRQHandler
weak_handler CAN1_RX1_IRQHandler
weak_handler CAN1_SCE_IRQHandler
weak_handler EXTI9_5_IRQHandler
weak_handler TIM1_BRK_TIM9_IRQHandler
weak_handler TIM1_UP_TIM10_IRQHandler
weak_handler TIM1_TRG_COM_TIM11_IRQHandler
weak_handler TIM1_CC_IRQHandler
weak_handler TIM2_IRQHandler
weak_handler TIM3_IRQHandler
weak_handler TIM4_IRQHandler
weak_handler I2C1_EV_IRQHandler
weak_handler I2C1_ER_IRQHandler
weak_handler I2C2_EV_IRQHandler
weak_handler I2C2_ER_IRQHandler
weak_handler SPI1_IRQHandler
weak_handler SPI2_IRQHandler
weak_handler USART1_IRQHandler
weak_handler USART2_IRQHandler
weak_handler USART3_IRQHandler
weak_handler EXTI15_10_IRQHandler
weak_handler RTC_Alarm_IRQHandler
weak_handler OTG_FS_WKUP_IRQHandler
weak_handler TIM8_BRK_TIM12_IRQHandler
weak_handler TIM8_UP_TIM13_IRQHandler
weak_handler TIM8_TRG_COM_TIM14_IRQHandler
weak_handler TIM8_CC_IRQHandler
weak_handler DMA1_Stream7_IRQHandler
weak_handler SDIO_IRQHandler
weak_handler TIM5_IRQHandler
weak_handler SPI3_IRQHandler
weak_handler UART4_IRQHandler
weak_handler UART5_IRQHandler
weak_handler TIM6_DAC_IRQHandler
weak_handler TIM7_IRQHandler
weak_handler DMA2_Stream0_IRQHandler
weak_handler DMA2_Stream1_IRQHandler
weak_handler DMA2_Stream2_IRQHandler
weak_handler DMA2_Stream3_IRQHandler
weak_handler DMA2_Stream4_IRQHandler
weak_handler CAN2_TX_IRQHandler
weak_handler CAN2_RX0_IRQHandler
weak_handler CAN2_RX1_IRQHandler
weak_handler CAN2_SCE_IRQHandler
weak_handler OTG_FS_IRQHandler
weak_handler DMA2_Stream5_IRQHandler
weak_handler DMA2_Stream6_IRQHandler
weak_handler DMA2_Stream7_IRQHandler
weak_handler USART6_IRQHandler
weak_handler I2C3_EV_IRQHandler
weak_handler I2C3_ER_IRQHandler
weak_handler FPU_IRQHandler
