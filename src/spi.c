/*
 * spi.c — SPI1 slave driver for LinuxCNC communication
 * Replaces Mbed RemoraComms (TARGET_STM32F446RE/drivers/comms/RemoraComms.cpp)
 *
 * SPI1: PA_15 = NSS, PB_3 = SCK, PB_4 = MISO, PB_5 = MOSI
 * DMA2: Stream 3 = TX (memory→peripheral), Stream 0 = RX (peripheral→memory)
 * EXTI15_10: NSS rising edge (PA_15 = line 15) → processPacket()
 *
 * DMA is configured once in spi_comms_init() and enabled in spi_comms_start().
 * It runs in circular mode continuously; spi_process_packet() does NOT restart
 * DMA (only the error-recovery path does). This avoids losing data between
 * LinuxCNC transactions.
 *
 * Flexi protocol (matches LinuxCNC/Components/Flexi/flexi.c):
 *   - Master sends PRU_WRITE + command data → STM32 copies commands to rxData
 *   - Master sends PRU_READ → STM32 responds with PRU_DATA + feedback data
 *   - PRU_ESTOP header → eStop active
 *   - Spike filter on joint feedback (n=2 periods, M=250 magnitude)
 *   - SPIstatus link health tracking (valid/invalid transfer counting)
 *
 * Note on NSS: SSM=1 + SSI=1 (software NSS management, internal NSS held high)
 * means the SPI hardware ignores the NSS pin. The NSS pin (PA_15) is still
 * routed to EXTI line 15 so we can detect NSS edges from the LinuxCNC master.
 * This matches the mbed RemoraComms behaviour for TARGET_FLEXIHAL.
 */

#include "spi.h"
#include "configuration.h"
#include "gpio.h"

#include <string.h>
#include <stdint.h>
#include <stdbool.h>

/* NVIC */
#define NVIC_ISER_BASE 0xE000E100
#define NVIC_IPR_BASE  0xE000E400
#define NVIC_ISER0     (*(volatile uint32_t *)(NVIC_ISER_BASE + 0x00))
#define NVIC_ISER1     (*(volatile uint32_t *)(NVIC_ISER_BASE + 0x04))

/* SPI1 base */
#define SPI1_BASE 0x40013000
#define SPI1      ((volatile uint32_t *)SPI1_BASE)

/* GPIO / RCC base addresses */
#define GPIOA_BASE 0x40020000
#define GPIOB_BASE 0x40020400
#define RCC_BASE   0x40023800

/* SYSCFG + EXTI for NSS edge detection (PA_15 = EXTI line 15 -> EXTI15_10_IRQHandler) */
#define SYSCFG_BASE 0x40013800
#define SYSCFG_EXTICR4 (*(volatile uint32_t *)(SYSCFG_BASE + 0x18))
#define EXTI_BASE  0x40013C00
#define EXTI_RTSR  (*(volatile uint32_t *)(EXTI_BASE + 0x08))
#define EXTI_IMR   (*(volatile uint32_t *)(EXTI_BASE + 0x00))
#define EXTI_PR    (*(volatile uint32_t *)(EXTI_BASE + 0x14))
#define EXTI_LINE15 (1U << 15)

/* SPI register offsets */
#define SPI_CR1   0x00
#define SPI_CR2   0x04
#define SPI_SR    0x08
#define SPI_DR    0x0C

/* SPI_CR1 bits */
#define SPI_CR1_CPHA      (1U << 0)
#define SPI_CR1_CPOL      (1U << 1)
#define SPI_CR1_MSTR      (1U << 2)
#define SPI_CR1_BR_POS    3
#define SPI_CR1_SPE       (1U << 6)
#define SPI_CR1_LSBFIRST  (1U << 7)
#define SPI_CR1_SSI       (1U << 8)
#define SPI_CR1_SSM       (1U << 9)
#define SPI_CR1_RXONLY    (1U << 10)
#define SPI_CR1_DFF       (1U << 11)
#define SPI_CR1_CRCNEXT   (1U << 12)
#define SPI_CR1_CRCEN     (1U << 13)
#define SPI_CR1_BIDIOE    (1U << 14)
#define SPI_CR1_BIDIMODE  (1U << 15)

/* SPI_CR2 bits */
#define SPI_CR2_RXDMAEN   (1U << 0)
#define SPI_CR2_TXDMAEN   (1U << 1)
#define SPI_CR2_SSOE      (1U << 2)
#define SPI_CR2_FRF       (1U << 4)
#define SPI_CR2_ERRIE     (1U << 5)
#define SPI_CR2_RXNEIE    (1U << 6)
#define SPI_CR2_TXEIE     (1U << 7)

/* SPI_SR bits */
#define SPI_SR_RXNE   (1U << 0)
#define SPI_SR_TXE    (1U << 1)
#define SPI_SR_BSY    (1U << 7)
#define SPI_SR_OVR    (1U << 6)

/* DMA2 base */
#define DMA2_BASE 0x40026400

/* DMA flag-clear registers (LIFCR covers streams 0-3, offset 0x08) */
#define DMA_LIFCR_OFFSET 0x08
/* Per-stream clear bits in LIFCR: FEIF(0) DMEIF(1) TEIF(2) HTIF(3) TCIF(4) */
#define DMA_STREAM_CLEAR_BITS 0x1F
#define DMA_STREAM0_CLEAR_POS 0   /* Stream 0 = bits [4:0]   */
#define DMA_STREAM3_CLEAR_POS 18  /* Stream 3 = bits [22:18] */

/* TX shadow buffer — DMA reads from this stable copy so it never sees a
 * partially-updated txData while module ISRs rewrite feedback/inputs.
 */
static uint8_t tx_shadow[SPI_BUFF_SIZE] __attribute__((aligned(4)));

/* DMA registers (per stream, offset 0x18 per stream) */
#define DMA_SxCR_OFFSET   0x00
#define DMA_SxNDTR_OFFSET 0x04
#define DMA_SxPAR_OFFSET  0x08
#define DMA_SxM0AR_OFFSET 0x0C
#define DMA_SxM1AR_OFFSET 0x10
#define DMA_SxFCR_OFFSET  0x14

/* Stream base addresses */
#define DMA2_S0_BASE (DMA2_BASE + 0x00)
#define DMA2_S1_BASE (DMA2_BASE + 0x18)
#define DMA2_S3_BASE (DMA2_BASE + 0x48)

/* DMA_SxCR bits */
#define DMA_SxCR_EN        (1U << 0)
#define DMA_SxCR_DIR_POS   6
#define DMA_SxCR_CHSEL_POS 25
#define DMA_SxCR_CIRC      (1U << 8)
#define DMA_SxCR_PFCTRL    (1U << 19)
#define DMA_SxCR_PRIO_POS  12
#define DMA_SxCR_MSIZE_POS 13
#define DMA_SxCR_PSIZE_POS 11
#define DMA_SxCR_MINC      (1U << 10)
#define DMA_SxCR_PINC      (1U << 9)
#define DMA_SxCR_TCIE      (1U << 4)

/* Status */
static volatile bool g_spi_status = false;
static volatile bool g_spi_error  = false;
static volatile int  g_reject_cnt = 0;

/* Spike filter state (matches Flexi flexi.c: n=2 periods, M=250 magnitude) */
#define SPI_SPIKE_N   2
#define SPI_SPIKE_M   250
static int32_t  g_fb_old_count[JOINTS]   = { 0 };
static int8_t   g_fb_filter_count[JOINTS] = { 0 };


/* DMA stream register access helpers */
#define DMA_STREAM(reg, stream) (*(volatile uint32_t *)((stream) + (reg)))

static void dma_stream_config(uint32_t stream_base, uint32_t config,
                              uint32_t periph_addr, uint32_t mem_addr,
                              uint32_t ndtr)
{
    DMA_STREAM(DMA_SxCR_OFFSET, stream_base) = config;
    DMA_STREAM(DMA_SxPAR_OFFSET, stream_base) = periph_addr;
    DMA_STREAM(DMA_SxM0AR_OFFSET, stream_base) = mem_addr;
    DMA_STREAM(DMA_SxNDTR_OFFSET, stream_base) = ndtr;
}

static void dma_stream_enable(uint32_t stream_base, bool enable)
{
    volatile uint32_t *cr = (volatile uint32_t *)((stream_base) + DMA_SxCR_OFFSET);
    if (enable) *cr |= DMA_SxCR_EN;
    else *cr &= ~DMA_SxCR_EN;
}

static void nvic_enable_irq(uint32_t irq_number, uint32_t priority)
{
    /* Enable interrupt in the correct NVIC ISER register (ISER0: IRQs 0-31, ISER1: 32-63) */
    if (irq_number < 32) {
        NVIC_ISER0 |= (1U << irq_number);
    } else {
        NVIC_ISER1 |= (1U << (irq_number - 32));
    }
    /* Write a single priority byte (upper 4 bits = priority, lower 4 bits = subpriority) */
    *(volatile uint8_t *)(NVIC_IPR_BASE + irq_number) = (uint8_t)((priority << 4) & 0xFF);
}

/* Apply spike filter to joint feedback counts.
 * Matches Flexi flexi.c: if abs(new-old) > M and filter_count < n, hold old value.
 */
static void spike_filter_feedback(volatile int32_t *feedback)
{
    for (int i = 0; i < JOINTS; i++) {
        int32_t new_count = feedback[i];
        int32_t diff = new_count - g_fb_old_count[i];
        if (diff < 0) diff = -diff;  /* absolute value */

        if (diff > SPI_SPIKE_M && g_fb_filter_count[i] < SPI_SPIKE_N) {
            /* recent big change: hold previous value */
            ++g_fb_filter_count[i];
            new_count = g_fb_old_count[i];
        } else {
            /* normal operation, or the big change is real after all */
            g_fb_filter_count[i] = 0;
        }
        g_fb_old_count[i] = new_count;
        feedback[i] = new_count;
    }
}

/* Prepare the TX shadow buffer with a PRU_DATA response containing current
 * feedback. Builds into tx_shadow (which DMA reads) with interrupts disabled
 * so the DMA never observes a partially-updated buffer while module ISRs
 * concurrently rewrite txData fields (jointFeedback, inputs, etc.).
 * Preserves PRU_ESTOP when the eStop is active so the master reliably sees it.
 */
static void spi_prepare_data_response(void)
{
    /* Snapshot txData into tx_shadow atomically w.r.t. module ISRs. */
    uint32_t primask;
    __asm__ volatile ("mrs %0, primask" : "=r"(primask));
    __asm__ volatile ("cpsid i" ::: "memory");

    tx_shadow[0] = (uint8_t)(g_pru_estop ? PRU_ESTOP : PRU_DATA);
    tx_shadow[1] = (uint8_t)(PRU_DATA >> 8);
    tx_shadow[2] = (uint8_t)(PRU_DATA >> 16);
    tx_shadow[3] = (uint8_t)(PRU_DATA >> 24);

    for (int i = 0; i < JOINTS; i++) {
        int32_t fb = txData.jointFeedback[i];
        tx_shadow[4 + i * 4 + 0] = (uint8_t)(fb);
        tx_shadow[4 + i * 4 + 1] = (uint8_t)(fb >> 8);
        tx_shadow[4 + i * 4 + 2] = (uint8_t)(fb >> 16);
        tx_shadow[4 + i * 4 + 3] = (uint8_t)(fb >> 24);
    }

    /* processVariable[0] (float) — 4 bytes at offset 24 */
    {
        float pv = txData.processVariable[0];
        uint32_t pv_bits;
        memcpy(&pv_bits, &pv, sizeof(pv_bits));
        tx_shadow[24 + 0] = (uint8_t)(pv_bits);
        tx_shadow[24 + 1] = (uint8_t)(pv_bits >> 8);
        tx_shadow[24 + 2] = (uint8_t)(pv_bits >> 16);
        tx_shadow[24 + 3] = (uint8_t)(pv_bits >> 24);
    }

    /* inputs (uint16_t) — 2 bytes at offset 28 */
    {
        uint16_t in = txData.inputs;
        tx_shadow[28] = (uint8_t)(in);
        tx_shadow[29] = (uint8_t)(in >> 8);
    }

    /* Remaining bytes (offsets 30-63) — spare, keep zero. */
    for (int i = 30; i < SPI_BUFF_SIZE; i++)
        tx_shadow[i] = 0;

    if (primask == 0)
        __asm__ volatile ("cpsie i" ::: "memory");

    /* Apply spike filter to the canonical jointFeedback (module source of truth). */
    spike_filter_feedback(txData.jointFeedback);
}

/* Re-enable SPI and DMA after an error. Called from the main loop error path.
 * Clears DMA error flags (FEIF/DMEIF/TEIF/HTIF/TCIF on both streams) and the
 * SPI overrun flag (SPI_SR_OVR) before restarting, so stale error state does
 * not immediately re-trigger.
 */
void spi_comms_error_recovery(void)
{
    /* Disable SPI and DMA streams */
    volatile uint32_t *spi_cr1 = (volatile uint32_t *)(SPI1_BASE + SPI_CR1);
    *spi_cr1 &= ~SPI_CR1_SPE;

    dma_stream_enable(DMA2_S3_BASE, false); /* TX */
    dma_stream_enable(DMA2_S0_BASE, false); /* RX */

    /* Clear DMA error flags for both streams via LIFCR */
    volatile uint32_t *dma_lifcr = (volatile uint32_t *)(DMA2_BASE + DMA_LIFCR_OFFSET);
    *dma_lifcr = (DMA_STREAM_CLEAR_BITS << DMA_STREAM0_CLEAR_POS)
               | (DMA_STREAM_CLEAR_BITS << DMA_STREAM3_CLEAR_POS);

    /* Clear SPI overrun flag (and any other SR flags) by reading DR then SR */
    volatile uint32_t *spi_sr  = (volatile uint32_t *)(SPI1_BASE + SPI_SR);
    volatile uint32_t *spi_dr  = (volatile uint32_t *)(SPI1_BASE + SPI_DR);
    (void)*spi_dr;
    (void)*spi_sr;

    /* Re-initialise TX buffer header */
    txData.header = PRU_DATA;

    /* Re-enable DMA streams */
    dma_stream_enable(DMA2_S3_BASE, true);  /* TX */
    dma_stream_enable(DMA2_S0_BASE, true);  /* RX */

    /* Re-enable SPI */
    *spi_cr1 |= SPI_CR1_SPE;

    /* Reset error state */
    g_spi_error = false;
    g_reject_cnt = 0;
}

void spi_comms_init(void)
{
    /* Initialize GPIO pins for SPI1 alternate function (AF5) */
    static GPIOPin gpio_nss, gpio_sck, gpio_miso, gpio_mosi;
    gpio_init(&gpio_nss,  "PA_15", GPIO_ALTERNATE, GPIO_PULLNONE);
    gpio_init(&gpio_sck,  "PB_3",  GPIO_ALTERNATE, GPIO_PULLNONE);
    gpio_init(&gpio_miso, "PB_4",  GPIO_ALTERNATE, GPIO_PULLNONE);
    gpio_init(&gpio_mosi, "PB_5",  GPIO_ALTERNATE, GPIO_PULLNONE);

    /* Set alternate function AF5 for SPI1 pins */
    volatile uint32_t *GPIOB_AFRL = (volatile uint32_t *)(GPIOB_BASE + 0x20);
    volatile uint32_t *GPIOA_AFRH = (volatile uint32_t *)(GPIOA_BASE + 0x24);

    /* PB_3, PB_4, PB_5 -> AF5 (0101) */
    *GPIOB_AFRL = (*GPIOB_AFRL & ~(0xF << (3 * 4))) | (0x5 << (3 * 4));
    *GPIOB_AFRL = (*GPIOB_AFRL & ~(0xF << (4 * 4))) | (0x5 << (4 * 4));
    *GPIOB_AFRL = (*GPIOB_AFRL & ~(0xF << (5 * 4))) | (0x5 << (5 * 4));

    /* PA_15 -> AF5 */
    *GPIOA_AFRH = (*GPIOA_AFRH & ~(0xF << ((15 - 8) * 4))) | (0x5 << ((15 - 8) * 4));

    /* Enable SPI1 clock (APB2ENR bit 12) */
    volatile uint32_t *RCC_APB2ENR = (volatile uint32_t *)(RCC_BASE + 0x44);
    *RCC_APB2ENR |= (1U << 12); /* SPI1EN */

    /* Enable DMA2 clock (AHB1ENR bit 22) */
    volatile uint32_t *RCC_AHB1ENR = (volatile uint32_t *)(RCC_BASE + 0x30);
    *RCC_AHB1ENR |= (1U << 22); /* DMA2EN */

    /* Enable SYSCFG clock for EXTI mapping (APB2ENR bit 0) */
    *RCC_APB2ENR |= (1U << 0); /* SYSCFGEN */

    /* Map PA_15 (NSS) to EXTI line 15: EXTICR4 bits[15:12] = 0000 (Port A) */
    SYSCFG_EXTICR4 = (SYSCFG_EXTICR4 & ~(0xF << 12)) | (0x0 << 12);

    /* Enable rising edge trigger on EXTI line 15 */
    EXTI_RTSR |= EXTI_LINE15;

    /* Unmask interrupt on EXTI line 15 */
    EXTI_IMR |= EXTI_LINE15;

    /* Enable EXTI15_10 IRQ in NVIC (IRQ 40, priority 4) */
    nvic_enable_irq(40, 4);

    /* Configure SPI1: Slave, 8-bit, CPOL=0, CPHA=1, MSB first, software NSS (SSM=1, SSI=1) */
    volatile uint32_t *spi_cr1 = (volatile uint32_t *)(SPI1_BASE + SPI_CR1);
    volatile uint32_t *spi_cr2 = (volatile uint32_t *)(SPI1_BASE + SPI_CR2);

    *spi_cr1 = 0;
    *spi_cr1 |= SPI_CR1_CPHA;      /* CPHA = 1 */
    *spi_cr1 &= ~SPI_CR1_CPOL;     /* CPOL = 0 */
    *spi_cr1 &= ~SPI_CR1_MSTR;     /* Slave mode */
    *spi_cr1 &= ~SPI_CR1_DFF;      /* 8-bit data */
    *spi_cr1 &= ~SPI_CR1_LSBFIRST; /* MSB first */
    *spi_cr1 |= SPI_CR1_SSM;       /* Software NSS management */
    *spi_cr1 |= SPI_CR1_SSI;       /* SSI = 1 (NSS high internally) */

    *spi_cr2 = 0;
    *spi_cr2 |= SPI_CR2_RXDMAEN;   /* RX DMA enable */
    *spi_cr2 |= SPI_CR2_TXDMAEN;   /* TX DMA enable */
    /* SSOE not used in slave mode with SSM=1 */

    /* --- Configure DMA streams once (enabled later in spi_comms_start) --- */

    /* DMA TX: memory → SPI1_DR, channel 3, memory-to-peripheral, circular */
    dma_stream_config(DMA2_S3_BASE,
        (0x3 << DMA_SxCR_CHSEL_POS) | /* Channel 3 */
        (0x1 << DMA_SxCR_DIR_POS) |   /* Memory-to-peripheral (TX) */
        DMA_SxCR_MINC |               /* Memory increment */
        (0x0 << DMA_SxCR_PSIZE_POS) | /* 8-bit peripheral */
        (0x0 << DMA_SxCR_MSIZE_POS) | /* 8-bit memory */
        DMA_SxCR_CIRC |               /* Circular mode */
        (0x3 << DMA_SxCR_PRIO_POS),   /* Very high priority */
        (uint32_t)&SPI1[SPI_DR / 4],  /* Peripheral address (SPI_DR) */
        (uint32_t)tx_shadow,          /* Memory address — DMA reads stable copy */
        SPI_BUFF_SIZE);

    /* DMA RX: SPI1_DR → memory, channel 3, peripheral-to-memory, circular */
    dma_stream_config(DMA2_S0_BASE,
        (0x3 << DMA_SxCR_CHSEL_POS) | /* Channel 3 */
        (0x0 << DMA_SxCR_DIR_POS) |   /* Peripheral-to-memory (RX) */
        DMA_SxCR_MINC |               /* Memory increment */
        (0x0 << DMA_SxCR_PSIZE_POS) | /* 8-bit peripheral */
        (0x0 << DMA_SxCR_MSIZE_POS) | /* 8-bit memory */
        DMA_SxCR_CIRC |               /* Circular mode */
        (0x3 << DMA_SxCR_PRIO_POS),   /* Very high priority */
        (uint32_t)&SPI1[SPI_DR / 4],  /* Peripheral address */
        (uint32_t)rxData.rxBuffer,    /* Memory address — DMA writes raw master data here */
        SPI_BUFF_SIZE);
}

void spi_comms_start(void)
{
    /* Initialize TX buffer header */
    txData.header = PRU_DATA;

    /* Enable DMA streams (they were configured in spi_comms_init) */
    dma_stream_enable(DMA2_S3_BASE, true); /* TX */
    dma_stream_enable(DMA2_S0_BASE, true); /* RX */

    /* Enable SPI */
    volatile uint32_t *spi_cr1 = (volatile uint32_t *)(SPI1_BASE + SPI_CR1);
    *spi_cr1 |= SPI_CR1_SPE;
}

void spi_process_packet(void)
{
    /* Monitor SPI overrun flag: set when a byte was overwritten before DMA
     * read it out of SPI_DR. Flag it as a communications error. */
    volatile uint32_t *spi_sr = (volatile uint32_t *)(SPI1_BASE + SPI_SR);
    if (*spi_sr & SPI_SR_OVR) {
        g_spi_error = true;
        /* Clear OVR: read DR then SR */
        volatile uint32_t *spi_dr = (volatile uint32_t *)(SPI1_BASE + SPI_DR);
        (void)*spi_dr;
        (void)*spi_sr;
    }

    /* DMA RX stream already wrote master bytes directly into rxData.rxBuffer.
     * Read the header and command data from there — no intermediate copy needed.
     */
    uint32_t header = *(volatile uint32_t *)rxData.rxBuffer;

    switch (header) {
    case PRU_READ:
        /* Master is requesting data. Respond with PRU_DATA + current feedback. */
        g_spi_status = true;
        g_reject_cnt = 0;
        spi_prepare_data_response();
        break;

    case PRU_WRITE:
        /* Master is sending commands. Commands already in rxData.rxBuffer via DMA.
         * Prepare data response with current feedback for the next PRU_READ.
         */
        g_spi_status = true;
        g_reject_cnt = 0;
        spi_prepare_data_response();
        break;

    case PRU_ESTOP:
        /* eStop notification from master */
        g_pru_estop = true;
        g_spi_status = false;
        g_reject_cnt = 0;
        txData.header = PRU_ESTOP;
        break;

    default:
        g_reject_cnt++;
        if (g_reject_cnt > 5) {
            g_spi_error = true;
            g_spi_status = false;
        }
        break;
    }
}

bool spi_get_status(void)  { return g_spi_status; }
void spi_set_status(bool s) { g_spi_status = s; }
bool spi_get_error(void)   { return g_spi_error; }
void spi_set_error(bool e) { g_spi_error = e; }

/* EXTI15_10_IRQHandler — called on NSS rising edge (PA_15 = EXTI line 15) */
void EXTI15_10_IRQHandler(void)
{
    if (EXTI_PR & EXTI_LINE15) {
        EXTI_PR = EXTI_LINE15; /* clear pending flag */
        spi_process_packet();
    }
}
