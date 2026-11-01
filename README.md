# STM32F446RE Bare-Metal Firmware for LinuxCNC (Flexi-HAL)

Firmware bare-metal viết lại từ Mbed OS (Remora-OS5) cho STM32F446RE, chạy như một **SPI Slave** giao tiếp với LinuxCNC qua thành phần HAL `flexi.c`. Dự án phục vụ nghiên cứu học thuật, không sử dụng Mbed OS, không cấp phát động, truy cập thanh ghi trực tiếp.

**Mục tiêu:** Thay thế toàn bộ stack Mbed OS bằng mã C thuần clàng, build bằng `arm-none-eabi-gcc` qua Makefile, chạy trên STM32F446RE (Cortex-M4F).

---

## 1. Kiến trúc hệ thống

```mermaid
flowchart TD
    subgraph LinuxCNC["LinuxCNC (Pi4/PC) - SPI Master"]
        HAL[flexi.c HAL component]
        SPI_M[SPI Master]
    end

    subgraph STM32["STM32F446RE - SPI Slave (bare-metal)"]
        NSS[PA_15 NSS]
        SCK[PB_3 SCK]
        MISO[PB_4 MISO]
        MOSI[PB_5 MOSI]

        subgraph DMA["DMA2 - Circular"]
            DMA_RX[Stream 0: RX - SPI1_DR → rxData]
            DMA_TX[Stream 3: TX - tx_shadow → SPI1_DR]
        end

        subgraph ISR["Interrupt Service Routines"]
            TIM9[TIM9 Base 120 kHz - stepgen, QEI]
            TIM10[TIM10 Servo 1 kHz - digital I/O, PWM, eStop, reset]
            TIM11[TIM11 Comms 57.6 kHz - trống]
            EXTI[EXTI15_10 - NSS edge → spi_process_packet]
        end

        subgraph Modules["Module System (static arrays, no malloc)"]
            STEP[stepgen ×5]
            DIN[Digital IN ×11]
            DOUT[Digital OUT ×10]
            PWM[PWM spindle PA_8]
            QEI[QEI TIM2 encoder]
            RST[Reset pin PC_3]
            ESTOP[eStop PA_2]
        end
    end

    LinuxCNC -->|MOSI| MOSI
    LinuxCNC <--|MISO| MISO
    LinuxCNC -->|SCK| SCK
    LinuxCNC -->|NSS| NSS

    MOSI --> DMA_RX
    DMA_TX --> MISO
    NSS --> EXTI
    EXTI -->|header switch| Modules
    TIM9 --> STEP
    TIM9 --> QEI
    TIM10 --> DIN
    TIM10 --> DOUT
    TIM10 --> PWM
    TIM10 --> ESTOP
    TIM10 --> RST
```

### Luồng dữ liệu SPI

1. **LinuxCNC (Master)** gửi 64 bytes qua MOSI (PRU_WRITE / PRU_READ / PRU_ESTOP).
2. **DMA2 Stream 0 (RX)** ghi trực tiếp vào `rxData.rxBuffer[64]`.
3. NSS rising edge → **EXTI15_10 IRQ** → `spi_process_packet()` phân tích header.
4. Nếu `PRU_READ`/`PRU_WRITE`: `spi_prepare_data_response()` chụp ảnh chớp (`tx_shadow`) các feedback vào bộ nhớ DMA TX.
5. **DMA2 Stream 3 (TX)** gửi `tx_shadow` về MISO cho LinuxCNC (PRU_DATA + jointFeedback + inputs + processVariable).

---

## 2. Cấu trúc thư mục

```
Firmware/MakefileFW/
├── README.md             ← Tài liệu này
├── Makefile              # Quy tắc biên dịch (make, make size, make clean)
├── STM32F446RE_FLASH.ld  # Linker script: FLASH @ 0x08010000, RAM @ 0x20000000
├── startup_stm32f446xx.s # Vector table + Reset_Handler (copy .data, zero .bss, SystemInit, main)
├── inc/                  # Header files
│   ├── configuration.h   # Hằng số: PRU_BASEFREQ, PRU_SERVOFREQ, SPI_ERR_MAX, STEPBIT, JOINTS...
│   ├── remora.h          # Union rxData_t / txData_t (packed 64 bytes) + PRU_* headers
│   ├── module.h          # Module base class: update / updatePost / slowUpdate / configure
│   ├── gpio.h            # GPIOPin struct + gpio_init/get/set/toggle
│   ├── spi.h             # SPI1 slave DMA API: spi_comms_init/start, status/error, recovery
│   ├── timer.h           # Timer init/start/stop, ISR declarations, module registration
│   ├── digitalpin.h      # Digital I/O (DO/DI)
│   ├── pwm.h             # Software PWM spindle
│   ├── qei.h             # QEI encoder (TIM2 hardware mode)
│   ├── resetpin.h        # Reset pin module
│   ├── estop.h           # eStop module
│   ├── miniprintf.h      # Minimal printf, _write retarget
│   ├── usart.h           # USART1 stdio (PA_9/PA_10, 115200 baud)
│   └── config.h          # JSON config struct + readJsonConfig/deserialiseJSON/configThreads
├── src/                  # Source files
│   ├── main.c            # State machine, main loop, watchdog, module loading
│   ├── spi.c             # SPI1 slave DMA + Flexi protocol + spike filter + error recovery
│   ├── timer.c           # TIM9/TIM10/TIM11/TIM2 ISR + NVIC byte-level IPR
│   ├── module.c          # module_init()
│   ├── system_stm32f4xx.c# Clock init: HSE 25MHz → PLL 168MHz
│   ├── stepgen.c         # DDS stepgen (5 joints, Base thread)
│   ├── digitalpin.c      # 10 DO + 11 DI (Servo thread)
│   ├── pwm.c             # Software PWM spindle (PA_8, Servo thread)
│   ├── qei.c             # TIM2 hardware encoder mode (Base thread poll)
│   ├── resetpin.c        # Reset pin PC_3 (Servo thread)
│   ├── estop.c           # eStop PA_2 → PRU_ESTOP header (Servo thread)
│   ├── usart.c           # USART1 115200 baud + _write retarget
│   ├── config.c          # JSON config parser (malloc-free)
│   └── miniprintf.c      # Minimal printf (%c %s %d %u %x %X %p %%)
└── build/                # Build artifacts (.o, .elf, .hex, .bin, .map) — không commit
```

---

## 3. Khái niệm kỹ thuật cốt बोल

| Thành phần | Chi tiết |
|------------|----------|
| MCU | STM32F446RE, ARM Cortex-M4F, FPU single-precision |
| Clock | HSE 25 MHz XTAL → PLL → SYSCLK 168 MHz, AHB 168 MHz, APB1 42 MHz, APB2 84 MHz |
| Toolchain | arm-none-eabi-gcc 14.3.1, `-mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16` |
| Build flags | `-ffreestanding -nostdlib -std=gnu11 -Wall -Wextra` |
| Linker script | FLASH bắt đầu tại `0x08010000` (sau bootloader), RAM tại `0x20000000` |
| SPI1 Slave | PA_15=NSS, PB_3=SCK, PB_4=MISO, PB_5=MOSI (AF5), Mode 0, SSM+SSI |
| DMA | DMA2 Stream3 TX (mem→peri, circular), Stream0 RX (peri→mem, circular) |
| EXTI | EXTI15_10 trên PA_15 (NSS rising edge), IRQ 40 |
| Timers | TIM9 Base 120 kHz (IRQ 24, prio 2), TIM10 Servo 1 kHz (IRQ 25, prio 3), TIM11 Comms 57.6 kHz (IRQ 26, prio 4), TIM2 QEI (IRQ 28) |
| Watchdog | IWDG ~2000 ms (LSI /64, reload 999), kick mỗi pass main loop |
| USART1 | PA_9=TX, PA_10=RX, AF7, 115200 baud, `_write()` retarget cho miniprintf |
| Flexi protocol | PRU_DATA / PRU_READ / PRU_WRITE / PRU_ESTOP, SPI_BUFF_SIZE=64, JOINTS=5, STEPBIT=22 |
| Module pattern | `struct Module` (update/updatePost/slowUpdate/configure/priv), mảng static, không malloc |

---

## 4. Các thread timer và module đăng ký

| Thread | Timer | Tần suất | IRQ | Ưu tiên NVIC | Module |
|--------|-------|----------|-----|--------------|--------|
| Base | TIM9 | 120 kHz | 24 | 2 | stepgen ×5, qei ×1 |
| Servo | TIM10 | 1 kHz | 25 | 3 | digitalpin DO ×10, digitalpin DI ×11, pwm ×1, resetpin ×1, estop ×1 |
| Comms | TIM11 | 57.6 kHz | 26 | 4 | **(trống — chưa đăng ký module)** |
| QEI | TIM2 | poll | 28 | mặc định | (chỉ clear UIF, QEI poll trong Base) |

Mỗi thread chạy `module_run()` cho tất cả module đã đăng ký, rồi `module_run_post()` cho các module có `updatePost`.

---

## 5. State machine (main.c)

```
ST_SETUP → ST_START → ST_IDLE → ST_RUNNING
              ↓           ↓           ↓
           (nếu reset) (nếu reset) (nếu reset → ST_WDRESET)
                                          ↓
                                      ST_WDRESET (while(1), force watchdog reset)

ST_STOP → ST_IDLE (tự động quay lại IDLE)
```

- **ST_SETUP:** Khởi tạo SPI, timer, load module, đọc config JSON.
- **ST_START:** Chờ threads chạy, kiểm tra `g_pru_reset`.
- **ST_IDLE:** Chờ SPI status (PRU_READ/WRITE) → vào RUNNING; kiểm tra lỗi SPI.
- **ST_RUNNING:** Giám sát SPI status/error, đếm lỗi liên tiếp (`SPI_ERR_MAX=5`).
- **ST_RESET:** Xóa rxBuffer, quay lại IDLE.
- **ST_WDRESET:** `while(1){}` để watchdog reset hệ thống sau 2000 ms.

---

## 6. SPI Flexi Protocol

Giao diện với LinuxCNC `flexi.c` HAL component. Bố trí 64 bytes giống hệt firmware Mbed gốc.

### rxData (Master → Slave, 64 bytes)

| Offset | Trường | Kiểu | Mô tả |
|--------|--------|------|-------|
| 0–3 | `header` | int32_t | PRU_DATA / PRU_READ / PRU_WRITE / PRU_ESTOP |
| 4–23 | `jointFreqCmd[5]` | int32_t | Lệnh tần số step cho 5 joint |
| 24–27 | `setPoint[1]` | float | Set point (PWM spindle) |
| 28 | `jointEnable` | uint8_t | Bật/tắt joint |
| 29–30 | `outputs` | uint16_t | Digital outputs |
| 31 | `spare0` | uint8_t | Dự trữ |
| 32–63 | — | — | Dự trữ (0) |

### txData (Slave → Master, 64 bytes)

| Offset | Trường | Kiểu | Mô tả |
|--------|--------|------|-------|
| 0–3 | `header` | int32_t | PRU_DATA / PRU_ESTOP |
| 4–23 | `jointFeedback[5]` | int32_t | Phản hồi vị step (DDS accumulator >> STEPBIT) |
| 24–27 | `processVariable[1]` | float | Biến quá trình (ví dụ vị trí QEI) |
| 28–29 | `inputs` | uint16_t | Digital inputs |
| 30–63 | — | — | Dự trữ (0) |

---

## 7. Hướng dẫn build

### Yêu cầu

- Toolchain: `arm-none-eabi-gcc` (Arm GNU Toolchain 14.3.1)
- make (GNU Make)

### Lệnh build

```bash
cd Firmware/MakefileFW

# Biên dịch đầy đủ
make

# Xem sử dụng bộ nhớ flash/RAM
make size

# Dọn dẹp artifacts
make clean
```

### Kết quả build (tham chiếu)

- `.text`: 8.564 bytes (FLASH)
- `.bss`: 3.300 bytes (RAM)
- 0 warning, 0 error với `-Wall -Wextra`

### Flash (tùy chọn, yêu cầu ST-Link)

```bash
make flash   # flash qua st-flash @ 0x08010000
```

---

## 8. Các quy tắc lập trình (ràng buộc thiết kế)

1. **Không cấp phát động:** Không dùng `malloc`/`free`. Tất cả mảng module static, kích thước cố định (`MAX_MODULES_PER_THREAD = 32`).
2. **Không Mbed OS:** Chỉ dùng register access + định nghĩa kiểu CMSIS-style. Không include header mbed nào.
3. **`-ffreestanding -nostdlib`:** Cung cấp `_write`, `_exit` nếu cần (nano.specs/nosys.specs).
4. **Bố trí SPI không thay đổi:** `rxData_t`/`txData_t` packed union giữ nguyên layout như Mbed `remora.h` để LinuxCNC HAL component hoạt động không đổi.
5. **ISR ngắn gọn:** Chỉ clear flag, chạy module, trả về. Không in printf trong ISR.
6. **Thanh ghi xác thực:** Địa chỉ thanh ghi theo STM32F446RE Reference Manual (RM0390), không猜测.

---

## 9. GPIO và peripherals

| Chức năng | Pin | AF | Mô tả |
|-----------|-----|----|-------|
| SPI1 NSS | PA_15 | AF5 | NSS (SSM+SSI, EXTI15_10 edge detect) |
| SPI1 SCK | PB_3 | AF5 | Clock SPI |
| SPI1 MISO | PB_4 | AF5 | Slave → Master |
| SPI1 MOSI | PB_5 | AF5 | Master → Slave |
| USART1 TX | PA_9 | AF7 | Debug stdio 115200 baud |
| USART1 RX | PA_10 | AF7 | Debug stdio |
| PWM spindle | PA_8 | GPIO output | Software PWM, Servo thread |
| Reset pin | PC_3 | GPIO input | → g_pru_reset |
| eStop | PA_2 | GPIO input | → PRU_ESTOP header, g_pru_estop |
| QEI CH1 | PA_0 | TIM2_CH1 | Encoder A |
| QEI CH2 | PA_1 | TIM2_CH2 | Encoder B |

---

## 10. Tài liệu liên quan

- `docs/architecture/` — Tài liệu thiết kế kiến trúc (Markdown + Draw.io XML): state machine, module registration, SPI communication, GPIO map, interrupt call tree, implementation status.
- `Firmware/FirmwareSource/Remora-OS5/` — Source gốc Mbed OS (tham chiếu, không sửa).
- `LinuxCNC/Components/Flexi/` — HAL component `flexi.c` chạy trên LinuxCNC (master phía Pi4/PC).

---

## 11. Trạng thái triển khai

- ✅ 24 chức năng đã triển khai hoàn toàn (build sạch, 0 warning)
- ⚠️ 8 chức năng chưa hoàn thành / cần cải thiện (xem `docs/architecture/07-implementation-status.md`)
- Ưu tiên CAO: đăng ký module cho TIM11 Comms thread (`g_comms_modules` hiện trống)

---

*Tài liệu này mô tả firmware bare-metal STM32F446RE cho LinuxCNC Flexi-HAL. Viết bằng tiếng Việt, phục vụ nghiên cứu học thuật.*
