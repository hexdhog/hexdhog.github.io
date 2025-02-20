---
title: "Bare blinky"
date: 2025-01-19
draft: true
showToc: true
---

I have messed with electronics for quite some time now, pretty much ever since I started programming. Actually, I learned basic C programming by playing in Arduino IDE. So I have a decent understanding of how to write basic programs that run on Arduino-like compatible microcontrollers; but I have a mediocre understanding of what is actually going on when I use the Arduino or, even, the chip's framework. So I want to go down the software stack and understand exactly what really happens under the hood. I think making an LED blink, an extremely basic task, in assembly and without any libraries/frameworks is a decent starting point.

## Setup

I bought a [CH32V003 kit](https://es.aliexpress.com/item/1005005834050641.html) a while back with this goal in mind.

It's a microcontroller based on QuingKe RISC-V2A with 2KB SRAM, 16KB FLASH, PFIC and comes with a bunch of very common peripherals (e.g. I2C, USART, SPI, ADC, etc). The kit also includes a WCH-LinkE, a USB to SWIO bridge, which is used to program the microcontroller and monitor the USART interface.

I am using PlatformIO to manage the project, although that isn't really relevant to this post.

## Plan

The CH32V003 kit's PCB has an LED connected to GPIO D4 which emits light when that pin is low. So we need a way to manipulate the GPIO pin. Then we need a way to time the actions perfomed on the GPIO pin, so that the LED actually blinks and it does so at a constant rate.

The following block diagram depicts the CH32V003 system architecture:

![CH32V003 system architecture block diagram](/img/ch32v003-block-diagram.png)

There are three GPIO ports: GPIOA (PA1-PA2), GPIOC (PC0-PC7) and GPIOD (PD0-PD7) which are completely seperate GPIO controllers. Like most peripherals, they are accessed through the [AHB bus](https://en.wikipedia.org/wiki/Advanced_Microcontroller_Bus_Architecture). Each of these peripheral controllers has a set of registers that control the behaviour of the actual peripheral, and each of these registers is wired to the bus. Additionally, although not specified in the block diagram, the CH32V003 core has a system tick counter which we can use to time the GPIO actions. Like the peripherals on the AHB bus, it has a set of registers that control its behaviour.

This is what's called [Memory Mapped IO](https://en.wikipedia.org/wiki/Memory-mapped_I/O_and_port-mapped_I/O), and it is very common in microcontrollers. The registers from the peripheral controllers are each assigned a unique memory address. Therefore, interfacing with each register translates to a read and/or write operation from/to the bus. The CH32V003 has the following memory map:

![CH32V003 memory map](/img/ch32v003-memory-map.png)

Note that FLASH and SRAM are also peripherals and, while they are connected on different buses, are accesed in the same way as any other peripheral. This means that all components are wired in a way such that when the RISC-V2A core selects an address to read/write to, the correct bus and peripheral controller register is enabled and given access to the bus. So to us, the RISC-V2A core programmers, accessing any peripheral is analogous to reading/writing data to memory.

According to the memory map diagram, the GPIO port D registers are located between addresses `0x40011400` and `0x40011800`, and, though not specified, the system tick timer's registers are located in the Core Private Peripherals section (`0xe000000` to `0xe0100000`).

## System clock setup

Before setting anything else up we should initialize the system clock. After reset, the CH32V003 uses the HSI (High Speed Internal) oscillator at 24MHz as a clock source. The [PLL (Phase Locked Loop)](https://en.wikipedia.org/wiki/Phase-locked_loop), used to multiply the input clock source, is disabled. An HSE (High Speed Extenal) oscillator 4-25MHz can also be used as a clock source which is disabled after reset; up to de user to set it up on every startup.

The following block diagram shows the system clock tree:

![CH32V003 clock tree block diagram](/img/ch32v003-clock-tree.png)

A few important notes about the clock tree that we should care about:
- HSE and HSI can multiplied by 2 through a PLL, so SYSCLK output is multiplexed between HSI, HSE, HSI\*2 and HSE\*2
- HCLK is prescaled (divisible by: 1, 2, ..., 256) from SYSCLK
- HCLK is used for HB peripherals (GPIO)
- HCLK is used for Core System Timer and can be divided by 8 (useful for longer timer delays)

Because HCLK depends on SYSCLK, let's configure SYSCLK first. We'll configure it at 48MHz; the maximum supported frequency. Because we don't have an external crystal (HSE), we'll use the internal oscillator (HSI) which runs at 24MHz and feed it to the PLL to multiply it by 2 (24MHz * 2 = 48MHz). So first of all, we have to enable both HSI and PLL.

Before enabling HSI*2, by selecting PLL as clock source, we should configure everything that depends on SYSCLK so that when we do enable it everything else is ready.

---

Before doing anything else the SYSCLK must be configured, so let's configure it at 48MHz, as it is the maximum supported frequency. The steps needed to do this are:

1. Enable HSI and PLL
2. Turn off prescaler (do not divide AHB clock) and select HSI as PLL source
3. Configure flash to use 1 cycle latency (recommended when 24MHz <= SYSCLK <= 48MHz)
4. Clear RCC interrupt flags
5. Wait until PLL is ready
6. Select PLL as system clock
7. Wait until PLL is used as system clock

All of this can be done through the RCC (Reset and Clock Control) registers, with base address `0x40021000`:

![CH32V003 RCC registers](/img/ch32v003-rcc-registers.png)

and the FLASH registers, with base address `0x40022000`:

![CH32V003 FLASH registers](/img/ch32v003-flash-registers.png)

More specifically, we'll need the following registers:

**R32_RCC_CTLR**
![CH32V003 R32_RCC_CTLR](/img/ch32v003-rcc-ctlr.png)

**R32_RCC_CFGR0**
![CH32V003 R32_RCC_CFGR0](/img/ch32v003-rcc-cfgr0.png)

**R32_RCC_INTR**
![CH32V003 R32_RCC_INTR](/img/ch32v003-rcc-intr.png)

**R32_FLASH_ACTLR**
![CH32V003 R32_FLASH_INTR](/img/ch32v003-flash-actlr.png)

**Note**: the description of each field for all registers is left out for brevity. More information can be found in the [reference manual](https://www.wch-ic.com/downloads/CH32V003RM_PDF.html).

### Enable HSI and PLL
HSI and PLL are enabled through `R32_RCC_CTLR` field `HSION` (bit 0) and field `PLLON` (bit 24). For both fields, writing a 1 will enable the device and writing a 0 will disable it. So let's write some RISC-V assembly code that enables them both:

```riscv
.equ rcc_base, 0x40021000
.equ flash_r_base, 0x40022000
.equ gpio_pd_base, 0x40011400
.equ systck_base, 0xe000f000

.equ led_pin, 4

.globl main
main:
        li a0, rcc_base # a0 -> RCC register base address
        li a1, flash_r_base # a1 -> FLASH register base address
        li a2, gpio_pd_base # a2 -> GPIO port d register base address

        # PLL_ON (bit 0): enable PLL
        # HSI_ON (bit 24): enable HSI
        #     RCC CTLR = 1 << 0 | 1 << 24
        #     RCC CTLR = 0x01000001
        li t0, 0x01000001
        sw t0, 0(a0)
        li a0, rcc_base # a0 -> RCC register base address
        li a1, flash_r_base # a1 -> FLASH register base address
        li a2, gpio_pd_base # a2 -> GPIO port d register base address

        # PLL_ON (bit 0): enable PLL
        # HSI_ON (bit 24): enable HSI
        #     RCC CTLR = 1 << 0 | 1 << 24
        #     RCC CTLR = 0x01000001
        li t0, 0x01000001
        sw t0, 0(a0)
```

**Note:** a few lines of code have been added, like constant definitions, which will be needed later.

### Turn off prescaler and select HSI as PLL source

The prescaler is turned off by writing 0 to `R32_RCC_CFGR0` field `HPRE` (bits 4-7) and HSI is selected as PLL source by writing 0 to field `PLLSRC` (bit 16):

```riscv
        # HPRE = 0: prescaler off; do not divide SYSCLK
        # PLLSRC = 0: HSI (instead of HSE) for PLL input
        #     RCC_CFGR0 = 0 << 4 | 0 << 16
        #     RCC_CFGR0 = 0
        li t0, 0x00000000
        sw t0, 4(a0)
```

### Configure flash to use 1 cycle latency

Flash latency is configured through `R32_FLASH_ACTLR` field `LATENCY` (bits 0-1); writing a 1 will select a 1 cycle latency:

```riscv
        # configure flash to recommended settings for 48MHz clock
        # LATENCY (bits 0-1) = 1
        #     FLASH_ACTLR = 1 << 0
        #     FLASH_ACTLR = 1
        li t0, 0x00000001
        sw t0, 0(a1)
```

### Clear RCC interrupt flags

Clearing the RCC interrupt flags, actually involves all `R32_RCC_INTR` fields, so let's take a closer look at them to better understand how this register works:

| bit | name       | access | description                                        |
|-----|------------|--------|----------------------------------------------------|
| 0   | `LSIRDYF`  | RO     | LSI clock-ready interrupt flag                     |
| 2   | `HSIRDYF`  | RO     | HSI clock-ready interrupt flag                     |
| 3   | `HSERDYF`  | RO     | HSE clock-ready interrupt flag                     |
| 4   | `PLLRDYF`  | RO     | PLL clock-ready lockout interrupt flag             |
| 7   | `CSSF`     | RO     | Clock security system interrupt flag bit           |
| 8   | `LSIRDYIE` | RW     | LSI-ready interrupt enable bit                     |
| 10  | `HSIRDYIE` | RW     | HSI-ready interrupt enable bit                     |
| 11  | `HSERDYIE` | RW     | HSE-ready interrupt enable bit                     |
| 12  | `PLLRDYIE` | RW     | PLL-ready interrupt enable bit                     |
| 16  | `LSIRDYC`  | WO     | Clear the LSI oscillator ready interrupt flag bit  |
| 19  | `HSERDYC`  | WO     | Clear the HSE oscillator ready interrupt flag bit  |
| 18  | `HSIRDYC`  | WO     | Clear the HSI oscillator ready interrupt flag bit  |
| 20  | `PLLRDYC`  | WO     | Clear the PLL-ready interrupt flag bit             |
| 23  | `CSSC`     | WO     | Clear the clock security system interrupt flag bit |

The first 5 table entries are interrupt flags, indicated by the trailing `F`, and are read-only because they are set by hardware. The last 5 table entries are fields used to clear the interrupt flags, indicated by the trailing `C`, and are write-only. Because the interrupt flags are set by hardware, these fields are needed to physically "reset" the corresponding hardware, which will in turn clear the corresponding interrupt flag. Finally, the middle 4 table entries are interrupt enable fields, indicated by the trailing `IE`. When set to 1 an interrupt will be generated when the corresponding interrupt flag is set.

For our use case, we actually only *need* to clear certain interrupt flags in order to know when certain events happend (e.g. we'll need to know when PLL is ready after we have enabled it), but clearing all of the interrupt flags is a good idea when changing the clock tree configuration anyway, so we'll do that. Also, we could write our program in a way that doesn't actively wait for the peripherals to be ready by utilizing interrupts but that would complicate our code, so we'll disable interrupts too:

```riscv
        # CSSC     (bit 23) = 1 -> clear CSSF (clock security system interrupt flag bit)
        # PLLRDYC  (bit 20) = 1 -> clear PLLRDYF (PLL-ready interrupt flag bit)
        # HSERDYC  (bit 19) = 1 -> clear HSERDYF (HSE oscillator ready interrupt flag bit)
        # HSIRDYC  (bit 18) = 1 -> clear HSIRDYF (HSI oscillator ready interrupt flag bit)
        # LSIRDYC  (bit 16) = 1 -> clear LSIRDYF (LSI oscillator ready interrupt flag bit)
        # PLLRDYIE (bit 12) = 0 -> disable PLL-ready interrupt
        # HSERDYIE (bit 11) = 0 -> disable HSE-ready interrupt
        # HSIRDYIE (bit 10) = 0 -> disable HSI-ready interrupt
        # LSIRDYIE (bit  8) = 0 -> disable LSI-ready interrupt
        #     RCC_INTR = 1<<23 | 1<<20 | 1<<19 | 1<<18 | 1<<16 | 0<<12 | 0<<11 | 0<<10 | 0<<8
        #     RCC_INTR = 0b 0000 0000 1001 1101 0000 0000 0000 0000
        #     RCC_INTR = 0x009d0000
        li t0, 0x009d0000
        sw t0, 8(a0)
```

### Wait until PLL is ready

When PLL is ready `RCC_CTLR` field `PLLRDY` (bit 25) will be set to 1. So we could write a loop that iterates until `PLLRDY` is set:

```riscv
        # wait until PLL is ready
        li t1, 0x02000000 # PLL_RDY mask = 1 << 25
.L_pll_rdy_wait:
        lw t0, 0(a0)
        and t0, t0, t1
        beq t0, zero, .L_pll_rdy_wait
```

### Select PLL as system clock

Once PLL is ready we can select it as system clock source, which is done by setting `R32_RCC_CFGR0` field `SW` (bits 0-1) to 2. Because we don't want to modify the rest of the fields we could read the register value, set the first two bits to 0 with a bitwise and mask (which would be `0b11 << 0 = 0x00000003`) and then bitwise or the result with 2:

```riscv
        # RCC_CFGR0 = RCC_CFGR0 & ~(0b11) | 0b10
        # RCC_CFGR0 = RCC_CFGR0 & ~(0x00000003) | 0x00000002
        # RCC_CFGR0 = RCC_CFGR0 & 0xfffffffc | 0x00000002
        lw t0, 4(a0) # t0 = RCC_CFGR0
        and t0, t0, 0xfffffffc # clear clock source selection ~(0x00000003) = 0xfffffffc
        or t0, t0, 0x00000002 # select PLL as clock source 0x00000002
        sw t0, 4(a0)
```

### Wait until PLL is used as system clock

When PLL is selected as clock source `R32_RCC_CFGR0` field `SWS` (bits 2-3) will be set to 2 (the same value we set field `SW` to in the previous step). We could write a loop that iterates until `SWS` is set to 2:

```riscv
        # wait until PLL is used as SYSCLK
        li t1, 0x0000000c # RCC_CFGR0 SWS mask
        li t2, 0x00000008 # RCC_CFGR0 SW PLL
.L_pll_use_wait:
        lw t0, 4(a0)
        and t0, t0, t1
        bne t0, t2, .L_pll_use_wait
```

## GPIO port setup

Before we can set a pin high or low we have to enable the corresponding GPIO port and configure the individual pin as output.

Enabling the GPIO port is done through `R32_RCC_APB2PCENR` field `IOPDEN` (bit 5) which enables (when set to 1) disables (when set to 0) GPIO port D clock.

```riscv
        # setup GPIO pin for led
        # enable GPIO port D clock
        # RCC_AP2PCENR = RCC_AP2PCENR | 1 << 5
        # RCC_AP2PCENR = RCC_AP2PCENR | 0x00000020
        lw t0, 24(a0) # t0 = APB2PCENR
        or t0, t0, 0x00000020 # APB2PCENR | EPB2PCENR_IOPDEN
        sw t0, 24(a0)
```

Configuring the GPIO pin involves the GPIO registers, where each port has a different base address:

![CH32V003 GPIO registers](/img/ch32v003-gpio-registers.png)

**Note**: `R32_GPIOX_CFGLR` address and the next register address, `R32_GPIOX_INDR`, have an 8 byte difference. Since each register occupies 4 bytes that means there's a reserved register between them. On other chips of the CH32 family this space is used for `R32_GPIOX_CFGHR` (Configuration High Register) which controls another 8 pins, doubling the amount of pins for each GPIO port. Even though this register is not present in the CH32V003 it is left blank to maintain register address consistency within the chip family.

More specifically, the `R32_GPIOX_CFGLR` (Configuration Low Register) is used to configure GPIO port D pins 0-7:

![CH32V003 GPIO CFGLR](/img/ch32v003-gpio-cfglr.png)

So, if we want to configure pin 4 we have to write to fields `MODE4` (bits 16-17) and `CNF4` (bits 18-19). To control the LED we want to set `MODE4` to 1, which indicates output at 10MHz maximum speed, and `CNF4` to 0, which indicates push-pull output mode. Because we don't want to overwrite the rest of pin configurations we could first perform a bitwise and with a mask and then bitwise or the result with the new configuration:

```riscv
        # clear current pin config with an and mask (shift count determined by pin number * pin conf bit count -> pin*4)
        # GPIOD_CFGLR = GPIOD_CFGLR & ~(0xf << (4*pin)) | ((0|1) << (4*pin))
        lw t0, 0(a2)
        li t1, ~(0x0f << (4 * led_pin))
        and t0, t0, t1

        li t1, 0x00000001 << (4 * led_pin)
        or t0, t0, t1
        sw t0, 0(a2)
```

## System tick counter as timer

## Blinky
