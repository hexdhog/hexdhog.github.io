---
title: "Test"
date: 2025-03-05
showToc: true
math: katex
tags: [ch32v003, asm]
---

```python
def add(a: int, b: int): return a + b
print(add(1, 2))
```

```asm
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