
# How to use with stm32f4xx

## Requirements

First make sure that the following files exists:
- startup
    - startup/startup_stm32f405xx.s
    - startup/startup_stm32f407xx.s
    - startup/startup_stm32f446xx.s
- ld scripts
    - STM32F405RGTx_FLASH.ld
    - STM32F407VGTx_FLASH.ld
    - STM32F446RETx_FLASH.ld

This files can be found in the internet on STM32 resources or similar

## Configure Makefile

Open Makefile and set `CHIP` variable:
```
######################################
# target
######################################
# Available values : 
# - STM32F405RG 
# - STM32F446RE
# - STM32F407VG
CHIP = STM32F407VG
CHIP_LOWER_CASE = $(shell echo $(CHIP) | tr A-Z a-z)
# e.g CHIP_LOWER_CASE = stm32f405rg
CHIP_BASE_LOWER_CASE = $(shell echo $(CHIP_LOWER_CASE) | cut -c1-9)
# e.g CHIP_BASE_LOWER_CASE = stm32f405
```

## Test debugging with `gdb`

*Note: Probably, on stm32f407, stm32f446 you will need to comment out the line 113 `MX_CAN1_Init();` in `Src/main.c`.*

```bash
> make flash
> make gdb
```

Set some breakpoints:
```
> b 95
> b 109
> c
> c
```

## Use python tools

```bash
> cd tools
> sudo python3 explore_odrive.py 
```






