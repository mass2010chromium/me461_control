# Project 3 submission: I2C communication

Jing-Chen Peng
ME 461 Fall 2021

## Non-C files

# Project Setup: `import.py _ccsproject _project`
- Template project files for easy setup on new computers in arbitrary folders
- `import.py` processes the templates; to copy this project just copy the folder
  anywhere and run `python import.py` to fix the project files based on the templates

# Misc: `cmd.py` `dump.py` `plot.py`
- `cmd.py`: opens windows command prompt (lol)
- `dump.py`: tera term but no need for config. For light debugging (no terminal input available)
- `plot.py`: template code for plotting values printed with `serial_printf`.

## Relevant files

# Main file: `project3_main.c`
- Some global initialization code
- Has `main` and the loop
    - In loop, writes to/reads from BQ32000 and DAN chips
    - Date is set with an interactive prompt
- Sets up interrupts (we only use timers and `serialRXA`)

# I2C comms: `i2c.h i2c.c`
- Setup code mirroring the tutorial given in lab
- Generic I2C read/write functions

# Interrupt handlers: `interrupt_handlers.h`
- Timer0, Timer2, and `serialRXA` interrupts are meaningfully implemented
- Timer0 sets I2C flag, Timer2 sets UARTPrint flag
- `serialRXA` listens for user input, saves it to a buffer, eventually uses it to update date

# Generic patterns: `common.h`
- Helper functions
    - converting to/from bcd
    - converting to/from lsb/msb in buffers
    - Basic string parsing for numbers

# GPIO macros and helpers: `gpio_decl.h`
- Helper functions and macros for condensing GPIO setup calls
- Reduce redundant code and do away with A, B, ... F lettering annoyance

## Unused files

# spi: `spi_devices/*`
- `spi.c/h`: Generic SPI interface
- `manual_spi.c/h`: "manual" spi without callback; reads it into a buffer and you DIY
- `MPU9250.c/h`: SPI interface for IMU with blocking and nonblocking IO modes

# ADC/DAC: `adc_dac.c adc_dac.h`
- Kinda boring files, macros and functions for reducing repeated code in ADC/DAC init

# Linear filtering and sliding window: `filter.c filter.h`
- Utilities/macros for creating and updating abritrary (fixed) size sliding windows
- Also helper function for applying a linear filter

# PWM: `pwm.c pwm.h`
- Utility funcs/macros for setting up PWM and doing motor/servo control
- Abstractions for duty cycle, normalize to 1, with offset for servos, etc
