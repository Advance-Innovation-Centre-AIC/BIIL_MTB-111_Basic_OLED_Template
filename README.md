# OLED screen control

# This is hardware command for easy use

This is a minimal starter application for Infineon MCU devices.
which include

- CY8CKIT-062s2-43012
- CY8CKIT-028-SENSE.
## Requirements
- [ModusToolbox™ software](https://www.infineon.com/modustoolbox) v3.0 or later (tested with v3.0)
- CY8CKIT-062s2-43012
- CY8CKIT-028-SENSE.
- Programming language: C
- Associated parts: All [PSoC™ 6 MCU](https://www.infineon.com/cms/en/product/microcontroller/32-bit-psoc-arm-cortex-microcontroller/psoc-6-32-bit-arm-cortex-m4-mcu) parts, [XMC7000 MCU](https://www.infineon.com/cms/en/product/microcontroller/32-bit-industrial-microcontroller-based-on-arm-cortex-m/), and [AIROC™ CYW20829 Bluetooth® LE SoC](https://www.infineon.com/cms/en/product/promopages/airoc20829)
## Supported toolchains (make variable 'TOOLCHAIN')
- GNU Arm® embedded compiler v10.3.1 (`GCC_ARM`) - Default value of `TOOLCHAIN`
- Arm® compiler v6.16 (`ARM`)
- IAR C/C++ compiler v9.30.1 (`IAR`)
## OLED functions

These functions will print text on OLED on CY8CKIT-028-SENSE following this

**lcd_print_top(const char * s)**

This function will let you print on top-left of the screen

char * s = text you want to print on OLED

**lcd_print_bot(const char * s)**

This function will let you print on bottom-left of the screen

char * s = text you want to print on OLED

**lcd_print_line_n(const char * s,int linenum)**

This function will let you print on selected line of the screen (from 1-6)

char * s = text you want to print on OLED

linenum = selected line

## How to use

Import this application to your modus project and go to file main.c
BSP and device are already configured all you need is to program your function in main() under for;; loop.

For example, the following code is to read the SENXIV sensor and print it on OLED.


     for (;;)
        {
            /* Get the pressure and temperature data and print the results to the UART */
            float pressure, temperature;
            xensiv_dps3xx_read(&pressure_sensor, &pressure, &temperature);
            sprintf(str,"Pressure: %4.2f",pressure);
            oled_print_top(str);
            sprintf(str,"Temp: %2.2f",temperature);
            oled_print_line_n(str,3);
        }

when you finish writing your code go to

- Build Application
- wait for your program to finish building
- go to Launch > [Your app name] debug (KitProg3_Miniprog4)

you should see tou OLED is lit up and show temperature and air pressure on OLED screen.

