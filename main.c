/*******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the Empty Application Example
*              for ModusToolbox.
*
* Related Document: See README.md
*
*
********************************************************************************
* Copyright 2021-2023, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

/*******************************************************************************
* Header Files
*******************************************************************************/
#include "cyhal.h"
#include "cybsp.h"


#include "cy8ckit_028_sense.h"
#include "GUI.h"
#include "hardware_cmd.h"

xensiv_dps3xx_t pressure_sensor;
cyhal_i2c_t i2c;
cyhal_i2c_cfg_t i2c_cfg = {
    .is_slave = false,
    .address = 0,
    .frequencyhal_hz = 400000
};


#define MTB_DPS3XX_I2C_ADDR_DEFAULT 0x77

/*******************************************************************************
* Macros
*******************************************************************************/


/*******************************************************************************
* Global Variables
*******************************************************************************/


/*******************************************************************************
* Function Prototypes
*******************************************************************************/
void floatToStr(uint8_t *out, float x,int decimalPoint);

/*******************************************************************************
* Function Definitions
*******************************************************************************/

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function for CPU. It...
*    1.
*    2.
*
* Parameters:
*  void
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;

    char str[80];

    /* Initialize the device and board peripherals */
    result = cybsp_init();
    CY_ASSERT(result == CY_RSLT_SUCCESS);
    __enable_irq();


    /* Initialize i2c */
    result = cyhal_i2c_init(&i2c, CY8CKIT_028_SENSE_PIN_I2C_SDA, CY8CKIT_028_SENSE_PIN_I2C_SCL, NULL);
    CY_ASSERT(result == CY_RSLT_SUCCESS);
    result = cyhal_i2c_configure(&i2c, &i2c_cfg);
    CY_ASSERT(result == CY_RSLT_SUCCESS);

    /* Initialize pressure sensor */
    result = xensiv_dps3xx_mtb_init_i2c(&pressure_sensor, &i2c, MTB_DPS3XX_I2C_ADDR_DEFAULT);
    CY_ASSERT(result == CY_RSLT_SUCCESS);

    /* Initialize the OLED display */
    result = mtb_ssd1306_init_i2c(&i2c);
    CY_ASSERT(result == CY_RSLT_SUCCESS);

    GUI_Init();

    for (;;)
    {
        /* Get the pressure and temperature data and print the results to the UART */
        float pressure, temperature;

		// Gets the current pressure and temperature values from the sensor
		// HOT_EXE_1
		// 1) Take reference from the "XENSIV DPS3xx Pressure Sensor (sensor-xensiv-dps3xx)"
		// 2) Use the "API Reference" >> "Functions"
		// 3) Use the API to read Pressure and Temperature
		// 4) Hint:
		//		- Use the below variables as the input of your API
		//		- Pressure sensor object >> "pressure_sensor"
		//		- Data >> "pressure, temperature"
       
        xensiv_dps3xx_read(&pressure_sensor, &pressure, &temperature);
        sprintf(str,"Pressure: %4.2f",pressure);
        lcd_print_top(str);
        sprintf(str,"Temp: %2.2f",temperature);
        lcd_print_line_n(str,3);
        led_blue_on();
        cyhal_system_delay_ms(1000);
        led_blue_off();
        led_green_on();
        cyhal_system_delay_ms(1000);
        led_green_off();
        led_red_on();
        cyhal_system_delay_ms(1000);
        led_red_off();
    }
}


/* [] END OF FILE */
