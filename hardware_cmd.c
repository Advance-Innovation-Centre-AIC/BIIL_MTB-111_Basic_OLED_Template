/*
 * hardware_cmd.c
 *
 *  Created on: Apr 27, 2023
 *      Author: Phuphu
 */

#include "cyhal.h"
#include "cybsp.h"
#include "cy8ckit_028_sense.h"
#include "GUI.h"

void lcd_print_top(const char * s){
	GUI_GotoXY(0,0);
	GUI_DispString(s);
}

void lcd_print_bot(const char * s){
	GUI_GotoXY(0,50);
	GUI_DispString(s);
}

void lcd_print_line_n(const char * s,int linenum){ //linenum is 1-6
	int line = (linenum-1)*10;
	GUI_GotoXY(0,line);
	GUI_DispString(s);
}

void led_green_on(void){
	cyhal_gpio_write(CYBSP_LED_RGB_GREEN, CYBSP_LED_STATE_ON);
}

void led_green_off(void){
	cyhal_gpio_write(CYBSP_LED_RGB_GREEN, CYBSP_LED_STATE_OFF);
}

void led_red_on(void){
	cyhal_gpio_write(CYBSP_LED_RGB_RED, CYBSP_LED_STATE_ON);
}

void led_red_off(void){
	cyhal_gpio_write(CYBSP_LED_RGB_RED, CYBSP_LED_STATE_OFF);
}

void led_blue_on(void){
	cyhal_gpio_write(CYBSP_LED_RGB_BLUE, CYBSP_LED_STATE_ON);
}

void led_blue_off(void){
	cyhal_gpio_write(CYBSP_LED_RGB_BLUE, CYBSP_LED_STATE_OFF);
}
