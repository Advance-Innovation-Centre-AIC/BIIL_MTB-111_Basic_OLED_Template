/*
 * hardware_cmd.h
 *
 *  Created on: Apr 27, 2023
 *      Author: Phuphu
 */

#ifndef HARDWARE_CMD_H_
#define HARDWARE_CMD_H_

#include "cyhal.h"
#include "cybsp.h"

#include "cy8ckit_028_sense.h"
#include "GUI.h"

void lcd_print_top(const char * s);
void lcd_print_bot(const char * s);
void lcd_print_line_n(const char * s,int linenum);

void led_green_on(void);
void led_green_off(void);
void led_red_on(void);
void led_red_off(void);
void led_blue_on(void);
void led_blue_off(void);




#endif /* HARDWARE_CMD_H_ */
