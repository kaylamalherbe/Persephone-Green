/** 
 **************************************************************
 * @file include/bme_lib.h
 * @author Kayla Malherbe - 47477157
 * @date 01042025
 * @brief Real time Clock Library header file
 * REFERENCE: 
 ***************************************************************
 * EXTERNAL FUNCTIONS 
 ***************************************************************
 * 
 *************************************************************** 
*/

#ifndef UART_H
#define UART_H

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>

#include <string.h>
#include <stdio.h>

#define MSG_SIZE 64

void print_uart(char *buf);

extern struct k_msgq uart_msgq; // uart reading queue

#endif