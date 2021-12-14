/*
 * retarget_uart.h
 *
 *  Created on: Dec 14, 2021
 *      Author: feder
 */

#ifndef INC_RETARGET_UART_H_
#define INC_RETARGET_UART_H_

void  RetargetInit(UART_HandleTypeDef *huart);
__attribute__((weak)) int _write(int fd, char *ptr, int len);


#endif /* INC_RETARGET_UART_H_ */
