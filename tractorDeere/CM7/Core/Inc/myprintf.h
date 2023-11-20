/*
 * myprintf.h
 *
 *  Created on: Nov 19, 2023
 *      Author: fsp
 */

#ifndef INC_MYPRINTF_H_
#define INC_MYPRINTF_H_
#include <stdio.h>
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
#endif /* INC_MYPRINTF_H_ */
