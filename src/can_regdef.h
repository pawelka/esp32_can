/**
 * @section License
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2017, Thomas Barth, barth-dev.de
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef __DRIVERS_CAN_REGDEF_H_
#define __DRIVERS_CAN_REGDEF_H_

// #include "esp32_can_builtin.h"	//CAN_FIR_t
#include "soc/twai_struct.h"

/** \brief Interrupt status register */
typedef enum  {
	__CAN_IRQ_RX=			BIT(0),					/**< \brief RX Interrupt */
	__CAN_IRQ_TX=			BIT(1),					/**< \brief TX Interrupt */
	__CAN_IRQ_ERR=			BIT(2),					/**< \brief Error Interrupt */
	__CAN_IRQ_DATA_OVERRUN=	BIT(3),					/**< \brief Date Overrun Interrupt */
	__CAN_IRQ_WAKEUP=		BIT(4),					/**< \brief Wakeup Interrupt */
	__CAN_IRQ_ERR_PASSIVE=	BIT(5),					/**< \brief Passive Error Interrupt */
	__CAN_IRQ_ARB_LOST=		BIT(6),					/**< \brief Arbitration lost interrupt */
	__CAN_IRQ_BUS_ERR=		BIT(7),					/**< \brief Bus error Interrupt */
}__CAN_IRQ_t;

#endif /* __DRIVERS_CAN_REGDEF_H_ */
