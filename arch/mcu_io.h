/*
 * Copyright 2011-2012 Ayla Networks, Inc.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are
 * those of the authors and should not be interpreted as representing official
 * policies, either expressed or implied, of Ayla Networks, Inc.
 */
#ifndef __AYLA_MCU_IO_H__
#define __AYLA_MCU_IO_H__

#define CPU_CLK_HZ 72000000
#define SYSTICK_HZ 100
#define BUTTON_ON_MIN_TICKS  (2 * SYSTICK_HZ)   /* button debounce time */
#define BUTTON_DEBOUNCE_TICKS (20 * SYSTICK_HZ / 1000) /* minimum of 10ms */

#define LED0_PIN	8		/* Blue LED */
#define LED1_PIN	15		/* Green LED */
#define LED2_PIN  9   /* Red LED */
#define LED_GPIO	GPIOE

#ifdef DEMO_UREG_LED
#define UREG_LED_PIN	13		/* Red LED */
#endif

#define BUTTON_GPIO	GPIOA
#define BUTTON_PIN	0
#define BUTTON_EXT_LINE	EXTI_Line0
#define BUTTON_IRQ	EXTI0_IRQn
#define BUTTON_PORT_SOURCE GPIO_PortSourceGPIOA
#define BUTTON_PIN_SOURCE GPIO_PinSource0

#define BUTTON1_GPIO	GPIOC
#define BUTTON1_PIN	0

#define READY_N_GPIO	GPIOB
#define READY_N_PIN	10

#define RESET_N_GPIO	GPIOB
#define RESET_N_PIN	0

#define WKUP_GPIO       GPIOB
#define WKUP_PIN        2

#define LINK_N_GPIO	GPIOB
#define LINK_N_PIN	1

struct intr_stats {
	u32 button;
	u32 intr_line;
};

extern struct intr_stats intr_stats;
extern volatile u32 tick;

void mcu_io_init(void);

/*
 * Return mask for bit number.
 */
static inline u32 bit(u32 i)
{
	return 1U << i;
}

#endif /* __AYLA_MCU_IO_H__ */
