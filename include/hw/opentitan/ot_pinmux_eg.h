/*
 * QEMU OpenTitan EarlGrey PinMux device
 *
 * Copyright (c) 2023-2025 Rivos, Inc.
 *
 * Author(s):
 *  Emmanuel Blot <eblot@rivosinc.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef HW_OPENTITAN_OT_PINMUX_EG_H
#define HW_OPENTITAN_OT_PINMUX_EG_H

#include "hw/opentitan/ot_pinmux.h"

#define TYPE_OT_PINMUX_EG TYPE_OT_PINMUX "-eg"
OBJECT_DECLARE_TYPE(OtPinmuxEgState, OtPinmuxEgClass, OT_PINMUX_EG)

#define OT_PINMUX_EG_MIO_PAD_COUNT          47u
#define OT_PINMUX_EG_INSEL_CONSTANT_ZERO    0u
#define OT_PINMUX_EG_INSEL_CONSTANT_ONE     1u
#define OT_PINMUX_EG_INSEL_MIO_BASE         2u
#define OT_PINMUX_EG_OUTSEL_CONSTANT_ZERO   0u
#define OT_PINMUX_EG_OUTSEL_CONSTANT_ONE    1u
#define OT_PINMUX_EG_OUTSEL_CONSTANT_HIGH_Z 2u
#define OT_PINMUX_EG_OUTSEL_GPIO_BASE       3u
#define OT_PINMUX_EG_OUTSEL_GPIO_END        35u
#define OT_PINMUX_EG_OUTSEL_I2C_FIRST       35u
#define OT_PINMUX_EG_OUTSEL_I2C_LAST        40u
#define OT_PINMUX_EG_OUTSEL_SPI_HOST1_SD0   41u
#define OT_PINMUX_EG_OUTSEL_SPI_HOST1_SD1   42u
#define OT_PINMUX_EG_OUTSEL_PATTGEN_BASE    49u
#define OT_PINMUX_EG_OUTSEL_PATTGEN_LAST    52u
#define OT_PINMUX_EG_OUTSEL_SPI_HOST1_SCK   53u
#define OT_PINMUX_EG_OUTSEL_SPI_HOST1_CSB   54u
#define OT_PINMUX_EG_OUTSEL_PWM_BASE        65u
#define OT_PINMUX_EG_OUTSEL_PWM_END         71u
#define OT_PINMUX_EG_OUTSEL_SYSRST_FIRST    72u
#define OT_PINMUX_EG_OUTSEL_SYSRST_LAST     77u
#define OT_PINMUX_EG_OUTSEL_SYSRST_DIO_BASE 86u

int ot_pinmux_eg_get_dio_sleep_val(unsigned dio_pad);
void ot_pinmux_eg_dio_pad_in(unsigned dio_pad, int level);
int ot_pinmux_eg_mio_to_host_pin(unsigned pad);
int ot_pinmux_eg_host_pin_to_gpio(OtPinmuxEgState *s, unsigned host_pin);
uint32_t ot_pinmux_eg_get_mio_outsel(OtPinmuxEgState *s, unsigned mio_pad);
uint32_t ot_pinmux_eg_get_mio_pad_attr(OtPinmuxEgState *s, unsigned mio_pad);
uint32_t ot_pinmux_eg_get_dio_pad_attr(unsigned dio_pad);
void ot_pinmux_eg_update_sysrst_inputs(OtPinmuxEgState *s);
uint32_t ot_pinmux_eg_get_gpio_outsel(OtPinmuxEgState *s, unsigned gpio_pin);
uint32_t ot_pinmux_eg_get_periph_insel(OtPinmuxEgState *s, unsigned periph_in);
bool ot_pinmux_eg_is_periph_in_zero(unsigned periph_in);
bool ot_pinmux_eg_trigger_mio_wkup(unsigned mio_pad);

#endif /* HW_OPENTITAN_OT_PINMUX_EG_H */
