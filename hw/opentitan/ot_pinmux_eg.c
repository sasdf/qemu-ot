/*
 * QEMU OpenTitan EarlGrey PinMux device
 *
 * Copyright (c) 2024-2025 Rivos, Inc.
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

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/main-loop.h"
#include "qemu/typedefs.h"
#include "qom/object.h"
#include "hw/core/cpu.h"
#include "hw/opentitan/ot_alert.h"
#include "hw/opentitan/ot_pinmux_eg.h"
#include "hw/opentitan/ot_rstmgr.h"
#include "hw/opentitan/ot_uart.h"
#include "hw/qdev-properties.h"
#include "hw/registerfields.h"
#include "hw/riscv/ibex_common.h"
#include "hw/riscv/ibex_irq.h"
#include "hw/sysbus.h"
#include "system/replay.h"
#include "trace.h"

#define PARAM_N_MIO_PERIPH_IN  OT_PINMUX_EG_PERIPH_IN_COUNT
#define PARAM_N_MIO_PERIPH_OUT OT_PINMUX_EG_PERIPH_OUT_COUNT
#define PARAM_N_MIO_PADS       OT_PINMUX_EG_MIO_COUNT
#define PARAM_N_DIO_PADS       OT_PINMUX_EG_DIO_COUNT
#define PARAM_N_PADRING_PADS   49u
#define PARAM_N_WKUP_DETECT    8u
#define PARAM_NUM_ALERTS       1u
#define BIT_PACKED_STATUS_REG  1

#define REG_SIZE(_n_) ((_n_) * sizeof(uint32_t))

#ifdef BIT_PACKED_STATUS_REG
#define MIO_SLEEP_STATUS_COUNT    DIV_ROUND_UP(PARAM_N_MIO_PADS, 32u)
#define DIO_SLEEP_STATUS_COUNT    DIV_ROUND_UP(PARAM_N_DIO_PADS, 32u)
#define MIO_PAD_SLEEP_STATUS_REM  (PARAM_N_MIO_PADS & 31u)
#define DIO_PAD_SLEEP_STATUS_REM  (PARAM_N_DIO_PADS & 31u)
#define DIO_PAD_SLEEP_STATUS_MASK UINT32_MAX
#define MIO_PAD_SLEEP_STATUS_MASK UINT32_MAX
#else
#define MIO_SLEEP_STATUS_COUNT    PARAM_N_MIO_PADS
#define DIO_SLEEP_STATUS_COUNT    PARAM_N_DIO_PADS
#define MIO_PAD_SLEEP_STATUS_REM  0
#define DIO_PAD_SLEEP_STATUS_REM  0
#define DIO_PAD_SLEEP_STATUS_MASK 1u
#define MIO_PAD_SLEEP_STATUS_MASK 1u
#endif
#define N_MAX_PADS MAX(PARAM_N_MIO_PADS, PARAM_N_DIO_PADS)

/* clang-format off */
REG32(ALERT_TEST, 0x0u)
    FIELD(ALERT_TEST, FATAL_FAULT, 0u, 1u)
REG32(MIO_PERIPH_INSEL_REGWEN, A_ALERT_TEST + REG_SIZE(1u))
    FIELD(MIO_PERIPH_INSEL_REGWEN, EN, 0u, 1u)
REG32(MIO_PERIPH_INSEL,
      A_MIO_PERIPH_INSEL_REGWEN + REG_SIZE(PARAM_N_MIO_PERIPH_IN))
REG32(MIO_OUTSEL_REGWEN,
      A_MIO_PERIPH_INSEL + REG_SIZE(PARAM_N_MIO_PERIPH_IN))
    FIELD(MIO_OUTSEL_REGWEN, EN, 0u, 1u)
REG32(MIO_OUTSEL,
      A_MIO_OUTSEL_REGWEN + REG_SIZE(PARAM_N_MIO_PADS))
REG32(MIO_PAD_ATTR_REGWEN,
      A_MIO_OUTSEL + REG_SIZE(PARAM_N_MIO_PADS))
    FIELD(MIO_PAD_ATTR_REGWEN, EN, 0u, 1u)
REG32(MIO_PAD_ATTR,
      A_MIO_PAD_ATTR_REGWEN + REG_SIZE(PARAM_N_MIO_PADS))
REG32(DIO_PAD_ATTR_REGWEN,
      A_MIO_PAD_ATTR + REG_SIZE(PARAM_N_MIO_PADS))
    FIELD(DIO_PAD_ATTR_REGWEN, EN, 0u, 1u)
REG32(DIO_PAD_ATTR,
      A_DIO_PAD_ATTR_REGWEN + REG_SIZE(PARAM_N_DIO_PADS))
REG32(MIO_PAD_SLEEP_STATUS,
      A_DIO_PAD_ATTR + REG_SIZE(PARAM_N_DIO_PADS))
REG32(MIO_PAD_SLEEP_REGWEN,
      A_MIO_PAD_SLEEP_STATUS + REG_SIZE(MIO_SLEEP_STATUS_COUNT))
    FIELD(MIO_PAD_SLEEP_REGWEN, EN, 0u, 1u)
REG32(MIO_PAD_SLEEP,
      A_MIO_PAD_SLEEP_REGWEN + REG_SIZE(PARAM_N_MIO_PADS))
    FIELD(MIO_PAD_SLEEP, EN, 0u, 1u)
REG32(MIO_PAD_SLEEP_MODE,
      A_MIO_PAD_SLEEP + REG_SIZE(PARAM_N_MIO_PADS))
    FIELD(MIO_PAD_SLEEP_MODE, OUT, 0u, 2u)
REG32(DIO_PAD_SLEEP_STATUS,
      A_MIO_PAD_SLEEP_MODE + REG_SIZE(PARAM_N_MIO_PADS))
REG32(DIO_PAD_SLEEP_REGWEN,
      A_DIO_PAD_SLEEP_STATUS + REG_SIZE(DIO_SLEEP_STATUS_COUNT))
    FIELD(DIO_PAD_SLEEP_REGWEN, EN, 0u, 1u)
REG32(DIO_PAD_SLEEP,
      A_DIO_PAD_SLEEP_REGWEN + REG_SIZE(PARAM_N_DIO_PADS))
    FIELD(DIO_PAD_SLEEP, EN, 0u, 1u)
REG32(DIO_PAD_SLEEP_MODE,
      A_DIO_PAD_SLEEP + REG_SIZE(PARAM_N_DIO_PADS))
    FIELD(DIO_PAD_SLEEP_MODE, OUT, 0u, 2u)
REG32(WKUP_DETECTOR_REGWEN,
     A_DIO_PAD_SLEEP_MODE + REG_SIZE(PARAM_N_DIO_PADS))
    FIELD(WKUP_DETECTOR_REGWEN, EN, 0u, 1u)
REG32(WKUP_DETECTOR,
      A_WKUP_DETECTOR_REGWEN + REG_SIZE(PARAM_N_WKUP_DETECT))
    FIELD(WKUP_DETECTOR, EN, 0u, 1u)
REG32(WKUP_DETECTOR_CFG,
      A_WKUP_DETECTOR + REG_SIZE(PARAM_N_WKUP_DETECT))
    FIELD(WKUP_DETECTOR_CFG, MODE, 0u, 3u)
    FIELD(WKUP_DETECTOR_CFG, FILTER, 3u, 1u)
    FIELD(WKUP_DETECTOR_CFG, MIODIO, 4u, 1u)
REG32(WKUP_DETECTOR_CNT_TH,
      A_WKUP_DETECTOR_CFG + REG_SIZE(PARAM_N_WKUP_DETECT))
    FIELD(WKUP_DETECTOR_CNT_TH, TH, 0u, 8u)
REG32(WKUP_DETECTOR_PADSEL,
      A_WKUP_DETECTOR_CNT_TH + REG_SIZE(PARAM_N_WKUP_DETECT))
    FIELD(WKUP_DETECTOR_PADSEL, SEL, 0u, 6u)
REG32(WKUP_CAUSE,
      A_WKUP_DETECTOR_PADSEL + REG_SIZE(PARAM_N_WKUP_DETECT))
/* clang-format on */

#define MIO_PAD_ATTR_MASK               0x0010008fu
#define DIO_PAD_ATTR_MASK               0x0010008fu
#define DIO_INPUT_STD_PAD_ATTR_MASK     0x0000008du
#define MIO_PAD_SLEEP_MODE_OUT_TIE_LOW  0x0u
#define MIO_PAD_SLEEP_MODE_OUT_TIE_HIGH 0x1u
#define MIO_PAD_SLEEP_MODE_OUT_HIGH_Z   0x2u
#define MIO_PAD_SLEEP_MODE_OUT_KEEP     0x3u
#define DIO_PAD_SLEEP_MODE_OUT_TIE_LOW  0x0u
#define DIO_PAD_SLEEP_MODE_OUT_TIE_HIGH 0x1u
#define DIO_PAD_SLEEP_MODE_OUT_HIGH_Z   0x2u
#define DIO_PAD_SLEEP_MODE_OUT_KEEP     0x3u
#define WKUP_DETECTOR_MODE_POSEDGE      0x0u
#define WKUP_DETECTOR_MODE_NEGEDGE      0x1u
#define WKUP_DETECTOR_MODE_EDGE         0x2u
#define WKUP_DETECTOR_MODE_TIMEDHIGH    0x3u
#define WKUP_DETECTOR_MODE_TIMEDLOW     0x4u
#define WKUP_CAUSE_MASK                 ((1u << PARAM_N_WKUP_DETECT) - 1u)
#define WKUP_DETECTOR_CFG_MASK \
    (R_WKUP_DETECTOR_CFG_MODE_MASK | R_WKUP_DETECTOR_CFG_FILTER_MASK | \
     R_WKUP_DETECTOR_CFG_MIODIO_MASK)

#define R32_OFF(_r_)             ((_r_) / sizeof(uint32_t))
#define R_LAST_REG               (R_WKUP_CAUSE)
#define REGS_COUNT               (R_LAST_REG + 1u)
#define REGS_SIZE                0x1000u
#define CASE_SCALAR(_reg_)       R_##_reg_
#define CASE_RANGE(_reg_, _rpt_) R_##_reg_...(R_##_reg_ + (_rpt_) - (1u))
#define PAD_ATTR_TO_IRQ(_pad_)   ((int)((_pad_) & INT32_MAX))
#define PAD_ATTR_ENABLE(_en_)    (((unsigned)!(_en_)) << 31u)

static_assert((OT_PINMUX_PAD_ATTR_MASK | OT_PINMUX_PAD_ATTR_FORCE_MODE_MASK) <
                  (1u << 31u),
              "Cannot encode PAD attr as IRQ");

/*
 * Declarative Padring-to-Pinmux map (0..48):
 *   0..42  -> MIO[0..42]  (IOA0..IOA8, IOB0..IOB12, IOC0..IOC12, IOR0..IOR7)
 *   43     -> DIO[10]     (IOR8 / SYSRST_CTRL_AON_EC_RST_L)
 *   44     -> DIO[11]     (IOR9 / SYSRST_CTRL_AON_FLASH_WP_L)
 *   45..48 -> MIO[43..46] (IOR10..IOR13)
 */
typedef struct {
    bool is_dio;
    uint8_t idx;
} OtPinmuxEgPadringMap;

static const OtPinmuxEgPadringMap
    OT_PINMUX_EG_PADRING_MAP[PARAM_N_PADRING_PADS] = {
        [0] = { false, OT_PINMUX_EG_MIO_IOA0 },
        [1] = { false, OT_PINMUX_EG_MIO_IOA1 },
        [2] = { false, OT_PINMUX_EG_MIO_IOA2 },
        [3] = { false, OT_PINMUX_EG_MIO_IOA3 },
        [4] = { false, OT_PINMUX_EG_MIO_IOA4 },
        [5] = { false, OT_PINMUX_EG_MIO_IOA5 },
        [6] = { false, OT_PINMUX_EG_MIO_IOA6 },
        [7] = { false, OT_PINMUX_EG_MIO_IOA7 },
        [8] = { false, OT_PINMUX_EG_MIO_IOA8 },
        [9] = { false, OT_PINMUX_EG_MIO_IOB0 },
        [10] = { false, OT_PINMUX_EG_MIO_IOB1 },
        [11] = { false, OT_PINMUX_EG_MIO_IOB2 },
        [12] = { false, OT_PINMUX_EG_MIO_IOB3 },
        [13] = { false, OT_PINMUX_EG_MIO_IOB4 },
        [14] = { false, OT_PINMUX_EG_MIO_IOB5 },
        [15] = { false, OT_PINMUX_EG_MIO_IOB6 },
        [16] = { false, OT_PINMUX_EG_MIO_IOB7 },
        [17] = { false, OT_PINMUX_EG_MIO_IOB8 },
        [18] = { false, OT_PINMUX_EG_MIO_IOB9 },
        [19] = { false, OT_PINMUX_EG_MIO_IOB10 },
        [20] = { false, OT_PINMUX_EG_MIO_IOB11 },
        [21] = { false, OT_PINMUX_EG_MIO_IOB12 },
        [22] = { false, OT_PINMUX_EG_MIO_IOC0 },
        [23] = { false, OT_PINMUX_EG_MIO_IOC1 },
        [24] = { false, OT_PINMUX_EG_MIO_IOC2 },
        [25] = { false, OT_PINMUX_EG_MIO_IOC3 },
        [26] = { false, OT_PINMUX_EG_MIO_IOC4 },
        [27] = { false, OT_PINMUX_EG_MIO_IOC5 },
        [28] = { false, OT_PINMUX_EG_MIO_IOC6 },
        [29] = { false, OT_PINMUX_EG_MIO_IOC7 },
        [30] = { false, OT_PINMUX_EG_MIO_IOC8 },
        [31] = { false, OT_PINMUX_EG_MIO_IOC9 },
        [32] = { false, OT_PINMUX_EG_MIO_IOC10 },
        [33] = { false, OT_PINMUX_EG_MIO_IOC11 },
        [34] = { false, OT_PINMUX_EG_MIO_IOC12 },
        [35] = { false, OT_PINMUX_EG_MIO_IOR0 },
        [36] = { false, OT_PINMUX_EG_MIO_IOR1 },
        [37] = { false, OT_PINMUX_EG_MIO_IOR2 },
        [38] = { false, OT_PINMUX_EG_MIO_IOR3 },
        [39] = { false, OT_PINMUX_EG_MIO_IOR4 },
        [40] = { false, OT_PINMUX_EG_MIO_IOR5 },
        [41] = { false, OT_PINMUX_EG_MIO_IOR6 },
        [42] = { false, OT_PINMUX_EG_MIO_IOR7 },
        [43] = { true, OT_PINMUX_EG_DIO_SYSRST_CTRL_AON_EC_RST_L },
        [44] = { true, OT_PINMUX_EG_DIO_SYSRST_CTRL_AON_FLASH_WP_L },
        [45] = { false, OT_PINMUX_EG_MIO_IOR10 },
        [46] = { false, OT_PINMUX_EG_MIO_IOR11 },
        [47] = { false, OT_PINMUX_EG_MIO_IOR12 },
        [48] = { false, OT_PINMUX_EG_MIO_IOR13 },
    };

typedef struct {
    const char *name;
    bool active;
    bool open_drain;
    int default_in;
    int default_out;
    int padring_idx;
    uint32_t attr_mask;
} OtPinmuxEgDioDesc;

static const OtPinmuxEgDioDesc OT_PINMUX_EG_DIO_DESC[PARAM_N_DIO_PADS] = {
    [OT_PINMUX_EG_DIO_USBDEV_USB_DP] = {
        "usbdev_usb_dp", false, false, 0, -1, -1, DIO_PAD_ATTR_MASK,
    },
    [OT_PINMUX_EG_DIO_USBDEV_USB_DN] = {
        "usbdev_usb_dn", false, false, 0, -1, -1, DIO_PAD_ATTR_MASK,
    },
    [OT_PINMUX_EG_DIO_SPI_HOST0_SD0] = {
        "spi_host0_sd0", false, false, 0, -1, -1, DIO_PAD_ATTR_MASK,
    },
    [OT_PINMUX_EG_DIO_SPI_HOST0_SD1] = {
        "spi_host0_sd1", false, false, 0, -1, -1, DIO_PAD_ATTR_MASK,
    },
    [OT_PINMUX_EG_DIO_SPI_HOST0_SD2] = {
        "spi_host0_sd2", false, false, 0, -1, -1, DIO_PAD_ATTR_MASK,
    },
    [OT_PINMUX_EG_DIO_SPI_HOST0_SD3] = {
        "spi_host0_sd3", false, false, 0, -1, -1, DIO_PAD_ATTR_MASK,
    },
    [OT_PINMUX_EG_DIO_SPI_DEVICE_SD0] = {
        "spi_device_sd0", true, false, 0, -1, -1, DIO_PAD_ATTR_MASK,
    },
    [OT_PINMUX_EG_DIO_SPI_DEVICE_SD1] = {
        "spi_device_sd1", false, false, 0, -1, -1, DIO_PAD_ATTR_MASK,
    },
    [OT_PINMUX_EG_DIO_SPI_DEVICE_SD2] = {
        "spi_device_sd2", false, false, 0, -1, -1, DIO_PAD_ATTR_MASK,
    },
    [OT_PINMUX_EG_DIO_SPI_DEVICE_SD3] = {
        "spi_device_sd3", false, false, 0, -1, -1, DIO_PAD_ATTR_MASK,
    },
    [OT_PINMUX_EG_DIO_SYSRST_CTRL_AON_EC_RST_L] = {
        "sysrst_ctrl_aon_ec_rst_l", true, true, 1, 1, 43, DIO_PAD_ATTR_MASK,
    },
    [OT_PINMUX_EG_DIO_SYSRST_CTRL_AON_FLASH_WP_L] = {
        "sysrst_ctrl_aon_flash_wp_l", true, true, 1, 0, 44, DIO_PAD_ATTR_MASK,
    },
    [OT_PINMUX_EG_DIO_SPI_DEVICE_SCK] = {
        "spi_device_sck", false, false, 0, -1, -1, DIO_INPUT_STD_PAD_ATTR_MASK,
    },
    [OT_PINMUX_EG_DIO_SPI_DEVICE_CSB] = {
        "spi_device_csb", true, false, 1, -1, -1, DIO_INPUT_STD_PAD_ATTR_MASK,
    },
    [OT_PINMUX_EG_DIO_SPI_HOST0_SCK] = {
        "spi_host0_sck", false, false, 0, -1, -1, DIO_PAD_ATTR_MASK,
    },
    [OT_PINMUX_EG_DIO_SPI_HOST0_CSB] = {
        "spi_host0_csb", false, false, 1, -1, -1, DIO_PAD_ATTR_MASK,
    },
};

typedef struct {
    const char *name;
    bool active;
} OtPinmuxEgPeriphInDesc;

#define PERIPH_IN_ENTRY(_id_, _active_) \
    [OT_PINMUX_EG_PERIPH_IN_##_id_] = { #_id_, (_active_) }

static const OtPinmuxEgPeriphInDesc
    OT_PINMUX_EG_PERIPH_IN_DESC[PARAM_N_MIO_PERIPH_IN] = {
        PERIPH_IN_ENTRY(GPIO_GPIO0, true),
        PERIPH_IN_ENTRY(GPIO_GPIO1, true),
        PERIPH_IN_ENTRY(GPIO_GPIO2, true),
        PERIPH_IN_ENTRY(GPIO_GPIO3, true),
        PERIPH_IN_ENTRY(GPIO_GPIO4, true),
        PERIPH_IN_ENTRY(GPIO_GPIO5, true),
        PERIPH_IN_ENTRY(GPIO_GPIO6, true),
        PERIPH_IN_ENTRY(GPIO_GPIO7, true),
        PERIPH_IN_ENTRY(GPIO_GPIO8, true),
        PERIPH_IN_ENTRY(GPIO_GPIO9, true),
        PERIPH_IN_ENTRY(GPIO_GPIO10, true),
        PERIPH_IN_ENTRY(GPIO_GPIO11, true),
        PERIPH_IN_ENTRY(GPIO_GPIO12, true),
        PERIPH_IN_ENTRY(GPIO_GPIO13, true),
        PERIPH_IN_ENTRY(GPIO_GPIO14, true),
        PERIPH_IN_ENTRY(GPIO_GPIO15, true),
        PERIPH_IN_ENTRY(GPIO_GPIO16, true),
        PERIPH_IN_ENTRY(GPIO_GPIO17, true),
        PERIPH_IN_ENTRY(GPIO_GPIO18, true),
        PERIPH_IN_ENTRY(GPIO_GPIO19, true),
        PERIPH_IN_ENTRY(GPIO_GPIO20, true),
        PERIPH_IN_ENTRY(GPIO_GPIO21, true),
        PERIPH_IN_ENTRY(GPIO_GPIO22, true),
        PERIPH_IN_ENTRY(GPIO_GPIO23, true),
        PERIPH_IN_ENTRY(GPIO_GPIO24, true),
        PERIPH_IN_ENTRY(GPIO_GPIO25, true),
        PERIPH_IN_ENTRY(GPIO_GPIO26, true),
        PERIPH_IN_ENTRY(GPIO_GPIO27, true),
        PERIPH_IN_ENTRY(GPIO_GPIO28, true),
        PERIPH_IN_ENTRY(GPIO_GPIO29, true),
        PERIPH_IN_ENTRY(GPIO_GPIO30, true),
        PERIPH_IN_ENTRY(GPIO_GPIO31, true),
        /* Unimplemented bitbang I2C / SPI_HOST1 MIO inputs */
        PERIPH_IN_ENTRY(I2C0_SDA, false),
        PERIPH_IN_ENTRY(I2C0_SCL, false),
        PERIPH_IN_ENTRY(I2C1_SDA, false),
        PERIPH_IN_ENTRY(I2C1_SCL, false),
        PERIPH_IN_ENTRY(I2C2_SDA, false),
        PERIPH_IN_ENTRY(I2C2_SCL, false),
        PERIPH_IN_ENTRY(SPI_HOST1_SD0, false),
        PERIPH_IN_ENTRY(SPI_HOST1_SD1, false),
        PERIPH_IN_ENTRY(SPI_HOST1_SD2, false),
        PERIPH_IN_ENTRY(SPI_HOST1_SD3, false),
        /* UART RX inputs are routed via ot_uart_update_pinmux */
        PERIPH_IN_ENTRY(UART0_RX, false),
        PERIPH_IN_ENTRY(UART1_RX, false),
        PERIPH_IN_ENTRY(UART2_RX, false),
        PERIPH_IN_ENTRY(UART3_RX, false),
        /* SPI Device TPM CSB input */
        PERIPH_IN_ENTRY(SPI_DEVICE_TPM_CSB, true),
        /* Unimplemented Flash Ctrl JTAG inputs */
        PERIPH_IN_ENTRY(FLASH_CTRL_TCK, false),
        PERIPH_IN_ENTRY(FLASH_CTRL_TMS, false),
        PERIPH_IN_ENTRY(FLASH_CTRL_TDI, false),
        /* Sysrst Ctrl AON MIO inputs */
        PERIPH_IN_ENTRY(SYSRST_CTRL_AON_AC_PRESENT, true),
        PERIPH_IN_ENTRY(SYSRST_CTRL_AON_KEY0_IN, true),
        PERIPH_IN_ENTRY(SYSRST_CTRL_AON_KEY1_IN, true),
        PERIPH_IN_ENTRY(SYSRST_CTRL_AON_KEY2_IN, true),
        PERIPH_IN_ENTRY(SYSRST_CTRL_AON_PWRB_IN, true),
        PERIPH_IN_ENTRY(SYSRST_CTRL_AON_LID_OPEN, true),
        /* USBDEV VBUS Sense input */
        PERIPH_IN_ENTRY(USBDEV_SENSE, true),
    };

typedef struct {
    const char *name;
    bool active;
    int default_val;
} OtPinmuxEgPeriphOutDesc;

#define PERIPH_OUT_ENTRY(_id_, _active_, _def_) \
    [OT_PINMUX_EG_PERIPH_OUT_##_id_] = { #_id_, (_active_), (_def_) }

static const OtPinmuxEgPeriphOutDesc
    OT_PINMUX_EG_PERIPH_OUT_DESC[PARAM_N_MIO_PERIPH_OUT] = {
        PERIPH_OUT_ENTRY(GPIO_GPIO0, true, -1),
        PERIPH_OUT_ENTRY(GPIO_GPIO1, true, -1),
        PERIPH_OUT_ENTRY(GPIO_GPIO2, true, -1),
        PERIPH_OUT_ENTRY(GPIO_GPIO3, true, -1),
        PERIPH_OUT_ENTRY(GPIO_GPIO4, true, -1),
        PERIPH_OUT_ENTRY(GPIO_GPIO5, true, -1),
        PERIPH_OUT_ENTRY(GPIO_GPIO6, true, -1),
        PERIPH_OUT_ENTRY(GPIO_GPIO7, true, -1),
        PERIPH_OUT_ENTRY(GPIO_GPIO8, true, -1),
        PERIPH_OUT_ENTRY(GPIO_GPIO9, true, -1),
        PERIPH_OUT_ENTRY(GPIO_GPIO10, true, -1),
        PERIPH_OUT_ENTRY(GPIO_GPIO11, true, -1),
        PERIPH_OUT_ENTRY(GPIO_GPIO12, true, -1),
        PERIPH_OUT_ENTRY(GPIO_GPIO13, true, -1),
        PERIPH_OUT_ENTRY(GPIO_GPIO14, true, -1),
        PERIPH_OUT_ENTRY(GPIO_GPIO15, true, -1),
        PERIPH_OUT_ENTRY(GPIO_GPIO16, true, -1),
        PERIPH_OUT_ENTRY(GPIO_GPIO17, true, -1),
        PERIPH_OUT_ENTRY(GPIO_GPIO18, true, -1),
        PERIPH_OUT_ENTRY(GPIO_GPIO19, true, -1),
        PERIPH_OUT_ENTRY(GPIO_GPIO20, true, -1),
        PERIPH_OUT_ENTRY(GPIO_GPIO21, true, -1),
        PERIPH_OUT_ENTRY(GPIO_GPIO22, true, -1),
        PERIPH_OUT_ENTRY(GPIO_GPIO23, true, -1),
        PERIPH_OUT_ENTRY(GPIO_GPIO24, true, -1),
        PERIPH_OUT_ENTRY(GPIO_GPIO25, true, -1),
        PERIPH_OUT_ENTRY(GPIO_GPIO26, true, -1),
        PERIPH_OUT_ENTRY(GPIO_GPIO27, true, -1),
        PERIPH_OUT_ENTRY(GPIO_GPIO28, true, -1),
        PERIPH_OUT_ENTRY(GPIO_GPIO29, true, -1),
        PERIPH_OUT_ENTRY(GPIO_GPIO30, true, -1),
        PERIPH_OUT_ENTRY(GPIO_GPIO31, true, -1),
        /* Unimplemented bitbang I2C / SPI_HOST1 / UART TX MIO outputs */
        PERIPH_OUT_ENTRY(I2C0_SDA, false, -1),
        PERIPH_OUT_ENTRY(I2C0_SCL, false, -1),
        PERIPH_OUT_ENTRY(I2C1_SDA, false, -1),
        PERIPH_OUT_ENTRY(I2C1_SCL, false, -1),
        PERIPH_OUT_ENTRY(I2C2_SDA, false, -1),
        PERIPH_OUT_ENTRY(I2C2_SCL, false, -1),
        PERIPH_OUT_ENTRY(SPI_HOST1_SD0, false, -1),
        PERIPH_OUT_ENTRY(SPI_HOST1_SD1, false, -1),
        PERIPH_OUT_ENTRY(SPI_HOST1_SD2, false, -1),
        PERIPH_OUT_ENTRY(SPI_HOST1_SD3, false, -1),
        PERIPH_OUT_ENTRY(UART0_TX, false, -1),
        PERIPH_OUT_ENTRY(UART1_TX, false, -1),
        PERIPH_OUT_ENTRY(UART2_TX, false, -1),
        PERIPH_OUT_ENTRY(UART3_TX, false, -1),
        /* Pattgen outputs */
        PERIPH_OUT_ENTRY(PATTGEN_PDA0_TX, true, 0),
        PERIPH_OUT_ENTRY(PATTGEN_PCL0_TX, true, 0),
        PERIPH_OUT_ENTRY(PATTGEN_PDA1_TX, true, 0),
        PERIPH_OUT_ENTRY(PATTGEN_PCL1_TX, true, 0),
        /* Unimplemented SPI_HOST1 / Flash JTAG / Sensor AST debug outputs */
        PERIPH_OUT_ENTRY(SPI_HOST1_SCK, false, -1),
        PERIPH_OUT_ENTRY(SPI_HOST1_CSB, false, -1),
        PERIPH_OUT_ENTRY(FLASH_CTRL_TDO, false, -1),
        PERIPH_OUT_ENTRY(SENSOR_CTRL_AON_AST_DEBUG_OUT0, false, -1),
        PERIPH_OUT_ENTRY(SENSOR_CTRL_AON_AST_DEBUG_OUT1, false, -1),
        PERIPH_OUT_ENTRY(SENSOR_CTRL_AON_AST_DEBUG_OUT2, false, -1),
        PERIPH_OUT_ENTRY(SENSOR_CTRL_AON_AST_DEBUG_OUT3, false, -1),
        PERIPH_OUT_ENTRY(SENSOR_CTRL_AON_AST_DEBUG_OUT4, false, -1),
        PERIPH_OUT_ENTRY(SENSOR_CTRL_AON_AST_DEBUG_OUT5, false, -1),
        PERIPH_OUT_ENTRY(SENSOR_CTRL_AON_AST_DEBUG_OUT6, false, -1),
        PERIPH_OUT_ENTRY(SENSOR_CTRL_AON_AST_DEBUG_OUT7, false, -1),
        PERIPH_OUT_ENTRY(SENSOR_CTRL_AON_AST_DEBUG_OUT8, false, -1),
        /* PWM AON outputs */
        PERIPH_OUT_ENTRY(PWM_AON_PWM0, true, 0),
        PERIPH_OUT_ENTRY(PWM_AON_PWM1, true, 0),
        PERIPH_OUT_ENTRY(PWM_AON_PWM2, true, 0),
        PERIPH_OUT_ENTRY(PWM_AON_PWM3, true, 0),
        PERIPH_OUT_ENTRY(PWM_AON_PWM4, true, 0),
        PERIPH_OUT_ENTRY(PWM_AON_PWM5, true, 0),
        /* Unimplemented OTP test output */
        PERIPH_OUT_ENTRY(OTP_CTRL_TEST0, false, -1),
        /* Sysrst Ctrl AON MIO outputs */
        PERIPH_OUT_ENTRY(SYSRST_CTRL_AON_BAT_DISABLE, true, 0),
        PERIPH_OUT_ENTRY(SYSRST_CTRL_AON_KEY0_OUT, true, 0),
        PERIPH_OUT_ENTRY(SYSRST_CTRL_AON_KEY1_OUT, true, 0),
        PERIPH_OUT_ENTRY(SYSRST_CTRL_AON_KEY2_OUT, true, 0),
        PERIPH_OUT_ENTRY(SYSRST_CTRL_AON_PWRB_OUT, true, 0),
        PERIPH_OUT_ENTRY(SYSRST_CTRL_AON_Z3_WAKEUP, true, 0),
    };

typedef struct {
    uint32_t alert_test;
    uint32_t mio_periph_insel_regwen[PARAM_N_MIO_PERIPH_IN];
    uint32_t mio_periph_insel[PARAM_N_MIO_PERIPH_IN];
    uint32_t mio_outsel_regwen[PARAM_N_MIO_PADS];
    uint32_t mio_outsel[PARAM_N_MIO_PADS];
    uint32_t mio_pad_attr_regwen[PARAM_N_MIO_PADS];
    uint32_t mio_pad_attr[PARAM_N_MIO_PADS];
    uint32_t dio_pad_attr_regwen[PARAM_N_DIO_PADS];
    uint32_t dio_pad_attr[PARAM_N_DIO_PADS];
    uint32_t mio_pad_sleep_status[MIO_SLEEP_STATUS_COUNT];
    uint32_t mio_pad_sleep_regwen[PARAM_N_MIO_PADS];
    uint32_t mio_pad_sleep[PARAM_N_MIO_PADS];
    uint32_t mio_pad_sleep_mode[PARAM_N_MIO_PADS];
    uint32_t dio_pad_sleep_status[DIO_SLEEP_STATUS_COUNT];
    uint32_t dio_pad_sleep_regwen[PARAM_N_DIO_PADS];
    uint32_t dio_pad_sleep[PARAM_N_DIO_PADS];
    uint32_t dio_pad_sleep_mode[PARAM_N_DIO_PADS];
    uint32_t wkup_detector_regwen[PARAM_N_WKUP_DETECT];
    uint32_t wkup_detector[PARAM_N_WKUP_DETECT];
    uint32_t wkup_detector_cfg[PARAM_N_WKUP_DETECT];
    uint32_t wkup_detector_cnt_th[PARAM_N_WKUP_DETECT];
    uint32_t wkup_detector_padsel[PARAM_N_WKUP_DETECT];
    uint32_t wkup_cause;
} OtPinmuxEgStateRegs;

struct OtPinmuxEgState {
    SysBusDevice parent_obj;

    MemoryRegion mmio;
    IbexIRQ alert;
    IbexIRQ wkup;
    IbexIRQ *dios;
    IbexIRQ *mios;
    IbexIRQ *periph_in;
    IbexIRQ *dio_periph_in;
    IbexIRQ *pad_out;
    IbexIRQ *pad_attr;
    int periph_out_level[PARAM_N_MIO_PERIPH_OUT];
    int dio_periph_out_level[PARAM_N_DIO_PADS];
    int pad_last_level[PARAM_N_PADRING_PADS];
    uint64_t mio_in_level;
    uint32_t dio_in_level;
    uint8_t mio_latched_sleep_mode[PARAM_N_MIO_PADS];
    uint8_t dio_latched_sleep_mode[PARAM_N_DIO_PADS];
    uint8_t wkup_prev_level[PARAM_N_WKUP_DETECT];

    OtPinmuxEgStateRegs *regs;
};

struct OtPinmuxEgClass {
    SysBusDeviceClass parent_class;
    ResettablePhases parent_phases;
};

static OtPinmuxEgState *ot_pinmux_eg_instance;

static uint32_t ot_pinmux_eg_encode_pad_attr(uint32_t raw_attr,
                                             bool sleep_active,
                                             uint8_t sleep_mode)
{
    uint32_t cfg = PAD_ATTR_TO_IRQ(raw_attr);
    if (sleep_active) {
        uint32_t force_mode;
        switch (sleep_mode) {
        case 0u: /* Drive Low */
            force_mode = OT_PINMUX_PAD_FORCE_LOW;
            break;
        case 1u: /* Drive High */
            force_mode = OT_PINMUX_PAD_FORCE_HIGH;
            break;
        case 2u: /* High-Z */
            force_mode = OT_PINMUX_PAD_FORCE_HIZ;
            break;
        case 3u: /* Keep */
        default:
            force_mode = OT_PINMUX_PAD_FORCE_IGNORE;
            break;
        }
        cfg |= SHARED_FIELD_DP32(0, OT_PINMUX_PAD_ATTR_FORCE_MODE, force_mode);
    }
    return cfg;
}

static void ot_pinmux_eg_eval_pad_out(OtPinmuxEgState *s, unsigned pad_idx)
{
    g_assert(pad_idx < PARAM_N_PADRING_PADS);
    const OtPinmuxEgPadringMap *map = &OT_PINMUX_EG_PADRING_MAP[pad_idx];
    int level = -1;
    uint32_t attr = 0;

    if (!map->is_dio) {
        unsigned mio_pad = map->idx;
        attr = s->regs->mio_pad_attr[mio_pad];
        bool sleep_active =
            (s->regs->mio_pad_sleep_status[mio_pad / 32u] >> (mio_pad % 32u)) &
            1u;
        if (sleep_active) {
            switch (s->mio_latched_sleep_mode[mio_pad]) {
            case 0u:
                level = 0;
                break;
            case 1u:
                level = 1;
                break;
            case 2u:
                level = -1;
                break;
            case 3u:
            default:
                level = s->pad_last_level[pad_idx];
                break;
            }
        } else {
            uint32_t outsel = s->regs->mio_outsel[mio_pad];
            if (outsel == 0u) {
                level = 0;
            } else if (outsel == 1u) {
                level = 1;
            } else if (outsel == 2u) {
                level = -1;
            } else if (outsel - 3u < PARAM_N_MIO_PERIPH_OUT) {
                unsigned periph_out = outsel - 3u;
                level = OT_PINMUX_EG_PERIPH_OUT_DESC[periph_out].active ?
                            s->periph_out_level[periph_out] :
                            OT_PINMUX_EG_PERIPH_OUT_DESC[periph_out].default_val;
            } else {
                level = -1;
            }
        }
    } else {
        unsigned dio_pad = map->idx;
        const OtPinmuxEgDioDesc *desc = &OT_PINMUX_EG_DIO_DESC[dio_pad];
        attr = s->regs->dio_pad_attr[dio_pad];
        bool sleep_active =
            (s->regs->dio_pad_sleep_status[dio_pad / 32u] >> (dio_pad % 32u)) &
            1u;
        if (sleep_active) {
            switch (s->dio_latched_sleep_mode[dio_pad]) {
            case 0u:
                level = 0;
                break;
            case 1u:
                level = 1;
                break;
            case 2u:
                level = -1;
                break;
            case 3u:
            default:
                level = s->pad_last_level[pad_idx];
                break;
            }
        } else {
            int out = desc->active ? s->dio_periph_out_level[dio_pad] :
                                     desc->default_out;
            level = desc->open_drain ? ((out == 0) ? 0 : -1) : out;
        }
    }

    s->pad_last_level[pad_idx] = level;
    ibex_irq_set(&s->pad_out[pad_idx], level);
    ibex_irq_set(&s->pad_attr[pad_idx], (int)attr);
}

static void ot_pinmux_eg_update_all_pad_outs(OtPinmuxEgState *s)
{
    for (unsigned i = 0; i < PARAM_N_PADRING_PADS; i++) {
        ot_pinmux_eg_eval_pad_out(s, i);
    }
}

static void ot_pinmux_eg_update_pads(OtPinmuxEgState *s)
{
    OtPinmuxEgStateRegs *regs = s->regs;

    for (unsigned pad = 0; pad < PARAM_N_MIO_PADS; pad++) {
        bool sleep_active =
            (regs->mio_pad_sleep_status[pad / 32u] >> (pad % 32u)) & 1u;
        uint32_t cfg =
            ot_pinmux_eg_encode_pad_attr(regs->mio_pad_attr[pad], sleep_active,
                                         s->mio_latched_sleep_mode[pad]);
        ibex_irq_set(&s->mios[pad], (int)cfg);
    }

    for (unsigned dio = 0; dio < PARAM_N_DIO_PADS; dio++) {
        bool sleep_active =
            (regs->dio_pad_sleep_status[dio / 32u] >> (dio % 32u)) & 1u;
        uint32_t cfg =
            ot_pinmux_eg_encode_pad_attr(regs->dio_pad_attr[dio], sleep_active,
                                         s->dio_latched_sleep_mode[dio]);
        ibex_irq_set(&s->dios[dio], (int)cfg);
    }

    ot_pinmux_eg_update_all_pad_outs(s);
}

static int ot_pinmux_eg_eval_mio_sel(const OtPinmuxEgState *s, uint32_t sel)
{
    if (sel == 1u) {
        return 1;
    }
    if (sel >= 2u && (sel - 2u) < PARAM_N_MIO_PADS) {
        unsigned mio_pad = sel - 2u;
        return (int)((s->mio_in_level >> mio_pad) & 1u);
    }
    return 0;
}

static void ot_pinmux_eg_eval_periph_in(OtPinmuxEgState *s, unsigned periph_in)
{
    g_assert(periph_in < PARAM_N_MIO_PERIPH_IN);
    if (!OT_PINMUX_EG_PERIPH_IN_DESC[periph_in].active) {
        return;
    }
    int level = ot_pinmux_eg_eval_mio_sel(s, s->regs->mio_periph_insel[periph_in]);
    ibex_irq_set(&s->periph_in[periph_in], level);
}

static void ot_pinmux_eg_update_all_periph_ins(OtPinmuxEgState *s)
{
    for (unsigned i = 0; i < PARAM_N_MIO_PERIPH_IN; i++) {
        ot_pinmux_eg_eval_periph_in(s, i);
    }
}

static void ot_pinmux_eg_eval_wkup_detectors(OtPinmuxEgState *s)
{
    OtPinmuxEgStateRegs *regs = s->regs;
    for (unsigned i = 0; i < PARAM_N_WKUP_DETECT; i++) {
        bool det_dio = (bool)(regs->wkup_detector_cfg[i] &
                              R_WKUP_DETECTOR_CFG_MIODIO_MASK);
        int level = 0;
        if (det_dio) {
            uint32_t dio_pad = regs->wkup_detector_padsel[i];
            if (dio_pad < PARAM_N_DIO_PADS) {
                level = (int)((s->dio_in_level >> dio_pad) & 1u);
            }
        } else {
            level = ot_pinmux_eg_eval_mio_sel(s, regs->wkup_detector_padsel[i]);
        }
        int prev = (int)s->wkup_prev_level[i];
        s->wkup_prev_level[i] = (uint8_t)level;
        if (!(regs->wkup_detector[i] & R_WKUP_DETECTOR_EN_MASK)) {
            continue;
        }
        uint32_t mode = regs->wkup_detector_cfg[i] & 0x7u;
        bool match = false;
        switch (mode) {
        case WKUP_DETECTOR_MODE_POSEDGE:
        default:
            match = (prev == 0 && level == 1);
            break;
        case WKUP_DETECTOR_MODE_NEGEDGE:
            match = (prev == 1 && level == 0);
            break;
        case WKUP_DETECTOR_MODE_EDGE:
            match = (prev != level);
            break;
        case WKUP_DETECTOR_MODE_TIMEDHIGH:
            /* pinmux_wkup.sv:52, 65-68: cnt_eq_th ungated by cnt_en */
            match = (level == 1) || (regs->wkup_detector_cnt_th[i] == 0u);
            break;
        case WKUP_DETECTOR_MODE_TIMEDLOW:
            /* pinmux_wkup.sv:52, 69-72: cnt_eq_th ungated by cnt_en */
            match = (level == 0) || (regs->wkup_detector_cnt_th[i] == 0u);
            break;
        }
        if (match) {
            regs->wkup_cause |= (1u << i);
            ibex_irq_set(&s->wkup, 1);
        }
    }
}

bool ot_pinmux_eg_trigger_mio_wkup(unsigned mio_pad)
{
    OtPinmuxEgState *s = ot_pinmux_eg_instance;
    if (!s || !s->regs || mio_pad >= PARAM_N_MIO_PADS) {
        return false;
    }
    CPUState *cpu = first_cpu;
    if (!cpu || !cpu->halted) {
        return false;
    }
    OtPinmuxEgStateRegs *regs = s->regs;
    bool triggered = false;
    for (unsigned i = 0; i < PARAM_N_WKUP_DETECT; i++) {
        if ((regs->wkup_detector[i] & R_WKUP_DETECTOR_EN_MASK) &&
            !(regs->wkup_detector_cfg[i] & R_WKUP_DETECTOR_CFG_MIODIO_MASK) &&
            regs->wkup_detector_padsel[i] == mio_pad + 2u) {
            regs->wkup_cause |= (1u << i);
            ibex_irq_set(&s->wkup, 0);
            ibex_irq_set(&s->wkup, 1);
            triggered = true;
        }
    }
    return triggered;
}

static void ot_pinmux_eg_dio_in(void *opaque, int irq, int level)
{
    OtPinmuxEgState *s = opaque;
    g_assert((unsigned)irq < PARAM_N_DIO_PADS);
    unsigned dio_pad = (unsigned)irq;

    uint32_t bit = 1u << dio_pad;
    if (level) {
        s->dio_in_level |= bit;
    } else {
        s->dio_in_level &= ~bit;
    }

    if (OT_PINMUX_EG_DIO_DESC[dio_pad].active) {
        ibex_irq_set(&s->dio_periph_in[dio_pad], level ? 1 : 0);
    }
    ot_pinmux_eg_eval_wkup_detectors(s);
}

static void ot_pinmux_eg_pad_in(void *opaque, int irq, int level)
{
    OtPinmuxEgState *s = opaque;
    g_assert((unsigned)irq < PARAM_N_PADRING_PADS);
    const OtPinmuxEgPadringMap *map = &OT_PINMUX_EG_PADRING_MAP[irq];

    if (map->is_dio) {
        ot_pinmux_eg_dio_in(s, map->idx, level);
        return;
    }

    unsigned mio_pad = map->idx;
    uint64_t bit = 1ULL << mio_pad;
    if (level) {
        s->mio_in_level |= bit;
    } else {
        s->mio_in_level &= ~bit;
    }

    for (unsigned p = 0; p < PARAM_N_MIO_PERIPH_IN; p++) {
        if (s->regs->mio_periph_insel[p] == mio_pad + 2u) {
            ot_pinmux_eg_eval_periph_in(s, p);
        }
    }
    ot_pinmux_eg_eval_wkup_detectors(s);
}

static void ot_pinmux_eg_periph_out_in(void *opaque, int irq, int level)
{
    OtPinmuxEgState *s = opaque;
    g_assert((unsigned)irq < PARAM_N_MIO_PERIPH_OUT);
    unsigned periph_out = (unsigned)irq;

    if (!OT_PINMUX_EG_PERIPH_OUT_DESC[periph_out].active) {
        return;
    }
    if (s->periph_out_level[periph_out] == level) {
        return;
    }
    s->periph_out_level[periph_out] = level;

    uint32_t target_outsel = periph_out + 3u;
    for (unsigned pad_idx = 0; pad_idx < PARAM_N_PADRING_PADS; pad_idx++) {
        const OtPinmuxEgPadringMap *map = &OT_PINMUX_EG_PADRING_MAP[pad_idx];
        if (!map->is_dio && s->regs->mio_outsel[map->idx] == target_outsel) {
            ot_pinmux_eg_eval_pad_out(s, pad_idx);
        }
    }
}

static void ot_pinmux_eg_dio_periph_out_in(void *opaque, int irq, int level)
{
    OtPinmuxEgState *s = opaque;
    g_assert((unsigned)irq < PARAM_N_DIO_PADS);
    unsigned dio_pad = (unsigned)irq;
    const OtPinmuxEgDioDesc *desc = &OT_PINMUX_EG_DIO_DESC[dio_pad];

    if (!desc->active) {
        return;
    }
    if (s->dio_periph_out_level[dio_pad] == level) {
        return;
    }
    s->dio_periph_out_level[dio_pad] = level;

    if (desc->padring_idx >= 0) {
        ot_pinmux_eg_eval_pad_out(s, (unsigned)desc->padring_idx);
    }
}

static void ot_pinmux_eg_sleep_en(void *opaque, int irq, int level)
{
    OtPinmuxEgState *s = opaque;
    OtPinmuxEgStateRegs *regs = s->regs;
    (void)irq;

    if (level) {
        for (unsigned k = 0; k < PARAM_N_MIO_PADS; k++) {
            if (regs->mio_pad_sleep[k] & R_MIO_PAD_SLEEP_EN_MASK) {
                regs->mio_pad_sleep_status[k / 32u] |= (1u << (k % 32u));
                s->mio_latched_sleep_mode[k] =
                    regs->mio_pad_sleep_mode[k] & R_MIO_PAD_SLEEP_MODE_OUT_MASK;
            }
        }
        for (unsigned k = 0; k < PARAM_N_DIO_PADS; k++) {
            if (regs->dio_pad_sleep[k] & R_DIO_PAD_SLEEP_EN_MASK) {
                regs->dio_pad_sleep_status[k / 32u] |= (1u << (k % 32u));
                s->dio_latched_sleep_mode[k] =
                    regs->dio_pad_sleep_mode[k] & R_DIO_PAD_SLEEP_MODE_OUT_MASK;
            }
        }
        ot_pinmux_eg_update_pads(s);
    }
}

static uint32_t ot_pinmux_eg_sel_mask(unsigned val)
{
    return (1u << (32 - clz32(val))) - 1u;
}

static uint64_t ot_pinmux_eg_regs_read(void *opaque, hwaddr addr, unsigned size)
{
    OtPinmuxEgState *s = opaque;
    (void)size;
    uint32_t val32;
    hwaddr reg = R32_OFF(addr);
    OtPinmuxEgStateRegs *regs = s->regs;

    switch (reg) {
    case CASE_RANGE(MIO_PERIPH_INSEL_REGWEN, PARAM_N_MIO_PERIPH_IN):
        val32 = regs->mio_periph_insel_regwen[reg - R_MIO_PERIPH_INSEL_REGWEN];
        break;
    case CASE_RANGE(MIO_PERIPH_INSEL, PARAM_N_MIO_PERIPH_IN):
        val32 = regs->mio_periph_insel[reg - R_MIO_PERIPH_INSEL];
        break;
    case CASE_RANGE(MIO_OUTSEL_REGWEN, PARAM_N_MIO_PADS):
        val32 = regs->mio_outsel_regwen[reg - R_MIO_OUTSEL_REGWEN];
        break;
    case CASE_RANGE(MIO_OUTSEL, PARAM_N_MIO_PADS):
        val32 = regs->mio_outsel[reg - R_MIO_OUTSEL];
        break;
    case CASE_RANGE(MIO_PAD_ATTR_REGWEN, PARAM_N_MIO_PADS):
        val32 = regs->mio_pad_attr_regwen[reg - R_MIO_PAD_ATTR_REGWEN];
        break;
    case CASE_RANGE(MIO_PAD_ATTR, PARAM_N_MIO_PADS):
        val32 = regs->mio_pad_attr[reg - R_MIO_PAD_ATTR];
        break;
    case CASE_RANGE(DIO_PAD_ATTR_REGWEN, PARAM_N_DIO_PADS):
        val32 = regs->dio_pad_attr_regwen[reg - R_DIO_PAD_ATTR_REGWEN];
        break;
    case CASE_RANGE(DIO_PAD_ATTR, PARAM_N_DIO_PADS):
        val32 = regs->dio_pad_attr[reg - R_DIO_PAD_ATTR];
        break;
    case CASE_RANGE(MIO_PAD_SLEEP_STATUS, MIO_SLEEP_STATUS_COUNT):
        val32 = regs->mio_pad_sleep_status[reg - R_MIO_PAD_SLEEP_STATUS];
        break;
    case CASE_RANGE(MIO_PAD_SLEEP_REGWEN, PARAM_N_MIO_PADS):
        val32 = regs->mio_pad_sleep_regwen[reg - R_MIO_PAD_SLEEP_REGWEN];
        break;
    case CASE_RANGE(MIO_PAD_SLEEP, PARAM_N_MIO_PADS):
        val32 = regs->mio_pad_sleep[reg - R_MIO_PAD_SLEEP];
        break;
    case CASE_RANGE(MIO_PAD_SLEEP_MODE, PARAM_N_MIO_PADS):
        val32 = regs->mio_pad_sleep_mode[reg - R_MIO_PAD_SLEEP_MODE];
        break;
    case CASE_RANGE(DIO_PAD_SLEEP_STATUS, DIO_SLEEP_STATUS_COUNT):
        val32 = regs->dio_pad_sleep_status[reg - R_DIO_PAD_SLEEP_STATUS];
        break;
    case CASE_RANGE(DIO_PAD_SLEEP_REGWEN, PARAM_N_DIO_PADS):
        val32 = regs->dio_pad_sleep_regwen[reg - R_DIO_PAD_SLEEP_REGWEN];
        break;
    case CASE_RANGE(DIO_PAD_SLEEP, PARAM_N_DIO_PADS):
        val32 = regs->dio_pad_sleep[reg - R_DIO_PAD_SLEEP];
        break;
    case CASE_RANGE(DIO_PAD_SLEEP_MODE, PARAM_N_DIO_PADS):
        val32 = regs->dio_pad_sleep_mode[reg - R_DIO_PAD_SLEEP_MODE];
        break;
    case CASE_RANGE(WKUP_DETECTOR_REGWEN, PARAM_N_WKUP_DETECT):
        val32 = regs->wkup_detector_regwen[reg - R_WKUP_DETECTOR_REGWEN];
        break;
    case CASE_RANGE(WKUP_DETECTOR, PARAM_N_WKUP_DETECT):
        val32 = regs->wkup_detector[reg - R_WKUP_DETECTOR];
        break;
    case CASE_RANGE(WKUP_DETECTOR_CFG, PARAM_N_WKUP_DETECT):
        val32 = regs->wkup_detector_cfg[reg - R_WKUP_DETECTOR_CFG];
        break;
    case CASE_RANGE(WKUP_DETECTOR_CNT_TH, PARAM_N_WKUP_DETECT):
        val32 = regs->wkup_detector_cnt_th[reg - R_WKUP_DETECTOR_CNT_TH];
        break;
    case CASE_RANGE(WKUP_DETECTOR_PADSEL, PARAM_N_WKUP_DETECT):
        val32 = regs->wkup_detector_padsel[reg - R_WKUP_DETECTOR_PADSEL];
        break;
    case CASE_SCALAR(WKUP_CAUSE):
        val32 = regs->wkup_cause;
        break;
    case CASE_SCALAR(ALERT_TEST):
        qemu_log_mask(LOG_GUEST_ERROR, "%s: W/O register 0x%03x\n", __func__,
                      (uint32_t)addr);
        val32 = 0;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: Bad offset 0x%03x\n", __func__,
                      (uint32_t)addr);
        val32 = 0;
        break;
    }

    uint32_t pc = ibex_get_current_pc();
    trace_ot_pinmux_io_read_out((uint32_t)addr, val32, pc);

    return (uint64_t)val32;
}

#define OT_PINMUX_EG_IS_REGWEN(_off_, _ren_, _rw_) \
    ((regs->_ren_##_regwen[(_off_) - (R_##_rw_)]) & 0x1u)

static void ot_pinmux_eg_regs_write(void *opaque, hwaddr addr, uint64_t val64,
                                    unsigned size)
{
    OtPinmuxEgState *s = opaque;
    (void)size;
    uint32_t val32 = (uint32_t)val64;
    hwaddr reg = R32_OFF(addr);
    OtPinmuxEgStateRegs *regs = s->regs;

    uint32_t pc = ibex_get_current_pc();
    trace_ot_pinmux_io_write((uint32_t)addr, val32, pc);

    switch (reg) {
    case R_ALERT_TEST:
        val32 &= R_ALERT_TEST_FATAL_FAULT_MASK;
        if (val32) {
            ibex_irq_set(&s->alert, 1);
            ibex_irq_set(&s->alert, 0);
        }
        break;
    case CASE_RANGE(MIO_PERIPH_INSEL_REGWEN, PARAM_N_MIO_PERIPH_IN):
        val32 &= R_MIO_PERIPH_INSEL_REGWEN_EN_MASK;
        regs->mio_periph_insel_regwen[reg - R_MIO_PERIPH_INSEL_REGWEN] &= val32;
        break;
    case CASE_RANGE(MIO_PERIPH_INSEL, PARAM_N_MIO_PERIPH_IN):
        if (OT_PINMUX_EG_IS_REGWEN(reg, mio_periph_insel, MIO_PERIPH_INSEL)) {
            if (val32 >= PARAM_N_MIO_PERIPH_IN + 2u) {
                qemu_log_mask(LOG_GUEST_ERROR, "%s: 0x%03x too large: %u\n",
                              __func__, (unsigned)reg, val32);
                uint32_t mask =
                    ot_pinmux_eg_sel_mask(PARAM_N_MIO_PERIPH_IN + 2u);
                val32 &= mask;
            }
            unsigned periph_in = reg - R_MIO_PERIPH_INSEL;
            regs->mio_periph_insel[periph_in] = val32;
            ot_uart_update_pinmux(regs->mio_outsel, regs->mio_periph_insel);
            ot_pinmux_eg_eval_periph_in(s, periph_in);
            ot_pinmux_eg_eval_wkup_detectors(s);
        } else {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: 0x%03x access is disabled\n",
                          __func__, (uint32_t)addr);
        }
        break;
    case CASE_RANGE(MIO_OUTSEL_REGWEN, PARAM_N_MIO_PADS):
        val32 &= R_MIO_OUTSEL_REGWEN_EN_MASK;
        regs->mio_outsel_regwen[reg - R_MIO_OUTSEL_REGWEN] &= val32;
        break;
    case CASE_RANGE(MIO_OUTSEL, PARAM_N_MIO_PADS):
        if (OT_PINMUX_EG_IS_REGWEN(reg, mio_outsel, MIO_OUTSEL)) {
            if (val32 >= PARAM_N_MIO_PERIPH_OUT + 3u) {
                qemu_log_mask(LOG_GUEST_ERROR, "%s: 0x%03x too large: %u\n",
                              __func__, (unsigned)reg, val32);
                uint32_t mask =
                    ot_pinmux_eg_sel_mask(PARAM_N_MIO_PERIPH_OUT + 3u);
                val32 &= mask;
            }
            regs->mio_outsel[reg - R_MIO_OUTSEL] = val32;
            ot_uart_update_pinmux(regs->mio_outsel, regs->mio_periph_insel);
            ot_pinmux_eg_update_pads(s);
        } else {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: 0x%03x access is disabled\n",
                          __func__, (uint32_t)addr);
        }
        break;
    case CASE_RANGE(MIO_PAD_ATTR_REGWEN, PARAM_N_MIO_PADS):
        val32 &= R_MIO_PAD_ATTR_REGWEN_EN_MASK;
        regs->mio_pad_attr_regwen[reg - R_MIO_PAD_ATTR_REGWEN] &= val32;
        break;
    case CASE_RANGE(MIO_PAD_ATTR, PARAM_N_MIO_PADS):
        if (OT_PINMUX_EG_IS_REGWEN(reg, mio_pad_attr, MIO_PAD_ATTR)) {
            unsigned pad_no = reg - R_MIO_PAD_ATTR;
            g_assert(pad_no < PARAM_N_MIO_PADS);
            val32 &= MIO_PAD_ATTR_MASK;
            regs->mio_pad_attr[pad_no] = val32;
            ot_pinmux_eg_update_pads(s);
        } else {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: 0x%03x access is disabled\n",
                          __func__, (uint32_t)addr);
        }
        break;
    case CASE_RANGE(DIO_PAD_ATTR_REGWEN, PARAM_N_DIO_PADS):
        val32 &= R_DIO_PAD_ATTR_REGWEN_EN_MASK;
        regs->dio_pad_attr_regwen[reg - R_DIO_PAD_ATTR_REGWEN] &= val32;
        break;
    case CASE_RANGE(DIO_PAD_ATTR, PARAM_N_DIO_PADS):
        if (OT_PINMUX_EG_IS_REGWEN(reg, dio_pad_attr, DIO_PAD_ATTR)) {
            unsigned pad_no = reg - R_DIO_PAD_ATTR;
            g_assert(pad_no < PARAM_N_DIO_PADS);
            val32 &= OT_PINMUX_EG_DIO_DESC[pad_no].attr_mask;
            regs->dio_pad_attr[pad_no] = val32;
            ot_pinmux_eg_update_pads(s);
        } else {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: 0x%03x access is disabled\n",
                          __func__, (uint32_t)addr);
        }
        break;
    case CASE_RANGE(MIO_PAD_SLEEP_STATUS, MIO_SLEEP_STATUS_COUNT):
        val32 &= MIO_PAD_SLEEP_STATUS_MASK;
#ifdef BIT_PACKED_STATUS_REG
        if ((MIO_PAD_SLEEP_STATUS_REM != 0) &&
            reg == R_MIO_PAD_SLEEP_STATUS + MIO_SLEEP_STATUS_COUNT - 1u) {
            val32 &= (1u << MIO_PAD_SLEEP_STATUS_REM) - 1u;
        }
#endif
        regs->mio_pad_sleep_status[reg - R_MIO_PAD_SLEEP_STATUS] &= val32;
        ot_pinmux_eg_update_pads(s);
        break;
    case CASE_RANGE(MIO_PAD_SLEEP_REGWEN, PARAM_N_MIO_PADS):
        val32 &= R_MIO_PAD_SLEEP_REGWEN_EN_MASK;
        regs->mio_pad_sleep_regwen[reg - R_MIO_PAD_SLEEP_REGWEN] &= val32;
        break;
    case CASE_RANGE(MIO_PAD_SLEEP, PARAM_N_MIO_PADS):
        if (OT_PINMUX_EG_IS_REGWEN(reg, mio_pad_sleep, MIO_PAD_SLEEP)) {
            val32 &= R_MIO_PAD_SLEEP_EN_MASK;
            regs->mio_pad_sleep[reg - R_MIO_PAD_SLEEP] = val32;
        } else {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: 0x%03x access is disabled\n",
                          __func__, (uint32_t)addr);
        }
        break;
    case CASE_RANGE(MIO_PAD_SLEEP_MODE, PARAM_N_MIO_PADS):
        if (OT_PINMUX_EG_IS_REGWEN(reg, mio_pad_sleep, MIO_PAD_SLEEP_MODE)) {
            val32 &= R_MIO_PAD_SLEEP_MODE_OUT_MASK;
            regs->mio_pad_sleep_mode[reg - R_MIO_PAD_SLEEP_MODE] = val32;
        } else {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: 0x%03x access is disabled\n",
                          __func__, (uint32_t)addr);
        }
        break;
    case CASE_RANGE(DIO_PAD_SLEEP_STATUS, DIO_SLEEP_STATUS_COUNT):
        val32 &= DIO_PAD_SLEEP_STATUS_MASK;
#ifdef BIT_PACKED_STATUS_REG
        if ((DIO_PAD_SLEEP_STATUS_REM != 0) &&
            reg == R_DIO_PAD_SLEEP_STATUS + DIO_SLEEP_STATUS_COUNT - 1u) {
            val32 &= (1u << DIO_PAD_SLEEP_STATUS_REM) - 1u;
        }
#endif
        regs->dio_pad_sleep_status[reg - R_DIO_PAD_SLEEP_STATUS] &= val32;
        ot_pinmux_eg_update_pads(s);
        break;
    case CASE_RANGE(DIO_PAD_SLEEP_REGWEN, PARAM_N_DIO_PADS):
        val32 &= R_DIO_PAD_SLEEP_REGWEN_EN_MASK;
        regs->dio_pad_sleep_regwen[reg - R_DIO_PAD_SLEEP_REGWEN] &= val32;
        break;
    case CASE_RANGE(DIO_PAD_SLEEP, PARAM_N_DIO_PADS):
        if (OT_PINMUX_EG_IS_REGWEN(reg, dio_pad_sleep, DIO_PAD_SLEEP)) {
            val32 &= R_DIO_PAD_SLEEP_EN_MASK;
            regs->dio_pad_sleep[reg - R_DIO_PAD_SLEEP] = val32;
        } else {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: 0x%03x access is disabled\n",
                          __func__, (uint32_t)addr);
        }
        break;
    case CASE_RANGE(DIO_PAD_SLEEP_MODE, PARAM_N_DIO_PADS):
        if (OT_PINMUX_EG_IS_REGWEN(reg, dio_pad_sleep, DIO_PAD_SLEEP_MODE)) {
            val32 &= R_DIO_PAD_SLEEP_MODE_OUT_MASK;
            regs->dio_pad_sleep_mode[reg - R_DIO_PAD_SLEEP_MODE] = val32;
        } else {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: 0x%03x access is disabled\n",
                          __func__, (uint32_t)addr);
        }
        break;
    case CASE_RANGE(WKUP_DETECTOR_REGWEN, PARAM_N_WKUP_DETECT):
        val32 &= R_WKUP_DETECTOR_REGWEN_EN_MASK;
        regs->wkup_detector_regwen[reg - R_WKUP_DETECTOR_REGWEN] &= val32;
        break;
    case CASE_RANGE(WKUP_DETECTOR, PARAM_N_WKUP_DETECT):
        if (OT_PINMUX_EG_IS_REGWEN(reg, wkup_detector, WKUP_DETECTOR)) {
            val32 &= R_WKUP_DETECTOR_EN_MASK;
            if (val32 && bql_locked()) {
                replay_mutex_unlock();
                bql_unlock();
                g_usleep(20000);
                replay_mutex_lock();
                bql_lock();
            }
            regs->wkup_detector[reg - R_WKUP_DETECTOR] = val32;
            ot_pinmux_eg_eval_wkup_detectors(s);
        } else {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: 0x%03x access is disabled\n",
                          __func__, (uint32_t)addr);
        }
        break;
    case CASE_RANGE(WKUP_DETECTOR_CFG, PARAM_N_WKUP_DETECT):
        if (OT_PINMUX_EG_IS_REGWEN(reg, wkup_detector, WKUP_DETECTOR_CFG)) {
            val32 &= WKUP_DETECTOR_CFG_MASK;
            regs->wkup_detector_cfg[reg - R_WKUP_DETECTOR_CFG] = val32;
            ot_pinmux_eg_eval_wkup_detectors(s);
        } else {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: 0x%03x access is disabled\n",
                          __func__, (uint32_t)addr);
        }
        break;
    case CASE_RANGE(WKUP_DETECTOR_CNT_TH, PARAM_N_WKUP_DETECT):
        if (OT_PINMUX_EG_IS_REGWEN(reg, wkup_detector, WKUP_DETECTOR_CNT_TH)) {
            val32 &= R_WKUP_DETECTOR_CNT_TH_TH_MASK;
            regs->wkup_detector_cnt_th[reg - R_WKUP_DETECTOR_CNT_TH] = val32;
            ot_pinmux_eg_eval_wkup_detectors(s);
        } else {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: 0x%03x access is disabled\n",
                          __func__, (uint32_t)addr);
        }
        break;
    case CASE_RANGE(WKUP_DETECTOR_PADSEL, PARAM_N_WKUP_DETECT):
        if (OT_PINMUX_EG_IS_REGWEN(reg, wkup_detector, WKUP_DETECTOR_PADSEL)) {
            if (val32 >= PARAM_N_MIO_PADS + 2u) {
                qemu_log_mask(LOG_GUEST_ERROR, "%s: 0x%03x too large: %u\n",
                              __func__, (unsigned)reg, val32);
                uint32_t mask = ot_pinmux_eg_sel_mask(PARAM_N_MIO_PADS + 2u);
                val32 &= mask;
            }
            regs->wkup_detector_padsel[reg - R_WKUP_DETECTOR_PADSEL] = val32;
            ot_pinmux_eg_eval_wkup_detectors(s);
        } else {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: 0x%03x access is disabled\n",
                          __func__, (uint32_t)addr);
        }
        break;
    case CASE_SCALAR(WKUP_CAUSE):
        regs->wkup_cause &= (val32 & WKUP_CAUSE_MASK);
        ibex_irq_set(&s->wkup, (int)(bool)regs->wkup_cause);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: Bad offset 0x%x\n", __func__,
                      (uint32_t)addr);
        break;
    }
}

static uint8_t ot_pinmux_eg_permit(hwaddr reg)
{
    if ((reg >= R_MIO_PAD_ATTR && reg < R_MIO_PAD_ATTR + PARAM_N_MIO_PADS) ||
        (reg >= R_DIO_PAD_ATTR && reg < R_DIO_PAD_ATTR + PARAM_N_DIO_PADS)) {
        return 0x7u;
    }
    if (reg == R_MIO_PAD_SLEEP_STATUS) {
        return 0xfu;
    }
    if (reg == R_MIO_PAD_SLEEP_STATUS + 1u || reg == R_DIO_PAD_SLEEP_STATUS) {
        return 0x3u;
    }
    return 0x1u;
}

static bool ot_pinmux_eg_regs_accepts(void *opaque, hwaddr addr, unsigned size,
                                      bool is_write, MemTxAttrs attrs)
{
    (void)opaque;
    (void)attrs;
    hwaddr reg = R32_OFF(addr);
    uint8_t reg_be = ((1u << size) - 1u) << (addr & 0x3u);
    return reg < REGS_COUNT &&
           (!is_write || (ot_pinmux_eg_permit(reg) & ~reg_be) == 0u);
}

static const MemoryRegionOps ot_pinmux_eg_regs_ops = {
    .read = &ot_pinmux_eg_regs_read,
    .write = &ot_pinmux_eg_regs_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl.min_access_size = 4u,
    .impl.max_access_size = 4u,
    .valid.accepts = &ot_pinmux_eg_regs_accepts,
};

static void ot_pinmux_eg_reset_enter(Object *obj, ResetType type)
{
    OtPinmuxEgClass *c = OT_PINMUX_EG_GET_CLASS(obj);
    OtPinmuxEgState *s = OT_PINMUX_EG(obj);

    if (c->parent_phases.enter) {
        c->parent_phases.enter(obj, type);
    }

    for (unsigned ix = 0; ix < PARAM_N_MIO_PERIPH_IN; ix++) {
        s->periph_in[ix].level = -1;
    }
    for (unsigned dio = 0; dio < PARAM_N_DIO_PADS; dio++) {
        s->dio_periph_in[dio].level = -1;
    }

    OtPinmuxEgStateRegs *regs = s->regs;
    bool is_por = (type == RESET_TYPE_COLD) || ot_rstmgr_is_por_reset();
    if (!is_por && ot_rstmgr_is_low_power_exit()) {
        ot_uart_update_pinmux(regs->mio_outsel, regs->mio_periph_insel);
        ot_pinmux_eg_update_all_periph_ins(s);
        ot_pinmux_eg_eval_wkup_detectors(s);
        ot_pinmux_eg_update_pads(s);
        ibex_irq_set(&s->wkup, (int)(bool)regs->wkup_cause);
        ibex_irq_set(&s->alert, 0);
        return;
    }

    uint32_t wkup_cause = regs->wkup_cause;
    uint32_t mio_sleep_st[MIO_SLEEP_STATUS_COUNT];
    uint32_t dio_sleep_st[DIO_SLEEP_STATUS_COUNT];
    memcpy(mio_sleep_st, regs->mio_pad_sleep_status, sizeof(mio_sleep_st));
    memcpy(dio_sleep_st, regs->dio_pad_sleep_status, sizeof(dio_sleep_st));

    memset(regs, 0, sizeof(*regs));
    if (!is_por) {
        regs->wkup_cause = wkup_cause;
        memcpy(regs->mio_pad_sleep_status, mio_sleep_st, sizeof(mio_sleep_st));
        memcpy(regs->dio_pad_sleep_status, dio_sleep_st, sizeof(dio_sleep_st));
    } else {
        memset(s->mio_latched_sleep_mode, 0, sizeof(s->mio_latched_sleep_mode));
        memset(s->dio_latched_sleep_mode, 0, sizeof(s->dio_latched_sleep_mode));
        for (unsigned i = 0; i < PARAM_N_PADRING_PADS; i++) {
            s->pad_last_level[i] = -1;
        }
    }

    for (unsigned ix = 0; ix < PARAM_N_MIO_PERIPH_IN; ix++) {
        regs->mio_periph_insel_regwen[ix] = 0x1u;
    }
    for (unsigned ix = 0; ix < PARAM_N_MIO_PADS; ix++) {
        regs->mio_outsel_regwen[ix] = 0x1u;
        regs->mio_outsel[ix] = 0x2u;
        regs->mio_pad_attr_regwen[ix] = 0x1u;
        regs->mio_pad_sleep_regwen[ix] = 0x1u;
        regs->mio_pad_sleep_mode[ix] = 0x2u;
    }
    for (unsigned ix = 0; ix < PARAM_N_DIO_PADS; ix++) {
        regs->dio_pad_attr_regwen[ix] = 0x1u;
        regs->dio_pad_sleep_regwen[ix] = 0x1u;
        regs->dio_pad_sleep_mode[ix] = 0x2u;
    }
    for (unsigned ix = 0; ix < PARAM_N_WKUP_DETECT; ix++) {
        regs->wkup_detector_regwen[ix] = 0x1u;
    }

    ot_uart_update_pinmux(regs->mio_outsel, regs->mio_periph_insel);
    ot_pinmux_eg_update_all_periph_ins(s);
    for (unsigned dio = 0; dio < PARAM_N_DIO_PADS; dio++) {
        if (OT_PINMUX_EG_DIO_DESC[dio].active) {
            ibex_irq_set(&s->dio_periph_in[dio],
                         (int)((s->dio_in_level >> dio) & 1u));
        }
    }
    ot_pinmux_eg_eval_wkup_detectors(s);
    ot_pinmux_eg_update_pads(s);
    ibex_irq_set(&s->wkup, (int)(bool)regs->wkup_cause);
    ibex_irq_set(&s->alert, 0);
}

static void ot_pinmux_eg_reset_exit(Object *obj, ResetType type)
{
    OtPinmuxEgState *s = OT_PINMUX_EG(obj);
    (void)type;

    for (unsigned ix = 0; ix < PARAM_N_MIO_PERIPH_IN; ix++) {
        s->periph_in[ix].level = -1;
    }
    for (unsigned dio = 0; dio < PARAM_N_DIO_PADS; dio++) {
        s->dio_periph_in[dio].level = -1;
    }
    ot_pinmux_eg_update_all_periph_ins(s);
    for (unsigned dio = 0; dio < PARAM_N_DIO_PADS; dio++) {
        if (OT_PINMUX_EG_DIO_DESC[dio].active) {
            ibex_irq_set(&s->dio_periph_in[dio],
                         (int)((s->dio_in_level >> dio) & 1u));
        }
    }
    ot_pinmux_eg_update_pads(s);
}

static void ot_pinmux_eg_init(Object *obj)
{
    OtPinmuxEgState *s = OT_PINMUX_EG(obj);

    memory_region_init_io(&s->mmio, obj, &ot_pinmux_eg_regs_ops, s,
                          TYPE_OT_PINMUX_EG, REGS_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->mmio);

    s->regs = g_new0(OtPinmuxEgStateRegs, 1u);
    ibex_qdev_init_irq(obj, &s->alert, OT_DEVICE_ALERT);
    ibex_qdev_init_irq(obj, &s->wkup, OT_PINMUX_WKUP);

    s->dios = g_new(IbexIRQ, PARAM_N_DIO_PADS);
    s->mios = g_new(IbexIRQ, PARAM_N_MIO_PADS);
    s->periph_in = g_new(IbexIRQ, PARAM_N_MIO_PERIPH_IN);
    s->dio_periph_in = g_new(IbexIRQ, PARAM_N_DIO_PADS);
    s->pad_out = g_new(IbexIRQ, PARAM_N_PADRING_PADS);
    s->pad_attr = g_new(IbexIRQ, PARAM_N_PADRING_PADS);

    ibex_qdev_init_irqs_default(obj, s->dios, OT_PINMUX_DIO, PARAM_N_DIO_PADS,
                                PAD_ATTR_ENABLE(false));
    ibex_qdev_init_irqs_default(obj, s->mios, OT_PINMUX_MIO, PARAM_N_MIO_PADS,
                                PAD_ATTR_ENABLE(false));
    ibex_qdev_init_irqs_default(obj, s->periph_in, OT_PINMUX_PERIPH_IN,
                                PARAM_N_MIO_PERIPH_IN, 0);
    ibex_qdev_init_irqs_default(obj, s->dio_periph_in, OT_PINMUX_DIO_PERIPH_IN,
                                PARAM_N_DIO_PADS, 0);
    ibex_qdev_init_irqs_default(obj, s->pad_out, OT_PINMUX_PAD_OUT,
                                PARAM_N_PADRING_PADS, -1);
    ibex_qdev_init_irqs_default(obj, s->pad_attr, OT_PINMUX_PAD_ATTR,
                                PARAM_N_PADRING_PADS, 0);

    for (unsigned i = 0; i < PARAM_N_MIO_PERIPH_OUT; i++) {
        s->periph_out_level[i] = OT_PINMUX_EG_PERIPH_OUT_DESC[i].default_val;
    }
    s->dio_in_level = 0u;
    for (unsigned i = 0; i < PARAM_N_DIO_PADS; i++) {
        s->dio_periph_out_level[i] = OT_PINMUX_EG_DIO_DESC[i].default_out;
        if (OT_PINMUX_EG_DIO_DESC[i].default_in) {
            s->dio_in_level |= (1u << i);
        }
    }
    for (unsigned i = 0; i < PARAM_N_PADRING_PADS; i++) {
        s->pad_last_level[i] = -1;
    }
    s->mio_in_level = (1ULL << OT_PINMUX_EG_MIO_IOB5) |
                      (1ULL << OT_PINMUX_EG_MIO_IOB6);

    qdev_init_gpio_in_named(DEVICE(obj), &ot_pinmux_eg_pad_in, OT_PINMUX_PAD_IN,
                            PARAM_N_PADRING_PADS);
    qdev_init_gpio_in_named(DEVICE(obj), &ot_pinmux_eg_periph_out_in,
                            OT_PINMUX_PERIPH_OUT, PARAM_N_MIO_PERIPH_OUT);
    qdev_init_gpio_in_named(DEVICE(obj), &ot_pinmux_eg_dio_in, OT_PINMUX_DIO_IN,
                            PARAM_N_DIO_PADS);
    qdev_init_gpio_in_named(DEVICE(obj), &ot_pinmux_eg_dio_periph_out_in,
                            OT_PINMUX_DIO_PERIPH_OUT, PARAM_N_DIO_PADS);
    qdev_init_gpio_in_named(DEVICE(obj), &ot_pinmux_eg_sleep_en,
                            OT_PINMUX_SLEEP_EN, 1u);
    ot_pinmux_eg_instance = s;
}

static void ot_pinmux_eg_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    (void)data;

    set_bit(DEVICE_CATEGORY_MISC, dc->categories);

    ResettableClass *rc = RESETTABLE_CLASS(klass);
    OtPinmuxEgClass *pc = OT_PINMUX_EG_CLASS(klass);
    resettable_class_set_parent_phases(rc, &ot_pinmux_eg_reset_enter, NULL,
                                       &ot_pinmux_eg_reset_exit,
                                       &pc->parent_phases);
}

static const TypeInfo ot_pinmux_eg_info = {
    .name = TYPE_OT_PINMUX_EG,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(OtPinmuxEgState),
    .instance_init = &ot_pinmux_eg_init,
    .class_size = sizeof(OtPinmuxEgClass),
    .class_init = &ot_pinmux_eg_class_init,
};

static void ot_pinmux_eg_register_types(void)
{
    type_register_static(&ot_pinmux_eg_info);
}

type_init(ot_pinmux_eg_register_types);
