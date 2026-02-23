#pragma once

// =============================================================================
// config.h — board-level constants (port of config.py)
// DO NOT MODIFY AT RUNTIME
// =============================================================================

// -----------------------------------------------------------------------------
// I2C
// -----------------------------------------------------------------------------
#define I2C_PORT_NUM        0
#define I2C_SDA_GPIO        5
#define I2C_SCL_GPIO        6
#define I2C_FREQ_HZ         400000

#define INA226_ADDR_BATT    0x40
#define INA226_ADDR_SYS     0x44
#define BQ25895_ADDR        0x6A
#define TCA9535_ADDR        0x20
#define OLED_ADDR           0x3C

// -----------------------------------------------------------------------------
// Shunt / circuit resistances
// -----------------------------------------------------------------------------
#define SHUNT_RESISTANCE_OHMS   0.008f      // 8 mΩ
#define CIRCUIT_R_CHARGE_MOHM   25
#define CIRCUIT_R_DISCHG_MOHM   25
#define DISCHARGE_RESISTOR_OHMS 0.83f

// -----------------------------------------------------------------------------
// TCA9535 port direction (1 = input, 0 = output)
// Port 0: CE(0), QON(1) outputs; CFG_0..CFG_5 inputs
// Port 1: FAN (bit0) output; rest inputs
// -----------------------------------------------------------------------------
#define TCA_DIR0    0xFC    // 0b11111100
#define TCA_DIR1    0xFE    // 0b11111110

// -----------------------------------------------------------------------------
// INA226 averaging / conversion time defaults
// -----------------------------------------------------------------------------
#define INA_AVG_SAMPLES     128
#define INA_VBUS_CT_US      332
#define INA_VSH_CT_US       332

// -----------------------------------------------------------------------------
// TCA9535 logical pin mapping (0-based, 0..7 = port0, 8..15 = port1)
// -----------------------------------------------------------------------------
#define P_CE        0
#define P_QON       1
#define P_CFG_5     2
#define P_CFG_4     3
#define P_CFG_3     4
#define P_CFG_2     5
#define P_CFG_1     6
#define P_CFG_0     7

#define P_FAN       8
#define P_BTN1      9   // MENU
#define P_BTN2      10  // LEFT
#define P_BTN3      11  // RIGHT
#define P_STAT      12
#define P_ALERT1    13
#define P_CHG_INT   14
#define P_ALERT2    15

// Button GPIO (always-present BOOT button, active-low)
#define GPIO_OK     9    // BOOT button, active-low, pull-up
#define TACH_GPIO   4    // fan tach input, active-low, pull-up

// Expander pin assignments (TCA9535)
// #define P_BTN1      0    // MENU button (in0, bit 0), active-low
// #define P_BTN2      1    // LEFT button (in0, bit 1), active-low
// #define P_BTN3      2    // RIGHT button (in0, bit 2), active-low
// #define P_FAN       8    // FAN enable (in1, bit 0), active-high output
// #define P_QON       9    // BQ25895 QON pulse (in1, bit 1), active-high
// #define P_CE        10   // BQ25895 /CE (in1, bit 2), active-LOW = charge enabled

// -----------------------------------------------------------------------------
// TWAI (CAN)
// Pin assignments — adapt to your PCB. No transceiver on-chip; needs e.g. SN65HVD230.
// -----------------------------------------------------------------------------
#define TWAI_TX_GPIO    3
#define TWAI_RX_GPIO    2
#define TWAI_BITRATE    500000  // 500 kbps — adjust as needed

// -----------------------------------------------------------------------------
// UI timing
// -----------------------------------------------------------------------------
#define STATUS_ROTATE_MS    5000

// -----------------------------------------------------------------------------
// Max cells
// -----------------------------------------------------------------------------
#define MAX_CELLS_PARALLEL  16
