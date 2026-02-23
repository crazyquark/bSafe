# config.py
# DO NOT MODIFY AT RUNTIME

from micropython import const

# ------------------------
# I2C
# ------------------------
I2C_SDA = const(5)
I2C_SCL = const(6)
I2C_FREQ = const(400_000)

INA226_ADDR_BATT = const(0x40)
INA226_ADDR_SYS  = const(0x44)
BQ25895_ADDR     = const(0x6A)
TCA9535_ADDR     = const(0x20)
OLED_ADDR        = const(0x3C)

# ------------------------
# Shunt & resistance
# ------------------------
SHUNT_RESISTANCE_OHMS = 0.008       # 8 milliohms
CIRCUIT_R_CHARGE_MOHM = 25
CIRCUIT_R_DISCHG_MOHM = 25
DISCHARGE_RESISTOR_OHMS = 0.85

# --- TCA9535 directions (1=input, 0=output)
# Port 0: CE(0), QON(1) outputs; CFG_0..CFG_5 inputs
TCA_DIR0 = const(0b11111100)

# Port 1: FAN (pin 8 -> port1 bit0) output; all others inputs
TCA_DIR1 = const(0b11111110)

# --- INA226 defaults (placeholders; tuned later)
# Precision-oriented defaults: moderate averaging + longer conv times
INA_AVG_SAMPLES = const(16)     # 1,4,16,64,128,256,512,1024
INA_VBUS_CT_US  = const(1100)   # 140,204,332,588,1100,2116,4156,8244
INA_VSH_CT_US   = const(1100)

# Alert defaults (we'll set at runtime based on chemistry/settings)
INA_ALERT_LATCHED = True
INA_ALERT_POLARITY_ACTIVE_LOW = True  # matches your pulled-up inputs

# ------------------------
# MCU GPIOs
# ------------------------
PIN_DISCHARGE = const(12)   # active-high
PIN_TACH      = const(18)
PIN_RGB       = const(22)
PIN_SERVO     = const(16)

PIN_CAN_RX    = const(5)
PIN_CAN_TX    = const(6)

PIN_IO_INT    = const(13)   # TCA9535 INT (active-low)

# ------------------------
# TCA9535 pin mapping
# ------------------------
# Port 0
P_CE       = const(0)
P_QON      = const(1)
P_CFG_5    = const(2)
P_CFG_4    = const(3)
P_CFG_3    = const(4)
P_CFG_2    = const(5)
P_CFG_1    = const(6)
P_CFG_0    = const(7)

# Port 1
P_FAN      = const(8)
P_BTN1     = const(9)
P_BTN2     = const(10)
P_BTN3     = const(11)
P_STAT     = const(12)
P_ALERT1   = const(13)
P_CHG_INT  = const(14)
P_ALERT2   = const(15)

# ------------------------
# UI Pages
# ------------------------
PAGE_STATUS = const(0)
PAGE_CONFIG = const(1)
PAGE_HEALTH = const(2)
PAGE_MODE   = const(3)

STATUS_ROTATE_MS = const(5000)

# ------------------------
# RGB colors (RGB tuples)
# ------------------------
RGB_PRECONDITION = (255,  20, 147)   # pink
RGB_CHARGE       = (160,  32, 240)   # purple
RGB_WAIT         = (255, 255,   0)   # yellow
RGB_DISCHARGE    = (255,   0,   0)   # red
RGB_RECHARGE     = (  0,   0, 255)   # blue
RGB_DONE         = (  0, 255,   0)   # green
RGB_IDLE         = (  0,   0,   0)

# ------------------------
# Chemistry definitions
# ------------------------
CHEMISTRIES = {
    "LTO":     2.40,
    "LFP":     3.60,
    "LCO":     4.20,
    "HV-LCO":  4.35,
    "LMNO":    4.10,
}

MAX_CELLS_PARALLEL = 16

# TS / NTC conversion (BQ25895 TS pin divider: REGN --10k-- TS -- NTC -- GND)
TS_PULLUP_OHMS = 10000
TS_NTC_R0_OHMS = 10000     # assumed 10k NTC @ 25C
TS_NTC_BETA_K = 3435
TS_NTC_T0_C = 25.0
