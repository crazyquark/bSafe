# drivers/tca9535.py
#
# TCA9535 16-bit I/O expander
# INT is active-low, open-drain
#
# Port 0: pins 0..7
# Port 1: pins 8..15

from micropython import const

# Registers
REG_INPUT_0   = const(0x00)
REG_INPUT_1   = const(0x01)
REG_OUTPUT_0  = const(0x02)
REG_OUTPUT_1  = const(0x03)
REG_POL_0     = const(0x04)
REG_POL_1     = const(0x05)
REG_CONFIG_0  = const(0x06)
REG_CONFIG_1  = const(0x07)

class TCA9535:
    def __init__(self, i2c, addr=0x20):
        self.i2c = i2c
        self.addr = addr

        self._dir0 = 0xFF
        self._dir1 = 0xFF
        self._out0 = 0x00
        self._out1 = 0x00

        self._last_in0 = 0x00
        self._last_in1 = 0x00

    # -------------------------
    # Low-level helpers
    # -------------------------
    def _write(self, reg, val):
        self.i2c.writeto_mem(self.addr, reg, bytes([val]))

    def _read(self, reg):
        return self.i2c.readfrom_mem(self.addr, reg, 1)[0]

    # -------------------------
    # Configuration
    # -------------------------
    def configure(self, dir0, dir1):
        """
        dir bit = 1 → input
        dir bit = 0 → output
        """
        self._dir0 = dir0 & 0xFF
        self._dir1 = dir1 & 0xFF
        self._write(REG_CONFIG_0, self._dir0)
        self._write(REG_CONFIG_1, self._dir1)

        # No polarity inversion
        self._write(REG_POL_0, 0x00)
        self._write(REG_POL_1, 0x00)

        # Initialize outputs
        self._write(REG_OUTPUT_0, self._out0)
        self._write(REG_OUTPUT_1, self._out1)

        # Prime input cache
        self._last_in0 = self._read(REG_INPUT_0)
        self._last_in1 = self._read(REG_INPUT_1)

    # -------------------------
    # Output control
    # -------------------------
    def write_pin(self, pin, value):
        if pin < 8:
            if value:
                self._out0 |= (1 << pin)
            else:
                self._out0 &= ~(1 << pin)
            self._write(REG_OUTPUT_0, self._out0)
        else:
            p = pin - 8
            if value:
                self._out1 |= (1 << p)
            else:
                self._out1 &= ~(1 << p)
            self._write(REG_OUTPUT_1, self._out1)

    def write_mask(self, mask0, mask1):
        self._out0 = mask0 & 0xFF
        self._out1 = mask1 & 0xFF
        self._write(REG_OUTPUT_0, self._out0)
        self._write(REG_OUTPUT_1, self._out1)

    # -------------------------
    # Input read
    # -------------------------
    def read_pin(self, pin):
        if pin < 8:
            return (self._read(REG_INPUT_0) >> pin) & 1
        else:
            return (self._read(REG_INPUT_1) >> (pin - 8)) & 1

    def read_all(self):
        return (
            self._read(REG_INPUT_0),
            self._read(REG_INPUT_1)
        )

    # -------------------------
    # Change detection (INT handling)
    # -------------------------
    def read_changes(self):
        """
        Returns:
          (changed_mask0, changed_mask1, new0, new1)
        """
        new0 = self._read(REG_INPUT_0)
        new1 = self._read(REG_INPUT_1)

        chg0 = new0 ^ self._last_in0
        chg1 = new1 ^ self._last_in1

        self._last_in0 = new0
        self._last_in1 = new1

        return chg0, chg1, new0, new1
