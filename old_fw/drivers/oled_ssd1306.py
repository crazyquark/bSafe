# drivers/oled_ssd1306.py
#
# Thin wrapper around MicroPython's ssd1306 module.
# Keeps UI code decoupled from raw display driver details.

from machine import Pin, I2C

class OLED_SSD1306:
    def __init__(self, i2c: I2C, addr: int = 0x3C, width: int = 128, height: int = 64):
        self.i2c = i2c
        self.addr = addr
        self.width = width
        self.height = height

        try:
            import ssd1306
        except ImportError:
            raise ImportError("Missing MicroPython 'ssd1306' module. Add ssd1306.py to filesystem/firmware.")

        self._drv = ssd1306.SSD1306_I2C(width, height, i2c, addr=addr)

    @property
    def fb(self):
        # expose framebuffer-like API
        return self._drv

    def clear(self):
        self._drv.fill(0)

    def show(self):
        self._drv.show()

    def text(self, s: str, x: int, y: int, color: int = 1):
        self._drv.text(s, x, y, color)

    def fill_rect(self, x, y, w, h, color):
        self._drv.fill_rect(x, y, w, h, color)

    def rect(self, x, y, w, h, color):
        self._drv.rect(x, y, w, h, color)
