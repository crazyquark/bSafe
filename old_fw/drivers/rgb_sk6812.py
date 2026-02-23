# drivers/rgb_sk6812.py
#
# SK6812 LED strip driver (NeoPixel-compatible).
# Works with MicroPython 'neopixel' module.

from machine import Pin

class RGBStrip:
    def __init__(self, pin, count=10, bpp=3):
        self.count = int(count)
        self.pin = Pin(pin, Pin.OUT)
        self._np = None

        try:
            import neopixel
            self._np = neopixel.NeoPixel(self.pin, self.count, bpp=bpp)
        except ImportError:
            raise ImportError("Missing MicroPython 'neopixel' module. Add neopixel.py or use firmware with it included.")

        self.clear()

    def clear(self):
        for i in range(self.count):
            self._np[i] = (0, 0, 0)
        self._np.write()

    def fill(self, rgb):
        r, g, b = rgb
        for i in range(self.count):
            self._np[i] = (int(r), int(g), int(b))
        self._np.write()

    def set_pixel(self, i, rgb):
        if 0 <= i < self.count:
            self._np[i] = (int(rgb[0]), int(rgb[1]), int(rgb[2]))

    def show(self):
        self._np.write()

    def set_status_color(self, rgb):
        """
        Convenience alias: fill entire strip with status color.
        """
        self.fill(rgb)
