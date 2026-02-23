# ui/renderer.py

import math

from ui import pages

CHAR_W = 8
TITLE_H = 12  # pixels

X_OFF = 2 * CHAR_W  # 2 characters -> 16px

class UIRenderer:
    def __init__(self, oled):
        self.oled = oled

    def _center_x(self, text, total_w=128):
        w = len(text) * CHAR_W
        x = (total_w - w) // 2
        return 0 if x < 0 else x

    def _draw_title_bar(self, page_name, entered):
        fb = self.oled.fb
        if not entered:
            fb.fill_rect(0, 0, 128, TITLE_H, 1)
            fb.text("<", 0, 2, 0)
            fb.text(">", 120, 2, 0)
            fb.text(page_name, self._center_x(page_name), 2, 0)
        else:
            fb.rect(0, 0, 128, TITLE_H, 1)
            fb.text(page_name, self._center_x(page_name), 2, 1)

    def render(self, model):
        fb = self.oled.fb
        fb.fill(0)

        page_name = pages.PAGES[model.page_selected]
        self._draw_title_bar(page_name, model.page_entered)

        if model.page_selected == pages.PAGE_STATUS:
            self._render_status(model)
        elif model.page_selected == pages.PAGE_CONFIG:
            self._render_config(model)
        elif model.page_selected == pages.PAGE_HEALTH:
            self._render_health(model)
        elif model.page_selected == pages.PAGE_MODE:
            self._render_mode(model)

        fb.show()

    # -------- STATUS --------
    def _render_status(self, m):
        fb = self.oled.fb

        # No subpage title line; content starts lower
        if m.status_subpage == pages.SUB_STATE:
            fb.text("{}".format(m.device_state_label), X_OFF, 18, 1)
            fb.text("TIME: {}s".format(int(m.status_seconds)), X_OFF, 32, 1)

        elif m.status_subpage == pages.SUB_INPUT:
            fb.text("VIN: {:.2f}V".format(m.input_voltage_v), X_OFF, 18, 1)
            fb.text("IIN: {:.2f}A".format(m.input_current_a), X_OFF, 32, 1)
            fb.text("MCU: {:.1f}C".format(m.mcu_temp_c), X_OFF, 46, 1)

        elif m.status_subpage == pages.SUB_BATTERY:
            fb.text("VBAT: {:.2f}V".format(m.batt_voltage_v), X_OFF, 18, 1)
            fb.text("IBAT: {:+.2f}A".format(m.batt_current_a), X_OFF, 32, 1)
            if m.batt_temp_c is None or (isinstance(m.batt_temp_c, float) and math.isnan(m.batt_temp_c)):
                fb.text("TBAT: N/A", X_OFF, 46, 1)
            else:
                fb.text("TBAT: {:.1f}C".format(m.batt_temp_c), X_OFF, 46, 1)

    # -------- CONFIG --------
    def _render_config(self, m):
        fb = self.oled.fb
        y0 = 16
        fb.text("CHEM: {}".format(m.cfg_chemistry), X_OFF, y0, 1)
        fb.text("dV: {}mV".format(m.cfg_voltage_offset_mv), X_OFF, y0+12, 1)
        fb.text("CAP: {}mAh".format(m.cfg_capacity_mah), X_OFF, y0+24, 1)
        fb.text("P: {} WAIT: {}".format(m.cfg_parallel_cells, m.cfg_wait_mode), X_OFF, y0+36, 1)

    # -------- HEALTH --------
    def _render_health(self, m):
        fb = self.oled.fb
        y0 = 18
        fb.text("Rchg: {}m".format(m.health_r_charge_mohm), X_OFF, y0, 1)
        fb.text("Rdis: {}m".format(m.health_r_discharge_mohm), X_OFF, y0+14, 1)
        fb.text("updated in tests", X_OFF, y0+32, 1)

    # -------- MODE --------
    def _render_mode(self, m):
        fb = self.oled.fb
        y0 = 16
        fb.text("CAN: {}".format("ON" if m.mode_can else "OFF"), X_OFF, y0, 1)
        fb.text("WIFI: {}".format("ON" if m.mode_wifi else "OFF"), X_OFF, y0+12, 1)
        fb.text("USB: {}".format("ON" if m.mode_usb else "OFF"), X_OFF, y0+24, 1)
        fb.text("LOCK: {}".format("YES" if m.lock_settings_on_net else "NO"), X_OFF, y0+36, 1)
