# ui/navigator.py

import time
from ui import pages

class UINavigator:
    def __init__(self, rotate_ms=5000):
        self.selected_page = pages.PAGE_STATUS
        self.entered = False

        self.status_subpage = pages.SUB_STATE
        self._rotate_ms = rotate_ms
        self._last_rotate = time.ticks_ms()

    def _wrap_page(self, idx):
        return idx % len(pages.PAGES)

    def _wrap_status_subpage(self, idx):
        return idx % len(pages.STATUS_SUBPAGES)

    def on_buttons(self, *, menu_pressed=False, left_pressed=False, right_pressed=False, ok_pressed=False):
        """
        Call on edge events (pressed once).
        MENU:
          - if entered -> exit to page-selection
          - if not entered -> force selection to STATUS
        OK:
          - toggles entered state
        LEFT/RIGHT:
          - if not entered -> change page
          - if entered -> change subpage (STATUS only for now)
        """
        if menu_pressed:
            if self.entered:
                self.entered = False
            else:
                self.selected_page = pages.PAGE_STATUS

        if ok_pressed:
            self.entered = not self.entered
            # When entering STATUS, keep current subpage (no rotation while entered)
            # When exiting, rotation will resume if STATUS is selected
            self._last_rotate = time.ticks_ms()

        if not self.entered:
            # page-selection mode
            if left_pressed:
                self.selected_page = self._wrap_page(self.selected_page - 1)
            if right_pressed:
                self.selected_page = self._wrap_page(self.selected_page + 1)

            # Auto-rotate STATUS subpages only when STATUS is selected and not entered
            if self.selected_page == pages.PAGE_STATUS:
                self._maybe_rotate_status()

        else:
            # subpage-selection mode (currently only STATUS has subpages)
            if self.selected_page == pages.PAGE_STATUS:
                if left_pressed:
                    self.status_subpage = self._wrap_status_subpage(self.status_subpage - 1)
                if right_pressed:
                    self.status_subpage = self._wrap_status_subpage(self.status_subpage + 1)

    def _maybe_rotate_status(self):
        now = time.ticks_ms()
        if time.ticks_diff(now, self._last_rotate) >= self._rotate_ms:
            self._last_rotate = now
            self.status_subpage = (self.status_subpage + 1) % len(pages.STATUS_SUBPAGES)

    def snapshot_to_model(self, model):
        model.page_selected = self.selected_page
        model.page_entered = self.entered
        model.status_subpage = self.status_subpage
