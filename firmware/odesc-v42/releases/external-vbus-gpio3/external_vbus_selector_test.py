#!/usr/bin/env python3
"""Behavioral model for the external VBUS selector in low_level.cpp."""

import math
import unittest


class Selector:
    def __init__(self):
        self.valid_samples = 0
        self.invalid_samples = 0
        self.accepted_once = False
        self.selected = 12.0
        self.valid = False
        self.fault = False
        self.status = 1

    def update(self, internal, external_adc, gpio_analog=True):
        external = external_adc * 18.702757793
        status = 0
        if not gpio_analog:
            status |= 2
        if not external >= 1.0:
            status |= 4
        if not external_adc < 3.25:
            status |= 8

        tolerance = max(1.0, 0.03 * internal)
        if external + tolerance < internal:
            status |= 16
        if internal < 34.0 and external > internal + tolerance:
            status |= 16

        if status == 0:
            self.invalid_samples = 0
            self.valid_samples = min(self.valid_samples + 1, 32)
            if self.valid_samples >= 32:
                self.accepted_once = True
                self.valid = True
                self.fault = False
                self.status = 0
                self.selected = external
            else:
                self.valid = False
                self.status = 1
                self.selected = internal
        else:
            self.valid_samples = 0
            self.invalid_samples = min(self.invalid_samples + 1, 64)
            self.valid = False
            self.status = status
            if self.invalid_samples >= 64:
                self.fault = True
                self.selected = math.nan
            elif not self.accepted_once and not self.fault:
                self.selected = internal
        return self.selected


def settle(selector, internal, external_adc, count=32, gpio_analog=True):
    for _ in range(count):
        selector.update(internal, external_adc, gpio_analog)


class ExternalVbusSelectorTest(unittest.TestCase):
    def test_nominal_source_is_selected_after_warmup(self):
        selector = Selector()
        settle(selector, 36.26, 36.2 / 18.702757793)
        self.assertTrue(selector.valid)
        self.assertFalse(selector.fault)
        self.assertAlmostEqual(selector.selected, 36.2, places=3)

    def test_42v_is_accepted_when_internal_adc_is_clipped(self):
        selector = Selector()
        settle(selector, 36.29, 42.0 / 18.702757793)
        self.assertTrue(selector.valid)
        self.assertAlmostEqual(selector.selected, 42.0, places=3)

    def test_disconnected_source_disarms_after_debounce(self):
        selector = Selector()
        settle(selector, 36.26, 36.2 / 18.702757793)
        settle(selector, 36.26, 0.0, count=63)
        self.assertFalse(selector.fault)
        selector.update(36.26, 0.0)
        self.assertTrue(selector.fault)
        self.assertTrue(math.isnan(selector.selected))

    def test_dangerous_external_under_read_is_rejected(self):
        selector = Selector()
        settle(selector, 36.29, 30.0 / 18.702757793, count=64)
        self.assertTrue(selector.fault)
        self.assertEqual(selector.status & 16, 16)

    def test_more_than_one_volt_under_clipped_internal_is_rejected(self):
        selector = Selector()
        settle(selector, 36.29, 35.0 / 18.702757793, count=64)
        self.assertTrue(selector.fault)
        self.assertEqual(selector.status & 16, 16)

    def test_wrong_gpio_mode_is_rejected(self):
        selector = Selector()
        settle(selector, 36.26, 36.2 / 18.702757793, count=64,
               gpio_analog=False)
        self.assertTrue(selector.fault)
        self.assertEqual(selector.status & 2, 2)


if __name__ == "__main__":
    unittest.main()
