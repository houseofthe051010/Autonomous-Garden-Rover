import pathlib
import sys
import unittest


TOOLS = pathlib.Path(__file__).resolve().parents[1] / "tools"
sys.path.insert(0, str(TOOLS))

from hall_analysis import VALID_HALL_CYCLE, analyze_states  # noqa: E402


class HallAnalysisTests(unittest.TestCase):
    def assert_detects(self, mask, sequence):
        report = analyze_states(sequence)
        self.assertFalse(report["ambiguous"])
        self.assertEqual(report["candidate"]["mask"], mask)
        self.assertEqual(report["candidate"]["jumps"], 0)
        self.assertEqual(len(report["candidate"]["coverage"]), 6)

    def test_normal_forward_and_reverse(self):
        sequence = list(VALID_HALL_CYCLE) + list(reversed(VALID_HALL_CYCLE))
        self.assert_detects(0b000, sequence)

    def test_each_single_channel_inversion(self):
        for mask in (0b001, 0b010, 0b100):
            with self.subTest(mask=mask):
                raw = [state ^ mask for state in VALID_HALL_CYCLE]
                self.assert_detects(mask, raw + list(reversed(raw)))

    def test_every_start_position_and_direction(self):
        for mask in (0b000, 0b001, 0b010, 0b100):
            for reverse in (False, True):
                cycle = list(reversed(VALID_HALL_CYCLE)) if reverse else list(VALID_HALL_CYCLE)
                for shift in range(6):
                    with self.subTest(mask=mask, reverse=reverse, shift=shift):
                        rotated = cycle[shift:] + cycle[:shift]
                        raw = []
                        for state in rotated * 3:
                            raw.extend([state ^ mask] * 3)
                        self.assert_detects(mask, raw)

    def test_duplicate_samples_are_ignored(self):
        raw = []
        for state in VALID_HALL_CYCLE:
            raw.extend([state ^ 0b100] * 5)
        self.assert_detects(0b100, raw)

    def test_partial_rotation_is_not_accepted(self):
        report = analyze_states(VALID_HALL_CYCLE[:4])
        self.assertTrue(report["ambiguous"])
        self.assertIsNone(report["candidate"])

    def test_skipped_transition_is_not_accepted(self):
        sequence = [1, 3, 6, 4, 5, 1, 3, 2, 6, 4, 5]
        report = analyze_states(sequence)
        self.assertTrue(report["ambiguous"])
        self.assertIsNone(report["candidate"])


if __name__ == "__main__":
    unittest.main()
