"""Pure Hall-state validation used by the ODESC bring-up tool."""

VALID_HALL_CYCLE = (0b001, 0b011, 0b010, 0b110, 0b100, 0b101)
VALID_HALL_STATES = frozenset(VALID_HALL_CYCLE)
SINGLE_CHANNEL_MASKS = (0b000, 0b001, 0b010, 0b100)
MASK_NAMES = {
    0b000: "none",
    0b001: "Hall A",
    0b010: "Hall B",
    0b100: "Hall C (Z input)",
}


def collapse_repeats(states):
    """Remove consecutive duplicate samples without hiding transitions."""
    output = []
    for state in states:
        state = int(state)
        if state < 0 or state > 7:
            raise ValueError("Hall states must be integers from 0 through 7")
        if not output or output[-1] != state:
            output.append(state)
    return output


def analyze_mask(states, mask):
    raw = collapse_repeats(states)
    corrected = [state ^ mask for state in raw]
    invalid = [state for state in corrected if state not in VALID_HALL_STATES]
    valid = [state for state in corrected if state in VALID_HALL_STATES]
    positions = {state: index for index, state in enumerate(VALID_HALL_CYCLE)}

    forward = 0
    reverse = 0
    jumps = 0
    for previous, current in zip(valid, valid[1:]):
        delta = (positions[current] - positions[previous]) % 6
        if delta == 1:
            forward += 1
        elif delta == 5:
            reverse += 1
        elif delta != 0:
            jumps += 1

    return {
        "mask": mask,
        "name": MASK_NAMES[mask],
        "raw": raw,
        "corrected": corrected,
        "invalid": invalid,
        "coverage": sorted(set(valid)),
        "forward": forward,
        "reverse": reverse,
        "jumps": jumps,
    }


def analyze_states(states):
    """Find a unique no-inversion or single-channel inversion candidate."""
    states = list(states)
    reports = [analyze_mask(states, mask) for mask in SINGLE_CHANNEL_MASKS]
    candidates = [
        report
        for report in reports
        if not report["invalid"]
        and len(report["coverage"]) == 6
        and report["jumps"] == 0
    ]
    return {
        "samples": len(states),
        "raw_transitions": collapse_repeats(states),
        "reports": reports,
        "candidate": candidates[0] if len(candidates) == 1 else None,
        "ambiguous": len(candidates) != 1,
    }


def format_state(state):
    return "{:03b}".format(state)

