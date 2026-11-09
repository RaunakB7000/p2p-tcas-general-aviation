"""Every tunable number in one place.

The logic reads these values at run time (P.H_SEP, P.PILOT_DELAY, ...), so a
scenario file, the command line or the simulator keyboard can change them
without touching the code.

Each entry: name -> (default, step for the +/- keys, min, max, unit, description)
"""

SPEC = {
    # --- collision check (logic/cpa.py) -------------------------------------
    "H_SEP":           (500.0, 50.0, 100.0, 2000.0, "m",   "horizontal protected radius"),
    "V_SEP":           (150.0, 10.0, 30.0, 600.0,   "m",   "vertical protected half-height"),
    "TA_TIME":         (45.0, 5.0, 10.0, 120.0,     "s",   "traffic advisory: time to closest approach"),
    "RA_TIME":         (25.0, 5.0, 5.0, 90.0,       "s",   "resolution advisory: time to closest approach"),
    "TA_DMOD":         (2000.0, 250.0, 0.0, 8000.0, "m",   "traffic advisory: distance trigger (slow closure)"),
    "RA_DMOD":         (1200.0, 100.0, 0.0, 5000.0, "m",   "resolution advisory: distance trigger"),
    # --- maneuver (logic/negotiation.py) ------------------------------------
    "TURN_RATE":       (3.0, 0.5, 0.5, 10.0,        "deg/s", "avoidance turn rate (3 = standard rate)"),
    "MAX_DEV":         (90.0, 15.0, 15.0, 180.0,    "deg", "largest heading change allowed"),
    # --- pilot and automation (logic/unit.py) -------------------------------
    "PILOT_DELAY":     (2.5, 0.5, 0.0, 15.0,        "s",   "pilot reaction time to an advisory"),
    "AUTO_DELAY":      (5.0, 0.5, 1.0, 30.0,        "s",   "auto-avoid engages if pilot hasn't turned"),
    "ACK_TIMEOUT":     (1.0, 0.25, 0.25, 5.0,       "s",   "proposer waits this long for ACK"),
    "PROPOSE_TIMEOUT": (1.5, 0.25, 0.25, 5.0,       "s",   "receiver waits this long for PROPOSE"),
    "TISB_MARGIN":     (1.6, 0.1, 1.0, 3.0,         "x",   "protected radius multiplier for radar (TIS-B) tracks"),
    # --- simulated world (sim/world.py) -------------------------------------
    "MSG_LOSS":        (0.0, 0.1, 0.0, 1.0,         "",    "chance each P2P radio message is lost (0-1)"),
    "RADIO_RANGE":     (8000.0, 1000.0, 1000.0, 30000.0, "m", "P2P radio range"),
    "GPS_NOISE":       (5.0, 5.0, 0.0, 200.0,       "m",   "GPS position error in broadcasts"),
    "TISB_NOISE":      (75.0, 25.0, 0.0, 500.0,     "m",   "radar (TIS-B) position error"),
    "TISB_PERIOD":     (5.0, 1.0, 1.0, 15.0,        "s",   "radar (TIS-B) update interval"),
}

# Shown in the simulator's parameter panel, in this order
UI_ORDER = ["H_SEP", "V_SEP", "TA_TIME", "RA_TIME", "TURN_RATE", "PILOT_DELAY",
            "AUTO_DELAY", "MSG_LOSS", "GPS_NOISE", "TISB_NOISE"]


class Params:
    def __init__(self):
        self.reset()

    def reset(self):
        for k, spec in SPEC.items():
            setattr(self, k, spec[0])

    def update(self, values: dict):
        for k, v in (values or {}).items():
            if k not in SPEC:
                raise KeyError(f"unknown parameter '{k}'. Known: {', '.join(SPEC)}")
            lo, hi = SPEC[k][2], SPEC[k][3]
            setattr(self, k, max(lo, min(hi, float(v))))

    def nudge(self, k, direction):
        default, step, lo, hi = SPEC[k][:4]
        v = round(getattr(self, k) + direction * step, 6)
        setattr(self, k, max(lo, min(hi, v)))

    def changed(self):
        """Parameters that differ from their defaults."""
        return {k: getattr(self, k) for k, s in SPEC.items() if getattr(self, k) != s[0]}

    def as_dict(self):
        return {k: getattr(self, k) for k in SPEC}


P = Params()
