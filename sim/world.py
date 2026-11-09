"""Simulated sky: aircraft, what each one carries, the radio link, a simple
airliner TCAS, and loading scenarios from JSON files.

No graphics here, so the same world runs in the pygame window and in the
headless test runner.
"""
import glob
import json
import math
import os
import random

from logic.models import Track, KT, FT, FPM, wrap180
from logic.cpa import straight_cpa
from logic.unit import Unit
from logic.params import P

RADIO_LATENCY = 0.05     # s
TCAS_RA_TIME = 25.0      # s   the airliner's own TCAS
TCAS_H, TCAS_V = 600.0, 200.0
TCAS_VZ = 1500 * FPM     # m/s climb/descent on a TCAS RA
PROX_MAX = 10000.0       # m   passive transponder replies heard out to here
REPORT_V = 150.0         # m   separation report counts pairs within this vertical band

EQUIPMENT = {
    # equipment: (has our unit, ADS-B Out default, transponder default, label)
    "device":      (True,  False, True,  "our device"),
    "adsb":        (False, True,  True,  "ADS-B Out only"),
    "tcas":        (False, True,  True,  "airliner: TCAS + ADS-B Out"),
    "transponder": (False, False, True,  "transponder only (Mode C)"),
    "none":        (False, False, False, "blind: no electronics"),
}
SCEN_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "scenarios")


class Aircraft:
    def __init__(self, aid, x, y, alt, heading, speed, vz=0.0, equipment="device",
                 adsb_out=None, transponder=None, name=None, pilot_ignores=False):
        has_unit, adsb_default, xpdr_default, _ = EQUIPMENT[equipment]
        self.id, self.name = aid, name or aid
        self.x, self.y, self.alt = x, y, alt
        self.heading = self.course = heading % 360
        self.speed, self.vz = speed, vz
        self.turn_rate = 0.0
        self.equipment = equipment
        self.adsb_out = adsb_default if adsb_out is None else adsb_out
        self.transponder = xpdr_default if transponder is None else transponder
        self.has_unit = has_unit
        self.pilot_ignores = pilot_ignores
        self.unit = None
        self.tcas_ra = None
        self.trail = []

    def track(self, now, source="own", noise=0.0, rng=None):
        returning = self.turn_rate and self.unit and self.unit.advisory is None
        nx = rng.gauss(0, noise) if rng and noise else 0.0
        ny = rng.gauss(0, noise) if rng and noise else 0.0
        return Track(self.id, self.x + nx, self.y + ny, self.alt, self.heading, self.speed,
                     self.vz, self.turn_rate, source, self.equipment == "tcas", now,
                     self.course if returning else None)

    def step(self, dt):
        self.heading = (self.heading + self.turn_rate * dt) % 360
        h = math.radians(self.heading)
        self.x += self.speed * math.sin(h) * dt
        self.y += self.speed * math.cos(h) * dt
        self.alt += self.vz * dt


class World:
    def __init__(self, aircraft, lose_negotiation=False, lora_down=False, system_on=True,
                 seed=1, duration=150.0):
        self.t = 0.0
        self.aircraft = {a.id: a for a in aircraft}
        self.lose_negotiation = lose_negotiation
        self.lora_down = lora_down
        self.system_on = system_on
        self.duration = duration
        self.rng = random.Random(seed)
        self.events = []
        self.queue = []
        self.min_sep = {}            # (p, q) -> (horizontal, vertical) closest while in the same band
        self.tisb_last = {}          # (receiver, target) -> time of last radar update
        self.prox_factor = {}        # (receiver, target) -> range error factor
        for a in aircraft:
            if a.has_unit:
                a.unit = Unit(a.id, self.log)

    def log(self, text):
        self.events.append((self.t, text))

    # ------------------------------------------------------------------ step
    def step(self, dt=0.1):
        now = self.t
        units = [a for a in self.aircraft.values() if a.unit] if self.system_on else []

        due = [m for m in self.queue if m[0] <= now]
        self.queue = [m for m in self.queue if m[0] > now]
        for _, msg in due:
            src = self.aircraft[msg["from"]]
            for a in units:
                if a.id != src.id and math.hypot(a.x - src.x, a.y - src.y) <= P.RADIO_RANGE:
                    outbox = []
                    a.unit.receive(msg, now, a.track(now), outbox)
                    self._send(outbox)

        for a in units:
            external, proximity = self._sensed_by(a, now)
            outbox = []
            a.unit.tick(now, a.track(now), external, outbox, proximity)
            self._send(outbox)
            self._fly(a, now, dt)

        for a in self.aircraft.values():
            if a.equipment == "tcas":
                self._airliner_tcas(a)

        for a in self.aircraft.values():
            a.step(dt)
            if int(round(now * 10)) % 10 == 0:
                a.trail.append((a.x, a.y))
        self._record_separation()
        self.t = round(self.t + dt, 6)

    def _sensed_by(self, a, now):
        """What aircraft `a`'s unit can sense of everyone else, given equipment."""
        external, proximity = [], []
        for o in self.aircraft.values():
            if o is a:
                continue
            if o.adsb_out:                                   # ADS-B In: exact, 2x per second
                external.append(o.track(now, "adsb", P.GPS_NOISE, self.rng))
            elif o.transponder and a.adsb_out:               # TIS-B: radar rebroadcast
                key = (a.id, o.id)
                if now - self.tisb_last.get(key, -99) >= P.TISB_PERIOD:
                    self.tisb_last[key] = now
                    tr = o.track(now, "tisb", P.TISB_NOISE, self.rng)
                    tr.heading = (tr.heading + self.rng.gauss(0, 4)) % 360
                    tr.speed *= 1 + self.rng.gauss(0, 0.05)
                    tr.turn_rate = 0.0
                    external.append(tr)
            elif o.transponder:                              # passive: altitude + rough range only
                rng_m = math.hypot(o.x - a.x, o.y - a.y)
                if rng_m < PROX_MAX:
                    f = self.prox_factor.setdefault((a.id, o.id), 1 + self.rng.uniform(-0.3, 0.3))
                    proximity.append((o.id, rng_m * f, o.alt - a.alt))
        return external, proximity

    def _send(self, outbox):
        for msg in outbox:
            if self.lora_down:
                continue
            if self.lose_negotiation and msg["type"] in ("PROPOSE", "ACK"):
                self.log(f"radio: {msg['type']} from {msg['from']} LOST")
                continue
            if P.MSG_LOSS and self.rng.random() < P.MSG_LOSS:
                if msg["type"] != "STATE":
                    self.log(f"radio: {msg['type']} from {msg['from']} LOST (random loss)")
                continue
            if msg["type"] == "STATE" and P.GPS_NOISE:
                tr = msg["track"]
                msg = dict(msg, track=Track(tr.id, tr.x + self.rng.gauss(0, P.GPS_NOISE),
                                            tr.y + self.rng.gauss(0, P.GPS_NOISE), tr.alt,
                                            tr.heading, tr.speed, tr.vz, tr.turn_rate, tr.source,
                                            tr.tcas, tr.t, tr.target_heading))
            self.queue.append((self.t + RADIO_LATENCY, msg))

    def _fly(self, a, now, dt):
        unit, adv = a.unit, a.unit.advisory
        cmd = unit.commanded_turn()
        if cmd:
            pilot = (not a.pilot_ignores) and now - adv.issued_at >= P.PILOT_DELAY
            unit.pilot_turning = pilot
            a.turn_rate = cmd if (pilot or adv.auto) else 0.0
            adv.deviation += abs(a.turn_rate) * dt
        elif adv is None and abs(wrap180(a.course - a.heading)) > 1.0 \
                and (a.turn_rate or unit.may_return_to_course(a.track(now), a.course)):
            err = wrap180(a.course - a.heading)
            a.turn_rate = max(-P.TURN_RATE, min(P.TURN_RATE, err))
            unit.pilot_turning = False
        else:
            a.turn_rate = 0.0
            unit.pilot_turning = False

    def _airliner_tcas(self, a):
        own = a.track(self.t)
        if a.tcas_ra is None:
            for o in self.aircraft.values():
                if o is a or not (o.transponder or o.adsb_out):
                    continue                          # TCAS cannot see a blind aircraft
                t, d_h, d_v = straight_cpa(own, o.track(self.t))
                if d_h < TCAS_H and abs(d_v) < TCAS_V and t < TCAS_RA_TIME:
                    a.tcas_ra = ("CLIMB" if a.alt >= o.alt else "DESCEND", o.id)
                    a.vz = TCAS_VZ if a.tcas_ra[0] == "CLIMB" else -TCAS_VZ
                    self.log(f"{a.name}: TCAS RA {a.tcas_ra[0]} vs {o.id}")
                    break
        else:
            o = self.aircraft[a.tcas_ra[1]]
            rx, ry = o.x - a.x, o.y - a.y
            vx = o.speed * math.sin(math.radians(o.heading)) - a.speed * math.sin(math.radians(a.heading))
            vy = o.speed * math.cos(math.radians(o.heading)) - a.speed * math.cos(math.radians(a.heading))
            if rx * vx + ry * vy > 0:
                self.log(f"{a.name}: TCAS clear of conflict, level off")
                a.tcas_ra, a.vz = None, 0.0

    def _record_separation(self):
        ids = sorted(self.aircraft)
        for i, p in enumerate(ids):
            for q in ids[i + 1:]:
                a, b = self.aircraft[p], self.aircraft[q]
                dh, dv = math.hypot(a.x - b.x, a.y - b.y), abs(a.alt - b.alt)
                if dv < REPORT_V and dh < self.min_sep.get((p, q), (float("inf"), 0))[0]:
                    self.min_sep[(p, q)] = (dh, dv)


# ---------------------------------------------------------------------- scenarios
def build(spec, user_params=None, system_on=True, extra_ignores=(), lose_negotiation=None,
          lora_down=None):
    """Create a World from a scenario dict (the contents of a JSON file).

    Parameters are applied as: defaults -> scenario "params" -> user_params
    (keyboard / command line), so user changes always win.
    """
    P.reset()
    P.update(spec.get("params", {}))
    P.update(user_params or {})
    opts = spec.get("options", {})

    craft = spec["aircraft"]
    own = next(c for c in craft if c.get("id") == "U1")
    T = max([c.get("conflict_in_s", 70.0) for c in craft if c is not own] or [70.0])
    oh = math.radians(own.get("heading", 0.0))
    ov = own.get("speed_kt", 100.0) * KT
    ox0, oy0 = -math.sin(oh) * ov * T / 2, -math.cos(oh) * ov * T / 2
    own_at = lambda t: (ox0 + math.sin(oh) * ov * t, oy0 + math.cos(oh) * ov * t)
    own_alt = own.get("alt_ft", 5000.0) * FT
    own_vz = own.get("vz_fpm", 0.0) * FPM

    ignores = set(opts.get("pilot_ignores", [])) | set(extra_ignores)
    out = []
    for c in craft:
        kw = dict(equipment=c.get("equipment", "device"), adsb_out=c.get("adsb_out"),
                  transponder=c.get("transponder"), name=c.get("name"),
                  pilot_ignores=c["id"] in ignores or c.get("pilot_ignores", False))
        speed, hdg, vz = c.get("speed_kt", 100.0) * KT, c.get("heading", 0.0), c.get("vz_fpm", 0.0) * FPM
        if c is own:
            out.append(Aircraft("U1", ox0, oy0, own_alt, hdg, speed, own_vz, **kw))
            continue
        if "x_m" in c:                                       # explicit start position
            x, y, alt = c["x_m"], c["y_m"], c.get("alt_ft", 5000.0) * FT
        else:                                                # placed to meet U1 after conflict_in_s
            t = c.get("conflict_in_s", 70.0)
            px, py = own_at(t)
            h = math.radians(hdg)
            miss = c.get("miss_m", 0.0)                      # + = passes to its own right of U1
            px += math.cos(h) * miss
            py -= math.sin(h) * miss
            x, y = px - math.sin(h) * speed * t, py - math.cos(h) * speed * t
            alt = own_alt + own_vz * t + c.get("alt_diff_ft", 0.0) * FT - vz * t
        out.append(Aircraft(c["id"], x, y, alt, hdg, speed, vz, **kw))

    return World(out,
                 lose_negotiation=opts.get("lose_negotiation", False) if lose_negotiation is None else lose_negotiation,
                 lora_down=opts.get("lora_down", False) if lora_down is None else lora_down,
                 system_on=system_on, seed=spec.get("seed", 1),
                 duration=spec.get("duration_s", 150.0))


def load_scenarios(folder=SCEN_DIR):
    """All scenario files, sorted by file name. Returns [(path, spec)]."""
    out = []
    for path in sorted(glob.glob(os.path.join(folder, "*.json"))):
        with open(path, encoding="utf-8") as f:
            spec = json.load(f)
        spec.setdefault("name", os.path.basename(path))
        out.append((path, spec))
    return out


def random_spec(seed):
    """A random two-aircraft encounter: random equipment, geometry, speed, altitude."""
    r = random.Random(seed)
    equipment = r.choice(["device", "device", "adsb", "tcas", "transponder", "none"])
    own_adsb = r.random() < 0.5
    rel = r.uniform(45, 315)                                 # avoid pure same-direction pairs
    speed = r.uniform(200, 260) if equipment == "tcas" else r.uniform(80, 150)
    vz = r.choice([-800, 0, 800]) if equipment == "tcas" else r.choice([0, 0, -300, 300])
    other = {"id": "U2" if equipment == "device" else "N" + str(r.randint(100, 999)),
             "equipment": equipment, "heading": rel, "speed_kt": round(speed),
             "conflict_in_s": r.uniform(60, 80), "miss_m": round(r.uniform(-150, 150)),
             "alt_diff_ft": r.choice([0, 0, 100, -100]), "vz_fpm": vz}
    if equipment == "device":
        other["adsb_out"] = r.random() < 0.5
    return {
        "name": f"Random encounter #{seed}",
        "table_row": f"Device{' + ADS-B Out' if own_adsb else ''} vs {EQUIPMENT[equipment][3]}",
        "seed": seed,
        "aircraft": [{"id": "U1", "equipment": "device", "adsb_out": own_adsb,
                      "speed_kt": round(r.uniform(90, 130)), "alt_ft": 5000, "heading": 0}, other],
    }
