"""One onboard unit: the logic that will run on each ESP32.

Each tick it:
  1. reads P2P radio messages (STATE / PROPOSE / ACK) and external tracks
     (ADS-B, TIS-B radar rebroadcast) plus passive transponder "proximity" hits
  2. keeps one track per aircraft, drops stale ones
  3. runs the collision check on every track
  4. raises TA / RA, negotiates a maneuver, or falls back to one-sided avoidance
  5. tells the pilot what to do, and takes over (auto-avoid) if ignored
"""
from dataclasses import dataclass, replace
from typing import Optional

from .models import Track, FT
from .cpa import assess, clock_position
from .negotiation import rule_plan, plan_pair, unilateral_choice, return_path_clear
from .params import P

MSG_MAX_AGE = 2.0         # s   ignore messages older than this
STATE_PERIOD = 1.0        # s   normal broadcast rate
STATE_PERIOD_ALERT = 0.5  # s   broadcast rate while an alert is active
STALE = {"unit": 3.0, "adsb": 3.0, "tisb": 12.0, "timing": 3.0}   # s  drop tracks not heard for this long
PROX_RANGE = 2 * 1609.34  # m   passive transponder alert range (estimated)
PROX_DV = 300 * FT        # m   ...and within this altitude band


@dataclass
class Advisory:
    threat_id: str
    action: str              # RIGHT / LEFT / HOLD
    mode: str                # NEGOTIATING / PROPOSED / NEGOTIATED / UNILATERAL
    reason: str
    issued_at: float
    deadline: float = 0.0
    done_turning: bool = False
    deviation: float = 0.0   # deg turned so far
    auto: bool = False       # unit has taken control


class Unit:
    def __init__(self, uid: str, log):
        self.id = uid
        self.log = log
        self.tracks: dict[str, Track] = {}
        self.advisory: Optional[Advisory] = None
        self.last_tx = -99.0
        self.top = None                      # most urgent Assessment
        self.status = "CLEAR"
        self.message = ""
        self.pilot_turning = False
        self.proximity = []                  # passive hits: (id, est_range, dv)
        self.cleared = {}                    # threat id -> time it was declared clear (hysteresis)
        self._prox_logged = set()

    # ---------------------------------------------------------------- radio
    def receive(self, msg, now, own: Track, outbox):
        if msg["from"] == self.id or now - msg["t"] > MSG_MAX_AGE:
            return
        kind = msg["type"]
        if kind == "STATE":
            self.tracks[msg["from"]] = replace(msg["track"], source="unit")
        elif kind == "PROPOSE" and msg["to"] == self.id:
            adv = self.advisory
            if adv and adv.threat_id == msg["from"] and adv.mode == "UNILATERAL":
                if adv.action == msg["your_action"]:
                    outbox.append({"type": "ACK", "from": self.id, "to": msg["from"], "t": now})
                return
            self.advisory = Advisory(msg["from"], msg["your_action"], "NEGOTIATED",
                                     msg["reason"], now)
            outbox.append({"type": "ACK", "from": self.id, "to": msg["from"], "t": now})
            self.log(f"{self.id}: PROPOSE from {msg['from']} accepted -> {msg['your_action']}; ACK sent")
        elif kind == "ACK" and msg["to"] == self.id:
            adv = self.advisory
            if adv and adv.threat_id == msg["from"] and adv.mode == "PROPOSED":
                adv.mode = "NEGOTIATED"
                self.log(f"{self.id}: ACK from {msg['from']} - maneuver agreed")

    # ---------------------------------------------------------------- main tick
    def tick(self, now, own: Track, external_tracks, outbox, proximity=()):
        for tr in external_tracks:
            if tr.id == self.id:
                continue
            existing = self.tracks.get(tr.id)
            # our own P2P link wins while it is fresh; ADS-B / radar fill in otherwise
            if existing is None or existing.source != "unit" or now - existing.t > STALE["unit"]:
                self.tracks[tr.id] = tr
        for k in [k for k, tr in self.tracks.items() if now - tr.t > STALE.get(tr.source, 3.0)]:
            del self.tracks[k]
        # passive transponder hits only matter for aircraft we can't otherwise locate
        self.proximity = [p for p in proximity if p[0] not in self.tracks]

        current = {k: tr.extrapolate(now) for k, tr in self.tracks.items()}
        results = {k: assess(own, tr) for k, tr in current.items()}
        rank = {"RA": 0, "TA": 1, "CLEAR": 2}
        top = min(results.values(), key=lambda a: (rank[a.level], a.t_cpa), default=None)
        self.top = top
        self.status = top.level if top else "CLEAR"

        self._update_advisory(now, own, current, results, outbox)

        close = [p for p in self.proximity if p[1] < PROX_RANGE and abs(p[2]) < PROX_DV]
        if self.status == "CLEAR" and close:
            self.status = "PROX"
            for pid, *_ in close:
                if pid not in self._prox_logged:
                    self._prox_logged.add(pid)
                    self.log(f"{self.id}: PROXIMITY alert, transponder {pid} nearby "
                             f"(altitude known, no position: cannot compute an avoidance)")

        period = STATE_PERIOD_ALERT if self.status != "CLEAR" or self.advisory else STATE_PERIOD
        if now - self.last_tx >= period:
            outbox.append({"type": "STATE", "from": self.id, "t": now,
                           "track": replace(own, source="unit")})
            self.last_tx = now
        self._make_message(own, current, close)

    def _update_advisory(self, now, own, current, results, outbox):
        adv = self.advisory
        if adv:
            a = results.get(adv.threat_id)
            if a is None or (a.level == "CLEAR" and not a.closing):
                self.log(f"{self.id}: clear of conflict with {adv.threat_id}")
                self.cleared[adv.threat_id] = now
                self.advisory = adv = None

        # hysteresis: right after "clear", the same aircraft must come well inside the radius again
        recent = self.top and now - self.cleared.get(self.top.other_id, -99) < 30 and \
            self.top.d_h > 0.85 * self.top.h_sep and self.top.range_now > self.top.h_sep
        if adv is None and self.top and self.top.level == "RA" and not recent:
            other = current[self.top.other_id]
            geo, ours, theirs, why = rule_plan(own, other)
            a = self.top
            info = f"CPA {a.d_h:.0f} m in {a.t_cpa:.0f} s"
            if other.source == "unit":
                if self.id < other.id:      # lower ID proposes
                    ours, theirs, why, sep = plan_pair(own, other)
                    why = f"{why} (predicted {sep:.0f} m)"
                    self.advisory = Advisory(other.id, ours, "PROPOSED", why, now, now + P.ACK_TIMEOUT)
                    outbox.append({"type": "PROPOSE", "from": self.id, "to": other.id, "t": now,
                                   "my_action": ours, "your_action": theirs, "reason": why})
                    self.log(f"{self.id}: RA vs {other.id} ({info}); {why}; PROPOSE me {ours} / you {theirs}")
                else:
                    self.advisory = Advisory(other.id, "HOLD", "NEGOTIATING", why, now,
                                             now + P.PROPOSE_TIMEOUT)
                    self.log(f"{self.id}: RA vs {other.id} ({info}); waiting for PROPOSE")
            else:
                if other.tcas:
                    kind = "TCAS aircraft: turn only, never climb/descend"
                elif other.source == "tisb":
                    kind = "radar (TIS-B) track: one-sided, wider margin"
                elif other.source == "timing":
                    kind = f"passively located (+/-{other.sigma:.0f} m): one-sided, margin {a.h_sep:.0f} m"
                else:
                    kind = "ADS-B-only traffic cannot negotiate"
                action, sep = unilateral_choice(own, [other], ours, a.h_sep)
                self.advisory = Advisory(other.id, action, "UNILATERAL", kind, now)
                self.log(f"{self.id}: RA vs {other.id} ({info}); {kind}; one-sided {action} "
                         f"(predicted {sep:.0f} m)")
            adv = self.advisory

        if adv is None:
            return

        if adv.mode in ("PROPOSED", "NEGOTIATING") and now > adv.deadline:
            other = current.get(adv.threat_id)
            if other is not None:
                action, _ = unilateral_choice(own, [other], adv.action)
            else:
                action = "RIGHT" if adv.action == "HOLD" else adv.action
            what = "no ACK" if adv.mode == "PROPOSED" else "no PROPOSE"
            self.log(f"{self.id}: {what} from {adv.threat_id} -> one-sided {action} (safe even if they do nothing)")
            if action != adv.action:
                adv.issued_at = now
            adv.action, adv.mode = action, "UNILATERAL"

        a = results.get(adv.threat_id)
        if adv.action in ("RIGHT", "LEFT") and not adv.done_turning:
            safe = a is None or a.d_h >= a.h_sep * 1.3 or abs(a.d_v) >= P.V_SEP * 1.3 or not a.closing
            if safe or adv.deviation >= P.MAX_DEV:
                adv.done_turning = True
                self.log(f"{self.id}: separation achieved, maintaining heading")

        if adv.action in ("RIGHT", "LEFT") and not adv.done_turning and not adv.auto \
                and not self.pilot_turning and now - adv.issued_at >= P.AUTO_DELAY:
            adv.auto = True
            self.log(f"{self.id}: pilot did not respond in {P.AUTO_DELAY:.1f} s - AUTO-AVOID engaged")

    # ---------------------------------------------------------------- outputs
    def may_return_to_course(self, own: Track, course=None):
        """Head back to the original course only if that path stays clear of everyone."""
        if self.advisory:
            return False
        others = [tr.extrapolate(own.t) for tr in self.tracks.values()]
        if course is not None:
            return return_path_clear(own, course, others)
        for tr in others:
            a = assess(own, tr)
            if a.range_now < 2 * a.h_sep and (a.closing or abs(a.dv_now) < P.V_SEP):
                return False
        return True

    def commanded_turn(self):
        adv = self.advisory
        if not adv or adv.done_turning or adv.action == "HOLD":
            return 0.0
        return P.TURN_RATE if adv.action == "RIGHT" else -P.TURN_RATE

    def _make_message(self, own, current, close):
        adv, top = self.advisory, self.top
        if adv:
            words = {"RIGHT": "TURN RIGHT", "LEFT": "TURN LEFT", "HOLD": "MAINTAIN HEADING"}
            if adv.mode == "NEGOTIATING":
                self.message = "RA: NEGOTIATING..."
            elif adv.done_turning:
                self.message = "RA: MAINTAIN HEADING"
            else:
                self.message = ("AUTO-AVOID: " if adv.auto else "RA: ") + words[adv.action]
        elif top and top.level == "TA":
            other = current[top.other_id]
            dv_ft = top.dv_now / FT
            alt = "same altitude" if abs(dv_ft) < 300 else (f"{abs(dv_ft):.0f} ft " +
                                                          ("above" if dv_ft > 0 else "below"))
            self.message = (f"TRAFFIC {clock_position(own, other)} o'clock, "
                            f"{top.range_now / 1609.34:.1f} mi, {alt}")
        elif close:
            pid, rng, dv = min(close, key=lambda p: p[1])
            self.message = f"PROXIMITY ~{rng / 1609.34:.1f} mi, {dv / FT:+.0f} ft, direction unknown"
        else:
            self.message = "Clear"
