"""Who turns which way: rules of the air, negotiated pairs, one-sided fallback.

All maneuvers are horizontal turns. We never climb or descend, so we can
never cancel out a TCAS aircraft's vertical resolution advisory.
"""
import math

from .models import Track, wrap180
from .params import P

SIGN = {"RIGHT": 1.0, "LEFT": -1.0, "HOLD": 0.0}


def relative_bearing(own: Track, other: Track) -> float:
    """Bearing of `other` from `own`, relative to own nose (-180..180, + = right)."""
    bearing = math.degrees(math.atan2(other.x - own.x, other.y - own.y))
    return wrap180(bearing - own.heading)


def classify(own: Track, other: Track) -> str:
    rb = relative_bearing(own, other)
    hd = abs(wrap180(other.heading - own.heading))
    if hd > 135 and abs(rb) < 60:
        return "HEAD_ON"
    if hd < 45:
        if abs(rb) < 90 and own.speed > other.speed:
            return "OVERTAKING"
        if abs(rb) >= 90 and other.speed > own.speed:
            return "BEING_OVERTAKEN"
    return "CONVERGING_RIGHT" if rb > 0 else "CONVERGING_LEFT"


# geometry -> (our action, their action, rule text)
RULES = {
    "HEAD_ON":          ("RIGHT", "RIGHT", "head-on: both turn right"),
    "OVERTAKING":       ("RIGHT", "HOLD",  "overtaking: overtaker passes on the right"),
    "BEING_OVERTAKEN":  ("HOLD",  "RIGHT", "being overtaken: overtaker passes on the right"),
    "CONVERGING_RIGHT": ("RIGHT", "HOLD",  "converging, traffic on our right: we give way"),
    "CONVERGING_LEFT":  ("HOLD",  "RIGHT", "converging, traffic on our left: they give way"),
}


def rule_plan(own: Track, other: Track):
    """Returns (geometry, our_action, their_action, reason)."""
    g = classify(own, other)
    ours, theirs, why = RULES[g]
    return g, ours, theirs, why


def _path(tr: Track, action, horizon, step, delay):
    """Predicted (x, y, alt) points if `tr` flies `action` after `delay` seconds."""
    sign = SIGN[action]
    x, y, h, dev = tr.x, tr.y, tr.heading, 0.0
    pts = []
    for k in range(int(horizon / step) + 1):
        t = k * step
        pts.append((x, y, tr.alt + tr.vz * t))
        if sign and t >= delay and dev < P.MAX_DEV:
            h += sign * P.TURN_RATE * step
            dev += P.TURN_RATE * step
        x += tr.speed * math.sin(math.radians(h)) * step
        y += tr.speed * math.cos(math.radians(h)) * step
    return pts


def _min_sep(p1, p2):
    best = float("inf")
    for (x1, y1, z1), (x2, y2, z2) in zip(p1, p2):
        if abs(z2 - z1) < P.V_SEP:
            best = min(best, math.hypot(x2 - x1, y2 - y1))
    return best


def min_separation_if(own: Track, others, action, horizon=40.0, step=0.5):
    """Smallest horizontal distance to any threat if we fly `action` and
    everyone else flies straight."""
    mine = _path(own, action, horizon, step, P.PILOT_DELAY)
    return min((_min_sep(mine, _path(o, "HOLD", horizon, step, 0)) for o in others),
               default=float("inf"))


def plan_pair(own: Track, other: Track):
    """Negotiated flight paths for two equipped aircraft.

    1. Start from the rules of the air (pilots expect them).
    2. If that pair does not reach 1.2x the protected radius in prediction,
       search all pairs of maneuvers and take the best one.
    Returns (our_action, their_action, reason, predicted_separation).
    """
    geo, ours, theirs, why = rule_plan(own, other)
    h, s, d = 40.0, 0.5, P.PILOT_DELAY

    def sep(a, b):
        return _min_sep(_path(own, a, h, s, d), _path(other, b, h, s, d))

    rule_sep = sep(ours, theirs)
    if rule_sep >= 1.2 * P.H_SEP:
        return ours, theirs, why, rule_sep
    best = (rule_sep, ours, theirs)
    for a in ("RIGHT", "LEFT", "HOLD"):
        for b in ("RIGHT", "LEFT", "HOLD"):
            if (a, b) == ("HOLD", "HOLD"):
                continue
            v = sep(a, b)
            if v > best[0] * 1.1:
                best = (v, a, b)
    v, a, b = best
    if (a, b) == (ours, theirs):
        return ours, theirs, why, v
    return a, b, f"{why} gives only {rule_sep:.0f} m -> optimized pair", v


def unilateral_choice(own: Track, others, preferred="RIGHT", h_sep=None):
    """A turn that is safe even if nobody else maneuvers. Never 'HOLD'."""
    h_sep = h_sep or P.H_SEP
    if preferred == "HOLD":
        preferred = "RIGHT"
    alt = "LEFT" if preferred == "RIGHT" else "RIGHT"
    s_pref = min_separation_if(own, others, preferred)
    if s_pref >= h_sep:
        return preferred, s_pref
    s_alt = min_separation_if(own, others, alt)
    return (alt, s_alt) if s_alt > s_pref * 1.2 else (preferred, s_pref)


def return_path_clear(own: Track, course, others, horizon=60.0, step=0.5):
    """True if turning back to `course` (at the standard rate) keeps every
    other aircraft outside its protected radius for the next `horizon` seconds."""
    from .cpa import margin_for
    x, y, h = own.x, own.y, own.heading
    paths = [(o, o.x, o.y) for o in others]
    for k in range(int(horizon / step) + 1):
        t = k * step
        for o, ox, oy in paths:
            if abs((o.alt + o.vz * t) - (own.alt + own.vz * t)) < P.V_SEP:
                if math.hypot(ox + o.vx * t - x, oy + o.vy * t - y) < P.H_SEP * margin_for(o) * 1.05:
                    return False
        err = wrap180(course - h)
        h += max(-P.TURN_RATE * step, min(P.TURN_RATE * step, err))
        x += own.speed * math.sin(math.radians(h)) * step
        y += own.speed * math.cos(math.radians(h)) * step
    return True
