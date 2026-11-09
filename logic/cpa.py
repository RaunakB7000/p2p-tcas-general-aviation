"""Collision check: closest point of approach (CPA) and alert level.

This is the code that should be ported 1:1 to the ESP32 firmware.
Thresholds live in logic/params.py (P.H_SEP, P.RA_TIME, ...).
"""
from dataclasses import dataclass
import math

from .models import Track, wrap180
from .params import P

TURNING = 0.5      # deg/s  above this, the other aircraft counts as turning
HORIZON = 60.0     # s      how far ahead curved paths are predicted


@dataclass
class Assessment:
    other_id: str
    t_cpa: float        # s until closest approach (0 if diverging)
    d_h: float          # m horizontal miss distance at CPA
    d_v: float          # m vertical separation at CPA (other minus own)
    range_now: float    # m current horizontal distance
    dv_now: float       # m current vertical separation
    closing: bool       # range currently decreasing
    level: str          # "CLEAR", "TA" or "RA"
    h_sep: float        # protected radius used (bigger for uncertain tracks)


def straight_cpa(own: Track, other: Track):
    """Closest approach assuming both fly straight. Returns (t, d_h, d_v)."""
    rx, ry = other.x - own.x, other.y - own.y
    vx, vy = other.vx - own.vx, other.vy - own.vy
    v2 = vx * vx + vy * vy
    t = 0.0 if v2 < 1e-9 else max(0.0, -(rx * vx + ry * vy) / v2)
    d_h = math.hypot(rx + vx * t, ry + vy * t)
    d_v = (other.alt + other.vz * t) - (own.alt + own.vz * t)
    return t, d_h, d_v


def curved_cpa(own: Track, other: Track, horizon=HORIZON, step=0.5):
    """Closest approach when the OTHER aircraft keeps its current turn rate
    (stopping at its announced target heading, if it sent one)."""
    best = (0.0, float("inf"), 0.0)
    x, y, h = other.x, other.y, other.heading
    for k in range(int(horizon / step) + 1):
        t = k * step
        ox, oy = own.x + own.vx * t, own.y + own.vy * t
        d_h = math.hypot(x - ox, y - oy)
        if d_h < best[1]:
            best = (t, d_h, (other.alt + other.vz * t) - (own.alt + own.vz * t))
        if other.target_heading is not None and \
                abs(wrap180(other.target_heading - h)) <= abs(other.turn_rate) * step:
            h = other.target_heading
        else:
            h += other.turn_rate * step
        x += other.speed * math.sin(math.radians(h)) * step
        y += other.speed * math.cos(math.radians(h)) * step
    return best


def margin_for(track: Track) -> float:
    """Uncertain tracks get a bigger protected radius."""
    if track.source == "tisb":
        return P.TISB_MARGIN
    if track.source == "timing":                       # passively located: radius + 2 sigma, capped
        return min(3.0, max(P.TISB_MARGIN, 1.0 + 2.0 * track.sigma / P.H_SEP))
    return 1.0


def assess(own: Track, other: Track) -> Assessment:
    h_sep = P.H_SEP * margin_for(other)
    t, d_h, d_v = straight_cpa(own, other)
    if abs(other.turn_rate) > TURNING:
        tc, dhc, dvc = curved_cpa(own, other)
        if dhc < d_h:
            t, d_h, d_v = tc, dhc, dvc

    rx, ry = other.x - own.x, other.y - own.y
    range_now = math.hypot(rx, ry)
    dv_now = other.alt - own.alt
    closing = (rx * (other.vx - own.vx) + ry * (other.vy - own.vy)) < 0

    conflict = d_h < h_sep and abs(d_v) < P.V_SEP
    # "already inside" uses at most 1.5x the normal radius: uncertainty alone shouldn't
    # make a distant aircraft count as inside the protected zone
    inside = range_now < min(h_sep, 1.5 * P.H_SEP) and abs(dv_now) < P.V_SEP
    # distance triggers (TCAS "DMOD") only count while the aircraft are closing
    if inside or (conflict and (t < P.RA_TIME or (closing and range_now < P.RA_DMOD))):
        level = "RA"
    elif conflict and (t < P.TA_TIME or (closing and range_now < P.TA_DMOD)):
        level = "TA"
    else:
        level = "CLEAR"
    return Assessment(other.id, t, d_h, d_v, range_now, dv_now, closing, level, h_sep)


def clock_position(own: Track, other: Track) -> int:
    """Where the other aircraft is, as a clock position (12 = straight ahead)."""
    bearing = math.degrees(math.atan2(other.x - own.x, other.y - own.y))
    c = round((wrap180(bearing - own.heading) % 360) / 30.0) % 12
    return 12 if c == 0 else c
