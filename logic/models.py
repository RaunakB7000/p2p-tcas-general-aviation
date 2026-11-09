"""Shared data types and constants.

Coordinates: local flat-earth frame in metres. x = east, y = north.
Heading: degrees, 0 = north, clockwise (90 = east).
"""
from dataclasses import dataclass, replace
import math

KT = 0.514444      # knots -> m/s
FT = 0.3048        # feet -> m
FPM = 0.00508      # feet per minute -> m/s


def wrap180(a):
    """Wrap an angle in degrees to [-180, 180)."""
    return (a + 180.0) % 360.0 - 180.0


@dataclass
class Track:
    """What a unit knows about one aircraft (its own or another)."""
    id: str
    x: float
    y: float
    alt: float               # m
    heading: float           # deg
    speed: float             # m/s
    vz: float = 0.0          # m/s, + = climbing
    turn_rate: float = 0.0   # deg/s, + = turning right
    source: str = "unit"     # "unit" (our LoRa link), "adsb", "own"
    tcas: bool = False       # other aircraft is TCAS-equipped
    t: float = 0.0           # time the report was made (s)
    target_heading: float = None   # intent: heading the current turn will stop at
    sigma: float = 0.0             # position uncertainty (m), for passively located tracks

    @property
    def vx(self):
        return self.speed * math.sin(math.radians(self.heading))

    @property
    def vy(self):
        return self.speed * math.cos(math.radians(self.heading))

    def extrapolate(self, now):
        """Move the report forward in time to `now` (straight line)."""
        dt = now - self.t
        if dt <= 0:
            return self
        return replace(self, x=self.x + self.vx * dt, y=self.y + self.vy * dt,
                       alt=self.alt + self.vz * dt, t=now)
