"""Locate a transponder-only aircraft without transmitting anything.

Measurements (all passive):
  "sum"   An interrogator (TCAS airliner, position from its ADS-B; or a ground
          radar, surveyed position) asks; the target answers; we time both.
          -> distance(interrogator, target) + distance(target, us) = z   (an ellipse)
  "az"    Ground radar beam direction when the target answered (a line from the radar)
  "range" Rough distance from signal strength (log-normal, about +/-40 %)
  "pos"   TIS-B position rebroadcast by ground stations (only if we have ADS-B Out)
  "tdoa"  Another of OUR units hears the same reply; both are GPS-timed, so
          distance(target, us) - distance(target, them) = c * (t_us - t_them)   (a hyperbola)
  "bearing" Direction antennas on our unit: angle of arrival (a line from us)
  Altitude comes straight from the Mode C reply.
  The math is written out in docs/MATH.md.

Estimator: a particle filter, the same predict/update loop as a Kalman filter,
but with a few thousand guesses instead of one bell curve, so it copes with the
curved ellipses and with the "two possible arcs" ambiguity until it resolves.
State per particle: [east m, north m, v_east m/s, v_north m/s].
"""
import math

import numpy as np

SIGMA_SUM = 120.0        # m   range-sum error (transponder delay tolerance + timing)
SIGMA_AZ = 1.5           # deg radar beam direction error
SIGMA_LOGR = 0.35        # signal-strength range error (log scale, about +/-40 %)
SIGMA_TDOA = 60.0        # m   distance-difference error between two GPS-timed units (~200 ns)
SIGMA_BRG = 15.0         # deg direction-antenna error (4-element array)
ACCEL = 1.0              # m/s^2  how much the target may maneuver between updates
FIX_SIGMA = 450.0        # m   "converged" when the position spread is below this


class PassiveLocator:
    def __init__(self, target_id, n=3000, seed=0):
        self.id = target_id
        self.n = n
        self.rng = np.random.default_rng(seed)
        self.p = None                 # particles, n x 4
        self.w = None                 # weights
        self.alt = None               # target altitude from Mode C (m)
        self.n_sum = self.n_az = self.n_pos = 0
        self.last_ellipse = []        # recent measurement geometry, for drawing
        self.curves = []              # recent ellipses / hyperbolas / bearings, for drawing
        self.n_tdoa = self.n_brg = 0
        self.sources = set()          # independent geometry sources seen, e.g. {"AAL12", "radar"}

    # ------------------------------------------------------------- helpers
    def _dist3(self, px, py, x, y, alt_b):
        dz = (self.alt if self.alt is not None else alt_b) - alt_b
        return np.sqrt((px - x) ** 2 + (py - y) ** 2 + dz ** 2)

    def _init_on_ellipse(self, ax, ay, a_alt, ux, uy, u_alt, z):
        """First measurement: spread guesses along that ellipse, any speed/direction."""
        c = math.hypot(ax - ux, ay - uy) / 2
        a = max(z / 2, c + 50)
        b = math.sqrt(max(a * a - c * c, 1.0))
        th = self.rng.uniform(0, 2 * math.pi, self.n)
        ang = math.atan2(ay - uy, ax - ux)
        cx, cy = (ax + ux) / 2, (ay + uy) / 2
        ex = a * np.cos(th) + self.rng.normal(0, 150, self.n)
        ey = b * np.sin(th) + self.rng.normal(0, 150, self.n)
        x = cx + ex * math.cos(ang) - ey * math.sin(ang)
        y = cy + ex * math.sin(ang) + ey * math.cos(ang)
        spd = self.rng.uniform(25, 85, self.n)
        hdg = self.rng.uniform(0, 2 * math.pi, self.n)
        self.p = np.column_stack([x, y, spd * np.sin(hdg), spd * np.cos(hdg)])
        self.w = np.full(self.n, 1.0 / self.n)

    def _weight(self, loglik):
        loglik = loglik - loglik.max()
        self.w = self.w * np.exp(loglik)
        s = self.w.sum()
        self.w = self.w / s if s > 0 and np.isfinite(s) else np.full(self.n, 1.0 / self.n)
        if 1.0 / np.sum(self.w ** 2) < self.n / 2:
            self._resample()

    def _resample(self):
        pos = (self.rng.random() + np.arange(self.n)) / self.n
        idx = np.minimum(np.searchsorted(np.cumsum(self.w), pos), self.n - 1)
        self.p = self.p[idx] + self.rng.normal(0, 1, (self.n, 4)) * np.array([25, 25, 2.0, 2.0])
        self.w = np.full(self.n, 1.0 / self.n)

    # ------------------------------------------------------------- loop
    def predict(self, dt):
        if self.p is None:
            return
        self.p[:, 0] += self.p[:, 2] * dt
        self.p[:, 1] += self.p[:, 3] * dt
        self.p[:, 2:] += self.rng.normal(0, ACCEL * math.sqrt(dt), (self.n, 2))

    def set_altitude(self, alt_m):
        self.alt = alt_m

    def update_sum(self, illum, us, z, t, kind):
        """illum/us = (x, y, alt). z = measured range sum (m)."""
        if self.p is None:
            self._init_on_ellipse(*illum, *us, z)
        h = self._dist3(self.p[:, 0], self.p[:, 1], illum[0], illum[1], illum[2]) + \
            self._dist3(self.p[:, 0], self.p[:, 1], us[0], us[1], us[2])
        self._weight(-0.5 * ((z - h) / SIGMA_SUM) ** 2)
        self.n_sum += 1
        self.sources.add(kind)
        self.last_ellipse.append((t, kind, illum[:2], us[:2], z))
        self.last_ellipse = self.last_ellipse[-6:]
        self._curve(dict(t=t, type="ellipse", kind=kind, f1=illum[:2], f2=us[:2], value=z))

    def _curve(self, c):
        self.curves.append(c)
        self.curves = self.curves[-12:]

    def update_tdoa(self, u, v, d, t, partner):
        """u = us, v = the other unit, both (x, y, alt). d = |X-u| - |X-v| (m)."""
        if self.p is None:
            return
        h = self._dist3(self.p[:, 0], self.p[:, 1], u[0], u[1], u[2]) - \
            self._dist3(self.p[:, 0], self.p[:, 1], v[0], v[1], v[2])
        self._weight(-0.5 * ((d - h) / SIGMA_TDOA) ** 2)
        self.n_tdoa += 1
        self.sources.add("tdoa:" + partner)
        self._curve(dict(t=t, type="hyperbola", kind=partner, f1=u[:2], f2=v[:2], value=d))

    def update_bearing(self, us_xy, brg_deg, t, sigma=SIGMA_BRG):
        if self.p is None:
            return
        b = np.degrees(np.arctan2(self.p[:, 0] - us_xy[0], self.p[:, 1] - us_xy[1]))
        d = (b - brg_deg + 180) % 360 - 180
        self._weight(-0.5 * (d / sigma) ** 2)
        self.n_brg += 1
        self.sources.add("bearing")
        self._curve(dict(t=t, type="bearing", kind="antennas", f1=us_xy, value=brg_deg, sigma=sigma))

    def update_az(self, radar_xy, az_deg, t):
        if self.p is None:
            return
        brg = np.degrees(np.arctan2(self.p[:, 0] - radar_xy[0], self.p[:, 1] - radar_xy[1]))
        d = (brg - az_deg + 180) % 360 - 180
        self._weight(-0.5 * (d / SIGMA_AZ) ** 2)
        self.n_az += 1
        self.sources.add("radar_az")       # beam direction: a second, independent line

    def update_range(self, us_xy, r_est):
        if self.p is None:
            return
        r = np.hypot(self.p[:, 0] - us_xy[0], self.p[:, 1] - us_xy[1]) + 1.0
        self._weight(-0.5 * (np.log(r_est / r) / SIGMA_LOGR) ** 2)

    def update_pos(self, x, y, sigma):
        if self.p is None:
            spd = self.rng.uniform(25, 85, self.n)
            hdg = self.rng.uniform(0, 2 * math.pi, self.n)
            self.p = np.column_stack([x + self.rng.normal(0, sigma, self.n), y + self.rng.normal(0, sigma, self.n),
                                      spd * np.sin(hdg), spd * np.cos(hdg)])
            self.w = np.full(self.n, 1.0 / self.n)
        d2 = (self.p[:, 0] - x) ** 2 + (self.p[:, 1] - y) ** 2
        self._weight(-0.5 * d2 / sigma ** 2)
        self.n_pos += 1
        self.sources.add("tisb")

    # ------------------------------------------------------------- results
    def estimate(self):
        """(x, y, vx, vy, sigma_pos_m, speed_sigma) or None."""
        if self.p is None:
            return None
        m = self.w @ self.p
        d = self.p - m
        cov = (d * self.w[:, None]).T @ d
        sig = math.sqrt(max(np.linalg.eigvalsh(cov[:2, :2]).max(), 0.0))
        vsig = math.sqrt(max(np.linalg.eigvalsh(cov[2:, 2:]).max(), 0.0))
        return m[0], m[1], m[2], m[3], sig, vsig

    def quality(self):
        """'FIX'   two or more independent sources and a tight spread
        'ROUGH' a single source (one airliner only): the arcs never fully resolve,
                and tests show the spread understates the true error, so we inflate it
        'NONE'  not enough data yet"""
        e = self.estimate()
        count = self.n_sum + self.n_az + self.n_pos + self.n_tdoa
        if e is None or count < 4:
            return "NONE"
        if self.score() >= 2:
            if e[4] < FIX_SIGMA:
                return "FIX"
            # a fix is still forming: wait a few seconds rather than act on a vague guess
            return "ROUGH" if count >= 12 and e[4] < 2.5 * FIX_SIGMA else "NONE"
        if count >= 6 and e[4] < 2.5 * FIX_SIGMA:
            return "ROUGH"
        return "NONE"

    def score(self):
        """How many independent geometric constraints we have. Each interrogator's
        ellipse, each partner unit's hyperbola and the radar beam line count 1;
        a TIS-B position counts 2 (it is a full fix); a +/-15 deg bearing only 0.5."""
        s = 0.0
        for src in self.sources:
            s += 2.0 if src == "tisb" else 0.5 if src == "bearing" else 1.0
        return s

    def honest_sigma(self):
        e = self.estimate()
        if e is None:
            return float("inf")
        if self.quality() == "FIX":
            return e[4]
        if "bearing" in self.sources:                  # tests: about 300 m typical, 550 m worst
            return 1.6 * max(e[4], 200.0)
        return 2.5 * max(e[4], 300.0)                  # one airliner only: 500 m typical, 1.7 km worst

    def converged(self):
        return self.quality() != "NONE"
