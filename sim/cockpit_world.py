"""The world behind the cockpit view: one mixed-traffic encounter that exercises
every detection layer, plus a tagged system log of how each aircraft was found.

Our aircraft (U1, "you") meets:
  N512   another aircraft with OUR DEVICE        -> P2P link, negotiation
  AAL12  airliner, TCAS + ADS-B Out (high above)  -> ADS-B; also the "interrogator"
  N88W   GA aircraft with ADS-B Out only           -> ADS-B
  N734   Cessna, TRANSPONDER ONLY                  -> passive: altitude + timing ellipses,
                                                      plus hyperbolas shared by N620
  N620   another aircraft with OUR DEVICE, 1,500 ft above, no conflict: it listens too,
         and shares when it heard each reply (network multilateration)
  GLIDER no electronics at all                     -> undetectable (truth only)

Ground radar is OFF by default: coverage at low altitude can't be counted on.
"""
import math

from logic.models import Track, KT, FT, FPM, wrap180
from logic.params import P
from logic.passive_locator import PassiveLocator
from sim.world import Aircraft, World

C_LIGHT = 299792458.0
TCAS_RANGE = 26000.0      # m   ~14 NM airliner TCAS surveillance
RX_RANGE = 25000.0        # m   our 1090 MHz receiver range for transponder replies
ADSB_RANGE = 60000.0      # m   ADS-B reception range
RADAR_PERIOD = 4.8        # s
RADAR_ALT = 400.0         # m   radar antenna height (site elevation + tower)


def _meet(heading, speed_kt, at, t):
    s, h = speed_kt * KT, math.radians(heading)
    return at[0] - s * math.sin(h) * t, at[1] - s * math.cos(h) * t


def build_traffic():
    own_kt, T0 = 110.0, 0.0
    start = (0.0, -6500.0)
    own_at = lambda t: (start[0], start[1] + own_kt * KT * t)
    alt = 5000 * FT
    u1 = Aircraft("U1", *start, alt, 0, own_kt * KT, equipment="device", adsb_out=False, name="YOU")

    # Cessna, transponder only: converging from the right, 100 ft below, would hit at t = 48 s
    cx, cy = _meet(230, 105, own_at(48), 48)
    n734 = Aircraft("N734", cx, cy, alt - 100 * FT, 230, 105 * KT, equipment="transponder", name="N734")

    # aircraft with our device: parked far away; the scenario puts it head-on with you
    # once you are back on course after the Cessna (see CockpitWorld._director)
    n512 = Aircraft("U2", 0.0, 60000.0, alt, 180, 115 * KT, equipment="device", name="N512")

    # airliner above everyone, crossing north of us: ADS-B + TCAS (it does the interrogating)
    aal = Aircraft("AAL12", -9000, 7000, 9000 * FT, 105, 250 * KT, equipment="tcas", name="AAL12")

    # ADS-B-only GA aircraft crossing 1,500 ft above us: detected, no threat
    gx, gy = _meet(80, 120, own_at(30), 30)
    n88w = Aircraft("N88W", gx, gy, alt + 1500 * FT, 80, 120 * KT, equipment="adsb", name="N88W")

    # blind glider, same altitude, passes 0.5 mi to our left at t = 72 s
    lx, ly = _meet(200, 60, (own_at(72)[0] - 800, own_at(72)[1]), 72)
    glider = Aircraft("GLIDER", lx, ly, alt + 100 * FT, 200, 60 * KT, equipment="none", name="GLIDER")
    # another aircraft with our device, 1,500 ft above us, flying roughly with us: a second "ear"
    n620 = Aircraft("N620", 3000.0, -3500.0, alt + 1500 * FT, 10, 100 * KT, equipment="device", name="N620")
    return [u1, n734, n512, aal, n88w, glider, n620]


def describe_sources(sources):
    names = []
    for s in sorted(sources):
        if s.startswith("tdoa:"):
            names.append(f"hyperbola via {s[5:]}")
        elif s == "radar":
            names.append("radar ellipse")
        elif s == "radar_az":
            names.append("radar beam line")
        elif s == "tisb":
            names.append("TIS-B")
        elif s == "bearing":
            names.append("our bearing")
        else:
            names.append(f"ellipse via {s}")
    return " + ".join(names)


class CockpitWorld(World):
    def __init__(self, radar_on=False, tcas_on=True, own_adsb_out=False, sim_pilot=True, seed=7,
                 net_on=True, antennas=False):
        super().__init__(build_traffic(), seed=seed, duration=140.0)
        self.syslog = []                     # dicts: t, tag, who, text
        self.radar = (-9000.0, -15000.0)
        self.radar_on, self.tcas_on = radar_on, tcas_on
        self.net_on, self.antennas = net_on, antennas
        self.own = self.aircraft["U1"]
        self.own.adsb_out = own_adsb_out
        self.sim_pilot = sim_pilot           # True: simulated pilot follows advisories
        self.own.pilot_ignores = not sim_pilot
        self.input_turn = 0                  # -1 / 0 / +1 from the keyboard
        self.input_climb = 0
        self.beam = 0.0
        self.next_tcas = 0.4
        self.next_tisb = 2.0
        self.loc = {a.id: PassiveLocator(a.id, seed=seed + i)
                    for i, a in enumerate(self.aircraft.values()) if a.equipment == "transponder"}
        self.heard = {}                      # transponder id -> last time a reply was heard
        self.rssi = {}                       # transponder id -> latest signal-strength range estimate
        self.flags = set()                   # one-time log events already written
        self.prev_level = {}
        self.glider_min = (float("inf"), 0.0)
        self.n512_placed = False
        self.slog("SYS", "YOU", "Unit on. Listening: LoRa 915 MHz (our devices), 1090 MHz (ADS-B + "
                  "transponder replies), 1030 MHz (interrogations). Transmitting: LoRa only.")
        if not own_adsb_out:
            self.slog("SYS", "YOU", "Own ADS-B Out is OFF, so ground stations won't send us TIS-B.")

    # ------------------------------------------------------------------ logging
    def slog(self, tag, who, text):
        self.syslog.append({"t": self.t, "tag": tag, "who": who, "text": text})

    def log(self, text):                     # messages from the logic units
        super().log(text)
        who, _, rest = text.partition(": ")
        who = {"U1": "YOU", "U2": "N512"}.get(who, who)
        tag = "RA" if "RA vs" in rest else "NEGOT" if ("PROPOSE" in rest or "ACK" in rest) else \
            "AUTO" if "AUTO-AVOID" in rest else "CLEAR" if "clear of conflict" in rest else \
            "TCAS" if "TCAS" in rest else "SEP" if "separation" in rest else "RADIO" if "LOST" in rest else "SYS"
        rest = rest.replace("U2", "N512").replace("U1", "YOU")
        if who != "YOU" and "PROXIMITY" in rest:
            return                                   # other aircraft's proximity alerts aren't ours
        self.slog(tag, who, rest)

    def once(self, key, tag, who, text):
        if key not in self.flags:
            self.flags.add(key)
            self.slog(tag, who, text)

    # ------------------------------------------------------------------ step
    def step(self, dt=0.1):
        for L in self.loc.values():
            L.predict(dt)
        self._passive(dt)
        self._director()
        super().step(dt)
        self._first_contacts()
        self._watch_levels()
        self._watch_glider()

    def _director(self, lead=45.0):
        """Place N512 head-on with you, `lead` seconds ahead, once you are flying straight again."""
        own, u2 = self.own, self.aircraft.get("U2")
        if self.n512_placed or u2 is None or self.t < 58:
            return
        if own.turn_rate or (own.unit and own.unit.advisory) or \
                abs(wrap180(own.course - own.heading)) > 2:
            return
        h = math.radians(own.heading)
        mx = own.x + own.speed * math.sin(h) * lead + 40 * math.cos(h)
        my = own.y + own.speed * math.cos(h) * lead - 40 * math.sin(h)
        u2.heading = u2.course = (own.heading + 180) % 360
        hh = math.radians(u2.heading)
        u2.x, u2.y = mx - u2.speed * math.sin(hh) * lead, my - u2.speed * math.cos(hh) * lead
        u2.alt = own.alt
        u2.trail = []
        self.n512_placed = True

    def _extra_listeners(self, L, o, us, now):
        """Each time the target answers: other units of ours that heard the same reply
        (hyperbolas), and our own direction antennas (a bearing line)."""
        x = (o.x, o.y, o.alt)
        if self.net_on and self.system_on:
            for v in self.aircraft.values():
                if v is self.own or v.equipment != "device" or not self.own.unit or \
                        v.id not in self.own.unit.tracks:          # need its LoRa STATE (position + clock)
                    continue
                if math.dist((v.x, v.y), (self.own.x, self.own.y)) > P.RADIO_RANGE or \
                        math.dist((v.x, v.y), (o.x, o.y)) > RX_RANGE:
                    continue
                vp = (v.x + self.rng.gauss(0, 5), v.y + self.rng.gauss(0, 5), v.alt)   # from its LoRa STATE
                d = math.dist(x, us) - math.dist(x, (v.x, v.y, v.alt)) + self.rng.gauss(0, 60)
                L.update_tdoa(us, vp, d, now, v.name)
                self.once(f"net-{o.id}-{v.id}", "NET", o.id,
                          f"{v.name} (our device) heard the same reply {abs(d) / C_LIGHT * 1e6:.2f} us "
                          f"{'earlier' if d > 0 else 'later'} than us; times shared over LoRa -> hyperbola")
        if self.antennas:
            brg = math.degrees(math.atan2(o.x - self.own.x, o.y - self.own.y)) % 360
            L.update_bearing((self.own.x, self.own.y), brg + self.rng.gauss(0, 15), now)
            self.once(f"dir-{o.id}", "DIR", o.id,
                      f"Direction antennas: reply arrived from about {brg:03.0f} deg (+/-15) -> a bearing line")

    # what our receivers pick up from transponder-only aircraft
    def _passive(self, dt):
        now, own = self.t, self.own
        us = (own.x, own.y, own.alt)
        prev = self.beam
        self.beam = (self.beam + 360 * dt / RADAR_PERIOD) % 360
        tcas_due = self.tcas_on and now >= self.next_tcas
        if tcas_due:
            self.next_tcas = now + 1.0
        for tid, L in self.loc.items():
            o = self.aircraft[tid]
            d_us = math.dist((o.x, o.y), (own.x, own.y))
            if d_us > RX_RANGE:
                continue
            mode_c = round(o.alt / FT / 100) * 100
            heard_any = False
            # 1) the airliner's TCAS asks; the Cessna answers; we time both
            for a in self.aircraft.values():
                if not (tcas_due and a.equipment == "tcas" and math.dist((a.x, a.y), (o.x, o.y)) < TCAS_RANGE):
                    continue
                true = math.dist((a.x, a.y, a.alt), (o.x, o.y, o.alt)) + math.dist((o.x, o.y, o.alt), us)
                z = true + self.rng.gauss(0, 100)
                illum = (a.x + self.rng.gauss(0, 10), a.y + self.rng.gauss(0, 10), a.alt)   # from its ADS-B
                L.update_sum(illum, us, z, now, a.id)
                heard_any = True
                base = math.dist((illum[0], illum[1], illum[2]), us)
                dt_us = (z - base) / C_LIGHT * 1e6 + 3.0
                self.once(f"tcas-{tid}", "MATCH", tid,
                          f"1030 MHz question from {a.name} (TCAS; position known from its ADS-B), "
                          f"answer {dt_us:.1f} us later -> distance sum {z / 1000:.1f} km -> ellipse #1")
                self._extra_listeners(L, o, us, now)
            # 2) the ground radar's beam sweeps over it
            if self.radar_on:
                brg = math.degrees(math.atan2(o.x - self.radar[0], o.y - self.radar[1])) % 360
                crossed = (prev <= brg < self.beam) if prev <= self.beam else (brg >= prev or brg < self.beam)
                if crossed:
                    rpos = (self.radar[0], self.radar[1], RADAR_ALT)
                    z = math.dist(rpos, (o.x, o.y, o.alt)) + math.dist((o.x, o.y, o.alt), us) + self.rng.gauss(0, 100)
                    L.update_sum(rpos, us, z, now, "radar")
                    L.update_az(self.radar, brg + self.rng.gauss(0, 1.5), now)
                    heard_any = True
                    self.once(f"radar-{tid}", "MATCH", tid,
                              f"Ground radar sweep: beam pointing {brg:03.0f} deg when it answered -> "
                              f"a line from the radar + a second ellipse")
                    self._extra_listeners(L, o, us, now)
            # 3) TIS-B, only if we have ADS-B Out
            if own.adsb_out and now >= self.next_tisb:
                L.update_pos(o.x + self.rng.gauss(0, P.TISB_NOISE), o.y + self.rng.gauss(0, P.TISB_NOISE), P.TISB_NOISE)
                L.set_altitude(mode_c * FT)
                self.heard[tid] = now
                self.once(f"tisb-{tid}", "TIS-B", tid, "Ground station rebroadcast its radar position (TIS-B)")
            if heard_any:
                self.heard[tid] = now
                L.set_altitude(mode_c * FT)
                r_est = d_us * math.exp(self.rng.gauss(0, 0.35))
                self.rssi[tid] = r_est
                L.update_range((own.x, own.y), r_est)
                self.once(f"rx-{tid}", "RX", tid,
                          f"1090 MHz Mode C reply (no ID): altitude {mode_c:,} ft "
                          f"({mode_c - round(own.alt / FT / 100) * 100:+,} ft). Signal -> ~{r_est / 1609:.1f} mi. "
                          f"Direction unknown yet")
            q = L.quality()
            if q != "NONE":
                e = L.estimate()
                if q == "FIX":
                    self.once(f"fix-{tid}", "FIX", tid,
                              f"Position FIX from {describe_sources(L.sources)}: +/-{L.honest_sigma():.0f} m")
                else:
                    self.once(f"rough-{tid}", "FIX", tid,
                              f"ROUGH position only ({describe_sources(L.sources)}): +/-{L.honest_sigma():.0f} m, "
                              f"safety margin widened to match")
                spd = math.hypot(e[2], e[3])
                if e[5] < 6 and f"trk-{tid}" not in self.flags:
                    hdg = math.degrees(math.atan2(e[2], e[3])) % 360
                    self.once(f"trk-{tid}", "TRACK", tid,
                              f"Track: heading {hdg:03.0f} deg, {spd / KT:.0f} kt (from how the fixes move)")
        if own.adsb_out and now >= self.next_tisb:
            self.next_tisb = now + P.TISB_PERIOD

    # ------------------------------------------------------------------ what U1's unit sees
    def _sensed_by(self, a, now):
        if a is not self.own:
            return super()._sensed_by(a, now)
        external, proximity = [], []
        for o in self.aircraft.values():
            if o is a:
                continue
            if o.adsb_out and math.dist((o.x, o.y), (a.x, a.y)) < ADSB_RANGE:
                external.append(o.track(now, "adsb", P.GPS_NOISE, self.rng))
            elif o.equipment == "transponder" and now - self.heard.get(o.id, -99) < max(8.0, P.TISB_PERIOD * 2):
                L = self.loc[o.id]
                if L.quality() != "NONE":
                    x, y, vx, vy, _, _ = L.estimate()
                    external.append(Track(o.id, x, y, L.alt, math.degrees(math.atan2(vx, vy)) % 360,
                                          math.hypot(vx, vy), 0.0, 0.0, "timing", False, now,
                                          None, L.honest_sigma()))
                else:
                    proximity.append((o.id, self.rssi.get(o.id, 9e9), L.alt - a.alt if L.alt else 0.0))
        return external, proximity

    # ------------------------------------------------------------------ flying our aircraft
    def _fly(self, a, now, dt):
        if a is not self.own:
            return super()._fly(a, now, dt)
        unit = a.unit
        if self.input_turn or self.input_climb:                  # the user is flying
            a.turn_rate = self.input_turn * P.TURN_RATE
            a.vz = self.input_climb * 700 * FPM
            cmd = unit.commanded_turn()
            unit.pilot_turning = bool(cmd and self.input_turn and math.copysign(1, cmd) == self.input_turn)
            if unit.advisory and a.turn_rate:
                unit.advisory.deviation += abs(a.turn_rate) * dt
            if not unit.advisory:
                a.course = (a.heading + a.turn_rate * dt) % 360   # new course = where you point it
            return
        a.vz = 0.0
        super()._fly(a, now, dt)

    # ------------------------------------------------------------------ one-time detection logs
    def _first_contacts(self):
        own = self.own
        for o in self.aircraft.values():
            if o is own:
                continue
            d = math.dist((o.x, o.y), (own.x, own.y))
            if o.adsb_out and d < ADSB_RANGE:
                kind = "TCAS operational flag set" if o.equipment == "tcas" else "no TCAS"
                dv = round((o.alt - own.alt) / FT / 100) * 100
                self.once(f"adsb-{o.id}", "ADS-B", o.name,
                          f"1090 MHz ADS-B (DF17) from {o.name}: GPS position +/-10 m, velocity, "
                          f"altitude {dv:+,} ft, {kind}")
            if o.equipment == "device" and own.unit and o.id in own.unit.tracks:
                self.once(f"link-{o.id}", "LINK", o.name,
                          f"LoRa 915 MHz STATE from {o.name} (our device): GPS position +/-5 m, "
                          f"heading, speed, intent. Can negotiate")

    def _watch_levels(self):
        u = self.own.unit
        if not u or not self.system_on:
            return
        top = u.top
        key = top.other_id if top else None
        lvl = top.level if top else "CLEAR"
        if key and lvl == "TA" and self.prev_level.get(key) != "TA" and not u.advisory:
            name = self.aircraft[key].name if key in self.aircraft else key
            self.slog("TA", name, f"Traffic advisory: closest approach {top.d_h:.0f} m in {top.t_cpa:.0f} s "
                                  f"(protected radius {top.h_sep:.0f} m)")
        if key:
            self.prev_level[key] = lvl

    def _watch_glider(self):
        g = self.aircraft.get("GLIDER")
        if not g:
            return
        d = math.dist((g.x, g.y), (self.own.x, self.own.y))
        if d < self.glider_min[0]:
            self.glider_min = (d, abs(g.alt - self.own.alt))
        elif d > self.glider_min[0] + 300 and self.glider_min[0] < 3000:
            self.once("glider", "TRUTH", "GLIDER",
                      f"(Truth, not detected) A glider with no electronics passed {self.glider_min[0] / 1609:.1f} mi "
                      f"away at your altitude. No radio system can see it: see-and-avoid only")
