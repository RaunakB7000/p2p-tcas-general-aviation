"""Passive transponder listening: an interactive demo.

Our aircraft never transmits on 1030 / 1090 MHz. Ground radar (and any TCAS
aircraft nearby) interrogates the Cessna; the Cessna's transponder answers on
1090 MHz; our unit simply listens to those answers and learns
  - the Cessna's ALTITUDE (Mode C reply, 100 ft steps)
  - a ROUGH DISTANCE from signal strength, and whether it is closing
  - (optional) a coarse DIRECTION, if the unit has several antennas
and gives a vertical advisory: "traffic below: climb / don't descend".

Run from the project root:  python -m sim.passive_demo

Mouse: click the buttons; drag the Cessna anywhere on the map.
Keys:  UP/DOWN Cessna altitude   LEFT/RIGHT Cessna heading
       G radar  T TCAS airliner  M Mode C/Mode S  A antennas  F pilot follows
       1-4 speed  SPACE pause  BACKSPACE restart  ESC quit
"""
import math
import os
import random
import sys

import pygame

FT, KT, MILE, FPM = 0.3048, 0.514444, 1609.34, 0.00508
W, H = 1280, 800
MAPW = 760
SCALE = 0.058                  # px per metre
CX, CY = 0.0, -600.0           # world point shown at the map centre
RING_SPEED = 6000.0            # m/s: radio waves drawn ~50,000x slower than light
RADAR_PERIOD = 4.8             # s per antenna revolution (terminal radar)
RADAR_BEAM = 3.0               # deg beam width
REPLIES_PER_SWEEP = 12         # replies while the beam passes over (half Mode A, half Mode C)
TCAS_PERIOD = 1.0              # s, a TCAS aircraft interrogates about once a second
TX_UNKNOWN_DB = 2.5            # the Cessna's transponder is a bit stronger than "nominal"
SAFE_V_FT = 500

BG, GRID = (12, 18, 30), (24, 34, 52)
TEXT, QUIET, DIM = (226, 233, 245), (140, 155, 180), (80, 92, 115)
PANEL, CARD, CARD2 = (18, 25, 40), (26, 36, 56), (34, 46, 70)
GREEN, AMBER, RED = (80, 210, 130), (245, 185, 40), (240, 75, 65)
BLUE, PURPLE, GREY = (90, 170, 255), (190, 120, 255), (150, 160, 180)
ORANGE, WHITE = (255, 150, 60), (240, 240, 240)
FONT = "segoeui,dejavusans,arial,helvetica"
MONO = "consolas,menlo,dejavusansmono,monospace"


def to_px(x, y):
    return (MAPW / 2 + (x - CX) * SCALE, H / 2 - (y - CY) * SCALE)


def to_world(px, py):
    return ((px - MAPW / 2) / SCALE + CX, -(py - H / 2) / SCALE + CY)


# ----------------------------------------------------------------- Mode A / C encoding
def mode_a_pulses(squawk="1200"):
    """12 code pulses in transmission order: C1 A1 C2 A2 C4 A4 (X) B1 D1 B2 D2 B4 D4."""
    a, b, c, d = (int(ch) for ch in squawk)
    bit = lambda v, k: (v >> k) & 1
    return [bit(c, 0), bit(a, 0), bit(c, 1), bit(a, 1), bit(c, 2), bit(a, 2),
            bit(b, 0), bit(d, 0), bit(b, 1), bit(d, 1), bit(b, 2), bit(d, 2)]


def mode_c_pulses(alt_ft):
    """Gillham altitude code in the same 12 pulse slots."""
    n = int(round((alt_ft + 1200) / 100.0))
    n500, n100 = divmod(n, 5)
    g = n500 ^ (n500 >> 1)                        # 500 ft steps: Gray code D2 D4 A1 A2 A4 B1 B2 B4
    d2, d4, a1, a2, a4, b1, b2, b4 = [(g >> k) & 1 for k in range(7, -1, -1)]
    seq = [0b001, 0b011, 0b010, 0b110, 0b100]      # 100 ft steps: C1 C2 C4
    cv = seq[n100] if n500 % 2 == 0 else seq[4 - n100]
    c1, c2, c4 = (cv >> 2) & 1, (cv >> 1) & 1, cv & 1
    return [c1, a1, c2, a2, c4, a4, b1, 0, b2, d2, b4, d4]


# ----------------------------------------------------------------- world objects
class Plane:
    def __init__(self, name, x, y, alt_ft, hdg, kt, color, vz_fpm=0.0):
        self.name, self.x, self.y, self.alt = name, x, y, alt_ft
        self.hdg, self.kt, self.color, self.vz = hdg, kt, color, vz_fpm
        self.trail = []

    def step(self, dt):
        h = math.radians(self.hdg)
        self.x += self.kt * KT * math.sin(h) * dt
        self.y += self.kt * KT * math.cos(h) * dt
        self.alt += self.vz / 60.0 * dt


class Ring:
    """A radio transmission drawn as an expanding circle."""
    def __init__(self, x, y, t0, color, reach, width=2):
        self.x, self.y, self.t0, self.color, self.reach, self.width = x, y, t0, color, reach, width


class Sim:
    def __init__(self, radar=True, tcas=False, mode_s=False, antennas=False, follow=True, seed=3):
        self.rng = random.Random(seed)
        self.t = 0.0
        self.radar_on, self.tcas_on, self.mode_s = radar, tcas, mode_s
        self.antennas, self.follow = antennas, follow
        T = 70.0
        self.own = Plane("Our plane", 0.0, -100 * KT * T, 5000, 0, 100, GREEN)
        self.cessna = Plane("Cessna", 110 * KT * T, 0.0, 4800, 270, 110, ORANGE)
        self.airliner = Plane("Airliner (TCAS)", -6500, 4200, 9000, 100, 250, PURPLE)
        self.radar = (-5200.0, -5600.0)
        self.beam = 0.0
        self.rings, self.events = [], []
        self.replies = []                 # what our unit heard
        self.bursts = []                  # (t, smoothed range estimate) per burst, for the trend
        self.next_tcas = 0.5
        self.next_squit = self.rng.uniform(0.8, 2.4)
        self.closest = None               # (horizontal m, vertical ft) at minimum horizontal distance
        self.adv = {"level": "SILENT"}
        self.climb_target = None

    # ------------------------------------------------------------ radio
    def interrogate(self, src_xy, color, source, n_replies):
        sx, sy = src_xy
        d1 = math.hypot(self.cessna.x - sx, self.cessna.y - sy)
        self.rings.append(Ring(sx, sy, self.t, color, d1 + 300))
        self.events.append((self.t + d1 / RING_SPEED, "reply", (source, n_replies)))

    def emit_reply(self, source, n, kind_override=None):
        c = self.cessna
        d2 = math.hypot(self.own.x - c.x, self.own.y - c.y)
        col = GREY if kind_override == "S" else ORANGE
        self.rings.append(Ring(c.x, c.y, self.t, col, d2 + 300, 1 if kind_override == "S" else 2))
        snap = (c.x, c.y, c.alt)
        self.events.append((self.t + d2 / RING_SPEED, "hear", (source, n, snap, kind_override)))

    def hear(self, source, n, snap, kind_override):
        x, y, alt = snap
        r = max(60.0, math.hypot(x - self.own.x, y - self.own.y))
        true_brg = math.degrees(math.atan2(x - self.own.x, y - self.own.y)) % 360
        for i in range(n):
            if kind_override == "S":
                kind = "S"                                   # Mode S squitter: ID only
            elif source == "tcas":
                kind = "C"                                   # TCAS asks for altitude
            else:
                kind = "A" if i % 2 == 0 else "C"            # radar interleaves Mode A and Mode C
            rssi = TX_UNKNOWN_DB + 60 - 20 * math.log10(r / 1000) + self.rng.gauss(0, 2.0)
            est = 1000 * 10 ** ((60 - rssi) / 20)            # assumes a nominal transponder
            self.replies.append({
                "t": self.t, "kind": kind, "source": source, "rssi": rssi, "est": est,
                "alt": int(round(alt / 100.0)) * 100 if kind == "C" else None,
                "squawk": "1200" if kind == "A" else None,
                "brg": (true_brg + self.rng.gauss(0, 12)) % 360 if self.antennas else None,
            })
        recent = [p["est"] for p in self.replies if self.t - p["t"] < 3.0]
        if recent:
            self.bursts.append((self.t, sorted(recent)[len(recent) // 2]))

    # ------------------------------------------------------------ step
    def step(self, dt):
        self.t += dt
        for p in (self.own, self.cessna, self.airliner):
            p.step(dt)
            if int(self.t * 10) % 10 == 0:
                p.trail.append((p.x, p.y))

        # ground radar: rotating beam; a burst of replies when it sweeps the Cessna
        prev = self.beam
        self.beam = (self.beam + 360 * dt / RADAR_PERIOD) % 360
        if self.radar_on:
            rx, ry = self.radar
            brg = math.degrees(math.atan2(self.cessna.x - rx, self.cessna.y - ry)) % 360
            crossed = (prev <= brg < self.beam) if prev <= self.beam else (brg >= prev or brg < self.beam)
            if crossed:
                self.interrogate(self.radar, BLUE, "radar", REPLIES_PER_SWEEP)

        # a TCAS airliner interrogates everything around it about once a second
        if self.tcas_on and self.t >= self.next_tcas:
            self.next_tcas = self.t + TCAS_PERIOD
            self.interrogate((self.airliner.x, self.airliner.y), PURPLE, "tcas", 1)

        # Mode S transponders also squitter their ID about once a second, unprompted
        if self.mode_s and self.t >= self.next_squit:
            self.next_squit = self.t + self.rng.uniform(0.8, 2.4)
            self.emit_reply("squitter", 1, "S")

        due = [e for e in self.events if e[0] <= self.t]
        self.events = [e for e in self.events if e[0] > self.t]
        for _, kind, args in due:
            if kind == "reply":
                self.emit_reply(*args)
            else:
                self.hear(*args)

        self.rings = [r for r in self.rings if (self.t - r.t0) * RING_SPEED < r.reach]
        self.replies = [p for p in self.replies if self.t - p["t"] < 30]
        self.bursts = [b for b in self.bursts if self.t - b[0] < 30]
        self.decide()
        self.fly_own()

        dh = math.hypot(self.cessna.x - self.own.x, self.cessna.y - self.own.y)
        dv = abs(self.cessna.alt - self.own.alt)
        if self.closest is None or dh < self.closest[0]:
            self.closest = (dh, dv)

    # ------------------------------------------------------------ what our unit concludes
    def decide(self):
        now = self.t
        recent = [p for p in self.replies if now - p["t"] < 8.0]
        if not recent:
            self.adv = {"level": "SILENT"}
            return
        alts = [p for p in recent if p["alt"] is not None]
        alt = alts[-1]["alt"] if alts else None
        own_baro = int(round(self.own.alt / 100.0)) * 100          # our own pressure altitude
        dv = None if alt is None else alt - own_baro
        ests = sorted(p["est"] for p in recent if now - p["t"] < 3.5) or [self.bursts[-1][1]]
        est = ests[len(ests) // 2]
        lo, hi = est * 0.65, est * 1.55
        rate = None
        pts = [b for b in self.bursts if now - b[0] < 12]
        if len(pts) >= 2 and pts[-1][0] - pts[0][0] > 1.5:
            n = len(pts)
            mt = sum(p[0] for p in pts) / n
            mr = sum(p[1] for p in pts) / n
            den = sum((p[0] - mt) ** 2 for p in pts)
            rate = sum((p[0] - mt) * (p[1] - mr) for p in pts) / den if den else None
        closing = rate is not None and rate < -10
        brgs = [p["brg"] for p in recent if p["brg"] is not None]
        brg = None
        if brgs:
            sx = sum(math.sin(math.radians(b)) for b in brgs[-6:])
            sy = sum(math.cos(math.radians(b)) for b in brgs[-6:])
            brg = math.degrees(math.atan2(sx, sy)) % 360

        near_v = dv is None or abs(dv) < SAFE_V_FT
        if est < 1.5 * MILE and near_v and (closing or est < 0.5 * MILE):
            level = "ALERT"
        elif est < 3 * MILE:
            level = "TRAFFIC"
        else:
            level = "FAR"
        if dv is None:
            advice = "ALTITUDE UNKNOWN - LOOK OUTSIDE"
        elif dv <= -100:
            advice = "CLIMB  /  DON'T DESCEND" if level == "ALERT" else "DON'T DESCEND"
        elif dv >= 100:
            advice = "DESCEND  /  DON'T CLIMB" if level == "ALERT" else "DON'T CLIMB"
        else:
            advice = "SAME ALTITUDE - LOOK OUTSIDE"
        self.adv = dict(level=level, alt=alt, dv=dv, est=est, lo=lo, hi=hi, rate=rate,
                        closing=closing, advice=advice, brg=brg, own_baro=own_baro,
                        last=recent[-1])

    def fly_own(self):
        a, own = self.adv, self.own
        if self.follow and a["level"] == "ALERT" and a.get("dv") is not None and abs(a["dv"]) < SAFE_V_FT:
            if a["dv"] <= -100:
                own.vz = 700                                       # a C172 climbs ~700 fpm
            elif a["dv"] >= 100:
                own.vz = -700
        if own.vz and a.get("dv") is not None and abs(a["dv"]) >= SAFE_V_FT + 50:
            own.vz = 0.0
        if own.vz and a["level"] in ("SILENT", "FAR"):
            own.vz = 0.0


# ----------------------------------------------------------------- user interface
class Button:
    def __init__(self, key, label, rect):
        self.key, self.label, self.rect = key, label, pygame.Rect(rect)
        self.flash = 0.0


class App:
    def __init__(self):
        pygame.init()
        pygame.display.set_caption("Passive transponder listening - demo")
        self.screen = pygame.display.set_mode((W, H))
        self.overlay = pygame.Surface((W, H), pygame.SRCALPHA)
        self.f = pygame.font.SysFont(FONT, 16)
        self.fs = pygame.font.SysFont(FONT, 14)
        self.fb = pygame.font.SysFont(FONT, 18, bold=True)
        self.fh = pygame.font.SysFont(FONT, 30, bold=True)
        self.fh2 = pygame.font.SysFont(FONT, 24, bold=True)
        self.fm = pygame.font.SysFont(MONO, 13)
        self.clock = pygame.time.Clock()
        self.speed, self.paused = 1.0, False
        self.dragging = False
        self.sim = Sim()
        x0, y0, bw, bh = MAPW + 16, 52, 246, 34
        keys = ["radar", "tcas", "mode", "ant", "follow", "speed", "pause", "restart"]
        self.buttons = {k: Button(k, "", (x0 + (i % 2) * (bw + 10), y0 + (i // 2) * (bh + 8), bw, bh))
                        for i, k in enumerate(keys)}

    # ------------------------------------------------------------ controls
    def press(self, key):
        s = self.sim
        self.buttons[key].flash = 0.6
        if key == "radar":
            s.radar_on = not s.radar_on
        elif key == "tcas":
            s.tcas_on = not s.tcas_on
        elif key == "mode":
            s.mode_s = not s.mode_s
        elif key == "ant":
            s.antennas = not s.antennas
        elif key == "follow":
            s.follow = not s.follow
        elif key == "speed":
            self.speed = {0.5: 1.0, 1.0: 2.0, 2.0: 4.0, 4.0: 0.5}[self.speed]
        elif key == "pause":
            self.paused = not self.paused
        elif key == "restart":
            self.sim = Sim(s.radar_on, s.tcas_on, s.mode_s, s.antennas, s.follow)

    def button_text(self, k):
        s = self.sim
        on = lambda b: "ON" if b else "OFF"
        return {
            "radar":   (f"[G] Ground radar: {on(s.radar_on)}", s.radar_on),
            "tcas":    (f"[T] TCAS airliner nearby: {on(s.tcas_on)}", s.tcas_on),
            "mode":    (f"[M] Cessna: {'Mode S' if s.mode_s else 'Mode C'} transponder", s.mode_s),
            "ant":     (f"[A] Direction antennas: {on(s.antennas)}", s.antennas),
            "follow":  (f"[F] Pilot follows advice: {on(s.follow)}", s.follow),
            "speed":   (f"[1-4] Speed: {self.speed:g}x", False),
            "pause":   ("[SPACE] " + ("Resume" if self.paused else "Pause"), self.paused),
            "restart": ("[BKSP] Restart", False),
        }[k]

    def handle(self, e):
        s = self.sim
        if e.type == pygame.MOUSEBUTTONDOWN and e.button == 1:
            for k, b in self.buttons.items():
                if b.rect.collidepoint(e.pos):
                    self.press(k)
                    return True
            cx, cy = to_px(s.cessna.x, s.cessna.y)
            if math.hypot(e.pos[0] - cx, e.pos[1] - cy) < 28:
                self.dragging = True
        elif e.type == pygame.MOUSEBUTTONUP:
            self.dragging = False
        elif e.type == pygame.MOUSEMOTION and self.dragging and e.pos[0] < MAPW:
            s.cessna.x, s.cessna.y = to_world(*e.pos)
        elif e.type == pygame.KEYDOWN:
            k = e.key
            keymap = {pygame.K_g: "radar", pygame.K_t: "tcas", pygame.K_m: "mode", pygame.K_a: "ant",
                      pygame.K_f: "follow", pygame.K_SPACE: "pause", pygame.K_BACKSPACE: "restart"}
            if k == pygame.K_ESCAPE:
                return False
            if k in keymap:
                self.press(keymap[k])
            elif k in (pygame.K_1, pygame.K_2, pygame.K_3, pygame.K_4):
                self.speed = {pygame.K_1: 0.5, pygame.K_2: 1.0, pygame.K_3: 2.0, pygame.K_4: 4.0}[k]
                self.buttons["speed"].flash = 0.6
            elif k == pygame.K_UP:
                s.cessna.alt += 100
            elif k == pygame.K_DOWN:
                s.cessna.alt -= 100
            elif k == pygame.K_LEFT:
                s.cessna.hdg = (s.cessna.hdg - 10) % 360
            elif k == pygame.K_RIGHT:
                s.cessna.hdg = (s.cessna.hdg + 10) % 360
        return True

    def update(self, real_dt):
        for b in self.buttons.values():
            b.flash = max(0.0, b.flash - real_dt)
        if not self.paused:
            steps = max(1, int(round(real_dt * self.speed / 0.02)))
            for _ in range(steps):
                self.sim.step(real_dt * self.speed / steps)

    def run(self):
        while True:
            for e in pygame.event.get():
                if e.type == pygame.QUIT or not self.handle(e):
                    return
            self.update(1 / 30)
            self.draw()
            pygame.display.flip()
            self.clock.tick(30)

    # ------------------------------------------------------------ drawing helpers
    def text(self, s, pos, color=TEXT, font=None, center=False, right=False):
        img = (font or self.f).render(s, True, color)
        x, y = pos
        if center:
            x -= img.get_width() / 2
        if right:
            x -= img.get_width()
        self.screen.blit(img, (x, y))
        return img.get_width()

    def plane_icon(self, p, size=1.0, hollow=False):
        px, py = to_px(p.x, p.y)
        pts = []
        for ang, r in ((0, 15), (130, 11), (180, 5), (230, 11)):
            t = math.radians(p.hdg + ang)
            pts.append((px + r * size * math.sin(t), py - r * size * math.cos(t)))
        pygame.draw.polygon(self.screen, p.color, pts, 2 if hollow else 0)
        return px, py

    # ------------------------------------------------------------ drawing
    def draw(self):
        self.screen.fill(BG)
        self.overlay.fill((0, 0, 0, 0))
        self.draw_map()
        self.screen.blit(self.overlay, (0, 0))
        self.draw_planes()
        self.draw_banner()
        self.draw_panel()

    def draw_map(self):
        s, scr, ov = self.sim, self.screen, self.overlay
        for i in range(-8, 9):
            gx, _ = to_px(i * 1000, 0)
            _, gy = to_px(0, CY + i * 1000)
            pygame.draw.line(scr, GRID, (gx, 0), (gx, H))
            pygame.draw.line(scr, GRID, (0, gy), (MAPW, gy))
        ox, oy = to_px(s.own.x, s.own.y)
        for r in (1, 2, 3):
            pygame.draw.circle(scr, (40, 55, 80), (ox, oy), r * MILE * SCALE, 1)
            self.text(f"{r} mi", (ox + 4, oy - r * MILE * SCALE - 18), DIM, self.fs)

        # radar site and its rotating beam
        rx, ry = to_px(*s.radar)
        if s.radar_on:
            for k in range(12):
                a0 = math.radians(s.beam - k * 3)
                a1 = math.radians(s.beam - (k + 1) * 3)
                L = 1500
                pts = [(rx, ry), (rx + L * math.sin(a0), ry - L * math.cos(a0)),
                       (rx + L * math.sin(a1), ry - L * math.cos(a1))]
                pygame.draw.polygon(ov, (90, 170, 255, max(0, 70 - k * 6)), pts)
        pygame.draw.rect(scr, BLUE if s.radar_on else DIM, (rx - 7, ry - 7, 14, 14), 0 if s.radar_on else 2)
        self.text("Ground radar", (rx + 12, ry - 30), BLUE if s.radar_on else DIM, self.fb)
        self.text("asks every 4.8 s" if s.radar_on else "OFF / out of coverage", (rx + 12, ry - 10),
                  QUIET if s.radar_on else DIM, self.fs)

        # radio rings
        for r in s.rings:
            rad = (s.t - r.t0) * RING_SPEED * SCALE
            if rad < 2:
                continue
            frac = min(1.0, (s.t - r.t0) * RING_SPEED / r.reach)
            alpha = int(230 * (1 - frac) + 25)
            pygame.draw.circle(ov, (*r.color, alpha), to_px(r.x, r.y), rad, r.width)

        # what our unit believes: a distance band (and a direction wedge with antennas)
        a = s.adv
        if a["level"] != "SILENT":
            col = {"ALERT": RED, "TRAFFIC": AMBER, "FAR": GREEN}[a["level"]]
            inner, outer = a["lo"] * SCALE, a["hi"] * SCALE
            if a.get("brg") is None:
                pygame.draw.circle(ov, (*col, 55), (ox, oy), outer, max(1, int(outer - inner)))
                pygame.draw.circle(ov, (*col, 160), (ox, oy), outer, 1)
                pygame.draw.circle(ov, (*col, 160), (ox, oy), inner, 1)
                if outer > 70:
                    self.text("Cessna is somewhere in this band", (ox - 110, oy + outer + 4), col, self.fs)
            else:
                b0, b1 = a["brg"] - 30, a["brg"] + 30
                pts = [(ox + inner * math.sin(math.radians(b)), oy - inner * math.cos(math.radians(b)))
                       for b in range(int(b0), int(b1) + 1, 3)]
                pts += [(ox + outer * math.sin(math.radians(b)), oy - outer * math.cos(math.radians(b)))
                        for b in range(int(b1), int(b0) - 1, -3)]
                pygame.draw.polygon(ov, (*col, 70), pts)
                pygame.draw.polygon(ov, (*col, 180), pts, 1)
                mx = ox + outer * math.sin(math.radians(a["brg"]))
                my = oy - outer * math.cos(math.radians(a["brg"]))
                if outer > 70:
                    self.text("Cessna is in the shaded zone", (ox - 230, oy - 8), col, self.fs)

        self.text("Radio waves drawn ~50,000x slower than reality", (12, H - 48), DIM, self.fs)
        self.text("Drag the Cessna   UP/DOWN its altitude   LEFT/RIGHT its heading", (12, H - 26), QUIET, self.fs)

    def draw_planes(self):
        s = self.sim
        for p in (s.airliner, s.cessna, s.own):
            if p is s.airliner and not s.tcas_on:
                continue
            for (tx, ty) in p.trail[-90:]:
                pygame.draw.circle(self.screen, tuple(v // 2 for v in p.color), to_px(tx, ty), 1)
            px, py = self.plane_icon(p, 1.2 if p is not s.airliner else 1.4)
            vz = "  climbing" if p.vz > 50 else "  descending" if p.vz < -50 else ""
            if p is s.own:
                self.text("OUR PLANE", (px + 16, py - 12), GREEN, self.fb)
                self.text(f"{p.alt:,.0f} ft{vz}   transmits nothing", (px + 16, py + 8), QUIET, self.fs)
            elif p is s.cessna:
                self.text("CESSNA", (px + 16, py - 12), ORANGE, self.fb)
                self.text(f"{p.alt:,.0f} ft   transponder only", (px + 16, py + 8), QUIET, self.fs)
            else:
                self.text("Airliner with TCAS", (px + 16, py - 12), PURPLE, self.fb)
                self.text("asks everyone ~1x per second", (px + 16, py + 8), QUIET, self.fs)

    def draw_banner(self):
        a = self.sim.adv
        scr = self.screen
        if a["level"] == "SILENT":
            col, l1 = DIM, "NOTHING HEARD"
            l2 = "Nobody is interrogating the Cessna, so its transponder stays silent"
        else:
            col = {"ALERT": RED, "TRAFFIC": AMBER, "FAR": GREEN}[a["level"]]
            word = {"ALERT": "TRAFFIC ALERT", "TRAFFIC": "TRAFFIC", "FAR": "TRAFFIC (far)"}[a["level"]]
            alt = "altitude ?" if a["dv"] is None else (
                "same altitude" if abs(a["dv"]) < 100 else f"{abs(a['dv']):,} ft {'BELOW' if a['dv'] < 0 else 'ABOVE'}")
            trend = "  CLOSING" if a["closing"] else ""
            l1 = f"{word}   ~{a['est'] / MILE:.1f} mi   {alt}{trend}"
            l2 = a["advice"] if a["level"] != "FAR" else "no action needed"
        pygame.draw.rect(scr, (10, 14, 22), (0, 0, MAPW, 84))
        pygame.draw.rect(scr, col, (0, 0, MAPW, 84), 3)
        self.text(l1, (MAPW / 2, 8), col, self.fh, center=True)
        self.text(("> " + l2) if a["level"] in ("ALERT", "TRAFFIC") else l2, (MAPW / 2, 46),
                  TEXT if a["level"] == "ALERT" else QUIET, self.fh2 if a["level"] == "ALERT" else self.f,
                  center=True)

    def card(self, rect, title):
        pygame.draw.rect(self.screen, CARD, rect, border_radius=8)
        self.text(title, (rect[0] + 12, rect[1] + 8), TEXT, self.fb)

    def draw_panel(self):
        s, scr = self.sim, self.screen
        pygame.draw.rect(scr, PANEL, (MAPW, 0, W - MAPW, H))
        x0 = MAPW + 16
        self.text("WHAT OUR UNIT HEARS", (x0, 14), TEXT, self.fb)
        self.text(f"t = {s.t:4.0f} s", (W - 16, 16), QUIET, self.fs, right=True)

        for k, b in self.buttons.items():
            label, on = self.button_text(k)
            fill = CARD2 if not on else (40, 70, 60)
            if b.flash > 0:
                fill = (90, 110, 150)
            pygame.draw.rect(scr, fill, b.rect, border_radius=6)
            pygame.draw.rect(scr, GREEN if on else DIM, b.rect, 1, border_radius=6)
            self.text(label, (b.rect.x + 10, b.rect.y + 8), TEXT if on else QUIET, self.fs)

        self.draw_timeline((x0, 230, W - MAPW - 32, 96))
        self.draw_packet((x0, 336, W - MAPW - 32, 150))
        self.draw_altitude((x0, 496, 220, 248))
        self.draw_range((x0 + 230, 496, W - MAPW - 32 - 230, 248))
        now_h = math.hypot(s.cessna.x - s.own.x, s.cessna.y - s.own.y)
        now_v = abs(s.cessna.alt - s.own.alt)
        self.text(f"Truth now: {now_h / MILE:.2f} mi apart, {now_v:,.0f} ft vertically", (x0, 750), QUIET, self.fs)
        dh, dv_ft = s.closest or (now_h, now_v)          # closest = (metres, feet)
        if s.closest and dh < 1.0 * MILE:
            safe = dv_ft >= SAFE_V_FT or dh >= 0.3 * MILE
            self.text(f"Closest pass: {dh / MILE:.2f} mi, {dv_ft:,.0f} ft vertically  "
                      + ("SAFE" if safe else "TOO CLOSE"), (x0, 770), GREEN if safe else RED, self.f)

    def draw_timeline(self, rect):
        s = self.sim
        x, y, w, h = rect
        self.card(rect, "1. Replies heard (last 20 s)")
        bx, by, bw, bh = x + 12, y + 36, w - 24, 34
        pygame.draw.rect(self.screen, (16, 22, 36), (bx, by, bw, bh))
        for p in s.replies:
            age = s.t - p["t"]
            if age > 20:
                continue
            px = bx + bw - age / 20 * bw
            col = {"radar": BLUE, "tcas": PURPLE, "squitter": GREY}[p["source"]]
            pygame.draw.line(self.screen, col, (px, by + 4), (px, by + bh - 4), 2)
        self.text("20 s ago", (bx, by + bh + 3), DIM, self.fs)
        self.text("now", (bx + bw, by + bh + 3), DIM, self.fs, right=True)
        lx = bx + 80
        for label, col in (("radar burst", BLUE), ("TCAS", PURPLE), ("Mode S ID", GREY)):
            pygame.draw.line(self.screen, col, (lx, by + bh + 11), (lx + 12, by + bh + 11), 3)
            lx += 18 + self.text(label, (lx + 16, by + bh + 3), QUIET, self.fs) + 14

    def draw_packet(self, rect):
        s = self.sim
        x, y, w, h = rect
        self.card(rect, "2. Last answer, decoded")
        a = s.adv
        p = a.get("last") if a["level"] != "SILENT" else None
        if p is None:
            self.text("(silence)", (x + 12, y + 40), DIM, self.f)
            return
        scr = self.screen
        # the question we overheard on 1030 MHz
        qy = y + 52
        spacing = 21 if p["kind"] == "C" else 8
        self.text("Question (1030):", (x + 12, qy - 12), QUIET, self.fs)
        if p["kind"] == "S":
            self.text("none - Mode S sends its ID on its own", (x + 140, qy - 12), GREY, self.fs)
        else:
            sx0 = x + 140
            for off in (0, spacing):
                pygame.draw.rect(scr, BLUE if p["source"] == "radar" else PURPLE, (sx0 + off * 4, qy - 14, 4, 16))
            what = "21 us apart = 'what is your ALTITUDE?'" if spacing == 21 else "8 us apart = 'what is your CODE?'"
            self.text(what, (sx0 + 96, qy - 12), QUIET, self.fs)
        # the answer on 1090 MHz
        ay = y + 92
        self.text("Answer (1090):", (x + 12, ay - 12), QUIET, self.fs)
        if p["kind"] == "S":
            self.text("ID A4B2C1 (no altitude, no position)", (x + 140, ay - 12), GREY, self.fs)
            return
        bits = mode_c_pulses(p["alt"]) if p["kind"] == "C" else mode_a_pulses(p["squawk"])
        sx0 = x + 140
        pygame.draw.rect(scr, ORANGE, (sx0, ay - 16, 4, 20))                       # F1
        for i, b in enumerate(bits):
            bxp = sx0 + 12 + i * 14
            if b:
                pygame.draw.rect(scr, ORANGE, (bxp, ay - 16, 4, 20))
            else:
                pygame.draw.line(scr, DIM, (bxp, ay + 3), (bxp + 4, ay + 3), 1)
        pygame.draw.rect(scr, ORANGE, (sx0 + 12 + 12 * 14 + 4, ay - 16, 4, 20))   # F2
        meaning = f"= ALTITUDE {p['alt']:,} ft" if p["kind"] == "C" else f"= squawk code {p['squawk']} (VFR)"
        self.text(meaning, (x + 12, ay + 18), ORANGE if p["kind"] == "C" else QUIET, self.fb)
        self.text(f"signal {p['rssi']:.0f} dB  ->  ~{p['est'] / MILE:.1f} mi", (x + w - 12, ay + 20),
                  QUIET, self.fs, right=True)

    def draw_altitude(self, rect):
        s = self.sim
        x, y, w, h = rect
        self.card(rect, "3. Altitude")
        a = s.adv
        cx, top, bot = x + 70, y + 44, y + h - 20
        mid = (top + bot) / 2
        own_alt = s.own.alt
        span = 1200.0
        Y = lambda alt: mid - (alt - own_alt) / span * (bot - top) / 2
        pygame.draw.line(self.screen, DIM, (cx, top), (cx, bot), 2)
        for k in range(-1000, 1001, 500):
            yy = Y(round(own_alt / 500) * 500 + k)
            if top <= yy <= bot:
                pygame.draw.line(self.screen, DIM, (cx - 6, yy), (cx + 6, yy), 1)
                self.text(f"{round(own_alt / 500) * 500 + k:,.0f}", (cx - 10, yy - 9), DIM, self.fs, right=True)
        # safety band +/- 500 ft around us
        pygame.draw.rect(self.screen, (40, 60, 52), (cx + 10, Y(own_alt + SAFE_V_FT), 16,
                                                     Y(own_alt - SAFE_V_FT) - Y(own_alt + SAFE_V_FT)))
        pygame.draw.polygon(self.screen, GREEN, [(cx + 2, mid), (cx + 14, mid - 7), (cx + 14, mid + 7)])
        self.text("us", (cx + 30, mid - 9), GREEN, self.fs)
        if a["level"] != "SILENT" and a.get("alt") is not None:
            ty = max(top, min(bot, Y(a["alt"])))
            pygame.draw.polygon(self.screen, ORANGE, [(cx + 2, ty), (cx + 14, ty - 7), (cx + 14, ty + 7)])
            dv = a["dv"]
            self.text(f"Cessna {dv:+,} ft", (cx + 30, ty - 9), ORANGE, self.fb)
        elif a["level"] != "SILENT":
            self.text("Cessna: ? (no altitude yet)", (x + 12, bot - 4), GREY, self.fs)
        self.text("green band = +/-500 ft", (x + 12, y + h - 20), DIM, self.fs)

    def draw_range(self, rect):
        s = self.sim
        x, y, w, h = rect
        self.card(rect, "4. Distance (rough)")
        a = s.adv
        bx, by, bw = x + 14, y + 62, w - 28
        maxm = 3 * MILE
        X = lambda m: bx + min(1.0, m / maxm) * bw
        pygame.draw.line(self.screen, DIM, (bx, by), (bx + bw, by), 2)
        for mi in range(4):
            pygame.draw.line(self.screen, DIM, (X(mi * MILE), by - 5), (X(mi * MILE), by + 5), 1)
            self.text(f"{mi} mi", (X(mi * MILE), by + 8), DIM, self.fs, center=True)
        true_d = math.hypot(s.cessna.x - s.own.x, s.cessna.y - s.own.y)
        if a["level"] != "SILENT":
            col = {"ALERT": RED, "TRAFFIC": AMBER, "FAR": GREEN}[a["level"]]
            pygame.draw.rect(self.screen, col, (X(a["lo"]), by - 10, max(3, X(a["hi"]) - X(a["lo"])), 20),
                             border_radius=4)
            self.text("our estimate", (X(a["lo"]), by - 30), col, self.fs)
            trend = "unknown yet" if a["rate"] is None else (
                "CLOSING" if a["closing"] else
                ("opening" if a["rate"] > 10 else "steady"))
            self.text(f"Trend: {trend}", (bx, by + 34), RED if a["closing"] else QUIET, self.fb)
        pygame.draw.line(self.screen, WHITE, (X(true_d), by - 14), (X(true_d), by + 14), 1)
        self.text("truth (unit can't see this)", (bx, y + h - 22), DIM, self.fs)
        pygame.draw.line(self.screen, WHITE, (bx + 168, y + h - 14), (bx + 182, y + h - 14), 1)
        # estimate history: one dot per burst, falling dots = getting closer
        gx, gy, gw, gh = bx, by + 64, bw, 90
        pygame.draw.rect(self.screen, (16, 22, 36), (gx, gy, gw, gh))
        self.text("estimate per burst (last 30 s)", (gx + 4, gy + 2), DIM, self.fs)
        for (t, est) in s.bursts:
            px = gx + gw - (s.t - t) / 30 * gw
            py = gy + gh - 6 - min(1.0, est / maxm) * (gh - 24)
            pygame.draw.circle(self.screen, ORANGE, (px, py), 3)


def main():
    App().run()
    pygame.quit()


if __name__ == "__main__":
    main()
