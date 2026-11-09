"""Cockpit view: you are the pilot.

Top:     out-the-window view with HUD boxes around traffic your unit has detected
         (placed where the UNIT THINKS the aircraft is, so passive estimates can be off)
Bottom:  flight instruments + unit status | traffic display | system log

Run from the project root:   python -m sim.cockpit

Keys
  LEFT / RIGHT   turn (hold)              UP / DOWN    climb / descend (hold)
  P   pilot: simulated pilot follows advisories  <->  you fly (auto-avoid takes over after 5 s)
  G   ground radar on/off (OFF by default: coverage can't be counted on)
  T   airliner TCAS interrogations on/off
  N   network: other units of ours share when they heard each reply (hyperbolas)
  D   direction antennas on your unit (bearing lines)
  O   your ADS-B Out on/off (enables TIS-B)
  TAB select a target: "how do we know about it?"     Z  traffic display range
  SPACE pause   + / -  speed   R  restart   ESC quit
"""
import math
import sys

import numpy as np
import pygame

from logic.models import FT, KT, FPM, wrap180
from logic.cpa import assess
from logic.params import P
from sim.cockpit_world import CockpitWorld

W, H = 1280, 800
OTW_H = 430                     # out-the-window view height
PANEL_Y = 452
MILE = 1609.34
FOCAL = 640.0                   # px: 90 degree horizontal field of view
SPAN = {"tcas": 36.0, "none": 15.0}

SKY_TOP, SKY_LOW = (70, 120, 185), (170, 200, 230)
GROUND, GROUND_FAR = (96, 112, 72), (140, 145, 110)
BG, PANEL, CARD = (10, 13, 20), (22, 26, 34), (30, 36, 48)
TEXT, QUIET, DIM = (228, 233, 242), (150, 160, 180), (90, 98, 115)
CYAN, GREEN, AMBER, RED, MAG = (90, 220, 240), (90, 220, 130), (250, 190, 50), (245, 70, 60), (230, 90, 235)
ORANGE, PURPLE, BLUE, TAN = (255, 150, 60), (180, 120, 255), (100, 160, 255), (210, 180, 130)
TAG_COL = {"ADS-B": CYAN, "LINK": GREEN, "RX": ORANGE, "MATCH": PURPLE, "FIX": ORANGE, "TRACK": ORANGE,
           "TIS-B": TAN, "TA": AMBER, "RA": RED, "NEGOT": GREEN, "AUTO": MAG, "CLEAR": (120, 190, 150),
           "SEP": (120, 190, 150), "TRUTH": (200, 110, 110), "SYS": QUIET, "TCAS": BLUE, "RADIO": QUIET,
           "NET": (120, 235, 170), "DIR": (120, 210, 255)}
SRC_LABEL = {"unit": "LINK", "adsb": "ADS-B", "tisb": "TIS-B", "timing": "PASSIVE"}
FONT = "segoeui,dejavusans,arial,helvetica"
MONO = "consolas,menlo,dejavusansmono,monospace"


class Cockpit:
    def __init__(self):
        pygame.init()
        pygame.display.set_caption("Peer-to-peer TCAS - cockpit")
        self.screen = pygame.display.set_mode((W, H))
        self.otw = pygame.Surface((W, OTW_H))
        self.alpha = pygame.Surface((W, H), pygame.SRCALPHA)
        self.f = pygame.font.SysFont(FONT, 15)
        self.fs = pygame.font.SysFont(FONT, 13)
        self.fb = pygame.font.SysFont(FONT, 16, bold=True)
        self.fbig = pygame.font.SysFont(FONT, 30, bold=True)
        self.fnum = pygame.font.SysFont(MONO, 22, bold=True)
        self.fm = pygame.font.SysFont(MONO, 13)
        self.clock = pygame.time.Clock()
        self.speed, self.paused = 1, False
        self.range_mi = 3
        self.sel = None
        self.settings = dict(radar_on=False, tcas_on=True, own_adsb_out=False, sim_pilot=True,
                             net_on=True, antennas=False)
        self.restart()
        rng = np.random.default_rng(1)
        self.ground_pts = rng.uniform(-30000, 30000, (900, 2))     # farmland dots for motion cues

    def restart(self):
        self.w = CockpitWorld(**self.settings)

    # ------------------------------------------------------------ input
    def handle(self, e):
        w = self.w
        if e.type == pygame.QUIT:
            return False
        if e.type != pygame.KEYDOWN:
            return True
        k = e.key
        if k == pygame.K_ESCAPE:
            return False
        if k == pygame.K_p:
            self.settings["sim_pilot"] = w.sim_pilot = not w.sim_pilot
            w.own.pilot_ignores = not w.sim_pilot
            w.slog("SYS", "YOU", "Simulated pilot follows advisories" if w.sim_pilot else
                   "You are flying. If you don't follow an advisory within 5 s, auto-avoid takes over")
        elif k == pygame.K_g:
            self.settings["radar_on"] = w.radar_on = not w.radar_on
            w.slog("SYS", "YOU", f"Ground radar {'in coverage' if w.radar_on else 'OFF / out of coverage'}")
        elif k == pygame.K_t:
            self.settings["tcas_on"] = w.tcas_on = not w.tcas_on
            w.slog("SYS", "YOU", f"Airliner TCAS interrogations {'ON' if w.tcas_on else 'OFF'}")
        elif k == pygame.K_n:
            self.settings["net_on"] = w.net_on = not w.net_on
            w.slog("SYS", "YOU", "Network listening ON: our other units share reply times over LoRa"
                   if w.net_on else "Network listening OFF")
        elif k == pygame.K_d:
            self.settings["antennas"] = w.antennas = not w.antennas
            w.slog("SYS", "YOU", f"Direction antennas {'ON' if w.antennas else 'OFF'}")
        elif k == pygame.K_o:
            self.settings["own_adsb_out"] = w.own.adsb_out = not w.own.adsb_out
            w.slog("SYS", "YOU", f"Own ADS-B Out {'ON: TIS-B available' if w.own.adsb_out else 'OFF: no TIS-B'}")
        elif k == pygame.K_z:
            self.range_mi = {2: 3, 3: 6, 6: 2}[self.range_mi]
        elif k == pygame.K_TAB:
            self.cycle_selection()
        elif k == pygame.K_SPACE:
            self.paused = not self.paused
        elif k in (pygame.K_PLUS, pygame.K_EQUALS, pygame.K_KP_PLUS):
            self.speed = min(8, self.speed * 2)
        elif k in (pygame.K_MINUS, pygame.K_KP_MINUS):
            self.speed = max(1, self.speed // 2)
        elif k == pygame.K_r:
            self.restart()
        return True

    def cycle_selection(self):
        ids = self.detected_ids()
        if not ids:
            self.sel = None
            return
        self.sel = ids[(ids.index(self.sel) + 1) % len(ids)] if self.sel in ids else ids[0]

    def detected_ids(self):
        u = self.w.own.unit
        ids = list(u.tracks) if u else []
        ids += [pid for pid, *_ in (u.proximity if u else []) if pid not in ids]
        return sorted(ids, key=lambda i: math.dist((self.w.aircraft[i].x, self.w.aircraft[i].y),
                                                   (self.w.own.x, self.w.own.y)))

    def tick(self, dt):
        keys = pygame.key.get_pressed()
        w = self.w
        w.input_turn = (1 if keys[pygame.K_RIGHT] else 0) - (1 if keys[pygame.K_LEFT] else 0)
        w.input_climb = (1 if keys[pygame.K_UP] else 0) - (1 if keys[pygame.K_DOWN] else 0)
        if not self.paused:
            for _ in range(self.speed):
                w.step(dt)

    def run(self):
        while True:
            for e in pygame.event.get():
                if not self.handle(e):
                    return
            self.tick(0.1)
            self.draw()
            pygame.display.flip()
            self.clock.tick(10)

    # ------------------------------------------------------------ helpers
    def text(self, s, pos, color=TEXT, font=None, center=False, right=False, surf=None):
        img = (font or self.f).render(s, True, color)
        x, y = pos
        if center:
            x -= img.get_width() / 2
        if right:
            x -= img.get_width()
        (surf or self.screen).blit(img, (x, y))
        return img.get_width()

    def attitude(self):
        own = self.w.own
        bank = math.degrees(math.atan(own.speed * math.radians(own.turn_rate) / 9.81))
        pitch = math.degrees(math.atan2(own.vz, own.speed))
        return bank, pitch

    def project(self, x, y, alt):
        """World point -> OTW screen (x, y, forward distance) or None if behind."""
        own = self.w.own
        h = math.radians(own.heading)
        dx, dy, dz = x - own.x, y - own.y, alt - own.alt
        f = dx * math.sin(h) + dy * math.cos(h)
        r = dx * math.cos(h) - dy * math.sin(h)
        if f < 30:
            return None
        bank, pitch = self.attitude()
        sx = FOCAL * r / f
        sy = -FOCAL * dz / f + FOCAL * math.tan(math.radians(pitch))
        b = math.radians(bank)
        X = sx * math.cos(b) + sy * math.sin(b)
        Y = -sx * math.sin(b) + sy * math.cos(b)
        return W / 2 + X, OTW_H / 2 + Y, f

    def rot(self, sx, sy):
        bank, _ = self.attitude()
        b = math.radians(bank)
        return W / 2 + sx * math.cos(b) + sy * math.sin(b), OTW_H / 2 - sx * math.sin(b) + sy * math.cos(b)

    # ------------------------------------------------------------ drawing
    def draw(self):
        self.screen.fill(BG)
        self.draw_otw()
        self.screen.blit(self.otw, (0, 0))
        self.draw_hud()
        self.draw_glareshield()
        self.draw_instruments()
        self.draw_mfd()
        self.draw_log()

    def draw_otw(self):
        s = self.otw
        for i in range(OTW_H):                                          # sky gradient
            k = i / OTW_H
            s.fill(tuple(int(SKY_TOP[j] * (1 - k) + SKY_LOW[j] * k) for j in range(3)), (0, i, W, 1))
        _, pitch = self.attitude()
        hy = FOCAL * math.tan(math.radians(pitch))
        ground = [self.rot(-4000, hy), self.rot(4000, hy), self.rot(4000, hy + 4000), self.rot(-4000, hy + 4000)]
        pygame.draw.polygon(s, GROUND, ground)
        pygame.draw.polygon(s, GROUND_FAR, [self.rot(-4000, hy), self.rot(4000, hy),
                                             self.rot(4000, hy + 14), self.rot(-4000, hy + 14)])
        own = self.w.own
        gx = self.ground_pts[:, 0] + math.floor(own.x / 60000) * 60000
        gy = self.ground_pts[:, 1] + math.floor(own.y / 60000) * 60000
        for x, y in zip(gx, gy):
            p = self.project(x, y, 0.0)
            if p and p[2] < 14000 and 0 <= p[1] < OTW_H:
                pygame.draw.circle(s, (70, 85, 55), (p[0], p[1]), 1 if p[2] > 4000 else 2)
        # other aircraft as you would see them (truth)
        for a in self.w.aircraft.values():
            if a is own:
                continue
            p = self.project(a.x, a.y, a.alt)
            if not p or p[2] > 12000:
                continue
            span = max(3.0, FOCAL * SPAN.get(a.equipment, 11.0) / p[2])
            bank, _ = self.attitude()
            b = math.radians(bank)
            dx, dy = math.cos(b) * span / 2, -math.sin(b) * span / 2
            pygame.draw.line(s, (35, 38, 45), (p[0] - dx, p[1] - dy), (p[0] + dx, p[1] + dy), max(1, int(span / 12)))
            pygame.draw.circle(s, (35, 38, 45), (p[0], p[1]), max(1, int(span / 9)))

    def threat_level(self, tid):
        u = self.w.own.unit
        if u and u.advisory and u.advisory.threat_id == tid:
            return "RA"
        tr = u.tracks.get(tid) if u else None
        if not tr:
            return "PROX"
        return assess(self.w.own.track(self.w.t), tr.extrapolate(self.w.t)).level

    def draw_hud(self):
        w, own, scr = self.w, self.w.own, self.screen
        u = own.unit
        hud = (120, 255, 160)
        # heading tape
        for d in range(-40, 41, 5):
            hdg = (own.heading + d) % 360
            x = W / 2 + (d - (own.heading % 5)) * 8
            mark = round(hdg / 5) * 5 % 360
            if 300 < x < W - 300:
                pygame.draw.line(scr, hud, (x, 8), (x, 16 if mark % 10 else 22), 1)
                if mark % 10 == 0:
                    self.text(f"{mark // 10:02d}", (x, 22), hud, self.fs, center=True)
        pygame.draw.polygon(scr, hud, [(W / 2, 6), (W / 2 - 6, 0), (W / 2 + 6, 0)])
        # flight path marker
        pygame.draw.circle(scr, hud, (W / 2, OTW_H / 2), 7, 2)
        pygame.draw.line(scr, hud, (W / 2 - 20, OTW_H / 2), (W / 2 - 7, OTW_H / 2), 2)
        pygame.draw.line(scr, hud, (W / 2 + 7, OTW_H / 2), (W / 2 + 20, OTW_H / 2), 2)
        if not u:
            return
        # boxes around detected traffic, drawn where the UNIT thinks it is
        for tid, tr in u.tracks.items():
            e = tr.extrapolate(w.t)
            p = self.project(e.x, e.y, e.alt)
            lvl = self.threat_level(tid)
            col = {"RA": RED, "TA": AMBER}.get(lvl, CYAN)
            name = w.aircraft[tid].name if tid in w.aircraft else tid
            rng = math.dist((e.x, e.y), (own.x, own.y)) / MILE
            dv = (e.alt - own.alt) / FT
            src = SRC_LABEL.get(tr.source, tr.source)
            if tr.source == "timing":
                src += f" +/-{tr.sigma:.0f} m"
            if p and 0 < p[0] < W and 0 < p[1] < OTW_H and p[2] < 20000:
                size = max(16, FOCAL * SPAN.get(w.aircraft[tid].equipment, 11.0) / p[2] + 12)
                if tr.source == "timing":
                    size = max(size, FOCAL * tr.sigma / p[2])          # box grows with uncertainty
                x, y, s2 = p[0], p[1], size / 2
                for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
                    cx, cy = x + sx * s2, y + sy * s2
                    pygame.draw.line(scr, col, (cx, cy), (cx - sx * 7, cy), 2)
                    pygame.draw.line(scr, col, (cx, cy), (cx, cy - sy * 7), 2)
                self.text(f"{name}  {src}", (x + s2 + 6, y - s2 - 2), col, self.fs)
                self.text(f"{rng:.1f} mi  {dv:+,.0f} ft", (x + s2 + 6, y - s2 + 13), col, self.fs)
            elif lvl in ("TA", "RA"):                                   # off-screen threat: edge pointer
                brg = math.radians(wrap180(math.degrees(math.atan2(e.x - own.x, e.y - own.y)) - own.heading))
                ex = W / 2 + math.sin(brg) * (W / 2 - 40)
                ey = OTW_H / 2 - math.cos(brg) * (OTW_H / 2 - 40)
                pygame.draw.circle(scr, col, (ex, ey), 9, 3)
                self.text(f"{name} {rng:.1f} mi", (ex, ey + 12), col, self.fs, center=True)
        # banner + guidance
        adv = u.advisory
        msg, col = None, None
        if adv:
            col = MAG if adv.auto else RED
            msg = u.message.replace("RA: ", "")
            if adv.action in ("RIGHT", "LEFT") and not adv.done_turning:
                sign = 1 if adv.action == "RIGHT" else -1
                cx = W / 2 + sign * 150
                pts = [(cx + sign * 40, OTW_H / 2), (cx - sign * 10, OTW_H / 2 - 30), (cx - sign * 10, OTW_H / 2 + 30)]
                pygame.draw.polygon(scr, col, pts)
        elif u.status in ("TA", "PROX"):
            col, msg = AMBER, u.message
        if msg:
            img = self.fbig.render(msg.upper(), True, col)
            bx = W / 2 - img.get_width() / 2 - 16
            pygame.draw.rect(scr, (10, 12, 18), (bx, 44, img.get_width() + 32, 44), border_radius=6)
            pygame.draw.rect(scr, col, (bx, 44, img.get_width() + 32, 44), 2, border_radius=6)
            scr.blit(img, (bx + 16, 49))
        if self.paused:
            self.text("PAUSED", (W - 16, 8), TEXT, self.fb, right=True)

    def draw_glareshield(self):
        pts = [(0, OTW_H), (W, OTW_H), (W, PANEL_Y), (0, PANEL_Y)]
        pygame.draw.polygon(self.screen, (16, 18, 22), pts)
        pygame.draw.line(self.screen, (40, 44, 52), (0, PANEL_Y), (W, PANEL_Y), 2)

    def box(self, rect, label, value, col=TEXT):
        pygame.draw.rect(self.screen, CARD, rect, border_radius=6)
        self.text(label, (rect[0] + 8, rect[1] + 4), QUIET, self.fs)
        self.text(value, (rect[0] + 8, rect[1] + 20), col, self.fnum)

    def draw_instruments(self):
        w, own = self.w, self.w.own
        x0, y0 = 10, PANEL_Y + 10
        self.box((x0, y0, 136, 50), "AIRSPEED", f"{own.speed / KT:.0f} KT")
        self.box((x0 + 144, y0, 136, 50), "ALTITUDE", f"{own.alt / FT:,.0f}")
        vs = own.vz / FPM
        self.box((x0, y0 + 56, 136, 50), "VERT SPEED", f"{vs:+,.0f}", GREEN if abs(vs) > 50 else TEXT)
        self.box((x0 + 144, y0 + 56, 136, 50), "HEADING", f"{own.heading:03.0f} deg")
        u = own.unit
        y = y0 + 116
        self.text("OUR UNIT", (x0, y), TEXT, self.fb)
        y += 22
        now = w.t
        link = [w.aircraft[k].name for k, t in (u.tracks.items() if u else []) if t.source == "unit"]
        adsb = [w.aircraft[k].name for k, t in (u.tracks.items() if u else []) if t.source == "adsb"]
        replies = [w.aircraft[k].name for k, t in w.heard.items() if now - t < 8]
        inter = (["radar"] if w.radar_on else []) + (["AAL12"] if w.tcas_on else [])
        partners = sorted({s[5:] for L in w.loc.values() for s in L.sources if s.startswith("tdoa:")}) \
            if w.net_on else []
        lines = [
            (GREEN if link else DIM, "LoRa 915 MHz", "link: " + ", ".join(link) if link else "listening"),
            (CYAN if adsb else DIM, "1090 ADS-B", ", ".join(adsb) if adsb else "none"),
            (ORANGE if replies else DIM, "1090 replies", ", ".join(replies) if replies else "none"),
            (PURPLE if inter else DIM, "1030 questions", ", ".join(inter) if inter else "nobody asking"),
            (BLUE if w.radar_on else DIM, "Ground radar", "in coverage" if w.radar_on else "no coverage"),
            ((120, 235, 170) if partners else DIM, "Network", ("shared by " + ", ".join(partners)) if partners
             else ("on, no partner yet" if w.net_on else "OFF")),
            ((120, 210, 255) if w.antennas else DIM, "Dir. antennas", "ON (+/-15 deg)" if w.antennas else "OFF"),
            (TAN if own.adsb_out else DIM, "Own ADS-B Out", "ON (TIS-B)" if own.adsb_out else "OFF"),
        ]
        for col, label, val in lines:
            pygame.draw.circle(self.screen, col, (x0 + 5, y + 8), 4)
            self.text(label, (x0 + 16, y), QUIET, self.fs)
            self.text(val[:22], (x0 + 128, y), col, self.fs)
            y += 18
        adv = u.advisory if u else None
        state = "ACTIVE" if adv and adv.auto else "ARMED"
        pilot = "SIM PILOT" if w.sim_pilot else "YOU FLY"
        self.text(f"Pilot: {pilot}   Auto-avoid: {state}", (x0, y + 4), MAG if state == "ACTIVE" else QUIET, self.fs)

    def draw_mfd(self):
        w, own, scr = self.w, self.w.own, self.screen
        X0, Y0, MW, MH = 300, PANEL_Y + 6, 372, H - PANEL_Y - 12
        pygame.draw.rect(scr, (4, 6, 10), (X0, Y0, MW, MH), border_radius=10)
        pygame.draw.rect(scr, (60, 66, 80), (X0, Y0, MW, MH), 2, border_radius=10)
        cx, cy = X0 + MW / 2, Y0 + MH * 0.62
        R = 150
        scale = R / (self.range_mi * MILE)
        clip = scr.get_clip()
        scr.set_clip(pygame.Rect(X0 + 2, Y0 + 2, MW - 4, MH - 4))
        h = math.radians(own.heading)

        def to_mfd(x, y):
            dx, dy = x - own.x, y - own.y
            r = dx * math.cos(h) - dy * math.sin(h)
            f = dx * math.sin(h) + dy * math.cos(h)
            return cx + r * scale, cy - f * scale

        for frac in (0.5, 1.0):
            pygame.draw.circle(scr, (45, 52, 66), (cx, cy), R * frac, 1)
            self.text(f"{self.range_mi * frac:g}", (cx + 4, cy - R * frac - 15), DIM, self.fs)
        u = own.unit
        self.alpha.fill((0, 0, 0, 0))
        # passive targets: particle cloud, ellipses, radar line, uncertainty circle
        for tid, L in w.loc.items():
            if L.p is None or w.t - w.heard.get(tid, -99) > 10:
                continue
            idx = np.random.default_rng(int(w.t * 10)).choice(L.n, 450, replace=False)
            spread = L.estimate()[4]
            dot = 2 if spread > 400 else 1
            for px, py in L.p[idx, :2]:
                pygame.draw.circle(self.alpha, (255, 150, 60, 150), to_mfd(px, py), dot)
            for cv in L.curves:
                age = w.t - cv["t"]
                if age > 4:
                    continue
                alpha = int(200 * (1 - age / 4)) + 30
                if cv["type"] == "bearing":
                    bx, by = cv["f1"]
                    for off, a_ in ((-cv["sigma"], alpha // 2), (0, alpha), (cv["sigma"], alpha // 2)):
                        th = math.radians(cv["value"] + off)
                        pygame.draw.line(self.alpha, (120, 210, 255, a_), to_mfd(bx, by),
                                         to_mfd(bx + 9000 * math.sin(th), by + 9000 * math.cos(th)), 1)
                    continue
                f1, f2 = cv["f1"], cv["f2"]
                c = math.dist(f1, f2) / 2
                mx, my = (f1[0] + f2[0]) / 2, (f1[1] + f2[1]) / 2
                ux, uy = (f2[0] - f1[0]) / (2 * c), (f2[1] - f1[1]) / (2 * c)    # axis from f1 to f2
                px_, py_ = -uy, ux
                pts = []
                if cv["type"] == "ellipse":
                    a = cv["value"] / 2
                    b = math.sqrt(max(a * a - c * c, 1))
                    for k in range(0, 361, 6):
                        th = math.radians(k)
                        ex, ey = a * math.cos(th), b * math.sin(th)
                        pts.append(to_mfd(mx + ex * ux + ey * px_, my + ex * uy + ey * py_))
                    col = BLUE if cv["kind"] == "radar" else PURPLE
                    pygame.draw.lines(self.alpha, (*col, alpha), True, pts, 1)
                else:                                   # hyperbola: |X-f1| - |X-f2| = value
                    a = max(-c + 1, min(c - 1, cv["value"] / 2))
                    b = math.sqrt(max(c * c - a * a, 1))
                    for k in range(-30, 31):
                        tt = k / 10
                        ex, ey = a * math.cosh(tt), b * math.sinh(tt)
                        pts.append(to_mfd(mx + ex * ux + ey * px_, my + ex * uy + ey * py_))
                    pygame.draw.lines(self.alpha, (120, 235, 170, alpha), False, pts, 1)
            q = L.quality()
            if q != "NONE":
                ex_, ey_, *_ = L.estimate()
                pygame.draw.circle(self.alpha, (255, 150, 60, 180), to_mfd(ex_, ey_),
                                   max(4, L.honest_sigma() * scale), 1)
        if w.radar_on:
            for tid, L in w.loc.items():
                if L.p is not None and L.n_az and w.t - w.heard.get(tid, -99) < 10:
                    ex_, ey_, *_ = L.estimate()
                    rx, ry = w.radar
                    d = math.dist((rx, ry), (ex_, ey_))
                    ux, uy = (ex_ - rx) / d, (ey_ - ry) / d
                    pygame.draw.line(self.alpha, (*BLUE, 120), to_mfd(ex_ - ux * 6000, ey_ - uy * 6000),
                                     to_mfd(ex_ + ux * 3000, ey_ + uy * 3000), 1)
        scr.blit(self.alpha, (0, 0))
        # proximity-only rings
        if u:
            for pid, rng_m, dv in u.proximity:
                if rng_m < self.range_mi * MILE * 1.2:
                    for k in range(0, 360, 12):
                        a0 = math.radians(k)
                        pygame.draw.arc(scr, AMBER, (cx - rng_m * scale, cy - rng_m * scale,
                                                     2 * rng_m * scale, 2 * rng_m * scale), a0, a0 + 0.1, 1)
                    self.text(f"{pid}: {dv / FT:+.0f} ft, direction?", (cx - 70, cy + rng_m * scale + 2), AMBER, self.fs)
        # tracks with TCAS-style symbols
        if u:
            for tid, tr in u.tracks.items():
                e = tr.extrapolate(w.t)
                x, y = to_mfd(e.x, e.y)
                lvl = self.threat_level(tid)
                if lvl == "RA":
                    pygame.draw.rect(scr, RED, (x - 6, y - 6, 12, 12))
                    col = RED
                elif lvl == "TA":
                    pygame.draw.circle(scr, AMBER, (x, y), 6)
                    col = AMBER
                else:
                    col = CYAN
                    pygame.draw.polygon(scr, CYAN, [(x, y - 7), (x + 6, y), (x, y + 7), (x - 6, y)], 2)
                rel = (e.alt - own.alt) / FT / 100
                arrow = "^" if e.vz > 2 else "v" if e.vz < -2 else ""
                self.text(f"{rel:+03.0f}{arrow}", (x + 9, y - 16 if rel >= 0 else y + 4), col, self.fs)
                name = w.aircraft[tid].name if tid in w.aircraft else tid
                self.text(f"{name} {SRC_LABEL.get(tr.source, '')}", (x + 9, y - 2 if rel >= 0 else y - 14), QUIET, self.fs)
                if tid == self.sel:
                    pygame.draw.circle(scr, TEXT, (x, y), 13, 1)
        # own aircraft
        pygame.draw.polygon(scr, TEXT, [(cx, cy - 10), (cx + 7, cy + 8), (cx, cy + 4), (cx - 7, cy + 8)])
        scr.set_clip(clip)
        self.text(f"TRAFFIC  {self.range_mi} mi  [Z]", (X0 + 10, Y0 + 6), QUIET, self.fs)
        self.text(f"HDG {own.heading:03.0f}", (X0 + MW - 10, Y0 + 6), QUIET, self.fs, right=True)
        self.text("dots: guesses  purple: ellipse  green: hyperbola  cyan: bearing", (X0 + 10, Y0 + MH - 18), DIM, self.fs)
        self.draw_selection(W - 262, 40)

    def draw_selection(self, x, y):
        """'How do we know about it?' card for the TAB-selected target."""
        w = self.w
        tid = self.sel
        u = w.own.unit
        if not tid or not u:
            self.text("TAB: how do we know about a target?", (x + 20, y), (230, 240, 250), self.fs)
            return
        name = w.aircraft[tid].name
        tr = u.tracks.get(tid)
        lines = []
        if tr is None:
            lines = ["Mode C replies only (no ID)", "Altitude known, range from signal", "No position: vertical advice only"]
        elif tr.source == "unit":
            lines = ["Our device over LoRa 915 MHz", "GPS position +/-5 m, heading, speed", "Can negotiate maneuvers"]
        elif tr.source == "adsb":
            lines = ["ADS-B DF17 on 1090 MHz", "GPS position +/-10 m, velocity",
                     "TCAS: yes (turn only vs it)" if tr.tcas else "No negotiation possible"]
        else:
            L = w.loc[tid]
            partners = sorted(s[5:] for s in L.sources if s.startswith("tdoa:"))
            lines = [f"Transponder only: altitude {L.alt / FT:,.0f} ft",
                     f"{L.n_sum} ellipses (airliner{', radar' if 'radar' in L.sources else ''})",
                     f"{L.n_tdoa} hyperbolas" + (f" (via {', '.join(partners)})" if partners else ""),
                     f"{L.n_brg} bearings, {L.n_az} radar lines, {L.n_pos} TIS-B",
                     f"{L.quality()}  +/-{L.honest_sigma():.0f} m, {tr.speed / KT:.0f} kt"]
        hgt = 22 + 16 * len(lines)
        pygame.draw.rect(self.screen, (18, 22, 30), (x, y, 246, hgt), border_radius=6)
        pygame.draw.rect(self.screen, (70, 80, 100), (x, y, 246, hgt), 1, border_radius=6)
        self.text(f"{name}: how we know", (x + 8, y + 3), TEXT, self.fb)
        for i, ln in enumerate(lines):
            self.text(ln, (x + 8, y + 22 + 16 * i), QUIET, self.fs)

    def draw_log(self):
        X0, Y0 = 684, PANEL_Y + 6
        MW, MH = W - X0 - 8, H - PANEL_Y - 12
        scr = self.screen
        pygame.draw.rect(scr, PANEL, (X0, Y0, MW, MH), border_radius=10)
        self.text("SYSTEM LOG: how each aircraft was detected", (X0 + 12, Y0 + 6), TEXT, self.fb)
        rows = []
        for e in self.w.syslog:
            head = f"{e['t']:5.1f}  "
            body = f"{e['who']}: {e['text']}"
            wrapped = self.wrap(body, 66)
            for i, ln in enumerate(wrapped):
                rows.append((e, ln, i == 0))
        room = (MH - 64) // 15
        y = Y0 + 30
        for e, ln, first in rows[-room:]:
            col = TAG_COL.get(e["tag"], QUIET)
            if first:
                self.text(f"{e['t']:5.1f}", (X0 + 10, y), DIM, self.fm)
                pygame.draw.rect(scr, col, (X0 + 54, y + 1, 50, 14), border_radius=3)
                self.text(e["tag"], (X0 + 79, y), (15, 18, 24), self.fs, center=True)
            self.text(ln, (X0 + 112, y), TEXT if e["tag"] in ("RA", "AUTO", "FIX") else QUIET, self.fs)
            y += 15
        self.text("arrows fly  P pilot  G radar  T TCAS  N network  D antennas  O ADS-B Out  TAB  Z",
                  (X0 + 12, Y0 + MH - 34), DIM, self.fs)
        self.text("SPACE pause  +/- speed  R restart  ESC quit", (X0 + 12, Y0 + MH - 18), DIM, self.fs)

    @staticmethod
    def wrap(s, n):
        out, line = [], ""
        for word in s.split(" "):
            if line and len(line) + len(word) + 1 > n:
                out.append(line)
                line = word
            else:
                line = f"{line} {word}" if line else word
        return out + [line]


def main():
    Cockpit().run()
    pygame.quit()


if __name__ == "__main__":
    main()
