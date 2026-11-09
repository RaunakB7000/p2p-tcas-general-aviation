"""Visual simulator: north-up traffic display centred on U1, plus control panel.

Run from the project root:
    python -m sim.simulator                         # first scenario file
    python -m sim.simulator 5                       # 5th scenario file
    python -m sim.simulator scenarios/my_test.json  # any scenario file
    python -m sim.simulator 1 --set RA_TIME=15 --set PILOT_DELAY=4

Keys
  1-9, 0        scenario 1-10            LEFT / RIGHT   previous / next scenario
  G             new random encounter     F5             reload scenario files from disk
  SPACE         pause / resume           R              restart
  + / -         simulation speed
  D             lose PROPOSE/ACK         K              P2P radio link down
  P             U1 pilot ignores alert   B              baseline: our units switched off
  TAB / S-TAB   select parameter         [ / ]          decrease / increase it (restarts)
  BACKSPACE     reset all parameters     ESC            quit
"""
import math
import os
import sys

import pygame

from sim.world import build, load_scenarios, random_spec, EQUIPMENT
from logic.models import FT, KT
from logic.params import P, SPEC, UI_ORDER

W, H = 1280, 800
MAP = 800
SCALE = 0.078                  # pixels per metre (3-mile ring fits the map)
MILE = 1609.34
DT = 0.1

BG, GRID, RING = (12, 18, 30), (26, 36, 54), (60, 80, 110)
TEXT, QUIET, DIM = (222, 230, 242), (135, 150, 175), (75, 85, 105)
PANEL, SEL = (20, 28, 44), (40, 56, 86)
COLORS = {"CLEAR": (70, 200, 120), "TA": (245, 185, 40), "PROX": (245, 185, 40),
          "RA": (235, 70, 60), "AUTO": (220, 80, 230), "adsb": (150, 160, 190),
          "tcas": (80, 190, 240), "transponder": (190, 150, 110), "none": (95, 100, 115),
          "device": (70, 200, 120)}
FONT = "consolas,menlo,dejavusansmono,monospace"


class App:
    def __init__(self, start=1, user_params=None):
        pygame.init()
        pygame.display.set_caption("Peer-to-peer TCAS for GA - simulator")
        self.screen = pygame.display.set_mode((W, H))
        self.f = pygame.font.SysFont(FONT, 15)
        self.fs = pygame.font.SysFont(FONT, 13)
        self.fb = pygame.font.SysFont(FONT, 15, bold=True)
        self.fbanner = pygame.font.SysFont(FONT, 28, bold=True)
        self.clock = pygame.time.Clock()
        self.speed, self.paused = 1, False
        self.lose = self.lora_down = self.ignore = self.baseline = False
        self.user_params = dict(user_params or {})
        self.sel = 0
        self.random_seed = 0
        self.files = load_scenarios()
        if isinstance(start, str) and os.path.exists(start):
            import json
            with open(start, encoding="utf-8") as fh:
                self.files.append((start, json.load(fh)))
            start = len(self.files)
        self.load(int(start))

    # ---------------------------------------------------------------- scenarios
    def load(self, idx=None, spec=None, label=None):
        if spec is None:
            idx = max(1, min(len(self.files), idx or self.idx))
            self.idx = idx
            self.path, self.spec = self.files[idx - 1]
            self.label = f"{idx}/{len(self.files)}  {os.path.basename(self.path)}"
        else:
            self.spec, self.path, self.label = spec, None, label
        self.restart()

    def restart(self):
        self.world = build(self.spec, self.user_params, system_on=not self.baseline,
                           extra_ignores=("U1",) if self.ignore else (),
                           lose_negotiation=self.lose or None, lora_down=self.lora_down or None)

    # ---------------------------------------------------------------- loop
    def run(self):
        while True:
            for e in pygame.event.get():
                if e.type == pygame.QUIT or (e.type == pygame.KEYDOWN and not self.key(e)):
                    return
            if not self.paused:
                for _ in range(self.speed):
                    self.world.step(DT)
            self.draw()
            pygame.display.flip()
            self.clock.tick(1 / DT)

    def key(self, e):
        k, mods = e.key, pygame.key.get_mods()
        if k == pygame.K_ESCAPE:
            return False
        digits = {pygame.K_1: 1, pygame.K_2: 2, pygame.K_3: 3, pygame.K_4: 4, pygame.K_5: 5,
                  pygame.K_6: 6, pygame.K_7: 7, pygame.K_8: 8, pygame.K_9: 9, pygame.K_0: 10}
        if k in digits and digits[k] <= len(self.files):
            self.load(digits[k])
        elif k == pygame.K_RIGHT:
            self.load((self.idx % len(self.files)) + 1)
        elif k == pygame.K_LEFT:
            self.load(((self.idx - 2) % len(self.files)) + 1)
        elif k == pygame.K_g:
            self.random_seed += 1
            self.load(spec=random_spec(self.random_seed), label=f"random seed {self.random_seed}")
        elif k == pygame.K_F5:
            self.files = load_scenarios()
            self.load(min(self.idx, len(self.files)))
        elif k == pygame.K_r:
            self.restart()
        elif k == pygame.K_SPACE:
            self.paused = not self.paused
        elif k in (pygame.K_PLUS, pygame.K_EQUALS, pygame.K_KP_PLUS):
            self.speed = min(8, self.speed * 2)
        elif k in (pygame.K_MINUS, pygame.K_KP_MINUS):
            self.speed = max(1, self.speed // 2)
        elif k in (pygame.K_d, pygame.K_k, pygame.K_p, pygame.K_b):
            if k == pygame.K_d: self.lose = not self.lose
            if k == pygame.K_k: self.lora_down = not self.lora_down
            if k == pygame.K_p: self.ignore = not self.ignore
            if k == pygame.K_b: self.baseline = not self.baseline
            self.restart()
        elif k == pygame.K_TAB:
            self.sel = (self.sel + (-1 if mods & pygame.KMOD_SHIFT else 1)) % len(UI_ORDER)
        elif k in (pygame.K_LEFTBRACKET, pygame.K_RIGHTBRACKET):
            name = UI_ORDER[self.sel]
            P.nudge(name, 1 if k == pygame.K_RIGHTBRACKET else -1)
            self.user_params[name] = getattr(P, name)
            self.restart()
        elif k == pygame.K_BACKSPACE:
            self.user_params = {}
            self.restart()
        return True

    # ---------------------------------------------------------------- drawing helpers
    def to_px(self, x, y):
        c = self.world.aircraft["U1"]
        return (MAP / 2 + (x - c.x) * SCALE, MAP / 2 - (y - c.y) * SCALE)

    def text(self, s, pos, color=TEXT, font=None, center=False):
        img = (font or self.f).render(s, True, color)
        if center:
            pos = (pos[0] - img.get_width() / 2, pos[1])
        self.screen.blit(img, pos)
        return img.get_width()

    @staticmethod
    def wrap(s, n, indent="   "):
        out, line = [], ""
        for word in s.split(" "):
            if line and len(line) + len(word) + 1 > n:
                out.append(line)
                line = indent + word
            else:
                line = f"{line} {word}" if line else word
        return out + [line]

    def color_of(self, a):
        if not a.unit:
            return COLORS[a.equipment]
        if not self.world.system_on:
            return COLORS["device"]
        adv = a.unit.advisory
        if adv and adv.auto:
            return COLORS["AUTO"]
        return COLORS["RA"] if adv else COLORS[a.unit.status]

    def draw(self):
        self.screen.fill(BG)
        self.draw_map()
        self.draw_banner()
        self.draw_panel()

    # ---------------------------------------------------------------- map
    def draw_map(self):
        scr, w = self.screen, self.world
        c = w.aircraft["U1"]
        step = 1000.0
        x0 = math.floor((c.x - MAP / 2 / SCALE) / step) * step
        y0 = math.floor((c.y - MAP / 2 / SCALE) / step) * step
        for i in range(int(MAP / SCALE / step) + 2):
            px, _ = self.to_px(x0 + i * step, 0)
            _, py = self.to_px(0, y0 + i * step)
            pygame.draw.line(scr, GRID, (px, 0), (px, MAP))
            pygame.draw.line(scr, GRID, (0, py), (MAP, py))
        centre = (MAP / 2, MAP / 2)
        for r, label in ((MILE, "1 mi"), (3 * MILE, "3 mi")):
            pygame.draw.circle(scr, RING, centre, r * SCALE, 1)
            self.text(label, (MAP / 2 + 4, MAP / 2 - r * SCALE - 16), QUIET, self.fs)
        pygame.draw.circle(scr, (70, 52, 52), centre, P.H_SEP * SCALE, 1)

        u = c.unit if w.system_on else None
        if u:
            self.draw_beliefs(u, c)

        for a in w.aircraft.values():
            if a.unit and a.unit.advisory and a.unit.advisory.threat_id in w.aircraft:
                o = w.aircraft[a.unit.advisory.threat_id]
                pygame.draw.line(scr, COLORS["RA"], self.to_px(a.x, a.y), self.to_px(o.x, o.y), 1)

        for a in w.aircraft.values():
            self.draw_aircraft(a, c)
        pygame.draw.line(scr, (40, 52, 74), (MAP, 0), (MAP, H), 2)

    def draw_beliefs(self, u, c):
        """What U1's unit believes: predicted closest point, radar estimates, proximity rings."""
        scr = self.screen
        top = u.top
        if top and top.level != "CLEAR" and top.other_id in u.tracks:
            o = u.tracks[top.other_id].extrapolate(self.world.t)
            t = top.t_cpa
            hc = math.radians(c.heading)
            mx, my = c.x + c.speed * math.sin(hc) * t, c.y + c.speed * math.cos(hc) * t
            ox, oy = o.x + o.vx * t, o.y + o.vy * t
            col = COLORS["RA"] if top.level == "RA" else COLORS["TA"]
            p1, p2 = self.to_px(mx, my), self.to_px(ox, oy)
            for p in (p1, p2):
                pygame.draw.line(scr, col, (p[0] - 6, p[1] - 6), (p[0] + 6, p[1] + 6), 2)
                pygame.draw.line(scr, col, (p[0] - 6, p[1] + 6), (p[0] + 6, p[1] - 6), 2)
            pygame.draw.line(scr, col, p1, p2, 1)
            self.text(f"predicted closest {top.d_h:.0f} m in {t:.0f} s",
                      (p1[0] + 10, p1[1] - 18), col, self.fs)
        for tr in u.tracks.values():                      # radar (TIS-B) estimate vs truth
            if tr.source == "tisb":
                e = tr.extrapolate(self.world.t)
                px, py = self.to_px(e.x, e.y)
                pygame.draw.rect(scr, COLORS["transponder"], (px - 7, py - 7, 14, 14), 1)
                self.text("radar estimate", (px + 9, py - 22), COLORS["transponder"], self.fs)
        for pid, rng, dv in u.proximity:                   # passive: range only, no direction
            if rng < 2 * MILE and abs(dv) < 300 * FT:
                pygame.draw.circle(scr, COLORS["PROX"], (MAP / 2, MAP / 2), rng * SCALE, 1)
                self.text(f"{pid}: somewhere on this ring ({dv / FT:+.0f} ft)",
                          (MAP / 2 - 120, MAP / 2 + rng * SCALE + 4), COLORS["PROX"], self.fs)

    def draw_aircraft(self, a, c):
        scr = self.screen
        col = self.color_of(a)
        dim = tuple(v // 2 for v in col)
        for (tx, ty) in a.trail[-120:]:
            pygame.draw.circle(scr, dim, self.to_px(tx, ty), 1)
        h = math.radians(a.heading)
        px, py = self.to_px(a.x, a.y)
        fx, fy = self.to_px(a.x + a.speed * math.sin(h) * 30, a.y + a.speed * math.cos(h) * 30)
        pygame.draw.line(scr, dim, (px, py), (fx, fy), 1)
        pts = []
        for ang, r in ((0, 13), (140, 10), (180, 4), (220, 10)):
            t = math.radians(a.heading + ang)
            pts.append((px + r * math.sin(t), py - r * math.cos(t)))
        if a.equipment == "none":
            pygame.draw.polygon(scr, col, pts, 1)            # hollow: nobody can detect it
        else:
            pygame.draw.polygon(scr, col, pts)
        # put the label on the side away from U1 so close pairs don't overlap
        left = a is not c and a.x < c.x
        dalt = (a.alt - c.alt) / FT / 100
        rel = "" if a is c else f" {dalt:+.0f}" + ("^" if a.vz > 0.5 else "v" if a.vz < -0.5 else "")
        tag = {"none": " (undetected)", "transponder": " (Mode C)"}.get(a.equipment, "")
        l1 = f"{a.name}{rel}{tag}"
        l2 = f"{a.alt / FT:,.0f} ft {a.speed / KT:.0f} kt"
        wmax = max(self.fs.size(l1)[0], self.fs.size(l2)[0])
        lx = px - 14 - wmax if left else px + 14
        self.text(l1, (lx, py - 10), col, self.fs)
        self.text(l2, (lx, py + 4), QUIET, self.fs)

    def draw_banner(self):
        """Large cockpit-style instruction for U1 across the top of the map."""
        w = self.world
        u = w.aircraft["U1"].unit
        if not w.system_on or u is None:
            msg, col = "OUR UNITS OFF (baseline)", DIM
        else:
            msg = u.message.upper()
            adv = u.advisory
            col = COLORS["AUTO"] if adv and adv.auto else COLORS["RA"] if adv else \
                COLORS.get(u.status, COLORS["CLEAR"])
        if msg in ("CLEAR",):
            self.text("U1: clear", (16, 12), COLORS["CLEAR"], self.f)
            return
        pygame.draw.rect(self.screen, (10, 14, 22), (0, 0, MAP, 52))
        pygame.draw.rect(self.screen, col, (0, 0, MAP, 52), 3)
        self.text(msg, (MAP / 2, 11), col, self.fbanner, center=True)

    # ---------------------------------------------------------------- panel
    def draw_panel(self):
        scr, w = self.screen, self.world
        pygame.draw.rect(scr, PANEL, (MAP, 0, W - MAP, H))
        x, y = MAP + 18, 12
        self.text(self.label, (x, y), QUIET, self.fs)
        y += 18
        self.text(self.spec.get("name", "")[:50], (x, y), TEXT, self.fb)
        y += 22
        self.text(self.spec.get("table_row", "")[:56], (x, y), COLORS["tcas"], self.fs)
        y += 16
        for line in self.wrap(self.spec.get("description", ""), 56, "")[:3]:
            self.text(line, (x, y), QUIET, self.fs)
            y += 15
        state = "PAUSED" if self.paused else f"x{self.speed}"
        self.text(f"t = {w.t:5.1f} s   {state}", (x, y + 2), QUIET, self.fs)
        y += 22

        flags = [("D", "lose PROPOSE/ACK", self.lose or w.lose_negotiation),
                 ("K", "P2P link down", self.lora_down or w.lora_down),
                 ("P", "U1 pilot ignores", self.ignore or w.aircraft["U1"].pilot_ignores),
                 ("B", "baseline: units off", self.baseline)]
        for i, (key, label, on) in enumerate(flags):
            self.text(f"[{key}] {label}: {'ON' if on else 'off'}", (x + (i % 2) * 220, y),
                      COLORS["TA"] if on else QUIET, self.fs)
            if i % 2:
                y += 16
        y += 8

        for a in w.aircraft.values():
            col = self.color_of(a)
            pygame.draw.rect(scr, col, (x, y + 4, 9, 9))
            equip = EQUIPMENT[a.equipment][3] + (" + ADS-B Out" if a.has_unit and a.adsb_out else "")
            self.text(f"{a.name}: {equip}", (x + 16, y), col, self.fs)
            y += 15
            if a.unit and w.system_on:
                adv = a.unit.advisory
                extra = ""
                if adv:
                    who = "AUTO" if adv.auto else "holding" if adv.done_turning or adv.action == "HOLD" \
                        else "pilot turning" if a.unit.pilot_turning else \
                        "pilot IGNORING" if a.pilot_ignores else "pilot reacting"
                    extra = f"  ({adv.mode.lower()}, {who})"
                self.text(f"{a.unit.message}{extra}"[:55], (x + 16, y), QUIET, self.fs)
                y += 15
            elif a.equipment == "tcas" and a.tcas_ra:
                self.text(f"TCAS RA: {a.tcas_ra[0]}", (x + 16, y), COLORS["tcas"], self.fs)
                y += 15
            y += 4

        y += 2
        self.text("Closest approach (within 150 m vertically)", (x, y), TEXT, self.fs)
        y += 16
        for (p, q), (dh, dv) in sorted(w.min_sep.items()):
            col = COLORS["RA"] if dh < 150 else (COLORS["TA"] if dh < 300 else COLORS["CLEAR"])
            self.text(f"{p}-{q}: {dh:5.0f} m ({dh / FT:5.0f} ft)" + ("  NEAR MID-AIR" if dh < 150 else ""),
                      (x + 8, y), col, self.fs)
            y += 15
        y += 8

        self.text("Parameters   TAB select  [ ] change  BKSP reset", (x, y), TEXT, self.fs)
        y += 17
        for i, name in enumerate(UI_ORDER):
            val, unit, desc = getattr(P, name), SPEC[name][4], SPEC[name][5]
            changed = val != SPEC[name][0]
            if i == self.sel:
                pygame.draw.rect(scr, SEL, (x - 4, y - 1, W - MAP - 28, 15))
            col = COLORS["TA"] if changed else (TEXT if i == self.sel else QUIET)
            num = f"{val:g} {unit}".strip()
            self.text(f"{name:<12s}{num:>9s}  {desc}"[:57], (x, y), col, self.fs)
            y += 15
        y += 8

        self.text("Event log", (x, y), TEXT, self.fs)
        y += 16
        lines = []
        for t, e in w.events[-30:]:
            lines += self.wrap(f"{t:5.1f}s {e}", 56)
        room = max(0, (H - y - 40) // 14)
        for line in lines[-room:] if room else []:
            self.text(line, (x, y), QUIET, self.fs)
            y += 14

        self.text("1-0/arrows scenario  G random  F5 reload  R restart", (x, H - 34), DIM, self.fs)
        self.text("SPACE pause  +/- speed  ESC quit", (x, H - 19), DIM, self.fs)


def parse_args(argv):
    start, params = 1, {}
    i = 0
    while i < len(argv):
        a = argv[i]
        if a == "--set":
            k, v = argv[i + 1].split("=")
            params[k.strip()] = float(v)
            i += 1
        elif a.isdigit():
            start = int(a)
        elif a.endswith(".json"):
            start = a
        i += 1
    return start, params


def main():
    start, params = parse_args(sys.argv[1:])
    App(start, params).run()
    pygame.quit()


if __name__ == "__main__":
    main()
