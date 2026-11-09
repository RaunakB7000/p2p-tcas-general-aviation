# Peer-to-peer TCAS for General Aviation (Devils Invent, PS 2)

Low-cost collision avoidance for small aircraft. Units talk to each other over a
P2P radio link, predict closest approach, warn the pilot, negotiate who turns
which way, and auto-avoid if the pilot ignores the warning. They also use ADS-B,
radar rebroadcast (TIS-B) and passive transponder listening for aircraft that
don't carry the device.

Airliners carry TCAS; small general-aviation aircraft often don't, so two small
planes closing on each other have little automated protection.
This project is a simulation of a cheap peer-to-peer alternative: a pure-Python
logic core, a 2D scenario simulator, an interactive cockpit view, and a test
harness that compares every encounter with and without the system.

## Highlights

- **Closest-point-of-approach (CPA) prediction** with traffic advisories (TA) and
  resolution advisories (RA), including distance triggers for slow-closure and
  overtaking encounters.
- **Peer-to-peer negotiation:** two units agree on who turns which way, with
  timeouts and a one-sided fallback when messages are lost.
- **Auto-avoid:** if the pilot ignores an alert, the unit manoeuvres on its own.
- **Layered detection for aircraft without our device:** ADS-B Out, radar
  rebroadcast (TIS-B), and *passive* listening to transponder replies.
- **Passive positioning with a particle filter** (`logic/passive_locator.py`): a
  Kalman-style predict/update loop over thousands of hypotheses locates a
  transponder-only aircraft from timing ellipses and hyperbolas, typically within
  20-80 m when two independent constraints exist. The maths is written up in
  [`docs/MATH.md`](docs/MATH.md).
- **Honest failure cases:** scenarios 07 and 08 (transponder-only with no
  TIS-B, and a fully blind aircraft) are included on purpose to show where the
  system cannot prevent a collision.

## Results (`python -m tests.run_scenarios`)

Closest horizontal distance while within 150 m vertically ("NMAC" marks a
near mid-air collision):

| Scenario | No system | System on |
| --- | --- | --- |
| 01 Head-on, both with device | 30 m NMAC | 673 m |
| 02 Converging, both with device | 0 m NMAC | 718 m |
| 03 P2P link down, ADS-B backup | 30 m NMAC | 652 m |
| 04 Device vs ADS-B Out only | 0 m NMAC | 518 m |
| 05 Device vs TCAS airliner | 3274 m | 3274 m |
| 06 Device vs transponder (TIS-B radar) | 0 m NMAC | 713 m |
| 07 Transponder only, no TIS-B | 0 m NMAC | 0 m NMAC (not prevented) |
| 08 Blind aircraft | 0 m NMAC | 0 m NMAC (not prevented) |
| 09 Three aircraft, mixed equipment | 30 m NMAC | 658 m |
| 10 Overtaking | 20 m NMAC | 576 m |

## Project layout

```
logic/      the avoidance logic (no graphics): CPA, negotiation, passive locator, unit state machine
sim/        pygame front-ends: scenario simulator, cockpit view, passive-listening demo
scenarios/  JSON encounter definitions (copy one and edit it)
tests/      headless scenario runner with/without the system, plus random encounters
docs/       MATH.md: the passive-positioning derivation with worked numbers
```

## Setup

Python 3.11 is the recommended version (3.14 has no pygame installers yet); it
has also been run on Python 3.13.

```bash
# pip
python -m venv .venv && source .venv/bin/activate   # Windows: .venv\Scripts\activate
pip install -r requirements.txt

# or Anaconda
conda create -n p2p_tcas python=3.11 -y
conda activate p2p_tcas
pip install -r requirements.txt
```

The tests (`python -m tests.run_scenarios`) do not need a display; the
simulator, cockpit and demo open a pygame window.

## Run (always from this folder)

```bash
python -m sim.simulator                         # scenario 1
python -m sim.simulator 6                       # scenario file number 6
python -m sim.simulator scenarios/my_test.json  # any scenario file
python -m sim.simulator 1 --set RA_TIME=15 --set PILOT_DELAY=4

python -m tests.run_scenarios                   # every scenario, with/without the system
python -m tests.run_scenarios 3 7 -v            # files 3 and 7, with full event log
python -m tests.run_scenarios --set H_SEP=800   # same, with changed parameters
python -m tests.run_scenarios --random 50       # 50 random encounters
```

## Simulator keys

| Key | Does |
| --- | --- |
| 1-9, 0 | Scenario file 1-10 |
| LEFT / RIGHT | Previous / next scenario file |
| G | New random encounter (random equipment and geometry) |
| F5 | Reload scenario files from disk (edit a JSON, press F5) |
| R | Restart |
| SPACE | Pause |
| + / - | Speed x1 to x8 |
| D | Lose PROPOSE/ACK messages (shows the one-sided fallback) |
| K | P2P radio link down (shows ADS-B as backup, if fitted) |
| P | U1's pilot ignores the alert (shows AUTO-AVOID) |
| B | Baseline: our units off (shows the collision we prevent) |
| TAB / SHIFT+TAB | Select a parameter in the panel |
| [ / ] | Decrease / increase it (restarts the scenario) |
| BACKSPACE | Reset all parameters to defaults |

On the map: the banner is U1's instruction to the pilot. Red X marks show where
U1's unit predicts the closest approach. A hollow square is a radar (TIS-B)
estimate; an amber ring is a passive transponder hit (range known, direction
unknown). A hollow aircraft is blind: nothing can detect it.

## Cockpit view (you are the pilot)

```bash
python -m sim.cockpit
```

One encounter that uses every detection layer: N512 (our device, P2P link and
negotiation), AAL12 (airliner, ADS-B + TCAS), N88W (ADS-B only), N734 (transponder
only, located passively), N620 (our device, 1,500 ft above: it listens too and shares
its timings), and a glider with no electronics (undetectable).

No ground radar by default. N734 is located from the airliner's ellipse crossed with
N620's hyperbola: a FIX within about 5 s, 20-80 m accuracy. The math, with worked
numbers and measured accuracy for each option, is in `docs/MATH.md`.

- Windshield: what you'd see, plus HUD boxes drawn where YOUR UNIT THINKS each
  aircraft is (a passive estimate's box grows with its uncertainty).
- Traffic display: TCAS-style symbols; for passive targets the orange dots are the
  particle filter's guesses, purple/blue curves are timing ellipses (airliner / radar),
  green curves are hyperbolas from our other units, cyan lines are antenna bearings.
- System log: every detection step, tagged (ADS-B, LINK, RX, MATCH, FIX, TRACK, TA,
  RA, NEGOT, AUTO, TRUTH).

| Key | Does |
| --- | --- |
| LEFT / RIGHT, UP / DOWN | Fly: turn, climb / descend (hold) |
| P | Simulated pilot follows advisories / you fly (auto-avoid after 5 s) |
| G | Ground radar (OFF by default: low-altitude coverage can't be counted on) |
| T | Airliner TCAS interrogations |
| N | Network: other units of ours share when they heard each reply (hyperbolas) |
| D | Direction antennas on your unit (bearing lines) |
| O | Your ADS-B Out (enables TIS-B) |
| TAB | Select a target: "how do we know about it?" |
| Z | Traffic display range |
| SPACE, + / -, R, ESC | Pause, speed, restart, quit |

Passive positioning lives in `logic/passive_locator.py` (a particle filter: the
Kalman-style predict/update loop with thousands of guesses). It reports a FIX only
with two independent constraints (airliner + a second unit of ours, two airliners,
radar, or TIS-B); with one interrogator it reports a ROUGH position with inflated
uncertainty, and the protected radius grows to match (radius + 2 sigma, capped at 3x).

| No radar; what's available | Typical error | Quality |
| --- | --- | --- |
| Airliner + a second unit of ours (default) | 20-80 m | FIX |
| Two airliners | ~60 m | FIX |
| Airliner + direction antennas | ~300 m | ROUGH |
| Airliner only | ~500 m (up to 1.7 km) | ROUGH |
| Nobody interrogating | not detectable | - |

## Passive transponder listening demo

```bash
python -m sim.passive_demo
```

Our plane meets a Cessna that has only a transponder. We transmit nothing: ground
radar (and any TCAS aircraft nearby) asks the Cessna; it answers on 1090 MHz; our
unit listens and learns its altitude, a rough distance, and whether it is closing.

| Control | Does |
| --- | --- |
| Click buttons, or G / T / M / A / F | Radar on/off, TCAS airliner, Mode C/S, direction antennas, pilot follows advice |
| Drag the Cessna | Put it anywhere on the map |
| UP / DOWN, LEFT / RIGHT | Cessna altitude (100 ft) / heading (10 deg) |
| 1-4 | Speed 0.5x, 1x, 2x, 4x |
| SPACE / BACKSPACE | Pause / restart |

Panel: (1) every reply heard, colour-coded by who asked; (2) the last question and
answer as pulses, decoded; (3) altitude tape, us vs Cessna; (4) distance estimate
band vs the truth (white line), with the trend.

## Scenario files (`scenarios/*.json`)

One file per encounter. They load in file-name order. Copy one and edit it.

```json
{
  "name": "Head-on: both have our device",
  "table_row": "Device vs Device",
  "description": "Shown under the title in the panel.",
  "duration_s": 150,
  "seed": 1,
  "params":  { "RA_TIME": 25, "PILOT_DELAY": 2.5 },
  "options": { "pilot_ignores": ["U1"], "lose_negotiation": false, "lora_down": false },
  "aircraft": [
    { "id": "U1", "equipment": "device", "adsb_out": false,
      "speed_kt": 100, "alt_ft": 5000, "heading": 0 },
    { "id": "U2", "equipment": "device", "speed_kt": 100, "heading": 180,
      "conflict_in_s": 70, "miss_m": 30, "alt_diff_ft": 0, "vz_fpm": 0 }
  ]
}
```

- `U1` is required: it is your aircraft and the map is centred on it.
- `equipment`: `device` (our unit), `adsb` (ADS-B Out only), `tcas` (airliner:
  TCAS + ADS-B Out), `transponder` (Mode C only), `none` (blind).
- `adsb_out` (true/false) adds ADS-B Out to a `device` aircraft. It matters twice:
  others can see you on ADS-B if the P2P link fails, and it is what makes the
  ground stations send you TIS-B radar tracks.
- Other aircraft are placed automatically so they would hit U1 after
  `conflict_in_s` seconds, missing by `miss_m` metres and `alt_diff_ft` feet.
  Or give an exact start with `"x_m"`, `"y_m"`, `"alt_ft"` (metres east and
  north of the point U1 reaches halfway through the encounter; U1 starts
  half the encounter's distance before that point).
- `params` override any parameter below for this scenario only.
- Everything is optional except `aircraft`, `id`, and U1.

## Parameters (`logic/params.py`)

| Name | Default | Meaning |
| --- | --- | --- |
| H_SEP | 500 m | horizontal protected radius |
| V_SEP | 150 m | vertical protected half-height |
| TA_TIME / RA_TIME | 45 / 25 s | time to closest approach for traffic / resolution advisory |
| TA_DMOD / RA_DMOD | 2000 / 1200 m | distance triggers for slow closure |
| TURN_RATE | 3 deg/s | avoidance turn rate |
| MAX_DEV | 90 deg | largest heading change |
| PILOT_DELAY | 2.5 s | pilot reaction time |
| AUTO_DELAY | 5 s | auto-avoid engages if the pilot hasn't turned |
| ACK_TIMEOUT / PROPOSE_TIMEOUT | 1 / 1.5 s | negotiation timeouts |
| TISB_MARGIN | 1.6 x | bigger protected radius for radar tracks |
| MSG_LOSS | 0 | chance each P2P message is lost |
| RADIO_RANGE | 8000 m | P2P radio range |
| GPS_NOISE | 5 m | GPS error in broadcasts |
| TISB_NOISE / TISB_PERIOD | 75 m / 5 s | radar track error and update interval |

Priority: defaults, then the scenario's `params`, then `--set` / keyboard changes.

## Included scenarios (match the capability table)

| File | Row | Expected result |
| --- | --- | --- |
| 01 | Device vs Device, head-on | Negotiated, both turn right |
| 02 | Device vs Device, converging | Negotiated better pair than the textbook rule |
| 03 | Device + ADS-B Out both, P2P link down | Each avoids via the other's ADS-B |
| 04 | Device vs ADS-B Out | One-sided avoidance |
| 05 | Device vs TCAS airliner | We warn; its TCAS climbs; we stand down |
| 06 | Device + ADS-B Out vs transponder only | TIS-B radar track, wider margin |
| 07 | Device (no ADS-B Out) vs transponder only | Proximity alert only: collision not prevented |
| 08 | Device vs blind | Undetectable: collision not prevented |
| 09 | Mixed three aircraft | All pairs separated |
| 10 | Overtaking | Distance trigger fires; overtaker passes right |

## Code map

| File | What it is | Ports to ESP32? |
| --- | --- | --- |
| `logic/params.py` | All tunable numbers | yes |
| `logic/models.py` | Track data type, units | yes |
| `logic/cpa.py` | Closest approach, TA/RA levels | yes |
| `logic/negotiation.py` | Rules of the air, negotiated pairs, one-sided fallback | yes |
| `logic/unit.py` | The onboard unit | yes |
| `sim/world.py` | Simulated sky, equipment, radio, airliner TCAS, scenario loader | no |
| `sim/simulator.py` | pygame display | no |
| `tests/run_scenarios.py` | Batch runner | no |
