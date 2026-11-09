# The math behind passive positioning

How our unit locates a transponder-only aircraft without transmitting anything.
Every measurement gives one equation relating the unknown position X to things we
know. Each equation is a curve of possible positions; combining curves leaves one point.

Code: `logic/passive_locator.py` (estimator), `sim/cockpit_world.py` (measurements).

## Setup

Positions in metres: x = east, y = north, z = altitude.

| Symbol | Meaning | Known from |
| --- | --- | --- |
| U | our aircraft | our GPS |
| A | a TCAS airliner | its ADS-B Out |
| V | another aircraft with our device | its GPS, in its LoRa STATE message |
| X | the transponder-only target (unknown) | what we solve for |
| c | speed of light | 299.79 m per microsecond |
| delta | transponder reply delay | fixed 3.0 us (+/-0.5 us tolerance) |

Distance: `|P - Q| = sqrt((x1-x2)^2 + (y1-y2)^2 + (z1-z2)^2)`

The Mode C reply gives the target's altitude, so z is known. Two unknowns remain
(x and y): one equation gives a curve, two give a point.

## 1. Timing an airliner's question -> ellipse

The airliner interrogates at an unknown time t0.

```
we hear the question:   t1 = t0 + |A - U| / c
we hear the answer:     t2 = t0 + |A - X| / c + delta + |X - U| / c

dt = t2 - t1 = ( |A - X| + |X - U| - |A - U| ) / c + delta

=>  |A - X| + |X - U| = c (dt - delta) + |A - U|  =  S
```

t0 cancels. A constant sum of distances to two points is an ellipse with foci A and U.

Worked example: U = (0, 0), A = (6000, 8000), |A - U| = 10,000 m. True X = (2400, 1600):
|A - X| = 7,343 m, |X - U| = 2,884 m, S = 10,227 m, so we would measure
dt = 227 / 299.79 + 3.0 = 3.76 us.

Error: +/-0.5 us of transponder tolerance = +/-150 m in S.

How tight one ellipse is: with a = S/2 and f = |A - U|/2, the target's distance from us
lies between a - f and a + f (here 114 m to 10,114 m: very vague).

## 2. Two of our units hear the same answer -> hyperbola

The target answers at an unknown time tr; we (U) and another unit (V) both hear it.

```
tU = tr + |X - U| / c
tV = tr + |X - V| / c

=>  |X - U| - |X - V| = c (tU - tV)  =  D
```

tr cancels, and we don't even need to know who asked. A constant difference of
distances to two points is a hyperbola with foci U and V. Both units timestamp with
GPS pulse-per-second (tens of ns); with receiver sampling, about 200 ns total, i.e.
+/-60 m in D.

Same example: V = (1500, 6000), |X - V| = 4,491 m, D = 2,884 - 4,491 = -1,607 m
(a 5.36 us difference). The ellipse and this hyperbola cross at (2400, 1600).

## 3. Direction antennas -> line

```
theta = atan2(x - xU, y - yU)        (bearing from north, clockwise)
```

Two antennas d apart see a phase difference `dphi = 2 pi d sin(theta) / lambda`,
lambda = c / 1090 MHz = 0.275 m. With d = lambda / 2 (about 14 cm), dphi = pi sin(theta).
Four antennas cover all directions. Typical accuracy +/-15 deg.

## 4. Signal strength -> rough circle

```
P_received = P_sent + G - 20 log10(4 pi r / lambda)
=>  r = (lambda / 4 pi) 10^((P_sent + G - P_received) / 20)
```

The target's transmit power is unknown (+/-3 dB between models) and the signal fades
with attitude, so the error is multiplicative, about +/-40 %. The code compares
ln(r_measured) with ln(r) with sigma 0.35.

## 5. Combining: likelihoods and the particle filter

For a candidate position X, each measurement has a residual:

```
residual = measured - predicted(X)       e.g.  S - (|A - X| + |X - U|)
L = exp( -0.5 (residual / sigma)^2 )
```

Multiply the likelihoods of all measurements; equivalently, the best X minimises
`sum (residual_i / sigma_i)^2` (weighted least squares).

The particle filter keeps 3,000 candidates, each with a velocity:

1. Predict: `x += vx dt`, `y += vy dt`, plus small random acceleration (target may turn).
2. Update: `weight *= L` for every new measurement.
3. Resample: copy high-weight particles, drop low-weight ones.

Example (ellipse, sigma = 120 m): a particle 20 m off the ellipse keeps
exp(-0.5 (20/120)^2) = 0.986 of its weight; one 400 m off keeps 0.004.

The weighted mean of the particles is the position; their spread is the uncertainty
sigma shown in the cockpit. This is the same predict/update loop as a Kalman filter,
but it copes with curved measurements and with two-candidate ambiguity.

## 6. Why crossing angle matters (dilution of precision)

```
position error  ~  sigma / sin(crossing angle)
```

Curves crossing at 90 deg: error ~ sigma. At 20 deg: about 3x worse.

- One airliner alone: successive ellipses share foci and shift slowly, so they cross at
  shallow angles -> 500 m+ errors.
- A second unit of ours or a second airliner: different foci, steep crossing -> ~50 m.

## 7. Measured in simulation (no ground radar, 12 runs each, error after 20 s)

| Available | Typical | 90th percentile |
| --- | --- | --- |
| One airliner only | 530 m | 1,750 m |
| + direction antennas (+/-30 deg) | 380 m | 875 m |
| + direction antennas (+/-15 deg) | 290 m | 540 m |
| Two airliners | 57 m | 67 m |
| One airliner + a second unit of ours | 41 m | 77 m |

## 8. How the unit stays honest about uncertainty

- FIX: at least two independent constraints (each interrogator's ellipse, each partner's
  hyperbola and the radar beam line count 1; TIS-B counts 2; a +/-15 deg bearing 0.5)
  and particle spread below 450 m. The reported sigma is the spread.
- ROUGH: a single source. Tests showed the spread understates the true error, so the
  reported sigma is inflated (x2.5, or x1.6 with direction antennas).
- The protected radius for a passive track is radius + 2 sigma, capped at 3x the
  normal radius; uncertainty alone never counts as "already inside" the radius.

## Summary

| Measurement | Equation | Shape | Typical error |
| --- | --- | --- | --- |
| Mode C | z = reported altitude | plane | +/-100 ft |
| Airliner timing | \|A-X\| + \|X-U\| = S | ellipse | +/-150 m in S |
| Two of our units | \|X-U\| - \|X-V\| = D | hyperbola | +/-60 m in D |
| Direction antennas | theta = atan2(dx, dy) | line | +/-15-30 deg |
| Signal strength | r from received power | circle | +/-40 % |
