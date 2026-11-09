"""Headless check of every scenario file, with and without the system.

Run from the project root:
    python -m tests.run_scenarios                 # all files in scenarios/
    python -m tests.run_scenarios 3 7             # only files 3 and 7
    python -m tests.run_scenarios 1 -v            # with the full event log
    python -m tests.run_scenarios --set RA_TIME=15 --set PILOT_DELAY=4
    python -m tests.run_scenarios --random 20     # 20 random encounters
"""
import sys

from sim.world import build, load_scenarios, random_spec

NMAC_H = 150.0     # m   near mid-air collision (horizontal, within the vertical band)


def parse_sets(argv):
    out = {}
    for i, a in enumerate(argv):
        if a == "--set":
            k, v = argv[i + 1].split("=")
            out[k.strip()] = float(v)
    return out


def run(spec, user_params=None, system_on=True, ignores=(), lose=None, verbose=False):
    w = build(spec, user_params, system_on=system_on, extra_ignores=ignores, lose_negotiation=lose)
    while w.t < w.duration:
        w.step(0.1)
    if verbose:
        for t, e in w.events:
            print(f"      {t:6.1f}s  {e}")
    return w


def closest(w):
    return min((v[0] for v in w.min_sep.values()), default=float("inf"))


def main():
    argv = sys.argv[1:]
    verbose = "-v" in argv
    params = parse_sets(argv)
    if "--random" in argv:
        n = int(argv[argv.index("--random") + 1])
        specs = [(f"random #{s}", random_spec(s)) for s in range(1, n + 1)]
    else:
        files = load_scenarios()
        pick = [int(a) for a in argv if a.isdigit() and argv[argv.index(a) - 1] != "--random"]
        specs = [(p, s) for i, (p, s) in enumerate(files, 1) if not pick or i in pick]
    if params:
        print(f"Parameter overrides: {params}\n")

    variants = [("no system", dict(system_on=False)),
                ("system on", dict()),
                ("negotiation lost", dict(lose=True)),
                ("U1 pilot ignores", dict(ignores=("U1",)))]
    print(f"{'scenario':58s} " + " ".join(f"{v[0]:>17s}" for v in variants))
    for path, spec in specs:
        cells = []
        for label, kw in variants:
            w = run(spec, params, verbose=verbose and label == "system on", **kw)
            d = closest(w)
            tag = "-" if d == float("inf") else f"{d:5.0f} m{' NMAC' if d < NMAC_H else '     '}"
            cells.append(f"{tag:>17s}")
        title = f"{spec['name']}  [{spec.get('table_row', '')}]"
        print(f"{title[:58]:58s} " + " ".join(cells))
    print("\nCells = closest horizontal distance while within 150 m vertically ('-' = never close).")


if __name__ == "__main__":
    main()
