"""CLI: generate the multi-sensor scenario battery to CSV fixtures + checksums.

    python -m generator.generate                 # write all scenarios, seed 0
    python -m generator.generate --seed 1        # a second seed
    python -m generator.generate --scenario sim_ms_headon
    python -m generator.generate --verify        # in-memory: generate twice,
                                                 # assert byte-identical (no writes)

Fixtures land in <out>/<scenario>_s<seed>/ with a CHECKSUMS.txt drift guard.
Everything is a pure function of (scenario, seed); numpy runs single-threaded.
"""

from __future__ import annotations

import argparse
import os
import sys

from .geo import Datum
from . import scenarios as scen
from . import sensors
from .truth import ScenarioSpec, build_truth
from .writer import (AIS_COLS, CAMERA_COLS, OWNSHIP_COLS, RADAR_COLS, TRUTH_COLS,
                     rows_to_csv, sha256_text, truth_rows)

_HERE = os.path.dirname(os.path.abspath(__file__))
_DEFAULT_OUT = os.path.dirname(_HERE)  # tests/fixtures/sim_multisensor/


def build_scenario_csvs(spec: ScenarioSpec) -> dict[str, str]:
    """Return {filename: csv_text} for one scenario. Pure; no I/O."""
    datum = Datum(spec.own.lat_deg, spec.own.lon_deg)
    own, targets = build_truth(spec)
    files = {
        "ownship.csv": rows_to_csv(
            sensors.ownship_poses(spec, datum, own), OWNSHIP_COLS),
        "ais.csv": rows_to_csv(
            sensors.ais_reports(spec, datum, targets), AIS_COLS),
        "radar_plots.csv": rows_to_csv(
            sensors.radar_plots(spec, own, targets), RADAR_COLS),
        "camera_bearings.csv": rows_to_csv(
            sensors.camera_bearings(spec, own, targets), CAMERA_COLS),
        "truth.csv": rows_to_csv(
            truth_rows(datum, targets, spec.duration_s), TRUTH_COLS),
    }
    files["meta.txt"] = _meta_text(spec, targets)
    return files


def _meta_text(spec: ScenarioSpec, targets) -> str:
    lines = [
        f"scenario: {spec.name}", f"seed: {spec.seed}",
        f"duration_s: {spec.duration_s}", f"base_dt_s: {spec.base_dt_s}",
        f"datum_lat_lon: {spec.own.lat_deg},{spec.own.lon_deg}",
        f"own_mmsi: {spec.own.mmsi}", f"clutter_model: {spec.clutter_model}",
        f"ais_dropout: {spec.ais_dropout}",
        f"ownship_stale_gap: {spec.ownship_stale_gap}",
        f"ownship_heading_fault: {spec.ownship_heading_fault}",
        "vessels:",
    ]
    for tk in targets:
        lines.append(
            f"  mmsi={tk.mmsi} name={tk.name} motion={tk.meta.get('motion')} "
            f"encounter={tk.meta.get('encounter')} nav_status={tk.nav_status} "
            f"radar={tk.radar} ais={tk.ais} camera={tk.camera}")
    return "\n".join(lines) + "\n"


def _scenario_dir(out: str, spec: ScenarioSpec) -> str:
    return os.path.join(out, f"{spec.name}_s{spec.seed}")


def write_all(out: str, seed: int, only: str | None) -> list[tuple[str, str, str]]:
    """Write scenarios; return [(scenario, filename, sha256)] manifest."""
    manifest = []
    for make in scen.BATTERY:
        spec = make(seed)
        if only and spec.name != only:
            continue
        files = build_scenario_csvs(spec)
        d = _scenario_dir(out, spec)
        os.makedirs(d, exist_ok=True)
        for fname, text in files.items():
            with open(os.path.join(d, fname), "w", newline="") as f:
                f.write(text)
            if fname.endswith(".csv"):
                manifest.append((f"{spec.name}_s{seed}", fname, sha256_text(text)))
        print(f"  wrote {spec.name}_s{seed}: {len(files)} files -> {d}")
    return manifest


def verify_determinism(seed: int) -> bool:
    """Generate every scenario twice in memory; assert byte-identical."""
    ok = True
    for make in scen.BATTERY:
        spec = make(seed)
        a = build_scenario_csvs(spec)
        b = build_scenario_csvs(spec)
        for fname in a:
            ha, hb = sha256_text(a[fname]), sha256_text(b[fname])
            status = "OK" if ha == hb else "MISMATCH"
            if ha != hb:
                ok = False
            print(f"  {spec.name}/{fname}: {status} {ha[:12]}")
    return ok


def main(argv=None):
    p = argparse.ArgumentParser(description="Generate sim multi-sensor fixtures.")
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--out", default=_DEFAULT_OUT)
    p.add_argument("--scenario", default=None, help="only this scenario name")
    p.add_argument("--verify", action="store_true",
                   help="in-memory double-generation determinism check (no writes)")
    args = p.parse_args(argv)

    if args.verify:
        print(f"Determinism verify (seed {args.seed}):")
        ok = verify_determinism(args.seed)
        print("PASS" if ok else "FAIL")
        return 0 if ok else 1

    print(f"Generating battery (seed {args.seed}) -> {args.out}")
    manifest = write_all(args.out, args.seed, args.scenario)
    checks_path = os.path.join(args.out, "CHECKSUMS.txt")
    # merge with any existing checksums for other seeds/scenarios
    existing = {}
    if os.path.exists(checks_path):
        with open(checks_path) as f:
            for line in f:
                parts = line.split()
                if len(parts) == 2:
                    existing[parts[1]] = parts[0]
    for scen_name, fname, h in manifest:
        existing[f"{scen_name}/{fname}"] = h
    with open(checks_path, "w") as f:
        for key in sorted(existing):
            f.write(f"{existing[key]}  {key}\n")
    print(f"Wrote {len(manifest)} checksums -> {checks_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
