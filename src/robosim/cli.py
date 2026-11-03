from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

from .engine import run_file
from .reporting import plot, write_csv, write_json, write_output_bundle, write_summary


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="robosim")
    sub = parser.add_subparsers(dest="command", required=True)
    run = sub.add_parser("run"); run.add_argument("scenario"); run.add_argument("--json"); run.add_argument("--csv"); run.add_argument("--plot"); run.add_argument("--output")
    verify = sub.add_parser("verify"); verify.add_argument("scenario"); verify.add_argument("--output", default="results/verify")
    bench = sub.add_parser("benchmark"); bench.add_argument("scenario"); bench.add_argument("--repeats", type=int, default=3)
    args = parser.parse_args(argv)
    if args.command == "benchmark":
        started = time.perf_counter()
        results = [run_file(args.scenario) for _ in range(args.repeats)]
        print(json.dumps({"repeats": args.repeats, "elapsed_seconds": time.perf_counter() - started,
                          "all_passed": all(r.passed for r in results)}))
        return 0 if all(r.passed for r in results) else 1
    if args.command == "verify" and Path(args.scenario).is_dir():
        paths = sorted(Path(args.scenario).glob("*.yaml"))
        outcomes = []
        for path in paths:
            result = run_file(str(path))
            write_output_bundle(result, Path(args.output) / path.stem)
            outcomes.append(result)
            print(json.dumps(result.metrics()))
        write_summary(outcomes, args.output)
        return 0 if all(item.passed for item in outcomes) else 1
    result = run_file(args.scenario)
    if args.command == "verify":
        print(json.dumps(result.metrics(), indent=2)); return 0 if result.passed else 1
    print(json.dumps(result.metrics(), indent=2))
    if args.json: write_json(result, args.json)
    if args.csv: write_csv(result, args.csv)
    if args.plot: plot(result, args.plot)
    if args.output: write_output_bundle(result, args.output)
    return 0


if __name__ == "__main__":
    sys.exit(main())
