"""Bounded live acceptance against a running loopback backend (no robot writes)."""
import argparse
import json
from pathlib import Path
import time

from board_verify import request


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--seconds", type=float, default=60)
    parser.add_argument("--output", type=Path, default=Path("test-results/rk3576/live.json"))
    args = parser.parse_args()
    original = request(args.port, "/api/config")
    samples = []
    started = time.monotonic()
    changed = False
    try:
        while time.monotonic() - started < args.seconds:
            elapsed = time.monotonic() - started
            if elapsed >= args.seconds / 3 and not changed:
                request(args.port, "/api/config", {"min_x": -5, "max_x": 5, "min_y": -5, "max_y": 5})
                changed = True
            state = request(args.port)
            samples.append({"elapsed_s": elapsed, **{key: state[key] for key in (
                "status", "frame", "error", "valid_count", "roi_count", "received_count",
                "content_version", "analyzed_content_version", "processed_count", "receive_hz",
                "content_hz", "receive_age_s", "processed_age_s", "processing_ms", "reconnects")},
                "objects": len(state["objects"]), "config": state["config"]})
            time.sleep(max(0, started + len(samples) * .2 - time.monotonic()))
    finally:
        request(args.port, "/api/config", original)
    healthy = [s for s in samples if s["status"] in ("clear", "detected")]
    report = {"elapsed_s": time.monotonic() - started, "samples": samples,
              "healthy_samples": len(healthy), "abnormal_samples": len(samples) - len(healthy),
              "max_roi_points": max(s["roi_count"] for s in samples),
              "max_candidates": max(s["objects"] for s in samples),
              "config_restored": request(args.port, "/api/config") == original,
              "source_timestamp_available": False,
              "fresh_acquisition_proven": False}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps({k: v for k, v in report.items() if k != "samples"}), flush=True)


if __name__ == "__main__":
    main()
