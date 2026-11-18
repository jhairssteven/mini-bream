#!/usr/bin/env python3
"""Local HTTP server for tools/results_viewer.html — browse controller run CSVs.

Usage (host or autonomy container):

  cd .../molo_wpt_follower/tools
  python3 serve_results_viewer.py
  # open http://127.0.0.1:8766/

Scans sibling controller packages' results/ trees for directories that contain
log.csv (gs / ilos / h0 / mpc experiments).
"""

from __future__ import annotations

import argparse
import json
import mimetypes
import re
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Dict, List, Optional
from urllib.parse import parse_qs, unquote, urlparse

TOOLS_DIR = Path(__file__).resolve().parent
MOLO_DIR = TOOLS_DIR.parent
VIEWER_HTML = TOOLS_DIR / "results_viewer.html"

CONTROLLER_RESULTS = [
    ("ground_speed_mpc_ilos", MOLO_DIR / "ground_speed_mpc_ilos" / "results"),
    ("ilos_boat", MOLO_DIR / "ilos_boat" / "results"),
    ("h0_boat", MOLO_DIR / "h0_boat" / "results"),
    ("mpc", MOLO_DIR / "mpc" / "experiments" / "results"),
]

_TS_RE = re.compile(r"^\d{8}T\d{6}Z$")


def _safe_under(root: Path, rel: str) -> Optional[Path]:
    try:
        p = (root / rel).resolve()
        root_r = root.resolve()
        if root_r not in p.parents and p != root_r:
            return None
        return p
    except (OSError, ValueError):
        return None


def _read_json(path: Path) -> Optional[dict]:
    if not path.is_file():
        return None
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None


def discover_runs(max_runs: int = 2000) -> List[Dict[str, Any]]:
    runs: List[Dict[str, Any]] = []
    for controller, root in CONTROLLER_RESULTS:
        if not root.is_dir():
            continue
        for log in sorted(root.rglob("log.csv"), reverse=True):
            run_dir = log.parent
            rel = run_dir.relative_to(MOLO_DIR).as_posix()
            meta = _read_json(run_dir / "result.json") or _read_json(run_dir / "score.json")
            parts = Path(rel).parts
            stamp = next((p for p in parts if _TS_RE.match(p) or p.startswith("tune_")), parts[-2] if len(parts) > 1 else parts[-1])
            label = f"{controller} · {stamp} · {run_dir.name}"
            entry: Dict[str, Any] = {
                "id": rel,
                "controller": controller,
                "label": label,
                "run_dir": rel,
                "stamp": stamp,
                "name": run_dir.name,
                "has_ref": (run_dir / "ref_path.csv").is_file(),
                "has_result": meta is not None,
                "has_config": (run_dir / "config.yaml").is_file(),
            }
            if meta:
                for k in ("rmse", "completion", "pass", "valid", "target_rmse_m", "hypothesis_id", "approach"):
                    if k in meta:
                        entry[k] = meta[k]
            runs.append(entry)
            if len(runs) >= max_runs:
                return runs
    return runs


class Handler(BaseHTTPRequestHandler):
    server_version = "ResultsViewer/1.0"

    def log_message(self, fmt: str, *args) -> None:
        print(f"[results-viewer] {self.address_string()} - {fmt % args}")

    def _send(self, code: int, body: bytes, content_type: str) -> None:
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _json(self, code: int, obj: Any) -> None:
        raw = json.dumps(obj, indent=2).encode("utf-8")
        self._send(code, raw, "application/json; charset=utf-8")

    def do_GET(self) -> None:  # noqa: N802
        parsed = urlparse(self.path)
        path = unquote(parsed.path)

        if path in ("/", "/index.html", "/results_viewer.html"):
            if not VIEWER_HTML.is_file():
                self._send(404, b"results_viewer.html missing", "text/plain")
                return
            self._send(200, VIEWER_HTML.read_bytes(), "text/html; charset=utf-8")
            return

        # Local Plotly (no CDN) — required offline / inside Docker
        if path.startswith("/vendor/"):
            name = Path(path).name
            if ".." in name or "/" in name or "\\" in name:
                self._send(403, b"forbidden", "text/plain")
                return
            fpath = (TOOLS_DIR / "vendor" / name).resolve()
            vendor_root = (TOOLS_DIR / "vendor").resolve()
            if vendor_root not in fpath.parents and fpath != vendor_root:
                self._send(403, b"forbidden", "text/plain")
                return
            if not fpath.is_file():
                self._send(404, b"vendor file missing", "text/plain")
                return
            ctype = "application/javascript; charset=utf-8" if fpath.suffix == ".js" else (
                mimetypes.guess_type(str(fpath))[0] or "application/octet-stream"
            )
            self._send(200, fpath.read_bytes(), ctype)
            return

        if path == "/api/runs":
            qs = parse_qs(parsed.query)
            controller = (qs.get("controller") or [None])[0]
            runs = discover_runs()
            if controller:
                runs = [r for r in runs if r["controller"] == controller]
            self._json(200, {"molo_dir": str(MOLO_DIR), "runs": runs})
            return

        if path == "/api/run":
            qs = parse_qs(parsed.query)
            rel = (qs.get("path") or [None])[0]
            if not rel:
                self._json(400, {"error": "missing path"})
                return
            run_dir = _safe_under(MOLO_DIR, rel)
            if run_dir is None or not run_dir.is_dir():
                self._json(404, {"error": "run not found"})
                return
            meta = _read_json(run_dir / "result.json") or _read_json(run_dir / "score.json") or {}
            self._json(
                200,
                {
                    "run_dir": rel,
                    "files": {
                        "log": (run_dir / "log.csv").is_file(),
                        "ref_path": (run_dir / "ref_path.csv").is_file(),
                        "result": (run_dir / "result.json").is_file(),
                        "score": (run_dir / "score.json").is_file(),
                        "config": (run_dir / "config.yaml").is_file(),
                    },
                    "meta": meta,
                    "urls": {
                        "log": f"/files/{rel}/log.csv",
                        "ref_path": f"/files/{rel}/ref_path.csv",
                        "result": f"/files/{rel}/result.json",
                        "score": f"/files/{rel}/score.json",
                        "config": f"/files/{rel}/config.yaml",
                    },
                },
            )
            return

        if path.startswith("/files/"):
            rel = path[len("/files/") :]
            # Disallow path tricks
            if ".." in Path(rel).parts:
                self._send(403, b"forbidden", "text/plain")
                return
            fpath = _safe_under(MOLO_DIR, rel)
            if fpath is None or not fpath.is_file():
                self._send(404, b"not found", "text/plain")
                return
            ctype = mimetypes.guess_type(str(fpath))[0] or "application/octet-stream"
            if fpath.suffix in (".csv", ".yaml", ".yml", ".json", ".log", ".txt"):
                ctype = {
                    ".csv": "text/csv; charset=utf-8",
                    ".yaml": "text/yaml; charset=utf-8",
                    ".yml": "text/yaml; charset=utf-8",
                    ".json": "application/json; charset=utf-8",
                    ".log": "text/plain; charset=utf-8",
                    ".txt": "text/plain; charset=utf-8",
                }.get(fpath.suffix, ctype)
            self._send(200, fpath.read_bytes(), ctype)
            return

        self._send(404, b"not found", "text/plain")


def main() -> None:
    ap = argparse.ArgumentParser(description="Serve interactive controller results viewer")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8766)
    args = ap.parse_args()

    if not VIEWER_HTML.is_file():
        raise SystemExit(f"missing {VIEWER_HTML}")

    httpd = ThreadingHTTPServer((args.host, args.port), Handler)
    print(f"Results viewer: http://{args.host}:{args.port}/")
    print(f"Scanning under: {MOLO_DIR}")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nbye")


if __name__ == "__main__":
    main()
