"""Run the Pi calibration dashboard: python calibration_dashboard.py."""

import argparse
import json
import logging
import math
import signal
import socket
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlsplit

from calibration.dashboard import Dashboard, encode, pixel_values

ROOT = Path(__file__).resolve().parent
ASSETS = ROOT / "calibration" / "dashboard_static"
USE_PCB = False


def json_safe(value):
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    return value


class DashboardServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, address, dashboard):
        self.dashboard = dashboard
        self.stream_slots = threading.BoundedSemaphore(24)
        super().__init__(address, Handler)


class Handler(BaseHTTPRequestHandler):
    def setup(self):
        super().setup()
        self.connection.settimeout(5)

    def log_message(self, fmt, *args):
        logging.getLogger("dashboard.http").debug(fmt, *args)

    def reply(self, body, content_type="application/json", status=200, filename=None):
        if not isinstance(body, bytes):
            body = json.dumps(json_safe(body), allow_nan=False).encode()
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Content-Security-Policy", "default-src 'self'; img-src 'self' blob:; style-src 'self'; script-src 'self'; connect-src 'self'; frame-ancestors 'none'")
        if filename:
            self.send_header("Content-Disposition", f'attachment; filename="{filename}"')
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        try:
            self._get()
        except (BrokenPipeError, ConnectionError, TimeoutError):
            pass
        except (ValueError, KeyError, TypeError) as exc:
            self.reply({"error": str(exc)}, status=400)

    def _get(self):
        dashboard = self.server.dashboard
        url = urlsplit(self.path)
        query = parse_qs(url.query)
        if url.path == "/api/state":
            self.reply(dashboard.state())
        elif url.path == "/api/pixel":
            frame = dashboard.frozen_frame(query["id"][0])
            self.reply(pixel_values(frame, int(query["x"][0]), int(query["y"][0])))
        elif url.path == "/frozen.png":
            self.reply(encode(dashboard.frozen_frame(query["id"][0]), ".png"), "image/png")
        elif url.path == "/snapshot.png":
            with dashboard.lock:
                if dashboard.latest is None:
                    raise ValueError("No image available")
                frame = dashboard.latest["frame"].copy()
                if query.get("view", ["raw"])[0] == "annotated":
                    from calibration.dashboard import scene
                    frame, _ = scene(frame, dashboard.latest["ball"], dashboard.latest["bots"], dashboard.calibration)
            self.reply(encode(frame, ".png"), "image/png", filename="camera-snapshot.png")
        elif url.path == "/debug.json":
            with dashboard.lock:
                logs = {"state": dashboard.state(), "history": list(dashboard.history), "events": list(dashboard.events)}
            self.reply(logs, filename="calibration-debug.json")
        elif url.path == "/stream.mjpg":
            self.stream(query.get("view", ["camera"])[0])
        else:
            assets = {"/": ("index.html", "text/html; charset=utf-8"),
                      "/app.js": ("app.js", "text/javascript; charset=utf-8"),
                      "/style.css": ("style.css", "text/css; charset=utf-8")}
            if url.path not in assets:
                self.reply({"error": "Not found"}, status=404)
                return
            name, content_type = assets[url.path]
            self.reply((ASSETS / name).read_bytes(), content_type)

    def stream(self, view):
        if view not in {"camera", "raw", "blue", "yellow", "goals"}:
            raise ValueError("Unknown preview")
        if not self.server.stream_slots.acquire(blocking=False):
            self.reply({"error": "Preview client limit reached"}, status=503)
            return
        dashboard = self.server.dashboard
        try:
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=FRAME")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            sequence = -1
            while not dashboard.closing.is_set():
                with dashboard.condition:
                    dashboard.condition.wait_for(
                        lambda previous=sequence: dashboard.stream_sequence != previous or dashboard.closing.is_set(), timeout=2,
                    )
                    sequence = dashboard.stream_sequence
                    frame = dashboard.streams.get(view)
                if frame is not None:
                    self.wfile.write(b"--FRAME\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                     + str(len(frame)).encode() + b"\r\n\r\n" + frame + b"\r\n")
                    self.wfile.flush()
        finally:
            self.server.stream_slots.release()

    def do_POST(self):
        try:
            origin = self.headers.get("Origin")
            expected = "http://" + self.headers.get("Host", "")
            if origin != expected or self.headers.get("Sec-Fetch-Site", "same-origin") not in ("same-origin", "none"):
                self.reply({"error": "Same-origin requests required"}, status=403)
                return
            if self.headers.get_content_type() != "application/json":
                raise ValueError("Expected application/json")
            length = int(self.headers.get("Content-Length", "0"))
            if not 0 < length <= 32768:
                raise ValueError("Invalid request size")
            data = json.loads(self.rfile.read(length))
            if not isinstance(data, dict):
                raise TypeError("Expected JSON object")
            action = self.path.removeprefix("/api/") if self.path.startswith("/api/") else ""
            dashboard = self.server.dashboard
            result = dashboard.freeze(for_bots=data.get("target") == "bot") if action == "freeze" else dashboard.command(
                action, data, self.headers.get("X-Control-Token"),
            )
            self.reply(result)
        except PermissionError as exc:
            self.reply({"error": str(exc)}, status=403)
        except (ValueError, TypeError, KeyError) as exc:
            self.reply({"error": str(exc)}, status=400)
        except (BrokenPipeError, ConnectionError, TimeoutError):
            pass
        except Exception as exc:
            self.server.dashboard.notify(f"Request failed: {exc}", error=True)
            self.reply({"error": str(exc)}, status=409)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--preview-fps", type=float, default=15)
    parser.add_argument("--lidar-port", default=None, help="Override lidar_port from config.txt")
    parser.add_argument("--lidar-baud", type=int, default=460800)
    args = parser.parse_args()
    if not 1 <= args.preview_fps <= 30:
        parser.error("--preview-fps must be between 1 and 30")
    logging.basicConfig(level=logging.INFO)
    # Prevent two dashboard processes sharing Pi devices.
    import fcntl
    lock_file = (ROOT / ".calibration_dashboard.lock").open("w")
    try:
        fcntl.flock(lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        parser.exit(1, "A calibration dashboard is already running.\n")
    dashboard = Dashboard(ROOT, fps=args.preview_fps, lidar_port=args.lidar_port, lidar_baud=args.lidar_baud,
                          use_pcb=USE_PCB)
    server = None
    try:
        server = DashboardServer((args.host, args.port), dashboard)
        def shutdown(_signum, _frame):
            with dashboard.lock:
                dashboard.lease.stop()
            threading.Thread(target=server.shutdown, daemon=True).start()
        signal.signal(signal.SIGTERM, shutdown)
        print(f"Dashboard: http://{socket.gethostname()}.local:{args.port} (or use the Pi IP address)", flush=True)
        print("Run standalone with main.py and other hardware scripts stopped.", flush=True)
        server.serve_forever(poll_interval=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        dashboard.close()
        if server is not None:
            server.server_close()
        lock_file.close()


if __name__ == "__main__":
    main()
