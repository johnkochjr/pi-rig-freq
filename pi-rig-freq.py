#!/usr/bin/env python3
"""
pi-rig-freq — Raspberry Pi frequency reader + on-screen band plan (TUI or GUI)

What it does
- Reads live frequency from either:
  1) Network rig server (Hamlib rigctld — recommended)
  2) **Ham Radio Deluxe (HRD) v5 TCP** — binary framed protocol (exactly as in your C#)
  3) Direct USB serial CAT (Kenwood/Yaesu-style commands like "FA;" / "IF;")
- TUI to configure settings (IP/port, serial, CAT) and save to ~/.config/pi-rig-freq/config.yaml
- GUI display window for Raspberry Pi screens that shows the current frequency and a band plan bar
- Optional JSON Lines logging

Tested on: Raspberry Pi OS (Bookworm) with Python 3.11; Windows 11 for dev

Dependencies:
  pip install pyserial pyyaml
GUI on Pi:
  sudo apt-get install -y python3-tk  # Tkinter
Windows TUI:
  pip install windows-curses

GPL-3.0 — Have fun and radio responsibly. 73!
"""

import argparse
import socket
import sys
import time
import curses
import threading
import queue
import pathlib
import json
import struct
from dataclasses import dataclass, asdict
from typing import Optional, Dict, List, Tuple

try:
    import serial  # pyserial
except Exception:  # pragma: no cover
    serial = None

try:
    import yaml  # pyyaml
except Exception:  # pragma: no cover
    yaml = None

# GUI imports are optional until --gui is used
try:  # pragma: no cover
    import tkinter as tk
    from tkinter import ttk
except Exception:
    tk = None
    ttk = None

APP_NAME = "pi-rig-freq"
CONFIG_DIR = pathlib.Path.home() / ".config" / APP_NAME
CONFIG_FILE = CONFIG_DIR / "config.yaml"
DEFAULT_LOG = pathlib.Path.home() / f".{APP_NAME}.log.jsonl"

# ---------------------- Config ----------------------
@dataclass
class Config:
    mode: str = "rigctld"            # "rigctld" | "hrd" | "usb"
    ip: str = "127.0.0.1"
    port: int = 4532                  # rigctld default; HRD commonly 7809
    poll_hz: float = 2.0              # 2 updates/sec
    # USB
    serial_port: str = "/dev/ttyUSB0"
    baud: int = 38400
    cat_cmd: str = "FA;"              # Kenwood/Yaesu get VFO-A frequency
    # Output
    jsonl_log: Optional[str] = str(DEFAULT_LOG)

    def save(self):
        if yaml is None:
            raise RuntimeError("pyyaml not installed: pip install pyyaml")
        CONFIG_DIR.mkdir(parents=True, exist_ok=True)
        with open(CONFIG_FILE, "w", encoding="utf-8") as f:
            yaml.safe_dump(asdict(self), f, sort_keys=False)

    @staticmethod
    def load() -> "Config":
        if yaml is None:
            return Config()
        if CONFIG_FILE.exists():
            with open(CONFIG_FILE, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f) or {}
            cfg = Config(**{**asdict(Config()), **data})
        else:
            cfg = Config()
        return cfg

# ---------------------- Band Plan (simplified, US Region 2 / ARRL-style) ----------------------
# NOTE: This is a high-level visualization (modes, not license-class slices). Consult ARRL for details.
BandSeg = Tuple[float, float, str]  # (start MHz, end MHz, label)
BANDS: Dict[str, Dict[str, List[BandSeg]]] = {
    "160m": {
        "range": [(1.800, 2.000, "160 m")],
        "segments": [
            (1.800, 1.875, "CW/RTTY/Data"),
            (1.875, 2.000, "Phone/Image"),
        ],
    },
    "80m": {
        "range": [(3.500, 4.000, "80 m")],
        "segments": [
            (3.500, 3.600, "CW/RTTY/Data"),
            (3.600, 4.000, "Phone/Image"),
        ],
    },
    "60m": {
        "range": [(5.330, 5.405, "60 m")],
        "segments": [
            (5.3305, 5.3305, "USB Phone"),
            (5.332, 5.332, "CW/Digi"),
            (5.3465, 5.3465, "USB Phone"),
            (5.348, 5.348, "CW/Digi"),
            (5.357, 5.357, "USB Phone"),
            (5.358, 5.358, "CW/Digi"),
            (5.3715, 5.3715, "USB Phone"),
            (5.373, 5.373, "CW/Digi"),
            (5.4035, 5.4035, "USB Phone"),
            (5.405, 5.405, "CW/Digi"),
        ],
    },
    "40m": {
        "range": [(7.000, 7.300, "40 m")],
        "segments": [
            (7.000, 7.125, "CW/RTTY/Data"),
            (7.125, 7.300, "Phone/Image"),
        ],
    },
    "30m": {
        "range": [(10.100, 10.150, "30 m")],
        "segments": [
            (10.100, 10.150, "CW/RTTY/Data (200W max)")
        ],
    },
    "20m": {
        "range": [(14.000, 14.350, "20 m")],
        "segments": [
            (14.000, 14.150, "CW/RTTY/Data"),
            (14.150, 14.350, "Phone/Image"),
        ],
    },
    "17m":{
        "range": [(18.068, 18.168, "17 m")],
        "segments": [
            (18.068, 18.110, "CW/RTTY/Data"),
            (18.110, 18.168, "Phone/Image"),
        ],
    },
    "15m": {
        "range": [(21.000, 21.450, "15 m")],
        "segments": [
            (21.000, 21.200, "CW/RTTY/Data"),
            (21.200, 21.450, "Phone/Image"),
        ],
    },
    "12m": {
        "range": [(24.890, 24.990, "12 m")],
        "segments": [
            (24.890, 24.930, "CW/RTTY/Data"),
            (24.930, 24.990, "Phone/Image"),
        ],
    },
    "10m": {
        "range": [(28.000, 29.700, "10 m")],
        "segments": [
            (28.000, 28.300, "CW/RTTY/Data"),
            (28.300, 29.000, "SSB Phone"),
            (29.000, 29.700, "AM/FM, Satellites")
        ],
    },
    "6m": {
        "range": [(50.000, 54.000, "6 m")],
        "segments": [
            (50.000, 50.300, "CW/Beacons"),
            (50.300, 50.600, "SSB/Weak Signal"),
            (50.600, 52.000, "General Mixed"),
            (52.000, 54.000, "FM/Repeaters")
        ],
    },
}

# Helper: match a MHz value to a band key
def band_for_mhz(mhz: float) -> Optional[str]:
    for k, v in BANDS.items():
        for (lo, hi, _label) in v["range"]:
            if lo <= mhz <= hi:
                return k
    return None

# ---------------------- Backends ----------------------
class Backend:
    def open(self):
        raise NotImplementedError

    def close(self):
        pass

    def get_frequency_hz(self) -> Optional[float]:
        """Return frequency in Hz, or None if not available."""
        raise NotImplementedError

    def get_mode(self) -> Optional[str]:
        return None

class RigctldBackend(Backend):
    """Hamlib rigctld TCP.
    Uses the documented 'f' (get freq) command and expects a numeric response.
    """
    def __init__(self, host: str, port: int, timeout=1.0):
        self.host = host
        self.port = port
        self.timeout = timeout
        self.sock: Optional[socket.socket] = None

    def open(self):
        self.sock = socket.create_connection((self.host, self.port), timeout=self.timeout)
        self.sock.settimeout(self.timeout)

    def close(self):
        try:
            if self.sock:
                self.sock.close()
        finally:
            self.sock = None

    def _send(self, cmd: str) -> str:
        if not self.sock:
            raise RuntimeError("rigctld socket not open")
        self.sock.sendall((cmd + "\n").encode("ascii"))
        chunks = []
        while True:
            try:
                data = self.sock.recv(4096)
            except socket.timeout:
                break
            if not data:
                break
            chunks.append(data)
            if b"\n" in data:
                break
        return b"".join(chunks).decode(errors="replace").strip()

    def get_frequency_hz(self) -> Optional[float]:
        try:
            resp = self._send("f")
            if resp.startswith("RPRT"):
                return None
            token = resp.split()[0]
            return float(token)
        except Exception:
            return None

    def get_mode(self) -> Optional[str]:
        try:
            resp = self._send("m")
            if resp.startswith("RPRT"):
                return None
            return resp.split()[0].upper()
        except Exception:
            return None


class HRDBackend(Backend):
    """Ham Radio Deluxe IP Server v5 (binary framed) — per your C# implementation.

    Frame format (little-endian):
      uint32 size = 16 + payload_bytes
      uint32 magic1 = 0x1234ABCD
      uint32 magic2 = 0xABCD1234
      uint32 checksum = 0 (unused)
      payload = UTF-16LE string with NUL terminator (e.g., "get frequency")
    """
    MAGIC1 = 0x1234ABCD
    MAGIC2 = 0xABCD1234

    def __init__(self, host: str, port: int, timeout=1.0):
        self.host = host
        self.port = port
        self.timeout = timeout
        self.sock: Optional[socket.socket] = None

    def open(self):
        self.sock = socket.create_connection((self.host, self.port), timeout=self.timeout)
        self.sock.settimeout(self.timeout)
        # Probe like the C# client: send "get context" once to establish framing works
        try:
            _ = self._send_command_v5("get context")
        except Exception:
            # Some servers may not respond to this probe; continue anyway
            pass

    def close(self):
        try:
            if self.sock:
                self.sock.close()
        finally:
            self.sock = None

    def _read_exact(self, n: int) -> bytes:
        assert self.sock is not None
        buf = bytearray()
        while len(buf) < n:
            chunk = self.sock.recv(n - len(buf))
            if not chunk:
                raise EOFError("HRD closed the connection")
            buf.extend(chunk)
        return bytes(buf)

    def _send_command_v5(self, command: str, prepend_context: bool = False) -> str:
        if not self.sock:
            raise RuntimeError("HRD socket not open")
        # Build payload: optional "[0] " prefix, then command, then NUL; UTF-16LE encoding
        payload_text = ("[0] " + command if prepend_context else command) + "\x00"
        payload = payload_text.encode("utf-16le")
        size = 16 + len(payload)
        # Use UNSIGNED fields to avoid sign-mismatch issues when comparing magics
        header = struct.pack("<IIII", size, self.MAGIC1 & 0xFFFFFFFF, self.MAGIC2 & 0xFFFFFFFF, 0)
        frame = header + payload
        self.sock.sendall(frame)
        # Read reply header
        hdr = self._read_exact(16)
        reply_size, r1, r2, _checksum = struct.unpack("<IIII", hdr)
        if (r1 & 0xFFFFFFFF) != (self.MAGIC1 & 0xFFFFFFFF) or (r2 & 0xFFFFFFFF) != (self.MAGIC2 & 0xFFFFFFFF):
            raise IOError("Bad HRD header (magic mismatch)")
        # Read payload
        remaining = reply_size - 16
        pl = self._read_exact(remaining)
        text = pl.decode("utf-16le", errors="replace")
        # Trim at first NUL
        nul = text.find("\x00")
        return text[:nul] if nul >= 0 else text

    def get_frequency_hz(self) -> Optional[float]:
        try:
            reply = self._send_command_v5("get frequency", prepend_context=False)
            val = reply.strip()
            if val.isdigit():
                return float(val)
            # Fallback: first integer token in the string
            token = "".join(ch for ch in val if ch.isdigit())
            return float(token) if token else None
        except Exception:
            return None
    
    def get_mode(self) -> Optional[str]:
        try:
            reply = self._send_command_v5("get mode", prepend_context=False)
            m = reply.strip()
            return m or None
        except Exception:
            return None

class USBBackend(Backend):
    """Direct USB serial CAT using pyserial."""
    def __init__(self, port: str, baud: int, cmd: str = "FA;", timeout=0.8):
        self.port = port
        self.baud = baud
        self.cmd = cmd
        self.timeout = timeout
        # self.ser: Optional[serial.Serial] = None if serial else None

    def open(self):
        if serial is None:
            raise RuntimeError("pyserial not installed: pip install pyserial")
        self.ser = serial.Serial(self.port, self.baud, timeout=self.timeout)
        time.sleep(0.1)
        if self.ser.in_waiting:
            _ = self.ser.read(self.ser.in_waiting)

    def close(self):
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        finally:
            self.ser = None

    def get_frequency_hz(self) -> Optional[float]:
        if not self.ser:
            return None
        try:
            self.ser.write(self.cmd.encode("ascii"))
            self.ser.flush()
            time.sleep(0.05)
            data = self.ser.read(128)
            if not data:
                return None
            text = data.decode(errors="replace")
            digits = ''.join(ch for ch in text if ch.isdigit())
            if digits:
                return float(digits)
        except Exception:
            return None
        return None

# ---------------------- Worker ----------------------
class Poller(threading.Thread):
    def __init__(self, backend: Backend, poll_hz: float, out_q: queue.Queue):
        super().__init__(daemon=True)
        self.backend = backend
        self.dt = 1.0 / max(0.1, poll_hz)
        self.out_q = out_q
        self._stop = threading.Event()
        self._i = 0

    def run(self):
        try:
            self.backend.open()
        except Exception as e:
            self.out_q.put({"type": "error", "msg": f"open failed: {e}"})
            return
        self.out_q.put({"type": "status", "msg": "connected"})
        try:
            while not self._stop.is_set():
                hz = self.backend.get_frequency_hz()
                ts = time.time()

                if hz is not None:
                    self.out_q.put({"type": "freq", "ts": ts, "hz": hz})
                else:
                    self.out_q.put({"type": "warn", "msg": "no data"})
                if (self._i % 10) == 0:
                    try:
                        mode = self.backend.get_mode()
                        if mode is not None:
                            self.out_q.put({"type": "mode", "ts": ts, "mode": mode})
                    except Exception:
                        pass
                self._i += 1
                time.sleep(self.dt)
        finally:
            self.backend.close()
            self.out_q.put({"type": "status", "msg": "disconnected"})

    def stop(self):
        self._stop.set()

# ---------------------- TUI ----------------------
class TUI:
    def __init__(self, cfg: Config):
        self.cfg = cfg
        self.out_q: queue.Queue = queue.Queue()
        self.poller: Optional[Poller] = None
        self.latest_hz: Optional[float] = None
        self.log_fp = None
        self.latest_mode: Optional[str] = None

    def _open_poller(self):
        if self.poller:
            self.poller.stop()
            self.poller.join(timeout=1)
        backend: Backend
        if self.cfg.mode == "rigctld":
            backend = RigctldBackend(self.cfg.ip, self.cfg.port)
        elif self.cfg.mode == "hrd":
            backend = HRDBackend(self.cfg.ip, self.cfg.port)
        else:
            backend = USBBackend(self.cfg.serial_port, self.cfg.baud, self.cfg.cat_cmd)
        self.poller = Poller(backend, self.cfg.poll_hz, self.out_q)
        self.poller.start()

    def _toggle_log(self):
        if self.log_fp:
            self.log_fp.close()
            self.log_fp = None
        else:
            if self.cfg.jsonl_log:
                self.log_fp = open(self.cfg.jsonl_log, "a", encoding="utf-8")

    def _write_log(self, rec: dict):
        if self.log_fp:
            self.log_fp.write(json.dumps(rec) + "\n")
            self.log_fp.flush()

    def run(self):
        curses.wrapper(self._loop)

    def _loop(self, stdscr):
        curses.curs_set(0)
        stdscr.nodelay(True)
        stdscr.timeout(200)
        self._open_poller()
        msg = "h:help  m:mode  e:edit net  u:edit usb  c:CAT cmd  s:save  l:log  q:quit  g:GUI"
        while True:
            try:
                while True:
                    ev = self.out_q.get_nowait()
                    if ev.get("type") == "freq":
                        self.latest_hz = ev["hz"]
                        self._write_log(ev)
                    # elif ev.get("type") == "mode":
                    #     self.latest_mode = ev["mode"]
                    #     self._write_log(ev)
                    elif ev.get("type") in ("status", "warn", "error"):
                        self._write_log(ev)
            except queue.Empty:
                pass

            stdscr.erase()
            stdscr.addstr(0, 0, f"{APP_NAME} — Raspberry Pi Frequency Reader")
            stdscr.addstr(1, 0, msg)
            stdscr.addstr(3, 0, f"Mode: {self.cfg.mode}   Poll: {self.cfg.poll_hz:.2f} Hz   Log: {'ON' if self.log_fp else 'off'}")
            if self.cfg.mode in ("rigctld", "hrd"):
                stdscr.addstr(4, 0, f"IP: {self.cfg.ip}  Port: {self.cfg.port}")
            else:
                stdscr.addstr(4, 0, f"Serial: {self.cfg.serial_port}  {self.cfg.baud} baud  CAT: {self.cfg.cat_cmd}")

            disp = "—"
            band = None
            if self.latest_hz is not None:
                mhz = self.latest_hz / 1e6
                band = band_for_mhz(mhz)
                disp = f"{mhz:0.6f} MHz ({int(self.latest_hz)} Hz)  [{band or '—'}]"
            stdscr.addstr(6, 0, f"Frequency: {disp}")
            stdscr.addstr(7, 0, f"Mode: {self.latest_mode or '—'}")

            stdscr.addstr(8, 0, "Keys: h=help  m=cycle mode  e=edit IP/port  u=edit serial  c=edit CAT  s=save  l=toggle log  g=open GUI  q=quit")

            ch = stdscr.getch()
            if ch == ord('q'):
                break
            elif ch == ord('m'):
                self.cfg.mode = {"rigctld":"hrd","hrd":"usb","usb":"rigctld"}[self.cfg.mode]
                self._open_poller()
            elif ch == ord('e'):
                if self.cfg.mode in ("rigctld", "hrd"):
                    self._edit_net(stdscr)
                    self._open_poller()
            elif ch == ord('u'):
                if self.cfg.mode == "usb":
                    self._edit_usb(stdscr)
                    self._open_poller()
            elif ch == ord('c'):
                self.cfg.cat_cmd = self._prompt(stdscr, "CAT command (e.g., FA; or IF;): ", self.cfg.cat_cmd)
                if self.cfg.mode == "usb":
                    self._open_poller()
            elif ch == ord('s'):
                self.cfg.save()
            elif ch == ord('l'):
                self._toggle_log()
            elif ch == ord('g'):
                # Launch GUI from TUI
                curses.endwin()
                run_gui(self.cfg)
                stdscr.refresh()
                self._open_poller()

            stdscr.refresh()
        if self.poller:
            self.poller.stop()
            self.poller.join(timeout=1)
        if self.log_fp:
            self.log_fp.close()

    # -------------- small helpers --------------
    def _prompt(self, stdscr, label: str, initial: str) -> str:
        curses.echo()
        stdscr.addstr(10, 0, " " * 120)
        stdscr.addstr(10, 0, label)
        stdscr.move(10, len(label))
        stdscr.addstr(10, len(label), initial)
        s = stdscr.getstr(10, len(label), 60)
        curses.noecho()
        return (s.decode() or initial).strip()

    def _prompt_int(self, stdscr, label: str, initial: int) -> int:
        val = self._prompt(stdscr, label, str(initial))
        try:
            return int(val)
        except Exception:
            return initial

    def _edit_net(self, _stdscr):
        self.cfg.ip = self._prompt(_stdscr, "IP address: ", self.cfg.ip)
        self.cfg.port = self._prompt_int(_stdscr, "Port: ", self.cfg.port)

    def _edit_usb(self, _stdscr):
        self.cfg.serial_port = self._prompt(_stdscr, "Serial device (e.g., /dev/ttyUSB0 or COM5): ", self.cfg.serial_port)
        self.cfg.baud = self._prompt_int(_stdscr, "Baud (e.g., 38400): ", self.cfg.baud)

# ---------------------- GUI ----------------------
class BandPlanView(ttk.Frame):
    def __init__(self, master, width=900, height=200):
        super().__init__(master)
        self.canvas = tk.Canvas(self, width=width, height=height, highlightthickness=0)
        self.canvas.pack(fill="both", expand=True)
        self.width = width
        self.height = height
        self.current_mhz: Optional[float] = None
        self.current_band: Optional[float] = None

    def draw_plan(self, band_key: str, mhz: Optional[float]):
        self.canvas.delete("all")
        band = BANDS[band_key]
        lo, hi, _ = band["range"][0]
        pad = 40
        x0, x1 = pad, self.width - pad
        y0, y1 = 60, self.height - 30
        # Axis
        self.canvas.create_rectangle(x0, y0, x1, y1, outline="#333")
        # Segments
        colors = ["#d0e6ff", "#ffe3c2", "#d8ffd8", "#ffd8ec", "#e8e8ff"]
        for idx, (s, e, label) in enumerate(band["segments"]):
            xs = x0 + (s - lo) / (hi - lo) * (x1 - x0)
            xe = x0 + (e - lo) / (hi - lo) * (x1 - x0)
            self.canvas.create_rectangle(xs, y0, xe, y1, fill=colors[idx % len(colors)], outline="")
            self.canvas.create_text((xs+xe)/2, (y0+y1)/2, text=label, font=("Arial", 10))
        # Ticks
        for tick in range(int(lo*10), int(hi*10)+1, max(1, int((hi-lo)*10//10))):
            f = tick/10.0
            x = x0 + (f - lo)/(hi - lo) * (x1 - x0)
            self.canvas.create_line(x, y1, x, y1+6, fill="#555")
            self.canvas.create_text(x, y1+18, text=f"{f:.1f}", font=("Arial", 8))
        # Cursor line for current frequency
        if mhz is not None and lo <= mhz <= hi:
            xf = x0 + (mhz - lo) / (hi - lo) * (x1 - x0)
            self.canvas.create_line(xf, y0-6, xf, y1+6, fill="#e11", width=2)

    def update_freq(self, mhz: Optional[float]):
        if mhz is None:
            self.canvas.delete("all")
            self.canvas.create_text(self.width/2, self.height/2, text="No data", font=("Arial", 14))
            return
        band = band_for_mhz(mhz)
        if band is not None:
            self.draw_plan(band, mhz)
        else:
            self.canvas.delete("all")
            self.canvas.create_text(self.width/2, self.height/2, text="Outside defined bands", font=("Arial", 14))

class GUIApp:
    def __init__(self, cfg: Config):
        if tk is None:
            raise RuntimeError("Tkinter not available. On Pi: sudo apt-get install -y python3-tk")
        self.cfg = cfg
        self.root = tk.Tk()
        self.root.attributes("-fullscreen", True)
        self.root.bind("<Escape>", lambda e: self.root.attributes("-fullscreen", False))
        self.root.title("pi-rig-freq — Live Frequency + Band Plan")
        self.root.geometry("480x320+0+0")
        self.root.configure(bg="white")
        self._after_id = None
        self._closing = False
        self.mode_var = tk.StringVar(value="")
        
        # Header
        top = ttk.Frame(self.root, padding=12)
        top.pack(fill="x")
        self.freq_var = tk.StringVar(value="—")
        ttk.Label(top, text="Frequency:", font=("Arial", 12)).pack(side="left")
        ttk.Label(top, textvariable=self.freq_var, font=("Consolas", 14)).pack(side="left", padx=10)
        self.band_var = tk.StringVar(value="")
        ttk.Label(top, textvariable=self.band_var, font=("Arial", 10)).pack(side="left", padx=12)
        ttk.Label(top, textvariable=self.mode_var, font=("Arial", 12)).pack(side="left", padx=12)

        # Band plan view
        self.plan = BandPlanView(self.root, width=460, height=200)
        self.plan.pack(fill="both", expand=True, padx=10, pady=(0,10))

        # Footer note
        bot = ttk.Frame(self.root, padding=(12,0,12,8))
        bot.pack(fill="x")
        ttk.Label(bot, text="Simplified US band plan visualization — verify details at ARRL / Part 97", font=("Arial", 9)).pack(side="left")

        # Queue + poller
        self.q: queue.Queue = queue.Queue()
        backend: Backend
        if self.cfg.mode == "rigctld":
            backend = RigctldBackend(self.cfg.ip, self.cfg.port)
        elif self.cfg.mode == "hrd":
            backend = HRDBackend(self.cfg.ip, self.cfg.port)
        else:
            backend = USBBackend(self.cfg.serial_port, self.cfg.baud, self.cfg.cat_cmd)
        self.poller = Poller(backend, self.cfg.poll_hz, self.q)
        self.poller.start()

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self._after_id = self.root.after(100, self._drain)

    def _drain(self):
        if self._closing:
            return
        try:
            while True:
                ev = self.q.get_nowait()
                if ev.get("type") == "freq":
                    hz = ev["hz"]
                    mhz = hz / 1e6
                    self.freq_var.set(f"{mhz:0.6f} MHz")
                    b = band_for_mhz(mhz)
                    self.band_var.set(f"[{b}]" if b else "")
                    self.plan.update_freq(mhz)
                elif ev.get("type") == "mode":                   # NEW
                    self.mode_var.set(f"Mode: {ev['mode']}")

        except queue.Empty:
            pass
        except tk.TclError:
            return
        if not self._closing:
            self._after_id = self.root.after(100, self._drain)

    def on_close(self):
        self._closing = True
        try:
            if self._after_id is not None:
                try:
                    self.root.after_cancel(self._after_id)
                except Exception:
                    pass
            if self.poller:
                self.poller.stop()
                self.poller.join(timeout=1)
        finally:
            try:
                self.root.quit()
            except Exception:
                pass
            try:
                self.root.destroy()
            except Exception:
                pass

    def run(self):
        self.root.mainloop()

# Convenience launcher
def run_gui(cfg: Config):
    GUIApp(cfg).run()

# ---------------------- CLI ----------------------

def parse_args():
    p = argparse.ArgumentParser(description=APP_NAME)
    p.add_argument("--once", action="store_true", help="print one reading and exit")
    p.add_argument("--mode", choices=["rigctld","hrd","usb"], help="override mode")
    p.add_argument("--gui", action="store_true", help="show GUI band plan display")
    p.add_argument("--ip", help="override host/IP for network modes")
    p.add_argument("--port", type=int, help="override TCP port for network modes")
    p.add_argument("--tui", action="store_true", help="show TUI")
    return p.parse_args()


def make_backend(cfg: Config) -> Backend:
    if cfg.mode == "rigctld":
        return RigctldBackend(cfg.ip, cfg.port)
    elif cfg.mode == "hrd":
        return HRDBackend(cfg.ip, cfg.port)
    else:
        return USBBackend(cfg.serial_port, cfg.baud, cfg.cat_cmd)


def main():
    args = parse_args()
    cfg = Config.load()
    if args.mode:
        cfg.mode = args.mode
    if args.ip:
        cfg.ip = args.ip
    if args.port:
        cfg.port = args.port
    # Smart default: if switching to HRD and no explicit port provided, use 7809 when default is still 4532
    if cfg.mode == "hrd" and not args.port and cfg.port == 4532:
        cfg.port = 7809
    if not args.gui and not args.tui:
        args.gui = True

    if args.once:
        be = make_backend(cfg)
        be.open()
        try:
            hz = be.get_frequency_hz()
            if hz is None:
                print("no data", file=sys.stderr)
                sys.exit(1)
            print(int(hz))
        finally:
            be.close()
        return

    if args.gui:
        run_gui(cfg)
        sys.exit(0)

    # Otherwise run the TUI
    TUI(cfg).run()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
