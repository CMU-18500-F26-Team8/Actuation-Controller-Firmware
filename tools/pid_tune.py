#!/usr/bin/env python3
"""
pid_tune.py - live PID tuning for the SnackBot drive (Pi control mode firmware).

Talks to the Nucleo over the ST-LINK USB port (115200). Turns on the firmware's
50 Hz telemetry, plots each wheel's setpoint, measured speed and duty, and lets
you change the gains and run timed straight / reverse tests from the window.

    pip3 install pyserial matplotlib
    python3 tools/pid_tune.py                 # auto-finds /dev/tty.usbmodem*
    python3 tools/pid_tune.py --port /dev/tty.usbmodem1103
    python3 tools/pid_tune.py --demo          # no hardware: simulated motors

Close any other serial monitor (screen, VS Code) first: only one program can
open the port. Gain changes last until the board resets; "Print config" prints
#define lines to paste into Core/Inc/robot_config.h.

Each wheel has its own PID (kp_l/ki_l/kd_l and kp_r/ki_r/kd_r): tune them
independently, and untick "show L" / "show R" to look at one wheel.

Firmware commands used (USB only): get | set,<name>,<value> | telem,on|off,
plus motor-instruction,straight|reverse|stop,,, (repeated every 0.2 s during a
run to keep the 500 ms watchdog happy).
"""

import argparse
import glob
import math
import queue
import random
import sys
import threading
import time
from collections import deque

TUNABLES = ["kp_l", "ki_l", "kd_l", "kp_r", "ki_r", "kd_r", "speed", "turn", "accel"]
CONFIG_NAMES = {"kp_l": "PID_L_KP", "ki_l": "PID_L_KI", "kd_l": "PID_L_KD",
                "kp_r": "PID_R_KP", "ki_r": "PID_R_KI", "kd_r": "PID_R_KD",
                "speed": "TARGET_SPEED_MPS", "turn": "TURN_SPEED_MPS", "accel": "ACCEL_MPS2"}
REPEAT_SEC = 0.2          # motor instruction resend period during a run
TAIL_SEC = 1.0            # keep plotting this long after a run stops
STEADY_SEC = 1.0          # steady-state window at the end of a run
SETTLE_BAND = 0.05        # +-5 % of target


# ── Links ────────────────────────────────────────────────────────────────────

class SerialLink:
    def __init__(self, port, baud):
        import serial
        self.ser = serial.Serial(port, baud, timeout=0.1)
        self.lock = threading.Lock()

    def write_line(self, line):
        with self.lock:
            self.ser.write((line + "\n").encode("ascii"))

    def read_line(self):
        raw = self.ser.readline()
        return raw.decode("ascii", errors="replace").strip() if raw else None

    def close(self):
        self.ser.close()


class DemoLink:
    """Simulates the firmware (drive.c per-wheel PID, 100 ms speed window,
    encoder quantisation) driving two slightly different first-order motors."""
    M_PER_COUNT = math.pi * 0.0889 / 1920.0
    DT = 0.02

    def __init__(self):
        self.t = dict(kp_l=200.0, ki_l=1000.0, kd_l=0.0, kp_r=200.0, ki_r=1000.0, kd_r=0.0,
                      speed=0.16, turn=0.16, accel=0.4)
        self.out = queue.Queue()
        self.state = 0                     # 0 stopped, 1 straight, 2 reverse
        self.last_cmd = 0.0
        self.telem = False
        self.ms = 0
        self.w = [self._wheel(0.00150, 0.09, 12), self._wheel(0.00140, 0.11, 15)]
        self.pos0 = [0, 0]
        threading.Thread(target=self._run, daemon=True).start()

    @staticmethod
    def _wheel(gain, tau, deadband):
        # gain: m/s per permille; deadband: permille needed to move at all
        return dict(gain=gain, tau=tau, dead=deadband, v=0.0, frac=0.0, pos=0,
                    win=deque([0] * 5, maxlen=5), speed=0.0, sp=0.0, integ=0.0, prev=None, duty=0)

    def write_line(self, line):
        f = [x.strip() for x in line.split(",")] + [""] * 5
        if f[0] == "get":
            self._g()
        elif f[0] == "telem":
            self.telem = f[1] in ("on", "1", "true")
            self.out.put(f"[telem] {'on' if self.telem else 'off'}")
        elif f[0] == "set" and f[1] in self.t:
            self.t[f[1]] = float(f[2])
            self._g()
        elif f[0] == "motor-instruction":
            if f[1] == "stop":
                self.state = 0
                for w in self.w:
                    w.update(sp=0.0, integ=0.0, prev=None, duty=0)
            else:
                new = 1 if f[1] == "straight" else 2
                if new != self.state:
                    self.state = new
                    self.pos0 = [w["pos"] for w in self.w]
                    for w in self.w:
                        w.update(integ=0.0, prev=None)
                self.last_cmd = time.monotonic()

    def _g(self):
        t = self.t
        self.out.put("G," + ",".join(f"{t[k]:g}" for k in TUNABLES))

    def read_line(self):
        try:
            return self.out.get(timeout=0.1)
        except queue.Empty:
            return None

    def close(self):
        pass

    def _run(self):
        nxt = time.monotonic()
        while True:
            nxt += self.DT
            time.sleep(max(0.0, nxt - time.monotonic()))
            self._tick()

    def _tick(self):
        t, dt = self.t, self.DT
        self.ms += 20
        if self.state and time.monotonic() - self.last_cmd > 0.5:
            self.state = 0
            for w in self.w:
                w.update(sp=0.0, integ=0.0, prev=None, duty=0)
            self.out.put("[watchdog] no motor instruction for 500 ms: stopped")
        for w in self.w:                              # plant + encoder
            drive = 0.0 if abs(w["duty"]) < w["dead"] else w["gain"] * (w["duty"] - math.copysign(w["dead"], w["duty"]))
            if self.state == 0:
                drive = 0.0
            w["v"] += (drive - w["v"]) * dt / w["tau"] + random.gauss(0, 0.002)
            c = w["v"] * dt / self.M_PER_COUNT + w["frac"]
            n = int(math.floor(c))
            w["frac"] = c - n
            w["pos"] += n
            w["win"].append(n)
            w["speed"] = sum(w["win"]) * self.M_PER_COUNT / (len(w["win"]) * dt)
        if self.state:
            tgt = t["speed"] if self.state == 1 else -t["speed"]
            for i, w in enumerate(self.w):
                kp, ki, kd = (t[k + ("_l", "_r")[i]] for k in ("kp", "ki", "kd"))
                step = t["accel"] * dt
                w["sp"] = min(max(tgt, w["sp"] - step), w["sp"] + step)
                err = w["sp"] - w["speed"]
                der = 0.0 if w["prev"] is None else (err - w["prev"]) / dt
                w["prev"] = err
                base = kp * err + kd * der
                trial = base + ki * (w["integ"] + err * dt)
                if abs(trial) <= 600 or (trial > 0) != (err > 0):
                    w["integ"] += err * dt
                w["duty"] = int(round(max(-600, min(600, base + ki * w["integ"]))))
        if self.telem:
            l, r = self.w
            self.out.put(f"T,{self.ms},{self.state},{round(l['sp']*1000)},{round(l['speed']*1000)},{l['duty']},"
                         f"{round(r['sp']*1000)},{round(r['speed']*1000)},{r['duty']}")


# ── Data + analysis ─────────────────────────────────────────────────────────

class Recorder:
    """Collects T samples. Each run starts a new trace."""
    def __init__(self):
        self.lock = threading.Lock()
        self.samples = []          # (t_s, state, spL, vL, dL, spR, vR, dR) in m/s and %
        self.t0_ms = None
        self.idle_window = 10.0

    def start_run(self):
        with self.lock:
            self.samples = []
            self.t0_ms = None

    def add(self, fields):
        ms, st, spl, vl, dl, spr, vr, dr = (int(x) for x in fields)
        with self.lock:
            if self.t0_ms is None:
                self.t0_ms = ms
            self.samples.append(((ms - self.t0_ms) / 1000.0, st, spl / 1000, vl / 1000, dl / 10,
                                 spr / 1000, vr / 1000, dr / 10))
            if len(self.samples) > 3000:        # 60 s cap
                del self.samples[:500]

    def snapshot(self):
        with self.lock:
            return list(self.samples)


def analyse(samples, target, run_start_s, run_end_s):
    """Per-wheel step metrics for one run. Returns a text block."""
    if not samples or target <= 0:
        return "no data"
    lines = []
    for name, vi, di in (("L", 3, 4), ("R", 6, 7)):
        run = [(s[0], abs(s[vi]), abs(s[di])) for s in samples
               if run_start_s <= s[0] <= run_end_s and s[1] != 0]       # only while driven
        if run:
            run_end_s = run[-1][0]
        if len(run) < 10:
            lines.append(f"{name}: too short")
            continue
        rise = next((t - run_start_s for t, v, _ in run if v >= 0.9 * target), None)
        peak = max(v for _, v, _ in run)
        settle = None
        for t, v, _ in run:
            if abs(v - target) > SETTLE_BAND * target:
                settle = t - run_start_s
        steady = [(v, d) for t, v, d in run if t >= run_end_s - STEADY_SEC]
        mean_v = sum(v for v, _ in steady) / len(steady)
        sd_v = math.sqrt(sum((v - mean_v) ** 2 for v, _ in steady) / len(steady))
        mean_d = sum(d for _, d in steady) / len(steady)
        lines.append(
            f"{name}: rise {'%.2fs' % rise if rise is not None else '--'}  "
            f"overshoot {max(0.0, (peak - target) / target * 100):.0f}%  "
            f"settle {'%.2fs' % settle if settle is not None else '0'}\n"
            f"    steady {mean_v:.3f} m/s ({(mean_v - target) / target * 100:+.1f}%)  "
            f"ripple {sd_v * 1000:.1f} mm/s\n"
            f"    steady duty {mean_d:.1f}%")
    return "\n".join(lines)


# ── App ─────────────────────────────────────────────────────────────────────

class Tuner:
    def __init__(self, link, run_sec):
        self.link = link
        self.rec = Recorder()
        self.tuning = {}
        self.tuning_dirty = False
        self.run_sec = run_sec
        self.run_lock = threading.Lock()
        self.run_info = None         # (direction, start_s, end_s or None)
        self.metrics = "Press Forward or Reverse to run a test."
        self.metrics_dirty = True
        self._stop_run = threading.Event()
        self._running = True
        threading.Thread(target=self._reader, daemon=True).start()
        link.write_line("telem,on")
        link.write_line("get")

    # serial side
    def _reader(self):
        while self._running:
            try:
                line = self.link.read_line()
            except Exception as e:
                print(f"serial error: {e}")
                time.sleep(0.5)
                continue
            if not line:
                continue
            if line.startswith("T,"):
                f = line.split(",")[1:]
                if len(f) == 8:
                    try:
                        self.rec.add(f)
                    except ValueError:
                        pass
            elif line.startswith("G,"):
                vals = line.split(",")[1:]
                if len(vals) == len(TUNABLES):
                    try:
                        self.tuning = {k: float(v) for k, v in zip(TUNABLES, vals)}
                        self.tuning_dirty = True
                    except ValueError:
                        pass
            elif "[rx usb]" not in line and "[tx] odometer" not in line:
                print(line)          # hide the echo of our own repeats and odometer chatter

    def set(self, name, text):
        try:
            float(text)
        except ValueError:
            print(f"{name}: not a number: {text!r}")
            return
        self.link.write_line(f"set,{name},{text.strip()}")

    def start_run(self, action):
        if not self.run_lock.acquire(blocking=False):
            print("a run is already in progress")
            return
        self._stop_run.clear()
        self.rec.start_run()

        def go():
            try:
                t_start = time.monotonic()
                while time.monotonic() - t_start < self.run_sec and not self._stop_run.is_set():
                    self.link.write_line(f"motor-instruction,{action},,,")
                    self._stop_run.wait(REPEAT_SEC)
                self.link.write_line("motor-instruction,stop,,,")
                elapsed = time.monotonic() - t_start
                time.sleep(TAIL_SEC)
                target = self.tuning.get("speed", 0.16)
                self.metrics = (f"{action} run, target {target:.3f} m/s\n"
                                + analyse(self.rec.snapshot(), target, 0.0, elapsed))
                self.metrics_dirty = True
                print("\n" + self.metrics + "\n")
            finally:
                self.run_lock.release()
        threading.Thread(target=go, daemon=True).start()

    def stop(self):
        self._stop_run.set()
        self.link.write_line("motor-instruction,stop,,,")

    def print_config(self):
        if not self.tuning:
            print("no G line received yet")
            return
        print("\n/* paste into Core/Inc/robot_config.h */")
        for k in TUNABLES:
            v = self.tuning[k]
            s = f"{v:.4f}".rstrip("0").rstrip(".")
            if "." not in s:
                s += ".0"
            print(f"#define {CONFIG_NAMES[k]:<23} {s}f")
        print()

    def close(self):
        try:
            self.link.write_line("motor-instruction,stop,,,")
            self.link.write_line("telem,off")
            time.sleep(0.1)
        finally:
            self._running = False
            self.link.close()

    # GUI
    def run_gui(self):
        import matplotlib.pyplot as plt
        from matplotlib.animation import FuncAnimation
        from matplotlib.widgets import Button, CheckButtons, TextBox

        fig = plt.figure(figsize=(13.5, 7.5))
        fig.canvas.manager.set_window_title("SnackBot PID tuning")
        ax_v = fig.add_axes([0.06, 0.55, 0.62, 0.40])
        ax_d = fig.add_axes([0.06, 0.08, 0.62, 0.40], sharex=ax_v)
        ax_v.set_ylabel("speed (m/s)")
        ax_d.set_ylabel("duty (%)")
        ax_d.set_xlabel("time since run start (s)")
        for a in (ax_v, ax_d):
            a.grid(True, alpha=0.3)
        cl, cr = "#1f77b4", "#d62728"
        ln = {
            "spL": ax_v.plot([], [], "--", color=cl, lw=1, label="L setpoint")[0],
            "vL": ax_v.plot([], [], "-", color=cl, lw=1.6, label="L measured")[0],
            "spR": ax_v.plot([], [], "--", color=cr, lw=1, label="R setpoint")[0],
            "vR": ax_v.plot([], [], "-", color=cr, lw=1.6, label="R measured")[0],
            "dL": ax_d.plot([], [], "-", color=cl, lw=1.4, label="L duty")[0],
            "dR": ax_d.plot([], [], "-", color=cr, lw=1.4, label="R duty")[0],
        }
        band_lo = ax_v.axhline(0, color="gray", lw=0.6, ls=":")
        band_hi = ax_v.axhline(0, color="gray", lw=0.6, ls=":")
        ax_v.legend(loc="lower right", fontsize=8, ncol=2)
        ax_d.legend(loc="lower right", fontsize=8)

        boxes = {}
        self._updating = False

        def add_box(key, label, x, y, w=0.075):
            bax = fig.add_axes([x, y, w, 0.045])
            tb = TextBox(bax, label + " ", initial="")
            tb.on_submit(lambda text, k=key: None if self._updating else self.set(k, text))
            boxes[key] = tb

        fig.text(0.785, 0.955, "Left", color=cl, fontsize=10, weight="bold", ha="center")
        fig.text(0.915, 0.955, "Right", color=cr, fontsize=10, weight="bold", ha="center")
        y = 0.90
        for g in ("kp", "ki", "kd"):
            add_box(g + "_l", g, 0.75, y)
            add_box(g + "_r", g, 0.88, y)
            y -= 0.055
        y -= 0.015
        for k in ("speed", "turn", "accel"):
            add_box(k, k, 0.80, y, 0.12)
            y -= 0.055
        rax = fig.add_axes([0.80, y, 0.12, 0.045])
        run_box = TextBox(rax, "run s ", initial=f"{self.run_sec:g}")

        def set_run_sec(text):
            try:
                self.run_sec = max(0.5, float(text))
            except ValueError:
                pass
        run_box.on_submit(set_run_sec)
        y -= 0.065
        buttons = []
        for label, fn, x in (("Forward", lambda e: self.start_run("straight"), 0.74),
                             ("Reverse", lambda e: self.start_run("reverse"), 0.86),
                             ("Stop", lambda e: self.stop(), 0.74),
                             ("Print config", lambda e: self.print_config(), 0.86)):
            bx = fig.add_axes([x, y, 0.11, 0.05])
            b = Button(bx, label)
            b.on_clicked(fn)
            buttons.append(b)
            if label == "Reverse":
                y -= 0.06
        cax = fig.add_axes([0.74, y - 0.065, 0.23, 0.055], frameon=False)
        checks = CheckButtons(cax, ["show L", "show R"], [True, True])

        def toggle(label):
            vis = checks.get_status()
            for key in ("spL", "vL", "dL"):
                ln[key].set_visible(vis[0])
            for key in ("spR", "vR", "dR"):
                ln[key].set_visible(vis[1])
            fig.canvas.draw_idle()
        checks.on_clicked(toggle)
        self._widgets = (boxes, buttons, checks, run_box)   # keep widgets alive
        y -= 0.07
        metrics_txt = fig.text(0.72, y - 0.04, "", fontsize=8.5, family="monospace", va="top")
        fig.text(0.72, 0.015, "Type a value and press Enter to send it. Changes last until reset.",
                 fontsize=7.5, color="gray")

        def update(_):
            if self.tuning_dirty:
                self.tuning_dirty = False
                self._updating = True
                for k, tb in boxes.items():
                    if k in self.tuning:
                        tb.set_val(f"{self.tuning[k]:g}")
                self._updating = False
            if self.metrics_dirty:
                self.metrics_dirty = False
                metrics_txt.set_text(self.metrics)
            s = self.rec.snapshot()
            if not s:
                return
            t = [x[0] for x in s]
            for i, key in enumerate(["spL", "vL", "dL", "spR", "vR", "dR"]):
                ln[key].set_data(t, [x[2 + i] for x in s])
            tgt = self.tuning.get("speed", 0.16)
            band_lo.set_ydata([tgt * (1 - SETTLE_BAND)] * 2)
            band_hi.set_ydata([tgt * (1 + SETTLE_BAND)] * 2)
            hi = max(t[-1], 1.0)
            lo = max(0.0, hi - 30.0)
            ax_v.set_xlim(lo, hi + 0.2)
            vs = [v for x in s for v in (x[2], x[3], x[5], x[6])] + [tgt, -tgt * 0.1]
            ax_v.set_ylim(min(vs) - 0.02, max(vs) + 0.02)
            ds = [d for x in s for d in (x[4], x[7])] + [0, 10]
            ax_d.set_ylim(min(ds) - 3, max(ds) + 3)

        self._anim = FuncAnimation(fig, update, interval=100, cache_frame_data=False)
        plt.show()


def find_port():
    ports = sorted(glob.glob("/dev/tty.usbmodem*") + glob.glob("/dev/ttyACM*"))
    if not ports:
        sys.exit("No /dev/tty.usbmodem* found. Is the Nucleo plugged in? Use --port to name it.")
    if len(ports) > 1:
        print(f"Several ports found, using {ports[0]}: {ports}")
    return ports[0]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="serial port (default: first /dev/tty.usbmodem*)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--run", type=float, default=4.0, help="seconds per test run (default 4)")
    ap.add_argument("--demo", action="store_true", help="simulated motors, no hardware")
    args = ap.parse_args()

    link = DemoLink() if args.demo else SerialLink(args.port or find_port(), args.baud)
    tuner = Tuner(link, args.run)
    try:
        tuner.run_gui()
    finally:
        tuner.close()


if __name__ == "__main__":
    main()
