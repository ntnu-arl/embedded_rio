#!/usr/bin/env python3
"""
RIO ESKF State Visualizer — full state + covariances
──────────────────────────────────────────────────────────
Serial output parsed:
  p_WI / v_WI / q_WI / b_a / b_g / p_IR / q_IR  — ESKF state
  cov_xy=[...] / cov_z=[...]                      — covariance std devs
  RATES imu_hz=N radar_hz=N baro_hz=N             — measurement rates

Usage:
  python eskf_visualizer.py
  python eskf_visualizer.py --port /dev/ttyACM0
  python eskf_visualizer.py --file log.txt [--speed 5]

Dependencies:
  pip install pyserial matplotlib
"""

import argparse
import math
import re
import sys
import threading
import time
from collections import deque

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.animation import FuncAnimation

# ── regex ──────────────────────────────────────────────────────────────────────
_F = r"([-\d.eE+]+)"

def _v3(tag):  return re.compile(rf"{tag}=\[\s*{_F},\s*{_F},\s*{_F}\]")
def _q4(tag):  return re.compile(rf"{tag}=\[\s*{_F},\s*{_F},\s*{_F},\s*{_F}\]")

RE = {
    "p_WI":   _v3("p_WI"),
    "v_WI":   _v3("v_WI"),
    "q_WI":   _q4("q_WI"),
    "b_a":    _v3("b_a"),
    "b_g":    _v3("b_g"),
    "p_IR":   _v3("p_IR"),
    "q_IR":   _q4("q_IR"),
    "corr":   re.compile(r"ESKF correct:\s*(\d+)\s*accepted,\s*(\d+)\s*rejected,\s*(\d+)\s*skipped"),
    "baro":   re.compile(r"BARO STATS:\s*(\d+)\s*accepted,\s*(\d+)\s*rejected,\s*(\d+)\s*skipped"),
    "att":    re.compile(r"Attitude initialized from gravity"),
    # Firmware prints "cov_z= [" (extra space, padded to align with "cov_xy=[")
    # — allow optional whitespace between '=' and '[' for both.
    "cov_xy": re.compile(rf"cov_xy=\s*\[\s*{_F},\s*{_F},\s*{_F},\s*{_F},\s*{_F},\s*{_F},\s*{_F}\]"),
    "cov_z":  re.compile(rf"cov_z=\s*\[\s*{_F},\s*{_F},\s*{_F},\s*{_F},\s*{_F},\s*{_F},\s*{_F}\]"),
    "rates":  re.compile(r"RATES imu_hz=([\d.]+) radar_hz=([\d.]+) baro_hz=([\d.]+)"),
}

# ── shared state ───────────────────────────────────────────────────────────────
MAXLEN = 500

FRAME_KEYS = ["t", "px","py","pz", "vx","vy","vz", "spd",
              "roll","pitch","yaw",
              "bax","bay","baz", "bgx","bgy","bgz",
              "pir_x","pir_y","pir_z",
              "qir_roll","qir_pitch","qir_yaw"]

# Covariance group labels: p, v, b_a, att, b_g, p_IR, q_IR
COV_LABELS = ["p", "v", "b_a", "att", "b_g", "p_IR", "q_IR"]
COV_COLORS = ["#5599ee", "#44bb88", "#ee7744", "#aa88ff",
              "#ff4466", "#ffcc44", "#66ddcc"]

bufs   = {k: deque(maxlen=MAXLEN) for k in FRAME_KEYS}
stats  = {"accepted": 0, "rejected": 0, "skipped": 0,
          "b_accepted": 0, "b_rejected": 0, "b_skipped": 0,
          "frames": 0}
status = ["waiting for data…"]
rate_bufs = {
    "t":        deque(maxlen=MAXLEN),
    "imu_hz":   deque(maxlen=MAXLEN),
    "radar_hz": deque(maxlen=MAXLEN),
    "baro_hz":  deque(maxlen=MAXLEN),
}
# cov_bufs["xy"][i] and cov_bufs["z"][i] for i in 0..6
cov_bufs = {
    "t":  deque(maxlen=MAXLEN),
    "xy": [deque(maxlen=MAXLEN) for _ in range(7)],
    "z":  [deque(maxlen=MAXLEN) for _ in range(7)],
}
lock   = threading.Lock()

# ESKF pending frame
_pending: dict = {}
t0 = time.time()


def _rpy(w, x, y, z):
    roll  = math.degrees(math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y)))
    sp    = max(-1.0, min(1.0, 2*(w*y - z*x)))
    pitch = math.degrees(math.asin(sp))
    yaw   = math.degrees(math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z)))
    return roll, pitch, yaw


def _commit(frame: dict):
    if "t" not in frame:
        return
    if not any(k in frame for k in ("px", "py", "pz")):
        return
    with lock:
        for k in FRAME_KEYS:
            bufs[k].append(frame.get(k, float("nan")))
        stats["frames"] += 1
        status[0] = "live"


def parse_line(line: str):
    global _pending

    # ── ESKF state lines ──────────────────────────────────────
    m = RE["p_WI"].search(line)
    if m:
        _commit(_pending)
        _pending = {
            "t":  time.time() - t0,
            "px": float(m[1]), "py": float(m[2]), "pz": float(m[3]),
        }
        return

    m = RE["v_WI"].search(line)
    if m:
        vx, vy, vz = float(m[1]), float(m[2]), float(m[3])
        _pending.update(vx=vx, vy=vy, vz=vz,
                        spd=math.sqrt(vx*vx + vy*vy + vz*vz))
        return

    m = RE["q_WI"].search(line)
    if m:
        r, p, y = _rpy(float(m[1]), float(m[2]), float(m[3]), float(m[4]))
        _pending.update(roll=r, pitch=p, yaw=y)
        return

    m = RE["b_a"].search(line)
    if m:
        _pending.update(bax=float(m[1]), bay=float(m[2]), baz=float(m[3]))
        return

    m = RE["b_g"].search(line)
    if m:
        _pending.update(bgx=float(m[1]), bgy=float(m[2]), bgz=float(m[3]))
        return

    m = RE["p_IR"].search(line)
    if m:
        _pending.update(pir_x=float(m[1]), pir_y=float(m[2]), pir_z=float(m[3]))
        return

    m = RE["q_IR"].search(line)
    if m:
        r, p, y = _rpy(float(m[1]), float(m[2]), float(m[3]), float(m[4]))
        _pending.update(qir_roll=r, qir_pitch=p, qir_yaw=y)
        _commit(_pending)
        _pending = {}
        return

    m = RE["corr"].search(line)
    if m:
        with lock:
            stats["accepted"] = int(m[1])
            stats["rejected"]  = int(m[2])
            stats["skipped"]   = int(m[3])
        return

    m = RE["baro"].search(line)
    if m:
        with lock:
            stats["b_accepted"] = int(m[1])
            stats["b_rejected"] = int(m[2])
            stats["b_skipped"]  = int(m[3])
        return

    # ── covariances ───────────────────────────────────────────
    m = RE["cov_xy"].search(line)
    if m:
        with lock:
            cov_bufs["t"].append(time.time() - t0)
            for i in range(7):
                cov_bufs["xy"][i].append(float(m[i + 1]))
        return

    m = RE["cov_z"].search(line)
    if m:
        with lock:
            for i in range(7):
                cov_bufs["z"][i].append(float(m[i + 1]))
        return

    m = RE["rates"].search(line)
    if m:
        with lock:
            rate_bufs["t"].append(time.time() - t0)
            rate_bufs["imu_hz"].append(float(m[1]))
            rate_bufs["radar_hz"].append(float(m[2]))
            rate_bufs["baro_hz"].append(float(m[3]))
        return

    if RE["att"].search(line):
        with lock:
            status[0] = "attitude init OK"


# ── reader threads ─────────────────────────────────────────────────────────────

def serial_reader(port, baud):
    import serial
    print(f"Opening {port} @ {baud} baud…")
    with serial.Serial(port, baud, timeout=1) as ser:
        while True:
            try:
                line = ser.readline().decode("utf-8", errors="replace").rstrip()
                if line:
                    print(line)
                    parse_line(line)
            except Exception as e:
                print(f"Serial error: {e}", file=sys.stderr)
                time.sleep(0.5)


def file_reader(path, speed=1.0):
    print(f"Replaying {path} (speed ×{speed})…")
    with open(path) as f:
        lines = f.readlines()
    for line in lines:
        parse_line(line.rstrip())
        time.sleep(0.015 / max(speed, 0.01))


def autodetect_port():
    try:
        import serial.tools.list_ports
        ports = list(serial.tools.list_ports.comports())
        for p in ports:
            if any(k in p.device for k in ("ACM", "usbmodem", "ttyUSB")):
                return p.device
        return ports[0].device if ports else None
    except Exception:
        return None


# ── plot helpers ───────────────────────────────────────────────────────────────
BG, FG, GRID = "#1a1a1a", "#cccccc", "#2e2e2e"
C = {"x": "#5599ee", "y": "#44bb88", "z": "#ee7744", "s": "#aa88ff"}
D = {"x": [],        "y": [4, 2],    "z": [1, 2]}


def _style(ax):
    ax.set_facecolor(BG)
    ax.tick_params(colors=FG, labelsize=7)
    for sp in ax.spines.values():
        sp.set_edgecolor("#3a3a3a")
    ax.xaxis.label.set_color(FG)
    ax.yaxis.label.set_color(FG)
    ax.title.set_color(FG)
    ax.grid(True, color=GRID, linewidth=0.4, linestyle="--")


def _ln(ax, axis, label=""):
    ln, = ax.plot([], [], color=C[axis], lw=1.2, dashes=D.get(axis, []), label=label)
    return ln


def _leg(ax):
    ax.legend(fontsize=6, facecolor="#222", edgecolor="#3a3a3a", labelcolor=FG,
              loc="upper left", ncol=3, handlelength=2,
              borderpad=0.4, labelspacing=0.2)


def _xyz(ax, title, ylabel):
    ax.set_title(title, fontsize=8); ax.set_ylabel(ylabel, fontsize=7)
    lx, ly, lz = _ln(ax,"x","x"), _ln(ax,"y","y"), _ln(ax,"z","z")
    _leg(ax)
    return lx, ly, lz


def _autoscale(ax, xs, *ybufs):
    ys = [v for b in ybufs for v in b if not (isinstance(v, float) and math.isnan(v))]
    if not xs or not ys:
        return
    pad = max(abs(max(ys) - min(ys)) * 0.12, 1e-4)
    ax.set_xlim(min(xs), max(max(xs), min(xs) + 1))
    ax.set_ylim(min(ys) - pad, max(ys) + pad)


def _set(ln, t, y):
    n = min(len(t), len(y))
    if n > 0:
        ln.set_data(list(t)[-n:], list(y)[-n:])


# ── figure ─────────────────────────────────────────────────────────────────────

def build_figure():
    fig = plt.figure(figsize=(20, 10), facecolor=BG)
    fig.canvas.manager.set_window_title("RIO ESKF — full state + covariance visualizer")

    gs = gridspec.GridSpec(4, 4, figure=fig,
                           left=0.05, right=0.98,
                           top=0.93, bottom=0.06,
                           hspace=0.65, wspace=0.38)

    ax = {
        "pos":    fig.add_subplot(gs[0, 0]),
        "vel":    fig.add_subplot(gs[1, 0]),
        "att":    fig.add_subplot(gs[2, 0]),
        "spd":    fig.add_subplot(gs[3, 0]),
        "traj":   fig.add_subplot(gs[0:2, 1]),
        "ba":     fig.add_subplot(gs[2, 1]),
        "bg":     fig.add_subplot(gs[3, 1]),
        "p_ir":   fig.add_subplot(gs[0, 2]),
        "q_ir":   fig.add_subplot(gs[1, 2]),
        "bars":   fig.add_subplot(gs[2, 2]),
        "rates":  fig.add_subplot(gs[3, 2]),
        "cov_xy": fig.add_subplot(gs[0:2, 3]),
        "cov_z":  fig.add_subplot(gs[2:4, 3]),
    }
    for a in ax.values():
        _style(a)

    ln = {}
    ln["px"],  ln["py"],  ln["pz"]   = _xyz(ax["pos"],  "position  p_WI",          "m")
    ln["vx"],  ln["vy"],  ln["vz"]   = _xyz(ax["vel"],  "velocity  v_WI",          "m/s")
    ln["roll"],ln["pitch"],ln["yaw"] = _xyz(ax["att"],  "attitude  roll/pitch/yaw","deg")
    ln["bax"], ln["bay"], ln["baz"]  = _xyz(ax["ba"],   "accel bias  b_a",         "m/s²")
    ln["bgx"], ln["bgy"], ln["bgz"]  = _xyz(ax["bg"],   "gyro bias  b_g",          "rad/s")
    ln["pirx"],ln["piry"],ln["pirz"] = _xyz(ax["p_ir"],"extrinsic  p_IR",          "m")
    ln["qirr"],ln["qirp"],ln["qiry"] = _xyz(ax["q_ir"],"extrinsic  q_IR (RPY)",    "deg")

    ax["spd"].set_title("|v| speed", fontsize=8)
    ax["spd"].set_ylabel("m/s", fontsize=7)
    ax["spd"].set_xlabel("time (s)", fontsize=7)
    ln["spd"], = ax["spd"].plot([], [], color=C["s"], lw=1.2)

    ax["bg"].set_xlabel("time (s)", fontsize=7)

    ax["traj"].set_title("trajectory  x–y", fontsize=8)
    ax["traj"].set_xlabel("x (m)", fontsize=7)
    ax["traj"].set_ylabel("y (m)", fontsize=7)
    ax["traj"].set_aspect("equal", adjustable="datalim")
    ln["tpath"], = ax["traj"].plot([], [], color=C["x"], lw=0.8, alpha=0.6)
    ln["tdot"],  = ax["traj"].plot([], [], "o", color="#ff4466", ms=5, zorder=5)

    ax["bars"].set_title("last counts  (RAD batch | BAR /s)", fontsize=8)
    ax["bars"].set_ylabel("count", fontsize=7)
    # Six bars: 3 radar (last batch) + 3 baro (last 1-second window).
    # Lighter shades distinguish baro from radar.
    bars = ax["bars"].bar(
        ["R-acc", "R-rej", "R-skp", "B-acc", "B-rej", "B-skp"],
        [0, 0, 0, 0, 0, 0],
        color=["#44bb88", "#ee5555", "#eeaa33",
               "#88dda8", "#f08888", "#f0c477"],
        width=0.6)
    btxt = [ax["bars"].text(b.get_x()+b.get_width()/2, 0.2, "0",
                            ha="center", va="bottom", fontsize=8, color=FG)
            for b in bars]
    ax["bars"].set_ylim(0, 10)
    for lbl in ax["bars"].get_xticklabels():
        lbl.set_fontsize(7)

    # ── rates panel: IMU + Radar + Baro ──────────────────────────────────────
    ax["rates"].set_title("measurement rates", fontsize=8)
    ax["rates"].axis("off")
    rtxt_imu   = ax["rates"].text(0.5, 0.78, "IMU:    — Hz",
                                  ha="center", va="center", fontsize=11,
                                  color="#5599ee", fontfamily="monospace",
                                  transform=ax["rates"].transAxes)
    rtxt_radar = ax["rates"].text(0.5, 0.48, "Radar:  — Hz",
                                  ha="center", va="center", fontsize=11,
                                  color="#ee7744", fontfamily="monospace",
                                  transform=ax["rates"].transAxes)
    rtxt_baro  = ax["rates"].text(0.5, 0.18, "Baro:   — Hz",
                                  ha="center", va="center", fontsize=11,
                                  color="#44bb88", fontfamily="monospace",
                                  transform=ax["rates"].transAxes)

    ax["cov_xy"].set_title("covariance σ  (xy avg)", fontsize=8)
    ax["cov_xy"].set_ylabel("σ", fontsize=7)
    ax["cov_xy"].set_yscale("log")

    ax["cov_z"].set_title("covariance σ  (z)", fontsize=8)
    ax["cov_z"].set_ylabel("σ", fontsize=7)
    ax["cov_z"].set_xlabel("time (s)", fontsize=7)
    ax["cov_z"].set_yscale("log")

    cov_ln_xy, cov_ln_z = [], []
    for i, (label, color) in enumerate(zip(COV_LABELS, COV_COLORS)):
        lxy, = ax["cov_xy"].plot([], [], color=color, lw=1.2, label=label)
        lz,  = ax["cov_z"].plot( [], [], color=color, lw=1.2, label=label)
        cov_ln_xy.append(lxy)
        cov_ln_z.append(lz)

    for a in ("cov_xy", "cov_z"):
        ax[a].legend(fontsize=6, facecolor="#222", edgecolor="#3a3a3a",
                     labelcolor=FG, loc="upper right", ncol=1,
                     handlelength=2, borderpad=0.4, labelspacing=0.2)

    stxt = fig.text(0.5, 0.97, "RIO ESKF — waiting for data…",
                    ha="center", va="top", color=FG,
                    fontsize=9, fontfamily="monospace")

    return (fig, ax, ln, bars, btxt, stxt,
            rtxt_imu, rtxt_radar, rtxt_baro, cov_ln_xy, cov_ln_z)


# ── animation ──────────────────────────────────────────────────────────────────

def animate(_, artists):
    (fig, ax, ln, bars, btxt, stxt,
     rtxt_imu, rtxt_radar, rtxt_baro, cov_ln_xy, cov_ln_z) = artists

    with lock:
        t   = list(bufs["t"])
        d   = {k: list(bufs[k]) for k in FRAME_KEYS if k != "t"}
        s   = dict(stats)
        msg = status[0]
        r_imu_hz   = list(rate_bufs["imu_hz"])
        r_radar_hz = list(rate_bufs["radar_hz"])
        r_baro_hz  = list(rate_bufs["baro_hz"])
        ct         = list(cov_bufs["t"])
        cxy        = [list(cov_bufs["xy"][i]) for i in range(7)]
        cz         = [list(cov_bufs["z"][i])  for i in range(7)]

    # ── ESKF time-series ──────────────────────────────────────────────────
    panels = [
        ("pos",  ["px","py","pz"]),
        ("vel",  ["vx","vy","vz"]),
        ("att",  ["roll","pitch","yaw"]),
        ("spd",  ["spd"]),
        ("ba",   ["bax","bay","baz"]),
        ("bg",   ["bgx","bgy","bgz"]),
        ("p_ir", ["pir_x","pir_y","pir_z"]),
        ("q_ir", ["qir_roll","qir_pitch","qir_yaw"]),
    ]
    lnmap = {
        "px":"px","py":"py","pz":"pz",
        "vx":"vx","vy":"vy","vz":"vz",
        "roll":"roll","pitch":"pitch","yaw":"yaw",
        "spd":"spd",
        "bax":"bax","bay":"bay","baz":"baz",
        "bgx":"bgx","bgy":"bgy","bgz":"bgz",
        "pir_x":"pirx","pir_y":"piry","pir_z":"pirz",
        "qir_roll":"qirr","qir_pitch":"qirp","qir_yaw":"qiry",
    }

    if t:
        for panel, keys in panels:
            for k in keys:
                _set(ln[lnmap[k]], t, d[k])
            _autoscale(ax[panel], t, *[d[k] for k in keys])

        # Trajectory: filter out NaN pairs before plotting
        px_raw, py_raw = d["px"], d["py"]
        valid = [(x, y) for x, y in zip(px_raw, py_raw)
                 if not (math.isnan(x) or math.isnan(y))]
        if valid:
            vx_list, vy_list = zip(*valid)
            ln["tpath"].set_data(vx_list, vy_list)
            ln["tdot"].set_data([vx_list[-1]], [vy_list[-1]])
        ax["traj"].relim(); ax["traj"].autoscale_view()

    vals  = [s["accepted"],   s["rejected"],   s["skipped"],
             s["b_accepted"], s["b_rejected"], s["b_skipped"]]
    max_v = max(vals + [1])
    ax["bars"].set_ylim(0, max_v * 1.5)
    for bar, txt, v in zip(bars, btxt, vals):
        bar.set_height(v)
        txt.set_text(str(v))
        txt.set_y(v + max_v * 0.04)

    stxt.set_text(f"RIO ESKF — frames: {s['frames']}   {msg}")

    if r_imu_hz:
        rtxt_imu.set_text(f"IMU:    {r_imu_hz[-1]:.1f} Hz")
    if r_radar_hz:
        rtxt_radar.set_text(f"Radar:  {r_radar_hz[-1]:.1f} Hz")
    if r_baro_hz:
        rtxt_baro.set_text(f"Baro:   {r_baro_hz[-1]:.1f} Hz")

    if ct:
        all_xy, all_z = [], []
        for i in range(7):
            _set(cov_ln_xy[i], ct, cxy[i])
            _set(cov_ln_z[i],  ct, cz[i])
            all_xy += [v for v in cxy[i] if v > 0]
            all_z  += [v for v in cz[i]  if v > 0]

        for aname, vals in (("cov_xy", all_xy), ("cov_z", all_z)):
            if vals:
                ax[aname].set_xlim(min(ct), max(max(ct), min(ct) + 1))
                ax[aname].set_ylim(min(vals) * 0.5, max(vals) * 2.0)


# ── main ───────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description="RIO ESKF full state + covariance visualizer")
    ap.add_argument("--port",  help="Serial port (e.g. /dev/ttyACM0)")
    ap.add_argument("--baud",  type=int,   default=115200)
    ap.add_argument("--file",  help="Replay from a log file")
    ap.add_argument("--speed", type=float, default=1.0)
    args = ap.parse_args()

    if args.file:
        th = threading.Thread(target=file_reader,
                              args=(args.file, args.speed), daemon=True)
    else:
        port = args.port or autodetect_port()
        if not port:
            print("No serial port found. Use --port /dev/ttyACM0 or --file log.txt")
            sys.exit(1)
        th = threading.Thread(target=serial_reader,
                              args=(port, args.baud), daemon=True)
    th.start()

    artists = build_figure()
    _ani = FuncAnimation(artists[0], animate, fargs=(artists,),
                         interval=100, cache_frame_data=False)
    plt.show()


if __name__ == "__main__":
    main()