#!/usr/bin/env python3
"""Plot rio_replay outputs. Pass one or more run directories.

Each run dir is expected to contain state.csv, cov_diag.csv, radar_innov.csv,
baro_innov.csv produced by rio_replay. Multiple runs are overlaid for visual
comparison.

Usage:
    python tools/replay/scripts/plot_runs.py runs/baseline runs/sigma_vr_high
    python tools/replay/scripts/plot_runs.py runs/baseline --save
"""
import argparse
import json
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

import plot_style
from plot_style import ABLATION_COLORS, plot_extrinsic_convergence


def quat_to_euler_zyx(q_w, q_x, q_y, q_z):
    """Convert quaternions (w, x, y, z) to ZYX (roll, pitch, yaw) Euler angles."""
    r = np.arctan2(2 * (q_w * q_x + q_y * q_z), 1 - 2 * (q_x**2 + q_y**2))
    sinp = 2 * (q_w * q_y - q_z * q_x)
    p = np.where(np.abs(sinp) >= 1, np.sign(sinp) * np.pi / 2, np.arcsin(sinp))
    y = np.arctan2(2 * (q_w * q_z + q_x * q_y), 1 - 2 * (q_y**2 + q_z**2))
    return r, p, y


def load_run(d: Path):
    state = pd.read_csv(d / "state.csv")
    cov = pd.read_csv(d / "cov_diag.csv")
    rinn = pd.read_csv(d / "radar_innov.csv")
    binn = pd.read_csv(d / "baro_innov.csv")
    # state.csv and cov_diag.csv emit one row per filter step in lockstep.
    assert len(state) == len(cov), f"row count mismatch in {d}"
    # Prefer a human label from config.json (written by run_ablation.sh) so
    # ablation runs read as "baro+UW" etc. rather than the raw dir name.
    name = d.name
    cfg = d / "config.json"
    if cfg.is_file():
        try:
            name = json.loads(cfg.read_text()).get("label", name)
        except (ValueError, OSError):
            pass
    return {"name": name, "dir": d, "state": state, "cov": cov,
            "rinn": rinn, "binn": binn}


def plot_trajectory_xy(ax, runs, colors):
    for r, c in zip(runs, colors):
        s = r["state"]
        ax.plot(s.p_x, s.p_y, color=c, lw=0.8, label=r["name"])
        ax.scatter(s.p_x.iloc[0], s.p_y.iloc[0], color=c, marker="o", s=20)
        ax.scatter(s.p_x.iloc[-1], s.p_y.iloc[-1], color=c, marker="x", s=30)
    ax.set_xlabel("p_x [m]"); ax.set_ylabel("p_y [m]")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(alpha=0.3); ax.legend(fontsize=8)


def plot_z_over_time(ax, runs, colors):
    for r, c in zip(runs, colors):
        s, cov = r["state"], r["cov"]
        sig = np.sqrt(np.clip(cov.P02, 0, None))
        ax.fill_between(s.t, s.p_z - sig, s.p_z + sig, color=c, alpha=0.15)
        ax.plot(s.t, s.p_z, color=c, lw=0.6, label=r["name"])
    ax.set_xlabel("t [s]"); ax.set_ylabel("p_z [m]")
    ax.grid(alpha=0.3); ax.legend(fontsize=8)


def plot_speed(ax, runs, colors):
    for r, c in zip(runs, colors):
        s = r["state"]
        v = np.sqrt(s.v_x**2 + s.v_y**2 + s.v_z**2)
        ax.plot(s.t, v, color=c, lw=0.6, label=r["name"])
    ax.set_xlabel("t [s]"); ax.set_ylabel("|v| [m/s]")
    ax.grid(alpha=0.3); ax.legend(fontsize=8)


def plot_sigmas(ax, runs, colors):
    """Position/velocity/attitude RMS sigma vs time, log scale."""
    for r, c in zip(runs, colors):
        cov = r["cov"]
        sp = np.sqrt(np.clip(cov.P00 + cov.P01 + cov.P02, 0, None) / 3)
        sv = np.sqrt(np.clip(cov.P03 + cov.P04 + cov.P05, 0, None) / 3)
        sa = np.sqrt(np.clip(cov.P09 + cov.P10 + cov.P11, 0, None) / 3)
        ax.plot(cov.t, sp, color=c, lw=0.7, label=f"{r['name']}  σ_p")
        ax.plot(cov.t, sv, color=c, lw=0.7, ls="--", label=f"{r['name']}  σ_v")
        ax.plot(cov.t, sa, color=c, lw=0.7, ls=":", label=f"{r['name']}  σ_θ")
    ax.set_yscale("log")
    ax.set_xlabel("t [s]"); ax.set_ylabel("σ (rms over xyz)")
    ax.grid(alpha=0.3, which="both"); ax.legend(fontsize=7)


def plot_radar_innov_hist(ax, runs, colors):
    """Normalized radar innovation histogram for ACCEPTED measurements.
    Should match N(0, 1) when σ_vr is well-tuned. A peak narrower than N(0,1)
    means σ_vr is over-estimated; broader means under-estimated."""
    bins = np.linspace(-5, 5, 80)
    for r, c in zip(runs, colors):
        rinn = r["rinn"]
        acc = rinn[rinn.status == 0]
        z = acc.residual / np.sqrt(acc.S.clip(lower=1e-12))
        z = z[np.isfinite(z)]
        ax.hist(z, bins=bins, density=True, alpha=0.45, color=c,
                label=f"{r['name']}  n={len(z)}  std={z.std():.3f}")
    xs = np.linspace(-5, 5, 200)
    ax.plot(xs, np.exp(-xs**2 / 2) / np.sqrt(2 * np.pi),
            color="k", lw=1, ls="--", label="N(0,1)")
    ax.set_yscale("log")
    ax.set_xlabel("residual / √S"); ax.set_ylabel("density (log)")
    ax.grid(alpha=0.3, which="both"); ax.legend(fontsize=8)


def plot_baro_innov(ax, runs, colors):
    """Baro residual over time, with ±√S band."""
    any_data = False
    for r, c in zip(runs, colors):
        b = r["binn"]
        b = b[(b.accepted == 1) | (b.rejected == 1)]
        if b.empty:
            continue
        # Force float64 — pandas occasionally hands back object-dtype arrays
        # which break matplotlib's internal np.isfinite() in fill_between.
        t   = np.asarray(b.t,        dtype=np.float64)
        sig = np.sqrt(np.clip(np.asarray(b.S, dtype=np.float64), 0, None))
        res = np.asarray(b.residual, dtype=np.float64)
        ax.fill_between(t, -sig, sig, color=c, alpha=0.15)
        ax.plot(t, res, color=c, lw=0.5, label=r["name"])
        any_data = True
    ax.axhline(0, color="k", lw=0.5)
    ax.set_xlabel("t [s]"); ax.set_ylabel("Δz residual [m]")
    ax.grid(alpha=0.3)
    if any_data:
        ax.legend(fontsize=8)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("dirs", nargs="+", type=Path,
                    help="run directories produced by rio_replay")
    ap.add_argument("--save", action="store_true",
                    help="save PNG instead of opening a window")
    args = ap.parse_args()

    plot_style.setup_mpl()

    for d in args.dirs:
        if not d.is_dir():
            raise SystemExit(f"not a directory: {d}")

    runs = [load_run(d) for d in args.dirs]
    colors = [ABLATION_COLORS[i % len(ABLATION_COLORS)] for i in range(len(runs))]

    fig, axes = plt.subplots(2, 3, figsize=(17, 10))
    plot_trajectory_xy   (axes[0, 0], runs, colors)
    plot_z_over_time     (axes[0, 1], runs, colors)
    plot_speed           (axes[0, 2], runs, colors)
    plot_sigmas          (axes[1, 0], runs, colors)
    plot_radar_innov_hist(axes[1, 1], runs, colors)
    plot_baro_innov      (axes[1, 2], runs, colors)
    fig.tight_layout()

    # Radar extrinsic convergence (p_IR / q_IR + ±σ) for the same run set.
    fig_ext = plot_extrinsic_convergence(
        [r["dir"] for r in runs], labels=[r["name"] for r in runs],
        colors=colors, bands="first" if len(runs) > 1 else "all")

    if args.save:
        out = args.dirs[0] / "compare.png"
        fig.savefig(out, dpi=120)
        out_ext = args.dirs[0] / "compare_extrinsics.png"
        fig_ext.savefig(out_ext, dpi=120)
        print(f"wrote {out}\nwrote {out_ext}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
