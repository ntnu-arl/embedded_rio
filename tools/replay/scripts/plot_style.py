#!/usr/bin/env python3
"""Shared plotting style for the rio_teensy41 replay/comparison tooling.

Single source of truth for colors, line styles, matplotlib rcParams, and the
radar-extrinsic convergence plot — so every figure (compare_with_ulog.py,
plot_runs.py, compare_ablation.py) uses the same thesis-quality convention.

Convention (locked for the thesis):
  * ground truth → grey, DOTTED. GT = mocap when a bag is present, else EKF2.
  * estimates → solid, distinct colors:
        replay (our RIO) ... blue   (#1f77b4)
        EKF2 (PX4)       ... orange (#ff7f0e)
        live VVO         ... green  (#2ca02c)
  * ablation (4 replay variants, all "the signal" under different configs):
        baro+UW (baseline) blue, baro·noUW orange, noBaro·UW green, none red.
"""
import warnings
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib as mpl
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation

# ── palette ────────────────────────────────────────────────────────────────
GREY   = "0.45"
BLUE   = "#1f77b4"
ORANGE = "#ff7f0e"
GREEN  = "#2ca02c"
RED    = "#d62728"

GT_LS = ":"      # ground truth is always dotted
GT_LW = 1.6

# X/Y/Z component colors (used by per-axis error plots, raw component traces).
AXIS_COLORS = [BLUE, ORANGE, GREEN]
NORM_COLOR  = RED

# Ordered colors for the baro×underweighting ablation (4 configs).
ABLATION_COLORS = [BLUE, ORANGE, GREEN, RED]

# Solid "estimate" style per logical trace. The trace that is the ground
# truth is overridden to the grey-dotted GT style by role_styles() below.
_ROLE = {
    "replay": dict(color=BLUE,   ls="-", lw=1.1, alpha=0.95, label="replay (RIO)"),
    "ekf2":   dict(color=ORANGE, ls="-", lw=1.1, alpha=0.95, label="EKF2 (PX4)"),
    "vvo":    dict(color=GREEN,  ls="-", lw=1.0, alpha=0.90, label="live VVO"),
    "mocap":  dict(color=GREY,   ls="-", lw=1.1, alpha=0.90, label="mocap"),
}
_GT_KW = dict(color=GREY, ls=GT_LS, lw=GT_LW, alpha=0.90, zorder=1)


def setup_mpl():
    """Apply thesis rcParams. Call once at the top of each script's main()."""
    # Benign matplotlib 3.6+ noise emitted when tight_layout() + bbox='tight'
    # are both in play (as handle_fig does); keeps the console readable.
    warnings.filterwarnings(
        "ignore", message="The figure layout has changed to tight")
    plt.rcParams.update({
        "figure.dpi":      110,
        "savefig.dpi":     200,
        "font.family":     "serif",
        "font.size":       11,
        "axes.titlesize":  11,
        "axes.labelsize":  10,
        "legend.fontsize": 8,
        "axes.grid":       True,
        "grid.alpha":      0.3,
        "axes.prop_cycle": mpl.cycler(color=ABLATION_COLORS),
    })


def role_styles(present, gt_name):
    """Return ``{name: kwargs}`` ready to splat into ``ax.plot``.

    ``present``  iterable of trace names ⊆ {"ekf2","replay","vvo","mocap"}.
    ``gt_name``  the ground-truth trace ("mocap" if present else "ekf2").

    The GT trace gets the grey-dotted style; every other present trace gets
    its solid role color. The color the GT trace would otherwise have used is
    simply left unused (per the locked convention).
    """
    styles = {}
    for name in present:
        if name == gt_name:
            kw = dict(_GT_KW)
            kw["label"] = f"{_ROLE[name]['label']} (GT)"
        else:
            kw = dict(_ROLE[name])
        styles[name] = kw
    return styles


def ablation_style(i, label):
    """Style kwargs for ablation config ``i`` (0-based)."""
    return dict(color=ABLATION_COLORS[i % len(ABLATION_COLORS)],
                ls="-", lw=1.1, alpha=0.95, label=label)


def band(kw):
    """Color-only kwargs for a ±σ ``fill_between`` matching a trace style."""
    return dict(color=kw.get("color", GREY), alpha=0.15, lw=0)


# ── radar extrinsic convergence ─────────────────────────────────────────────

_EUL_LABELS = ["roll", "pitch", "yaw"]


def _euler_deg(q_wxyz):
    q_xyzw = q_wxyz[:, [1, 2, 3, 0]]
    eul = np.rad2deg(np.unwrap(
        Rotation.from_quat(q_xyzw).as_euler("xyz"), axis=0))
    eul[:, 2] = ((eul[:, 2] + 180.0) % 360.0) - 180.0
    return eul


def load_extrinsics(run_dir: Path):
    """Read radar→IMU extrinsic estimate + ±σ from a run directory.

    Returns (t_rel, p_IR (N,3) [m], q_IR_euler (N,3) [deg],
             sig_p (N,3) [m], sig_a (N,3) [deg]).

    σ for the rotation is the error-state δθ_IR std (P18..20, rad) shown on the
    euler axes — an approximation, but adequate for a convergence band.
    """
    run_dir = Path(run_dir)
    s = pd.read_csv(run_dir / "state.csv")
    c = pd.read_csv(run_dir / "cov_diag.csv")
    n = min(len(s), len(c))
    s, c = s.iloc[:n], c.iloc[:n]
    t = s["t"].to_numpy(dtype=float)
    t = t - t[0]
    p = s[["p_IR_x", "p_IR_y", "p_IR_z"]].to_numpy(dtype=float)
    q = s[["q_IR_w", "q_IR_x", "q_IR_y", "q_IR_z"]].to_numpy(dtype=float)
    eul = _euler_deg(q)
    sig_p = np.sqrt(np.clip(c[["P15", "P16", "P17"]].to_numpy(dtype=float), 0, None))
    sig_a = np.rad2deg(
        np.sqrt(np.clip(c[["P18", "P19", "P20"]].to_numpy(dtype=float), 0, None)))
    return t, p, eul, sig_p, sig_a


def plot_extrinsic_convergence(run_dirs, labels=None, colors=None,
                               bands="first", prior_marker=True):
    """Plot radar extrinsic convergence for one or more runs.

    ``run_dirs``  list of run directory Paths (N≥1).
    ``labels``    legend labels (default: dir names).
    ``colors``    per-run colors (default: BLUE for N==1, else ABLATION_COLORS).
    ``bands``     "first" (only run 0), "all", or "none" — which runs get ±σ.
    ``prior_marker`` dotted horizontal line at each run's initial value.

    Layout: 2×3 — top p_IR x/y/z [m], bottom q_IR roll/pitch/yaw [deg].
    Returns the matplotlib Figure.
    """
    run_dirs = [Path(d) for d in run_dirs]
    if labels is None:
        labels = [d.name for d in run_dirs]
    if colors is None:
        colors = [BLUE] if len(run_dirs) == 1 else \
            [ABLATION_COLORS[i % len(ABLATION_COLORS)] for i in range(len(run_dirs))]

    fig, axs = plt.subplots(2, 3, figsize=(15, 8), sharex=True)
    p_labels = ["p_IR_x [m]", "p_IR_y [m]", "p_IR_z [m]"]
    for ri, d in enumerate(run_dirs):
        t, p, eul, sig_p, sig_a = load_extrinsics(d)
        c = colors[ri % len(colors)]
        show_band = (bands == "all") or (bands == "first" and ri == 0)
        for i in range(3):
            # translation row
            ax = axs[0, i]
            if show_band:
                ax.fill_between(t, p[:, i] - sig_p[:, i], p[:, i] + sig_p[:, i],
                                color=c, alpha=0.15, lw=0)
            ax.plot(t, p[:, i], color=c, lw=1.1, label=labels[ri])
            if prior_marker:
                ax.axhline(p[0, i], color=c, ls=":", lw=0.7, alpha=0.6)
            # rotation row
            ax = axs[1, i]
            if show_band:
                ax.fill_between(t, eul[:, i] - sig_a[:, i], eul[:, i] + sig_a[:, i],
                                color=c, alpha=0.15, lw=0)
            ax.plot(t, eul[:, i], color=c, lw=1.1, label=labels[ri])
            if prior_marker:
                ax.axhline(eul[0, i], color=c, ls=":", lw=0.7, alpha=0.6)

    for i in range(3):
        axs[0, i].set_ylabel(p_labels[i])
        axs[1, i].set_ylabel(f"q_IR {_EUL_LABELS[i]} [deg]")
        axs[1, i].set_xlabel("t [s] (since first sample)")
    axs[0, 0].legend(fontsize=8)
    fig.tight_layout()
    return fig


# Single estimated 3-vector + its ±σ band, stacked x/y/z. ``kind`` picks the
# state columns from state.csv and the matching error-state covariance diagonal
# from cov_diag.csv (filter error-state order: 0-2 pos, 3-5 vel, 6-8 att,
# 9-11 accel bias, 12-14 gyro bias, 15-17 radar pos, 18-20 radar att).
_CONVERGENCE_KINDS = {
    "accel_bias": dict(
        cols=["ba_x", "ba_y", "ba_z"], cov=["P09", "P10", "P11"],
        ylabels=["b_a,x [m/s²]", "b_a,y [m/s²]", "b_a,z [m/s²]"]),
    "gyro_bias": dict(
        cols=["bg_x", "bg_y", "bg_z"], cov=["P12", "P13", "P14"],
        ylabels=["b_g,x [rad/s]", "b_g,y [rad/s]", "b_g,z [rad/s]"]),
    "radar_pos": dict(
        cols=["p_IR_x", "p_IR_y", "p_IR_z"], cov=["P15", "P16", "P17"],
        ylabels=["p_IR,x [m]", "p_IR,y [m]", "p_IR,z [m]"]),
    "radar_att": dict(
        cols=["q_IR_w", "q_IR_x", "q_IR_y", "q_IR_z"], cov=["P18", "P19", "P20"],
        ylabels=[f"q_IR {l} [deg]" for l in _EUL_LABELS], quat=True, deg=True),
}


def plot_state_convergence(run_dir, kind, label="estimate", color=BLUE,
                           prior_marker=True, figsize=(9, 7)):
    """Plot one estimated 3-vector state with its ±σ band, stacked x/y/z.

    ``kind`` ∈ {"accel_bias", "gyro_bias", "radar_pos", "radar_att"} selects
    which state (from state.csv) and covariance diagonal (from cov_diag.csv) to
    read. The line carries the ``label`` legend entry (default "estimate") and
    the shaded band carries its own "±σ" entry. ``prior_marker`` draws a dotted
    line at the initial value. Returns the matplotlib Figure.
    """
    cfg = _CONVERGENCE_KINDS[kind]
    run_dir = Path(run_dir)
    s = pd.read_csv(run_dir / "state.csv")
    c = pd.read_csv(run_dir / "cov_diag.csv")
    n = min(len(s), len(c))
    s, c = s.iloc[:n], c.iloc[:n]
    t = s["t"].to_numpy(dtype=float)
    t = t - t[0]
    vals = s[cfg["cols"]].to_numpy(dtype=float)
    if cfg.get("quat"):
        vals = _euler_deg(vals)
    sig = np.sqrt(np.clip(c[cfg["cov"]].to_numpy(dtype=float), 0, None))
    if cfg.get("deg"):
        sig = np.rad2deg(sig)

    fig, axs = plt.subplots(3, 1, figsize=figsize, sharex=True)
    for i in range(3):
        ax = axs[i]
        # Line created first so it leads the legend ("estimate", then "±σ");
        # zorder keeps the line drawn above the translucent band.
        ax.plot(t, vals[:, i], color=color, lw=1.1, label=label, zorder=3)
        ax.fill_between(t, vals[:, i] - sig[:, i], vals[:, i] + sig[:, i],
                        color=color, alpha=0.15, lw=0, label="±σ", zorder=0)
        if prior_marker:
            ax.axhline(vals[0, i], color=color, ls=":", lw=0.7, alpha=0.6)
        ax.set_ylabel(cfg["ylabels"][i])
        ax.grid(alpha=0.3)
    axs[0].legend(fontsize=8)
    axs[-1].set_xlabel("t [s] (since first sample)")
    fig.tight_layout()
    return fig
