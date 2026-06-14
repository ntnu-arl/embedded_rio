#!/usr/bin/env python3
"""Overlay + summarize a baro×underweighting ablation (typically 4 configs).

Each positional argument is a rio_replay output directory (state.csv,
cov_diag.csv, ...) plus a config.json written by run_ablation.sh. Every run is
expected to come from the SAME flight, so they share one PX4 .ulg.

Ground truth = mocap when a bag is given (--mocap), otherwise EKF2 from the
.ulg. All traces are put in the same NED frame and origin-aligned to
(0, identity) at the first sample after `--skip-seconds` past the first VVO
publish — the exact convention compare_with_ulog.py uses, so these plots match
the single-run cmp/.

Mocap is time-synced to the PX4 clock (|v| cross-correlation) and Umeyama-
aligned against the BASELINE replay run — the config with both barometer and
underweighting enabled (baro+UW) — never EKF2, whose velocity is not a reliable
sync reference. When a mocap bag is given, EKF2 is not loaded, used, or plotted
at all.
    f
  python compare_ablation.py R1 R2 R3 R4 --ulg flight.ulg [--mocap m.bag]

Outputs (in --out, default <first-dir-parent>/cmp/):
  trajectory_xy.svg  altitude.svg  speed.svg  sigmas.svg  extrinsics.svg
  rmse_bars.svg  ablation.png  ablation_metrics.json
"""
import argparse
import json
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
from pyulog import ULog
from evo.core import sync, metrics
from evo.core.units import Unit
from evo.core.lie_algebra import se3

import plot_style
from plot_style import (ABLATION_COLORS, GREY, GT_LS, GT_LW,
                        plot_extrinsic_convergence)
from compare_with_ulog import (
    load_replay_state, transform_replay_to_ned,
    compute_time_offset_via_first_publish, align_origin_pose,
    make_pose_trajectory, load_visual_odometry, load_estimator_local_position,
    load_mocap_bag, auto_align_mocap_via_velocity)
import plot_runs

_REF_Q = np.array([1.0, 0.0, 0.0, 0.0])


def read_config(d: Path) -> dict:
    cfg = d / "config.json"
    if cfg.is_file():
        try:
            return json.loads(cfg.read_text())
        except (ValueError, OSError):
            pass
    return {}


def config_label(cfg: dict, default: str = "") -> str:
    """Canonical ablation legend name derived from the baro/underweight flags:
    (U)nderweighted (B)arometric Radar-Inertial Odometry. So baro+UW → UBRIO,
    baro only → BRIO, UW only → URIO, neither → RIO. Falls back to the
    config.json free-text label (then `default`) when the flags are absent."""
    baro, uw = cfg.get("baro"), cfg.get("underweight")
    if baro is None or uw is None:
        return cfg.get("label", default)
    return ("U" if uw else "") + ("B" if baro else "") + "RIO"


def load_vvo_anchor(ulg: Path, skip_s: float):
    """Open the .ulg and read the VVO stream that anchors replay time.

    Returns (ulog, ts_vvo, t_cut_us). EKF2 is intentionally NOT read here, so a
    mocap-GT run never loads it. (VVO = vehicle_visual_odometry, the firmware's
    own stream — distinct from EKF2 — and is needed to map each replay's clock
    onto PX4 time regardless of which trace is the ground truth.)"""
    ulog = ULog(str(ulg))
    ts_vvo, *_ = load_visual_odometry(ulog)
    return ulog, ts_vvo, float(ts_vvo[0]) + skip_s * 1e6


def load_ekf_gt(ulog, t_cut_us):
    """EKF2 ground truth (only used when no mocap bag): trimmed + origin-aligned.
    Returns (t0_s, ekf) where ekf is a plot-ready GT dict."""
    ts_ekf, pos_ekf, q_ekf, vel_ekf, _ = load_estimator_local_position(ulog)
    keep = ts_ekf >= t_cut_us
    ts_ekf, pos_ekf, q_ekf, vel_ekf = (ts_ekf[keep], pos_ekf[keep],
                                       q_ekf[keep], vel_ekf[keep])
    if len(ts_ekf) == 0:
        raise SystemExit("--skip-seconds left EKF2 empty; pick a smaller value")
    pos_rel, q_rel = align_origin_pose(pos_ekf, q_ekf, 0, target_q_wxyz=_REF_Q)
    t0_s = float(ts_ekf[0]) / 1e6
    ekf = dict(label="EKF2 (GT)", t_rel=ts_ekf / 1e6 - t0_s, pos=pos_rel,
               spd=np.linalg.norm(vel_ekf, axis=1),
               traj=make_pose_trajectory(ts_ekf, pos_rel, q_rel))
    return t0_s, ekf


def load_one_trace(d: Path, ts_vvo, t_cut_us, t0_s, color, apply_mount=True):
    """Load + align one replay run into the common frame (no metrics yet).
    Keeps the PX4-clock timestamps + NED velocity so the baseline run can be
    used as the mocap sync / Umeyama reference."""
    off = compute_time_offset_via_first_publish(d, ts_vvo)
    t_s, pos_W, vel_W, q_W = load_replay_state(d)
    pos_N, vel_N, q_N = transform_replay_to_ned(pos_W, vel_W, q_W,
                                                apply_mount=apply_mount)
    ts = t_s * 1e6 + off
    keep = ts >= t_cut_us
    ts, pos_N, vel_N, q_N = ts[keep], pos_N[keep], vel_N[keep], q_N[keep]
    if len(ts) == 0:
        raise SystemExit(f"{d}: empty after --skip-seconds trim")
    pos_rel, q_rel = align_origin_pose(pos_N, q_N, 0, target_q_wxyz=_REF_Q)
    cfg = read_config(d)
    return dict(dir=d, label=config_label(cfg, d.name), color=color, config=cfg,
                t=ts / 1e6 - t0_s, pos=pos_rel, spd=np.linalg.norm(vel_N, axis=1),
                ts_us=ts, vel=vel_N, cov=pd.read_csv(d / "cov_diag.csv"),
                traj=make_pose_trajectory(ts, pos_rel, q_rel))


def pick_baseline(runs):
    """The baro+UW run is the mocap reference. Identify it from config.json
    (baro and underweight both true); fall back to the first run."""
    for r in runs:
        c = r["config"]
        if c.get("baro") is True and c.get("underweight") is True:
            return r
    print("  WARNING: no baro+UW config.json found — using first run as "
          "mocap reference")
    return runs[0]


def build_mocap_gt(args, ref, t_cut_us, t0_s, save_dir):
    """Mocap ground truth, synced + Umeyama-aligned against the BASELINE replay
    run `ref` (baro+UW) — not EKF2. Returns a gt dict matching load_ekf()'s
    plot-ready shape (label/t_rel/pos/spd/traj)."""
    ts_s, pos, q, vel = load_mocap_bag(args.mocap, args.mocap_topic)
    print(f"\nmocap bag (GT)            : {args.mocap}  ({len(ts_s)} msgs)")
    print(f"  sync/Umeyama reference   : {ref['label']} (replay baseline)")
    if args.mocap_body_frame == "bld":
        qz = Rotation.from_quat([0.0, 0.0, 1.0, 0.0])
        q_corr = (Rotation.from_quat(q[:, [1, 2, 3, 0]]) * qz).as_quat()
        q = q_corr[:, [3, 0, 1, 2]]
        vel = vel * np.array([-1.0, -1.0, 1.0])
        print("  applied BLD→FRD body-frame correction")

    if args.mocap_offset is not None:
        off_s = float(args.mocap_offset)
        print(f"  using manual --mocap-offset = {off_s:+.3f} s (vs {ref['label']})")
    else:
        off_s = auto_align_mocap_via_velocity(
            ts_s, vel, ref["ts_us"], ref["vel"], ref["label"], save_dir)
    ts_us = (ts_s - ts_s[0]) * 1e6 + float(ref["ts_us"][0]) + off_s * 1e6

    keep = (ts_us >= t_cut_us) & (ts_us <= float(ref["ts_us"][-1]))
    ts_us, pos, q, vel = ts_us[keep], pos[keep], q[keep], vel[keep]
    if len(ts_us) < 3:
        raise SystemExit("mocap empty after sync/trim — check --mocap-offset")

    # Umeyama (rotation only) onto the baseline replay's origin-aligned
    # positions, then origin-align so mocap starts at (0, identity) like every
    # other trace.
    identity_q = np.tile(_REF_Q, (len(ref["ts_us"]), 1))
    traj_ref = make_pose_trajectory(ref["ts_us"], ref["pos"], identity_q)
    traj_moc = make_pose_trajectory(ts_us, pos, q)
    sa, sb = sync.associate_trajectories(traj_ref, traj_moc, max_diff=args.tol)
    if sb.num_poses < 3:
        r_a = np.eye(3)
        print("  mocap Umeyama skipped (too few paired poses)")
    else:
        r_a, _, _ = sb.align(sa, correct_scale=False)
        print(f"  mocap aligned to {ref['label']} via Umeyama "
              f"({sb.num_poses} paired)")
    traj_moc = make_pose_trajectory(ts_us, pos, q)
    traj_moc.transform(se3(r_a, np.zeros(3)))
    pos_rel, q_rel = align_origin_pose(
        traj_moc.positions_xyz, traj_moc.orientations_quat_wxyz, 0,
        target_q_wxyz=_REF_Q)
    return dict(label="mocap (GT)", t_rel=ts_us / 1e6 - t0_s, pos=pos_rel,
                spd=np.linalg.norm(vel, axis=1),
                traj=make_pose_trajectory(ts_us, pos_rel, q_rel))


def add_metrics(run, gt_traj, tol):
    """Compute APE/RPE RMSE of one run vs the chosen GT trajectory."""
    a_gt, a_est = sync.associate_trajectories(gt_traj, run["traj"], max_diff=tol)
    ape = metrics.APE(metrics.PoseRelation.translation_part)
    ape.process_data((a_gt, a_est))
    run["ape"] = float(ape.get_statistic(metrics.StatisticsType.rmse))
    rpe = metrics.RPE(metrics.PoseRelation.translation_part, delta=10.0,
                      delta_unit=Unit.meters, all_pairs=True)
    try:
        rpe.process_data((a_gt, a_est))
        run["rpe"] = float(rpe.get_statistic(metrics.StatisticsType.rmse))
    except Exception:
        run["rpe"] = float("nan")
    run["n_pairs"] = a_est.num_poses


# ── panels (each draws on a given Axes; reused for SVGs and the overview) ────

def _trace_kw(run):
    """No transparency, equal width. Underweighting-ON configs (baro+UW,
    noBaro,UW) are solid; their no-UW twins (baro,noUW, none) are dashed so that
    — since underweighting is often a near-no-op and the twins overlap — the
    solid trace shows through the dash gaps instead of two solid lines fighting
    for the same pixels. GT stays grey dotted, so dashed keeps all three line
    styles distinct."""
    uw = run.get("config", {}).get("underweight") is True
    return dict(color=run["color"], lw=1.4, ls="-" if uw else (0, (6, 3)))


def _fit_xy_to(ax, pos, pad=0.30):
    """Lock the XY view to `pos`'s extent (+pad), square, so a diverging
    estimate can't blow up the autoscale and hide the ground truth. Returns
    (xlim, ylim)."""
    x, y = pos[:, 0], pos[:, 1]
    cx, cy = 0.5 * (x.min() + x.max()), 0.5 * (y.min() + y.max())
    half = 0.5 * max(x.max() - x.min(), y.max() - y.min())
    half = half * (1.0 + pad) if half > 0 else 1.0
    xlim, ylim = (cx - half, cx + half), (cy - half, cy + half)
    ax.set_xlim(*xlim); ax.set_ylim(*ylim)
    return xlim, ylim


def panel_trajectory(ax, gt, gt_kw, runs):
    ax.plot(gt["pos"][:, 0], gt["pos"][:, 1], **gt_kw)
    for r in runs:
        ax.plot(r["pos"][:, 0], r["pos"][:, 1],
                **{**_trace_kw(r), "label": r["label"]})
    ax.set_xlabel("p_x [m]"); ax.set_ylabel("p_y [m]")
    # Fit the view to the GT (+margin); a diverging config would otherwise
    # dominate the autoscale and squash the GT to an unreadable dot.
    ax.set_aspect("equal", adjustable="box")
    xlim, ylim = _fit_xy_to(ax, gt["pos"])
    clipped = [r["label"] for r in runs
               if r["pos"][:, 0].min() < xlim[0] or r["pos"][:, 0].max() > xlim[1]
               or r["pos"][:, 1].min() < ylim[0] or r["pos"][:, 1].max() > ylim[1]]
    if clipped:
        ax.text(0.02, 0.02, "off-view: " + ", ".join(clipped),
                transform=ax.transAxes, fontsize=7, va="bottom", ha="left",
                color="0.4")
    ax.grid(alpha=0.3); ax.legend(fontsize=8)


def panel_altitude(ax, gt, gt_kw, runs):
    ax.plot(gt["t_rel"], gt["pos"][:, 2], **gt_kw)
    for r in runs:
        ax.plot(r["t"], r["pos"][:, 2],
                **{**_trace_kw(r), "label": r["label"]})
    ax.set_xlabel("t [s]"); ax.set_ylabel("p_z [m]")
    ax.grid(alpha=0.3); ax.legend(fontsize=8)


def panel_speed(ax, gt, gt_kw, runs):
    ax.plot(gt["t_rel"], gt["spd"], **gt_kw)
    for r in runs:
        ax.plot(r["t"], r["spd"],
                **{**_trace_kw(r), "label": r["label"]})
    ax.set_xlabel("t [s]"); ax.set_ylabel("|v| [m/s]")
    ax.grid(alpha=0.3); ax.legend(fontsize=8)


def panel_bars(ax, runs, key, title):
    labels = [r["label"] for r in runs]
    vals = [r[key] for r in runs]
    colors = [r["color"] for r in runs]
    bars = ax.bar(labels, vals, color=colors)
    for b, v in zip(bars, vals):
        ax.annotate(f"{v:.3f}", (b.get_x() + b.get_width() / 2, v),
                    ha="center", va="bottom", fontsize=8)
    ax.set_ylabel("RMSE [m]")
    ax.grid(alpha=0.3, axis="y")
    ax.tick_params(axis="x", labelrotation=20)


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("dirs", nargs="+", type=Path,
                    help="rio_replay run dirs (one per ablation config)")
    ap.add_argument("--ulg", type=Path, required=True,
                    help="PX4 .ulg shared by all runs (VVO time anchor always; "
                         "EKF2 used as GT only when --mocap is absent)")
    ap.add_argument("--out", type=Path, default=None,
                    help="output dir (default: <first-dir-parent>/cmp)")
    ap.add_argument("--skip-seconds", type=float, default=0.0, dest="skip",
                    help="drop this many seconds after first VVO before align")
    ap.add_argument("--tol", type=float, default=0.2,
                    help="evo association tolerance [s] (default 0.2)")
    ap.add_argument("--mocap", type=Path, default=None,
                    help="ROS1 mocap bag → use as ground truth instead of EKF2 "
                         "(synced/aligned against the baro+UW replay)")
    ap.add_argument("--mocap-topic", default="/qualisys/Body_1/odom",
                    help="nav_msgs/Odometry topic in the mocap bag")
    ap.add_argument("--mocap-offset", type=float, default=None,
                    help="manual mocap→PX4 offset [s] (default: |v| auto-sync)")
    ap.add_argument("--mocap-body-frame", choices=("frd", "bld"), default="frd",
                    help="mocap rigid-body frame convention (default frd)")
    ap.add_argument("--no-mount-correction", dest="mount_correction",
                    action="store_false",
                    help="skip the my-IMU -> Pixhawk-IMU mount tilt (_R_M, "
                         "firmware q_m) on the replay attitude")
    args = ap.parse_args()

    plot_style.setup_mpl()
    for d in args.dirs:
        if not d.is_dir():
            raise SystemExit(f"not a directory: {d}")
    if not args.ulg.is_file():
        raise SystemExit(f"missing .ulg: {args.ulg}")
    if args.mocap is not None and not args.mocap.is_file():
        raise SystemExit(f"missing --mocap bag: {args.mocap}")

    out = args.out or (args.dirs[0].resolve().parent / "cmp")
    out.mkdir(parents=True, exist_ok=True)

    ulog, ts_vvo, t_cut_us = load_vvo_anchor(args.ulg, args.skip)
    # When mocap is the GT, EKF2 is never loaded, used, or plotted — the time
    # origin comes from the alignment cutoff instead of EKF2's first sample.
    t0_s = (t_cut_us / 1e6) if args.mocap else None
    ekf = None
    if not args.mocap:
        t0_s, ekf = load_ekf_gt(ulog, t_cut_us)

    runs = [load_one_trace(d, ts_vvo, t_cut_us, t0_s,
                           ABLATION_COLORS[i % len(ABLATION_COLORS)],
                           apply_mount=args.mount_correction)
            for i, d in enumerate(args.dirs)]

    # GT: mocap (synced to the baro+UW replay) when a bag is given, else EKF2.
    if args.mocap:
        gt = build_mocap_gt(args, pick_baseline(runs), t_cut_us, t0_s, out)
    else:
        gt = ekf
    gt_kw = dict(color=GREY, ls=GT_LS, lw=GT_LW, label=gt["label"], zorder=1)
    gt_tag = gt["label"].replace(" (GT)", "")

    for r in runs:
        add_metrics(r, gt["traj"], args.tol)

    print(f"\nablation summary (GT = {gt_tag}):")
    print(f"  {'config':>16}  {'APE RMSE':>9}  {'RPE RMSE':>9}  {'pairs':>6}")
    for r in runs:
        print(f"  {r['label']:>16}  {r['ape']:>9.3f}  {r['rpe']:>9.3f}  "
              f"{r['n_pairs']:>6d}")

    # Individual SVGs ────────────────────────────────────────────────────────
    def _save(draw, name, figsize):
        fig, ax = plt.subplots(figsize=figsize)
        draw(ax)
        fig.tight_layout(); fig.savefig(out / name); plt.close(fig)
        print(f"  saved → {out / name}")

    _save(lambda ax: panel_trajectory(ax, gt, gt_kw, runs), "trajectory_xy.svg", (8, 8))
    _save(lambda ax: panel_altitude(ax, gt, gt_kw, runs),   "altitude.svg",     (12, 5))
    _save(lambda ax: panel_speed(ax, gt, gt_kw, runs),      "speed.svg",        (12, 5))
    _save(lambda ax: plot_runs.plot_sigmas(
        ax, [{"name": r["label"], "cov": r["cov"]} for r in runs],
        [r["color"] for r in runs]), "sigmas.svg", (12, 6))

    # RMSE bars (APE + RPE side by side).
    fig, axb = plt.subplots(1, 2, figsize=(12, 5))
    panel_bars(axb[0], runs, "ape", f"APE RMSE (vs {gt_tag})")
    panel_bars(axb[1], runs, "rpe", f"RPE RMSE  δ=10 m (vs {gt_tag})")
    fig.tight_layout(); fig.savefig(out / "rmse_bars.svg"); plt.close(fig)
    print(f"  saved → {out / 'rmse_bars.svg'}")

    # Extrinsic convergence overlay (frame-independent; reads run dirs).
    fig_ext = plot_extrinsic_convergence(
        [r["dir"] for r in runs], labels=[r["label"] for r in runs],
        colors=[r["color"] for r in runs], bands="first")
    fig_ext.savefig(out / "extrinsics.svg"); plt.close(fig_ext)
    print(f"  saved → {out / 'extrinsics.svg'}")

    # Combined overview PNG.
    fig, axes = plt.subplots(2, 3, figsize=(18, 10))
    panel_trajectory(axes[0, 0], gt, gt_kw, runs)
    panel_altitude(axes[0, 1], gt, gt_kw, runs)
    panel_speed(axes[0, 2], gt, gt_kw, runs)
    plot_runs.plot_sigmas(axes[1, 0],
                          [{"name": r["label"], "cov": r["cov"]} for r in runs],
                          [r["color"] for r in runs])
    panel_bars(axes[1, 1], runs, "ape", f"APE RMSE (vs {gt_tag})")
    panel_bars(axes[1, 2], runs, "rpe", f"RPE RMSE δ=10 m (vs {gt_tag})")
    fig.tight_layout(); fig.savefig(out / "ablation.png", dpi=120); plt.close(fig)
    print(f"  saved → {out / 'ablation.png'}")

    # Metrics table for the thesis.
    metrics_json = {"gt": gt_tag,
                    "configs": {r["label"]: {"dir": str(r["dir"]),
                                             "ape_rmse_m": r["ape"],
                                             "rpe_rmse_m": r["rpe"],
                                             "n_pairs": r["n_pairs"]}
                                for r in runs}}
    (out / "ablation_metrics.json").write_text(json.dumps(metrics_json, indent=2))
    print(f"  saved → {out / 'ablation_metrics.json'}")
    print(f"\nDone — ablation outputs in {out}/")


if __name__ == "__main__":
    main()
