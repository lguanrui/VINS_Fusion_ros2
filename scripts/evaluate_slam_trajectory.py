#!/usr/bin/env python3
"""Benchmark VINS trajectory output against bag ground truth and save plots.

Example:
  /usr/bin/python3 scripts/evaluate_slam_trajectory.py \
    --vins-csv /home/guanrui/ws/vins_fusion_ws/test_results/MH_01_easy_vio_short.csv \
    --bag /home/guanrui/ws/vins_fusion_ws/dataset_ros2/machine_hall/MH_01_easy/MH_01_easy \
    --out-dir /home/guanrui/ws/vins_fusion_ws/test_results/eval/MH_01_easy
"""

from __future__ import annotations

import argparse
import json
import math
import site
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Sequence, Tuple


def _deprioritize_user_site() -> None:
    """Keep user site-packages available, but prefer system packages first.

    This avoids importing a user-installed NumPy that can be ABI-incompatible
    with the system matplotlib build.
    """
    try:
        user_sites = site.getusersitepackages()
    except Exception:
        return

    if isinstance(user_sites, str):
        user_sites = [user_sites]

    for user_site in user_sites:
        if not user_site:
            continue
        count = sum(1 for p in sys.path if p == user_site)
        if count == 0:
            continue
        sys.path[:] = [p for p in sys.path if p != user_site]
        sys.path.append(user_site)


_deprioritize_user_site()

import matplotlib
import numpy as np
from scipy.spatial.transform import Rotation, Slerp

try:
    from rosbags.highlevel import AnyReader
except ImportError:
    AnyReader = None

try:
    from rclpy.serialization import deserialize_message
    from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
    from rosidl_runtime_py.utilities import get_message
except ImportError:
    ConverterOptions = None
    SequentialReader = None
    StorageOptions = None
    deserialize_message = None
    get_message = None

matplotlib.use("Agg")
import matplotlib.pyplot as plt


@dataclass
class Trajectory:
    t: np.ndarray  # seconds, shape (N,)
    p: np.ndarray  # position xyz, shape (N, 3)
    q_wxyz: Optional[np.ndarray] = None  # optional quaternion, shape (N, 4)
    topic: str = ""


@dataclass
class BagConnection:
    topic: str
    msgtype: str


SUPPORTED_GT_TYPES = {
    "geometry_msgs/msg/PointStamped",
    "geometry_msgs/msg/TransformStamped",
    "geometry_msgs/msg/PoseStamped",
    "nav_msgs/msg/Odometry",
    "geometry_msgs/PointStamped",
    "geometry_msgs/TransformStamped",
    "geometry_msgs/PoseStamped",
    "nav_msgs/Odometry",
}

ORIENTED_TYPES = {
    "geometry_msgs/msg/TransformStamped",
    "geometry_msgs/msg/PoseStamped",
    "nav_msgs/msg/Odometry",
    "geometry_msgs/TransformStamped",
    "geometry_msgs/PoseStamped",
    "nav_msgs/Odometry",
}

KNOWN_GT_TOPIC_PRIORITY = [
    "/vicon/firefly_sbx/firefly_sbx",
    "/leica/position",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Benchmark VINS output against ground truth from rosbag."
    )
    parser.add_argument("--vins-csv", required=True, type=Path, help="Path to VINS csv output.")
    parser.add_argument(
        "--bag",
        required=True,
        type=Path,
        help="Path to rosbag (ROS2 bag directory or ROS1 .bag file).",
    )
    parser.add_argument(
        "--out-dir",
        required=True,
        type=Path,
        help="Directory for metrics and plots.",
    )
    parser.add_argument(
        "--gt-topic",
        default="",
        help="Ground-truth topic. If omitted, auto-select supported topic.",
    )
    parser.add_argument(
        "--rpe-delta",
        type=float,
        default=1.0,
        help="Time interval in seconds for RPE computation.",
    )
    parser.add_argument(
        "--sim3",
        action="store_true",
        help="Use similarity alignment (scale + rotation + translation). Default is SE3.",
    )
    return parser.parse_args()


def _stamp_to_sec(stamp_obj: object) -> Optional[float]:
    for sec_key, nsec_key in (("sec", "nanosec"), ("sec", "nsec"), ("secs", "nsecs")):
        if hasattr(stamp_obj, sec_key) and hasattr(stamp_obj, nsec_key):
            sec = float(getattr(stamp_obj, sec_key))
            nsec = float(getattr(stamp_obj, nsec_key))
            return sec + nsec * 1e-9
    return None


def _header_stamp_to_sec(msg: object) -> Optional[float]:
    if not hasattr(msg, "header"):
        return None
    header = getattr(msg, "header")
    if not hasattr(header, "stamp"):
        return None
    return _stamp_to_sec(getattr(header, "stamp"))


def _extract_pose_and_quat(msg: object, msgtype: str) -> Tuple[np.ndarray, Optional[np.ndarray]]:
    if "PointStamped" in msgtype:
        p = np.array([msg.point.x, msg.point.y, msg.point.z], dtype=float)
        return p, None

    if "TransformStamped" in msgtype:
        tr = msg.transform.translation
        rot = msg.transform.rotation
        p = np.array([tr.x, tr.y, tr.z], dtype=float)
        q = np.array([rot.w, rot.x, rot.y, rot.z], dtype=float)
        return p, q

    if "PoseStamped" in msgtype:
        pose = msg.pose
        p = np.array([pose.position.x, pose.position.y, pose.position.z], dtype=float)
        q = np.array(
            [pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z],
            dtype=float,
        )
        return p, q

    if "Odometry" in msgtype:
        pose = msg.pose.pose
        p = np.array([pose.position.x, pose.position.y, pose.position.z], dtype=float)
        q = np.array(
            [pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z],
            dtype=float,
        )
        return p, q

    raise ValueError(f"Unsupported message type for GT extraction: {msgtype}")


def _pick_gt_connection(connections: Sequence[object], gt_topic: str) -> object:
    if gt_topic:
        for conn in connections:
            if conn.topic == gt_topic:
                if conn.msgtype not in SUPPORTED_GT_TYPES:
                    raise ValueError(
                        f"Topic '{gt_topic}' has unsupported type '{conn.msgtype}'. "
                        f"Supported: {sorted(SUPPORTED_GT_TYPES)}"
                    )
                return conn
        raise ValueError(f"Ground-truth topic '{gt_topic}' not found in bag.")

    # 1) Prefer known dataset GT topics.
    for topic in KNOWN_GT_TOPIC_PRIORITY:
        for conn in connections:
            if conn.topic == topic and conn.msgtype in SUPPORTED_GT_TYPES:
                return conn

    # 2) Fall back to first supported type, with orientation-bearing topics first.
    oriented = [c for c in connections if c.msgtype in ORIENTED_TYPES]
    if oriented:
        return oriented[0]
    candidates = [c for c in connections if c.msgtype in SUPPORTED_GT_TYPES]
    if candidates:
        return candidates[0]

    raise ValueError("No supported ground-truth topic found in bag.")


def list_bag_connections(bag_path: Path) -> Sequence[BagConnection]:
    if not bag_path.exists():
        raise FileNotFoundError(f"Bag path does not exist: {bag_path}")

    if bag_path.is_dir() and SequentialReader is not None:
        reader = SequentialReader()
        reader.open(
            StorageOptions(uri=str(bag_path), storage_id="sqlite3"),
            ConverterOptions(
                input_serialization_format="cdr",
                output_serialization_format="cdr",
            ),
        )
        return [
            BagConnection(topic=topic.name, msgtype=topic.type)
            for topic in reader.get_all_topics_and_types()
        ]

    if AnyReader is not None:
        with AnyReader([bag_path]) as reader:
            return [BagConnection(topic=conn.topic, msgtype=conn.msgtype) for conn in reader.connections]

    raise ImportError(
        "No supported rosbag reader is available. Install 'rosbags' or source a ROS 2 "
        "environment with rosbag2_py."
    )


def load_groundtruth_from_bag(bag_path: Path, gt_topic: str) -> Trajectory:
    if not bag_path.exists():
        raise FileNotFoundError(f"Bag path does not exist: {bag_path}")

    t_vals = []
    p_vals = []
    q_vals = []

    if bag_path.is_dir() and SequentialReader is not None:
        connections = list_bag_connections(bag_path)
        conn = _pick_gt_connection(connections, gt_topic)
        reader = SequentialReader()
        reader.open(
            StorageOptions(uri=str(bag_path), storage_id="sqlite3"),
            ConverterOptions(
                input_serialization_format="cdr",
                output_serialization_format="cdr",
            ),
        )
        msg_cls = get_message(conn.msgtype)

        while reader.has_next():
            topic, rawdata, timestamp_ns = reader.read_next()
            if topic != conn.topic:
                continue
            msg = deserialize_message(rawdata, msg_cls)
            ts = _header_stamp_to_sec(msg)
            if ts is None:
                ts = float(timestamp_ns) * 1e-9
            p, q = _extract_pose_and_quat(msg, conn.msgtype)
            t_vals.append(ts)
            p_vals.append(p)
            if q is not None:
                q_vals.append(q)
    elif AnyReader is not None:
        with AnyReader([bag_path]) as reader:
            conn = _pick_gt_connection(reader.connections, gt_topic)

            for _, timestamp_ns, rawdata in reader.messages(connections=[conn]):
                msg = reader.deserialize(rawdata, conn.msgtype)
                ts = _header_stamp_to_sec(msg)
                if ts is None:
                    ts = float(timestamp_ns) * 1e-9
                p, q = _extract_pose_and_quat(msg, conn.msgtype)
                t_vals.append(ts)
                p_vals.append(p)
                if q is not None:
                    q_vals.append(q)
    else:
        raise ImportError(
            "No supported rosbag reader is available. Install 'rosbags' or source a ROS 2 "
            "environment with rosbag2_py."
        )

    if not t_vals:
        raise ValueError(f"No ground-truth messages on topic '{conn.topic}'.")

    t = np.asarray(t_vals, dtype=float)
    p = np.asarray(p_vals, dtype=float)
    order = np.argsort(t)
    t = t[order]
    p = p[order]

    q_wxyz = None
    if q_vals:
        q_wxyz = np.asarray(q_vals, dtype=float)[order]
        # Normalize to protect against tiny numeric drift.
        q_wxyz = q_wxyz / np.linalg.norm(q_wxyz, axis=1, keepdims=True)

    return Trajectory(t=t, p=p, q_wxyz=q_wxyz, topic=conn.topic)


def load_vins_csv(csv_path: Path) -> Trajectory:
    if not csv_path.exists():
        raise FileNotFoundError(f"VINS csv does not exist: {csv_path}")

    rows = []
    with csv_path.open("r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = [x for x in line.split(",") if x != ""]
            if len(parts) < 8:
                continue
            rows.append([float(x) for x in parts[:8]])

    if not rows:
        raise ValueError(f"No valid rows found in VINS csv: {csv_path}")

    arr = np.asarray(rows, dtype=float)
    t = arr[:, 0]
    p = arr[:, 1:4]
    q_wxyz = arr[:, 4:8]
    q_wxyz = q_wxyz / np.linalg.norm(q_wxyz, axis=1, keepdims=True)

    # VINS currently logs timestamps with precision(0), i.e. integer seconds.
    # Reconstruct sub-second ordering when many duplicates exist.
    t = densify_second_timestamps(t)

    return Trajectory(t=t, p=p, q_wxyz=q_wxyz, topic="vins_csv")


def densify_second_timestamps(t: np.ndarray) -> np.ndarray:
    if t.size <= 1:
        return t
    unique_ratio = np.unique(t).size / t.size
    if unique_ratio > 0.9:
        return t

    out = t.astype(float).copy()
    i = 0
    n = out.size
    while i < n:
        j = i + 1
        while j < n and out[j] == out[i]:
            j += 1
        k = j - i
        if k > 1:
            out[i:j] = out[i] + np.arange(k, dtype=float) / float(k)
        i = j
    return out


def restrict_to_overlap(est: Trajectory, gt: Trajectory) -> Tuple[Trajectory, Trajectory]:
    t_min = max(float(est.t.min()), float(gt.t.min()))
    t_max = min(float(est.t.max()), float(gt.t.max()))
    if t_max <= t_min:
        raise ValueError("No time overlap between estimate and ground-truth trajectories.")

    est_mask = (est.t >= t_min) & (est.t <= t_max)
    gt_mask = (gt.t >= t_min) & (gt.t <= t_max)

    est_q = est.q_wxyz[est_mask] if est.q_wxyz is not None else None
    gt_q = gt.q_wxyz[gt_mask] if gt.q_wxyz is not None else None
    est_crop = Trajectory(est.t[est_mask], est.p[est_mask], est_q, est.topic)
    gt_crop = Trajectory(gt.t[gt_mask], gt.p[gt_mask], gt_q, gt.topic)
    return est_crop, gt_crop


def interpolate_groundtruth(gt: Trajectory, query_t: np.ndarray) -> Trajectory:
    # Ensure monotonic unique timestamps for interpolation/slerp.
    uniq_t, uniq_idx = np.unique(gt.t, return_index=True)
    gt_p = gt.p[uniq_idx]

    query_t_clamped = np.clip(query_t, uniq_t[0], uniq_t[-1])

    p_interp = np.column_stack(
        [np.interp(query_t_clamped, uniq_t, gt_p[:, k]) for k in range(3)]
    ).astype(float)

    q_interp = None
    if gt.q_wxyz is not None and gt.q_wxyz.shape[0] >= 2:
        gt_q = gt.q_wxyz[uniq_idx]
        gt_q_xyzw = gt_q[:, [1, 2, 3, 0]]
        gt_rot = Rotation.from_quat(gt_q_xyzw)
        slerp = Slerp(uniq_t, gt_rot)
        q_xyzw = slerp(query_t_clamped).as_quat()
        q_interp = q_xyzw[:, [3, 0, 1, 2]]

    return Trajectory(t=query_t, p=p_interp, q_wxyz=q_interp, topic=gt.topic)


def umeyama_alignment(src: np.ndarray, dst: np.ndarray, with_scale: bool = False) -> Tuple[float, np.ndarray, np.ndarray]:
    """Estimate transform that maps src onto dst: dst ~= s * R * src + t."""
    if src.shape != dst.shape or src.ndim != 2 or src.shape[1] != 3:
        raise ValueError("src and dst must be Nx3 arrays.")
    n = src.shape[0]
    if n < 3:
        raise ValueError("Need at least 3 points for alignment.")

    mu_src = np.mean(src, axis=0)
    mu_dst = np.mean(dst, axis=0)
    src_c = src - mu_src
    dst_c = dst - mu_dst

    cov = (dst_c.T @ src_c) / n
    u, d, vt = np.linalg.svd(cov)

    s_mat = np.eye(3)
    if np.linalg.det(u) * np.linalg.det(vt) < 0:
        s_mat[-1, -1] = -1

    r = u @ s_mat @ vt
    scale = 1.0
    if with_scale:
        var_src = np.mean(np.sum(src_c * src_c, axis=1))
        if var_src < 1e-12:
            raise ValueError("Degenerate source trajectory variance for Sim3 alignment.")
        scale = float(np.sum(d * np.diag(s_mat)) / var_src)

    t = mu_dst - scale * (r @ mu_src)
    return scale, r, t


def apply_alignment(est: Trajectory, scale: float, r: np.ndarray, t: np.ndarray) -> Trajectory:
    p_aligned = (scale * (r @ est.p.T)).T + t
    q_aligned = None
    if est.q_wxyz is not None:
        rot_align = Rotation.from_matrix(r)
        est_rot = Rotation.from_quat(est.q_wxyz[:, [1, 2, 3, 0]])
        aligned_rot = rot_align * est_rot
        q_xyzw = aligned_rot.as_quat()
        q_aligned = q_xyzw[:, [3, 0, 1, 2]]
    return Trajectory(t=est.t.copy(), p=p_aligned, q_wxyz=q_aligned, topic=est.topic)


def _basic_stats(arr: np.ndarray) -> dict:
    if arr.size == 0:
        return {
            "count": 0,
            "rmse": float("nan"),
            "mean": float("nan"),
            "median": float("nan"),
            "std": float("nan"),
            "max": float("nan"),
        }
    return {
        "count": int(arr.size),
        "rmse": float(np.sqrt(np.mean(arr**2))),
        "mean": float(np.mean(arr)),
        "median": float(np.median(arr)),
        "std": float(np.std(arr)),
        "max": float(np.max(arr)),
    }


def compute_rpe(
    t: np.ndarray,
    p_est: np.ndarray,
    p_gt: np.ndarray,
    q_est_wxyz: Optional[np.ndarray],
    q_gt_wxyz: Optional[np.ndarray],
    delta_sec: float,
) -> dict:
    if t.size < 3:
        return {"delta_sec": delta_sec, "pairs": 0}

    j_idx = np.searchsorted(t, t + delta_sec, side="left")
    i_idx = np.arange(t.size)
    mask = j_idx < t.size
    i_idx = i_idx[mask]
    j_idx = j_idx[mask]

    # Drop tiny-separation pairs (can happen with duplicate/near-duplicate times).
    dt = t[j_idx] - t[i_idx]
    sep_mask = dt >= 0.5 * delta_sec
    i_idx = i_idx[sep_mask]
    j_idx = j_idx[sep_mask]

    if i_idx.size == 0:
        return {"delta_sec": delta_sec, "pairs": 0}

    dp_est = p_est[j_idx] - p_est[i_idx]
    dp_gt = p_gt[j_idx] - p_gt[i_idx]
    trans_err = np.linalg.norm(dp_est - dp_gt, axis=1)
    gt_step = np.linalg.norm(dp_gt, axis=1)
    valid = gt_step > 1e-6
    drift_pct = np.full_like(trans_err, np.nan)
    drift_pct[valid] = 100.0 * trans_err[valid] / gt_step[valid]

    result = {
        "delta_sec": float(delta_sec),
        "pairs": int(i_idx.size),
        "trans_m": _basic_stats(trans_err),
        "trans_drift_percent": _basic_stats(drift_pct[valid]) if np.any(valid) else {"count": 0},
    }

    if q_est_wxyz is not None and q_gt_wxyz is not None:
        est_rot = Rotation.from_quat(q_est_wxyz[:, [1, 2, 3, 0]])
        gt_rot = Rotation.from_quat(q_gt_wxyz[:, [1, 2, 3, 0]])
        est_rel = est_rot[i_idx].inv() * est_rot[j_idx]
        gt_rel = gt_rot[i_idx].inv() * gt_rot[j_idx]
        rot_err = (gt_rel.inv() * est_rel).magnitude() * 180.0 / math.pi
        result["rot_deg"] = _basic_stats(rot_err)
    else:
        result["rot_deg"] = {"count": 0}

    return result


def compute_metrics(est_aligned: Trajectory, gt_sync: Trajectory, rpe_delta: float) -> dict:
    err_xyz = est_aligned.p - gt_sync.p
    trans_err = np.linalg.norm(err_xyz, axis=1)

    axis_rmse = {
        "x": float(np.sqrt(np.mean(err_xyz[:, 0] ** 2))),
        "y": float(np.sqrt(np.mean(err_xyz[:, 1] ** 2))),
        "z": float(np.sqrt(np.mean(err_xyz[:, 2] ** 2))),
    }

    metrics = {
        "samples": int(est_aligned.p.shape[0]),
        "ape_translation_m": _basic_stats(trans_err),  # often called ATE when aligned.
        "ape_axis_rmse_m": axis_rmse,
        "rpe": compute_rpe(
            t=est_aligned.t,
            p_est=est_aligned.p,
            p_gt=gt_sync.p,
            q_est_wxyz=est_aligned.q_wxyz,
            q_gt_wxyz=gt_sync.q_wxyz,
            delta_sec=rpe_delta,
        ),
    }
    return metrics


def _set_axes_equal_3d(ax: plt.Axes) -> None:
    xlim = np.array(ax.get_xlim3d())
    ylim = np.array(ax.get_ylim3d())
    zlim = np.array(ax.get_zlim3d())
    ranges = np.array([xlim[1] - xlim[0], ylim[1] - ylim[0], zlim[1] - zlim[0]])
    radius = 0.5 * np.max(ranges)
    centers = np.array([np.mean(xlim), np.mean(ylim), np.mean(zlim)])
    ax.set_xlim3d([centers[0] - radius, centers[0] + radius])
    ax.set_ylim3d([centers[1] - radius, centers[1] + radius])
    ax.set_zlim3d([centers[2] - radius, centers[2] + radius])


def save_plots(est: Trajectory, gt: Trajectory, out_dir: Path) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    t_rel = est.t - est.t[0]

    # XYZ vs time
    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    labels = ["x", "y", "z"]
    for idx, ax in enumerate(axes):
        ax.plot(t_rel, gt.p[:, idx], label="ground truth", linewidth=1.6)
        ax.plot(t_rel, est.p[:, idx], label="vins (aligned)", linewidth=1.1, alpha=0.9)
        ax.set_ylabel(f"{labels[idx]} [m]")
        ax.grid(True, alpha=0.3)
        if idx == 0:
            ax.legend(loc="upper right")
    axes[-1].set_xlabel("time [s]")
    fig.suptitle("Position Components vs Time")
    fig.tight_layout()
    fig.savefig(out_dir / "xyz_vs_time.png", dpi=200)
    plt.close(fig)

    # 3D trajectory
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")
    ax.plot(gt.p[:, 0], gt.p[:, 1], gt.p[:, 2], label="ground truth", linewidth=1.8)
    ax.plot(est.p[:, 0], est.p[:, 1], est.p[:, 2], label="vins (aligned)", linewidth=1.2)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("z [m]")
    ax.set_title("3D Trajectory")
    ax.grid(True)
    ax.legend(loc="best")
    _set_axes_equal_3d(ax)
    fig.tight_layout()
    fig.savefig(out_dir / "trajectory_3d.png", dpi=220)
    plt.close(fig)


def main() -> None:
    args = parse_args()
    out_dir: Path = args.out_dir
    out_dir.mkdir(parents=True, exist_ok=True)

    est_raw = load_vins_csv(args.vins_csv)
    gt_raw = load_groundtruth_from_bag(args.bag, args.gt_topic)
    est_crop, gt_crop = restrict_to_overlap(est_raw, gt_raw)
    gt_sync = interpolate_groundtruth(gt_crop, est_crop.t)

    scale, r, t = umeyama_alignment(est_crop.p, gt_sync.p, with_scale=args.sim3)
    est_aligned = apply_alignment(est_crop, scale, r, t)
    metrics = compute_metrics(est_aligned, gt_sync, rpe_delta=args.rpe_delta)

    metrics["alignment"] = {
        "mode": "Sim3" if args.sim3 else "SE3",
        "scale": float(scale),
        "rotation_matrix": r.tolist(),
        "translation": t.tolist(),
    }
    metrics["io"] = {
        "vins_csv": str(args.vins_csv),
        "bag": str(args.bag),
        "groundtruth_topic": gt_raw.topic,
        "out_dir": str(out_dir),
    }

    with (out_dir / "metrics.json").open("w", encoding="utf-8") as f:
        json.dump(metrics, f, indent=2)

    save_plots(est_aligned, gt_sync, out_dir)

    print(f"[OK] GT topic: {gt_raw.topic}")
    print(f"[OK] Samples: {metrics['samples']}")
    print(f"[OK] APE/ATE RMSE [m]: {metrics['ape_translation_m']['rmse']:.4f}")
    print(f"[OK] Metrics written: {out_dir / 'metrics.json'}")
    print(f"[OK] Plot written: {out_dir / 'xyz_vs_time.png'}")
    print(f"[OK] Plot written: {out_dir / 'trajectory_3d.png'}")


if __name__ == "__main__":
    main()
