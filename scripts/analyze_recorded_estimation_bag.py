#!/usr/bin/env python3
"""Benchmark a recorded VINS results bag against EuRoC ground truth."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from evaluate_slam_trajectory import (
    ORIENTED_TYPES,
    SUPPORTED_GT_TYPES,
    apply_alignment,
    compute_metrics,
    interpolate_groundtruth,
    list_bag_connections,
    load_groundtruth_from_bag,
    restrict_to_overlap,
    save_plots,
    umeyama_alignment,
)


KNOWN_RESULT_TOPIC_PRIORITY = [
    "/loop_fusion/odometry_rect",
    "/vins_estimator/odometry",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Benchmark a recorded VINS results bag against ground truth."
    )
    parser.add_argument(
        "--results-bag",
        required=True,
        type=Path,
        help="Bag containing recorded VINS result topics.",
    )
    parser.add_argument(
        "--gt-bag",
        required=True,
        type=Path,
        help="Original EuRoC ros2 bag with ground truth.",
    )
    parser.add_argument(
        "--traj-topic",
        default="",
        help="Result topic to evaluate. Auto-detected if omitted.",
    )
    parser.add_argument(
        "--gt-topic",
        default="",
        help="Ground-truth topic. Auto-detected if omitted.",
    )
    parser.add_argument(
        "--out-dir",
        required=True,
        type=Path,
        help="Directory for metrics and plots.",
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
        help="Use similarity alignment instead of rigid SE3 alignment.",
    )
    return parser.parse_args()


def pick_result_topic(results_bag: Path, requested_topic: str) -> str:
    connections = list_bag_connections(results_bag)

    if requested_topic:
        for conn in connections:
            if conn.topic == requested_topic:
                if conn.msgtype not in SUPPORTED_GT_TYPES:
                    raise ValueError(
                        f"Topic '{requested_topic}' has unsupported type '{conn.msgtype}'."
                    )
                return requested_topic
        raise ValueError(f"Result topic '{requested_topic}' not found in {results_bag}.")

    for topic in KNOWN_RESULT_TOPIC_PRIORITY:
        for conn in connections:
            if conn.topic == topic and conn.msgtype in ORIENTED_TYPES:
                return topic

    for conn in connections:
        if conn.msgtype in ORIENTED_TYPES:
            return conn.topic

    raise ValueError(f"No supported result trajectory topic found in {results_bag}.")


def main() -> None:
    args = parse_args()
    out_dir = args.out_dir
    out_dir.mkdir(parents=True, exist_ok=True)

    traj_topic = pick_result_topic(args.results_bag, args.traj_topic)
    est_raw = load_groundtruth_from_bag(args.results_bag, traj_topic)
    gt_raw = load_groundtruth_from_bag(args.gt_bag, args.gt_topic)
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
        "results_bag": str(args.results_bag),
        "result_topic": traj_topic,
        "gt_bag": str(args.gt_bag),
        "groundtruth_topic": gt_raw.topic,
        "out_dir": str(out_dir),
    }

    with (out_dir / "metrics.json").open("w", encoding="utf-8") as f:
        json.dump(metrics, f, indent=2)

    save_plots(est_aligned, gt_sync, out_dir)

    print(f"Result topic: {traj_topic}")
    print(f"Ground-truth topic: {gt_raw.topic}")
    print(
        "APE RMSE [m]: "
        f"{metrics['ape_translation_m']['rmse']:.4f}, "
        f"RPE RMSE [m]: {metrics['rpe']['trans_m']['rmse']:.4f}"
    )


if __name__ == "__main__":
    main()
