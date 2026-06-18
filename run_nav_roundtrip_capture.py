#!/usr/bin/env python3
# Copyright 2026 RBS contributors. SPDX-License-Identifier: Apache-2.0
# Author: XiaoYang <yangxiao@roboscience.ai>
# Last modified: 2026-06-18 by XiaoYang <yangxiao@roboscience.ai>

"""Run A/B navigation round trips and capture image plus pose after each arrival."""

from __future__ import annotations

import argparse
import json
import math
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path

import cv2
import rclpy

from rbs_msgs.msg import NavigateGoal, NavigateResult

from capture_nav_pose_image import CaptureNode, ImageSample, safe_name, wait_for_first_image


@dataclass(frozen=True)
class NavPoint:
  name: str
  x: float
  y: float
  yaw_degree: float


POINT_A = NavPoint("A", 0.56, 1.59, 15.5)
POINT_B = NavPoint("B", 0.97, -0.5, 12.65)


def angle_diff_deg(actual: float, target: float) -> float:
  return (actual - target + 180.0) % 360.0 - 180.0


def make_goal(point: NavPoint, goal_id: str, frame_id: str, timeout_sec: float,
              acceptable_radius_m: float) -> NavigateGoal:
  msg = NavigateGoal()
  msg.header.frame_id = frame_id
  msg.goal_id = goal_id
  msg.target_pose.header.frame_id = frame_id
  msg.target_pose.pose.position.x = point.x
  msg.target_pose.pose.position.y = point.y
  msg.target_pose.pose.position.z = 0.0
  msg.timeout_sec = float(timeout_sec)
  msg.poi_name = ""
  msg.precise = False
  msg.with_yaw = True
  msg.acceptable_radius_m = float(acceptable_radius_m)
  msg.max_retries = 0
  if hasattr(msg, "use_target_yaw"):
    msg.use_target_yaw = True
  if hasattr(msg, "target_yaw"):
    msg.target_yaw = float(point.yaw_degree)
  return msg


def save_roundtrip_capture(root: Path, point: NavPoint, round_index: int,
                           goal_id: str, result: NavigateResult,
                           pose: dict, sample: ImageSample) -> Path:
  actual_x = float(pose["translation"]["x"])
  actual_y = float(pose["translation"]["y"])
  actual_yaw = float(pose["rotation_rpy_degree"]["yaw"])
  dx = actual_x - point.x
  dy = actual_y - point.y
  yaw_error = angle_diff_deg(actual_yaw, point.yaw_degree)
  distance_error = math.hypot(dx, dy)

  folder = safe_name(
      f"round{round_index:02d}_{point.name}_x{actual_x:.2f}_y{actual_y:.2f}_yaw{actual_yaw:.2f}")
  out_dir = root / folder
  if out_dir.exists():
    out_dir = root / f"{folder}_{time.strftime('%Y%m%d_%H%M%S')}"
  out_dir.mkdir(parents=True, exist_ok=False)

  image_path = out_dir / "image.jpg"
  metadata_path = out_dir / "metadata.json"
  if not cv2.imwrite(str(image_path), sample.image_bgr):
    raise RuntimeError(f"failed to write {image_path}")

  metadata = {
      "navigation": {
          "round_index": round_index,
          "point_name": point.name,
          "goal_id": goal_id,
          "target": {
              "x": point.x,
              "y": point.y,
              "z": 0.0,
              "yaw_degree": point.yaw_degree,
          },
          "result": {
              "reached": bool(result.reached),
              "code": int(result.code),
              "message": result.message,
              "status": {
                  "code": int(result.status.code),
                  "message": result.status.message,
                  "source_node": result.status.source_node,
                  "recoverability": result.status.recoverability,
                  "details_json": result.status.details_json,
              },
          },
          "error": {
              "dx": dx,
              "dy": dy,
              "distance_m": distance_error,
              "yaw_degree": yaw_error,
          },
      },
      "image": {
          "file": image_path.name,
          "topic": sample.topic,
          "type": "sensor_msgs/msg/Image",
          "encoding": sample.encoding,
          "frame_id": sample.frame_id,
          "stamp": {"sec": sample.stamp_sec, "nanosec": sample.stamp_nanosec},
      },
      "pose": pose,
      "summary": {
          "target": {
              "x": round(point.x, 2),
              "y": round(point.y, 2),
              "yaw_degree": round(point.yaw_degree, 2),
          },
          "actual": {
              "x": round(actual_x, 2),
              "y": round(actual_y, 2),
              "yaw_degree": round(actual_yaw, 2),
          },
          "error": {
              "distance_m": round(distance_error, 4),
              "yaw_degree": round(yaw_error, 2),
          },
      },
      "created_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
  }
  metadata_path.write_text(json.dumps(metadata, ensure_ascii=False, indent=2) + "\n",
                           encoding="utf-8")
  return out_dir


class RoundTripNode(CaptureNode):
  def __init__(self, image_topic: str, source_frame: str, target_frame: str) -> None:
    super().__init__(image_topic, source_frame, target_frame)
    self._goal_pub = self.create_publisher(NavigateGoal, "/navigation/topic/goal", 10)
    self._result_sub = self.create_subscription(
        NavigateResult, "/navigation/topic/result", self._on_result, 10)
    self._result_cv = threading.Condition()
    self._results: dict[str, NavigateResult] = {}

  def _on_result(self, msg: NavigateResult) -> None:
    with self._result_cv:
      self._results[msg.goal_id] = msg
      self._result_cv.notify_all()

  def publish_goal(self, goal: NavigateGoal) -> None:
    self._goal_pub.publish(goal)

  def wait_result(self, goal_id: str, timeout_sec: float) -> NavigateResult:
    deadline = time.monotonic() + timeout_sec
    with self._result_cv:
      while rclpy.ok():
        if goal_id in self._results:
          return self._results.pop(goal_id)
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
          raise TimeoutError(f"no navigation result for {goal_id} within {timeout_sec:.1f}s")
        self._result_cv.wait(timeout=min(0.2, remaining))
    raise RuntimeError("rclpy stopped while waiting navigation result")


def parse_args() -> argparse.Namespace:
  repo_root = Path(__file__).resolve().parent
  parser = argparse.ArgumentParser(
      description="Navigate between two points and capture image/pose after each reached result.")
  parser.add_argument("--rounds", type=int, default=1,
                      help="number of A->B->A cycles to run")
  parser.add_argument("--start", choices=("A", "B"), default="A",
                      help="first point to visit")
  parser.add_argument("--image-topic", default="/cameras/head/rgb/image_raw")
  parser.add_argument("--target-frame", default="map")
  parser.add_argument("--source-frame", default="rbs_base_link")
  parser.add_argument("--nav-frame", default="odom")
  parser.add_argument("--timeout-sec", type=float, default=300.0)
  parser.add_argument("--acceptable-radius-m", type=float, default=0.03)
  parser.add_argument("--settle-sec", type=float, default=1.0,
                      help="sleep after reached result before capture")
  parser.add_argument("--image-timeout", type=float, default=5.0)
  parser.add_argument("--tf-timeout", type=float, default=3.0)
  parser.add_argument("--output-root", default=str(repo_root / "nav_roundtrip_captures"))
  return parser.parse_args()


def build_sequence(rounds: int, start: str) -> list[NavPoint]:
  first = POINT_A if start == "A" else POINT_B
  second = POINT_B if start == "A" else POINT_A
  sequence: list[NavPoint] = [first]
  for _ in range(rounds):
    sequence.extend([second, first])
  return sequence


def main() -> int:
  args = parse_args()
  if args.rounds <= 0:
    raise SystemExit("--rounds must be > 0")

  output_root = Path(args.output_root).expanduser().resolve()
  output_root.mkdir(parents=True, exist_ok=True)

  rclpy.init()
  node = RoundTripNode(args.image_topic, args.source_frame, args.target_frame)
  spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
  spin_thread.start()

  print(f"Image topic: {node.resolved_topic} [{node.resolved_type}]")
  print(f"Navigation goal topic: /navigation/topic/goal")
  print(f"Navigation result topic: /navigation/topic/result")
  print(f"Output root: {output_root}")
  print(f"Points: A=({POINT_A.x}, {POINT_A.y}, yaw={POINT_A.yaw_degree}), "
        f"B=({POINT_B.x}, {POINT_B.y}, yaw={POINT_B.yaw_degree})")

  try:
    wait_for_first_image(node, args.image_timeout)
    sequence = build_sequence(args.rounds, args.start)
    for index, point in enumerate(sequence, start=1):
      goal_id = f"roundtrip_{index:03d}_{point.name}_{int(time.time())}"
      goal = make_goal(point, goal_id, args.nav_frame, args.timeout_sec,
                       args.acceptable_radius_m)
      print(f"[{index}/{len(sequence)}] send {point.name}: "
            f"x={point.x:.2f}, y={point.y:.2f}, yaw={point.yaw_degree:.2f}, goal_id={goal_id}")
      node.publish_goal(goal)
      result = node.wait_result(goal_id, args.timeout_sec + 5.0)
      print(f"[{index}/{len(sequence)}] result: reached={result.reached} "
            f"code={result.code} message={result.message}")
      if not result.reached or result.code != NavigateResult.CODE_OK:
        print("navigation failed; stop roundtrip capture")
        return 2
      if args.settle_sec > 0.0:
        time.sleep(args.settle_sec)
      pose = node.lookup_pose(args.tf_timeout)
      sample = wait_for_first_image(node, args.image_timeout)
      out_dir = save_roundtrip_capture(output_root, point, index, goal_id, result, pose, sample)
      actual = pose["translation"]
      yaw = pose["rotation_rpy_degree"]["yaw"]
      print(f"[{index}/{len(sequence)}] saved: {out_dir}  "
            f"actual x={actual['x']:.2f}, y={actual['y']:.2f}, yaw={yaw:.2f}")
  except KeyboardInterrupt:
    pass
  finally:
    node.destroy_node()
    rclpy.shutdown()
    spin_thread.join(timeout=1.0)
  return 0


if __name__ == "__main__":
  sys.exit(main())
