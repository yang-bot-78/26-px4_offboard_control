#!/usr/bin/env python3
"""Use Intel RealSense D435i RGB video to detect plane/car/ship/house."""

import os
from pathlib import Path
import time

import cv2
import numpy as np
import pyrealsense2 as rs
import torch
from ultralytics import YOLO
from ultralytics.utils import torch_utils


BASE_DIR = Path(__file__).resolve().parent
MODEL_PATH = BASE_DIR / "best.pt"
CLASS_NAMES = {0: "plane", 1: "car", 2: "ship", 3: "house"}
WINDOW_NAME = "D435i - si.pt detection"
# Require 10 consecutive inference samples before displaying a class.
CONFIRM_FRAMES = 10
IMAGE_SIZE = 640
# This computer has Intel integrated graphics only, so CUDA is unavailable.
INFERENCE_DEVICE = "cpu"
# Limit model compute to three CPU threads on this 16-thread CPU.
CPU_THREADS = min(3, os.cpu_count() or 1)
# Leave headroom for RGB capture and window rendering while targeting 300% CPU.
TARGET_CPU_PERCENT = 280
CLASS_CONFIDENCES = {
    "plane": 0.83,
    "car": 0.83,
    "ship": 0.83,
    "house": 0.85,
}


def start_camera():
    """Start the sharpest D435i RGB profile available."""
    last_error = None
    for width, height, fps in ((1920, 1080, 30), (1280, 720, 30), (848, 480, 30)):
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        try:
            profile = pipeline.start(config)
            return pipeline, profile, (width, height, fps)
        except RuntimeError as error:
            last_error = error
    raise RuntimeError(
        "D435i RGB 启动失败，请检查 USB 3.0 连接和相机占用情况"
    ) from last_error


def main():
    if not MODEL_PATH.is_file():
        raise FileNotFoundError(f"找不到模型: {MODEL_PATH}")

    cv2.setNumThreads(1)
    torch.set_num_threads(CPU_THREADS)
    torch.set_num_interop_threads(1)
    # Ultralytics resets PyTorch's CPU threads during model.predict().
    torch_utils.NUM_THREADS = CPU_THREADS
    model = YOLO(str(MODEL_PATH), task="detect")
    names = {int(key): str(value).lower() for key, value in model.names.items()}
    missing = set(CLASS_NAMES.values()) - set(names.values())
    if missing:
        raise ValueError(f"si.pt 缺少类别: {', '.join(sorted(missing))}; 实际类别: {names}")
    class_ids = [key for key, value in names.items() if value in CLASS_NAMES.values()]
    class_confidences = {
        class_id: CLASS_CONFIDENCES[names[class_id]] for class_id in class_ids
    }
    inference_confidence = min(class_confidences.values())

    pipeline, profile, active = start_camera()
    print(f"模型: {MODEL_PATH}")
    print(f"类别: {', '.join(CLASS_NAMES.values())}")
    print(f"D435i RGB: {active[0]}x{active[1]}@{active[2]} FPS")
    print(
        f"推理设备: CPU ({CPU_THREADS} 线程), 输入尺寸: {IMAGE_SIZE}, "
        f"目标 CPU: 约 {TARGET_CPU_PERCENT}%"
    )
    print(
        "置信度阈值: "
        + ", ".join(
            f"{class_name}={CLASS_CONFIDENCES[class_name]:.2f}"
            for class_name in CLASS_NAMES.values()
        )
    )
    print(f"检测确认: 最高置信度类别连续 {CONFIRM_FRAMES} 帧后显示")
    print("窗口中按 q 或 Esc 退出")
    candidate_id = None
    candidate_frames = 0
    annotated = None
    next_inference_at = 0.0
    try:
        for _ in range(10):
            pipeline.wait_for_frames()
        while True:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue
            now = time.monotonic()
            if now >= next_inference_at:
                inference_started_at = now
                inference_cpu_started_at = time.process_time()
                frame = np.asanyarray(color_frame.get_data())
                result = model.predict(
                    source=frame,
                    imgsz=IMAGE_SIZE,
                    conf=inference_confidence,
                    classes=class_ids,
                    device=INFERENCE_DEVICE,
                    verbose=False,
                )[0]

                # Apply each class threshold before consecutive-frame counting.
                if result.boxes is not None and len(result.boxes):
                    thresholds = torch.tensor(
                        [
                            class_confidences[int(class_id)]
                            for class_id in result.boxes.cls.tolist()
                        ],
                        device=result.boxes.conf.device,
                        dtype=result.boxes.conf.dtype,
                    )
                    keep = result.boxes.conf >= thresholds
                    result.boxes.data = result.boxes.data[keep]

                # Track only the highest-confidence class, so the display can
                # never confirm or output more than one category at a time.
                detected_id = None
                if result.boxes is not None and len(result.boxes):
                    best_index = int(result.boxes.conf.argmax().item())
                    detected_id = int(result.boxes.cls[best_index].item())
                if detected_id == candidate_id:
                    candidate_frames += 1
                else:
                    candidate_id = detected_id
                    candidate_frames = int(detected_id is not None)

                confirmed_id = (
                    candidate_id if candidate_frames >= CONFIRM_FRAMES else None
                )
                if result.boxes is not None and len(result.boxes):
                    keep = torch.tensor(
                        [
                            int(class_id) == confirmed_id
                            for class_id in result.boxes.cls.tolist()
                        ],
                        device=result.boxes.data.device,
                        dtype=torch.bool,
                    )
                    result.boxes.data = result.boxes.data[keep]
                annotated = result.plot(labels=True, conf=True, line_width=2)
                if confirmed_id is None:
                    cv2.putText(
                        annotated,
                        f"Confirming detection: {CONFIRM_FRAMES} consecutive frames required",
                        (20, 35),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (0, 220, 255),
                        2,
                        cv2.LINE_AA,
                    )
                # Dynamically pace inference from actual process CPU time.
                # This keeps average use near the target on different scenes.
                inference_cpu_seconds = time.process_time() - inference_cpu_started_at
                target_cycle_seconds = inference_cpu_seconds / (
                    TARGET_CPU_PERCENT / 100
                )
                next_inference_at = max(
                    time.monotonic(),
                    inference_started_at + target_cycle_seconds,
                )

            if annotated is not None:
                cv2.imshow(WINDOW_NAME, annotated)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
