#!/usr/bin/env python3
"""Use Intel RealSense D435i RGB video to detect plane/car/ship/house."""

import os
from pathlib import Path
import time
from datetime import datetime
from zoneinfo import ZoneInfo

import cv2
import numpy as np
import pyrealsense2 as rs
import torch
from ultralytics import YOLO
from ultralytics.utils import torch_utils


BASE_DIR = Path(__file__).resolve().parent
MODEL_PATH = Path("/home/robot/egohx_ws/26-px4_offboard_control-main/bs/0819.pt")
CLASS_NAMES = {0: "plane", 1: "car", 2: "ship", 3: "house"}
WINDOW_NAME = "D435i - si.pt detection"
RESULT_WINDOW_NAME = "Recognition Results"
# Require 5 consecutive inference samples before confirming a class.
CONFIRM_FRAMES = 5
REPORT_INTERVAL_SEC = 10.0
IMAGE_SIZE = 640
TARGET_INFERENCE_FPS = 30.0
# Keep every inference worker below sustained single-core saturation. The
# yield is calculated from each completed inference instead of a fixed sleep,
# so slow and fast frames receive the same CPU-duty cap.
MAX_INFERENCE_CPU_DUTY_CYCLE = 0.70
CPU_IDLE_UTILIZATION_LIMIT = 0.25
CPU_OBSERVATION_SEC = 1.0
CPU_REBALANCE_SEC = 5.0
# This computer has Intel integrated graphics only, so CUDA is unavailable.
INFERENCE_DEVICE = "cpu"
# Limit model compute to three CPU threads on this 16-thread CPU.
CPU_THREADS = min(3, os.cpu_count() or 1)
CLASS_CONFIDENCES = {
    "plane": 0.83,
    "car": 0.83,
    "ship": 0.83,
    "house": 0.85,
}
DISPLAY_TIMEZONE = ZoneInfo("Asia/Shanghai")


def append_recognition_result(timestamp: str, detected_name: str) -> None:
    """Append one confirmed recognition to the state-machine supplied log."""
    configured_path = os.environ.get("RECOGNITION_RESULT_LOG_FILE")
    if not configured_path:
        return
    with Path(configured_path).expanduser().open("a", encoding="utf-8") as stream:
        stream.write(f"timestamp={timestamp} result={detected_name}\n")


def cpu_yield_seconds(work_seconds: float) -> float:
    """Return idle time needed to keep an inference cycle under the CPU cap."""
    return max(
        0.0,
        work_seconds * (1.0 / MAX_INFERENCE_CPU_DUTY_CYCLE - 1.0),
    )


def allowed_cpus_from_environment() -> tuple[int, ...]:
    """Resolve the recognition-only CPU whitelist against current affinity."""
    requested = os.environ.get("RECOGNITION_ALLOWED_CPUS")
    available = os.sched_getaffinity(0)
    if not requested:
        return tuple(sorted(available))
    try:
        cpu_ids = tuple(sorted({int(value) for value in requested.split(",")}))
    except ValueError as error:
        raise ValueError(
            "RECOGNITION_ALLOWED_CPUS must be a comma-separated CPU list"
        ) from error
    selected = tuple(cpu for cpu in cpu_ids if cpu in available)
    if not selected:
        raise RuntimeError(
            "recognition CPU whitelist has no usable CPUs"
        )
    return selected


def cpu_times() -> dict[int, tuple[int, int]]:
    """Read total and idle scheduler ticks for each logical CPU."""
    samples: dict[int, tuple[int, int]] = {}
    with Path("/proc/stat").open(encoding="utf-8") as stream:
        for line in stream:
            fields = line.split()
            if not fields or not fields[0].startswith("cpu") or fields[0] == "cpu":
                continue
            suffix = fields[0][3:]
            if not suffix.isdigit() or len(fields) < 5:
                continue
            values = [int(value) for value in fields[1:]]
            samples[int(suffix)] = (sum(values), values[3] + values[4])
    return samples


def set_process_affinity(cpu_ids: tuple[int, ...]) -> None:
    """Apply one CPU set to every current Python/PyTorch thread."""
    for task in Path("/proc/self/task").iterdir():
        try:
            os.sched_setaffinity(int(task.name), cpu_ids)
        except (FileNotFoundError, ProcessLookupError):
            continue


class NavigationPriorityCpuSelector:
    """Pick idle whitelisted CPUs without using samples polluted by inference."""

    def __init__(self, allowed_cpus: tuple[int, ...], worker_count: int) -> None:
        self.allowed_cpus = allowed_cpus
        self.worker_count = worker_count
        self.active_cpus: tuple[int, ...] = ()
        self._observation_started_at: float | None = None
        self._observation_start_ticks: dict[int, tuple[int, int]] | None = None
        self._next_observation_at = 0.0
        set_process_affinity(self.allowed_cpus)

    def inference_allowed(self, now: float) -> bool:
        if self._observation_started_at is None:
            if now < self._next_observation_at:
                return bool(self.active_cpus)
            # Stop inference while measuring so recognition work cannot make a
            # busy navigation CPU look unavailable.
            set_process_affinity(self.allowed_cpus)
            self.active_cpus = ()
            self._observation_started_at = now
            self._observation_start_ticks = cpu_times()
            return False
        if now - self._observation_started_at < CPU_OBSERVATION_SEC:
            return False

        end_ticks = cpu_times()
        loads: list[tuple[float, int]] = []
        for cpu in self.allowed_cpus:
            start = self._observation_start_ticks.get(cpu) if self._observation_start_ticks else None
            end = end_ticks.get(cpu)
            if start is None or end is None:
                continue
            total_delta = end[0] - start[0]
            if total_delta <= 0:
                continue
            loads.append((1.0 - (end[1] - start[1]) / total_delta, cpu))
        idle_cpus = tuple(cpu for load, cpu in sorted(loads) if load <= CPU_IDLE_UTILIZATION_LIMIT)
        self._observation_started_at = None
        if not idle_cpus:
            self._next_observation_at = now + CPU_OBSERVATION_SEC
            print(
                "[RECOGNITION_CPU_WAIT] "
                f"idle=0 requested={self.worker_count} "
                f"limit={CPU_IDLE_UTILIZATION_LIMIT:.0%}; inference_paused",
                flush=True,
            )
            return False

        self.active_cpus = idle_cpus[:self.worker_count]
        set_process_affinity(self.active_cpus)
        self._next_observation_at = now + CPU_REBALANCE_SEC
        print(
            "[RECOGNITION_CPU_SELECTED] "
            f"cpus={','.join(map(str, self.active_cpus))} "
            f"capacity={len(self.active_cpus)}/{self.worker_count} "
            f"idle_limit={CPU_IDLE_UTILIZATION_LIMIT:.0%}",
            flush=True,
        )
        return True


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


def show_recognition_result(timestamp: str, detected_name: str, confidence: float):
    """Show a confirmed, rate-limited result in its own window."""
    canvas = np.full((180, 760, 3), (28, 28, 28), dtype=np.uint8)
    cv2.putText(
        canvas, "RECOGNITION RESULT", (24, 45), cv2.FONT_HERSHEY_SIMPLEX,
        0.85, (80, 220, 80), 2, cv2.LINE_AA,
    )
    cv2.putText(
        canvas, f"timestamp: {timestamp}", (24, 92), cv2.FONT_HERSHEY_SIMPLEX,
        0.58, (230, 230, 230), 1, cv2.LINE_AA,
    )
    cv2.putText(
        canvas, f"type: {detected_name}    confidence: {confidence:.3f}",
        (24, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.72, (80, 220, 255), 2,
        cv2.LINE_AA,
    )
    cv2.namedWindow(RESULT_WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(RESULT_WINDOW_NAME, 760, 180)
    cv2.imshow(RESULT_WINDOW_NAME, canvas)


def main():
    startup_started_at = time.monotonic()
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
    cpu_selector = NavigationPriorityCpuSelector(
        allowed_cpus_from_environment(), CPU_THREADS)

    pipeline, profile, active = start_camera()
    print(f"模型: {MODEL_PATH}", flush=True)
    print(f"类别: {', '.join(CLASS_NAMES.values())}", flush=True)
    print(f"D435i RGB: {active[0]}x{active[1]}@{active[2]} FPS", flush=True)
    print(
        f"推理设备: CPU ({CPU_THREADS} 线程), 输入尺寸: {IMAGE_SIZE}, "
        f"目标推理帧率: {TARGET_INFERENCE_FPS:.0f} FPS, "
        f"单核持续占空比上限: {MAX_INFERENCE_CPU_DUTY_CYCLE:.0%}",
        flush=True,
    )
    print(
        "置信度阈值: "
        + ", ".join(
            f"{class_name}={CLASS_CONFIDENCES[class_name]:.2f}"
            for class_name in CLASS_NAMES.values()
        ), flush=True,
    )
    print(f"检测确认: 最高置信度类别连续 {CONFIRM_FRAMES} 帧后显示", flush=True)
    print(
        "[RECOGNITION_CPU_POLICY] "
        f"allowed={','.join(map(str, cpu_selector.allowed_cpus))} "
        f"workers={CPU_THREADS} idle_limit={CPU_IDLE_UTILIZATION_LIMIT:.0%}",
        flush=True,
    )
    print(
        "[RECOGNITION_READY] "
        f"startup_sec={time.monotonic() - startup_started_at:.3f} "
        f"target_fps={TARGET_INFERENCE_FPS:.0f}",
        flush=True,
    )
    print("窗口中按 q 或 Esc 退出", flush=True)
    candidate_id = None
    candidate_frames = 0
    last_reported_at: dict[str, float] = {}
    annotated = None
    next_inference_at = 0.0
    rate_window_started_at = time.monotonic()
    rate_window_count = 0
    try:
        for _ in range(10):
            pipeline.wait_for_frames()
        while True:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue
            now = time.monotonic()
            if cpu_selector.inference_allowed(now) and now >= next_inference_at:
                inference_started_at = now
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
                if confirmed_id is not None:
                    detected_name = names[confirmed_id]
                    if now - last_reported_at.get(
                            detected_name, float("-inf")) >= REPORT_INTERVAL_SEC:
                        confidence = float(result.boxes.conf.max().item())
                        timestamp = datetime.now(DISPLAY_TIMEZONE).isoformat(
                            timespec="milliseconds")
                        print(
                            "[RECOGNITION_RESULT] "
                            f"timestamp={timestamp} type={detected_name} "
                            f"confidence={confidence:.3f}",
                            flush=True,
                        )
                        append_recognition_result(timestamp, detected_name)
                        show_recognition_result(
                            timestamp, detected_name, confidence)
                        last_reported_at[detected_name] = now
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
                rate_window_count += 1
                rate_window_elapsed = time.monotonic() - rate_window_started_at
                if rate_window_elapsed >= 5.0:
                    print(
                        "[RECOGNITION_RATE] "
                        f"target_fps={TARGET_INFERENCE_FPS:.0f} "
                        f"actual_fps={rate_window_count / rate_window_elapsed:.2f} "
                        f"cpu_duty_cap={MAX_INFERENCE_CPU_DUTY_CYCLE:.0%}",
                        flush=True,
                    )
                    rate_window_started_at = time.monotonic()
                    rate_window_count = 0
                time.sleep(cpu_yield_seconds(time.monotonic() - inference_started_at))
                next_inference_at = max(
                    time.monotonic(),
                    inference_started_at + 1.0 / TARGET_INFERENCE_FPS,
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
