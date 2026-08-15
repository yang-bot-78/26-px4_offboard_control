#!/usr/bin/env python3
"""D435i CPU-only detector using the fine-tuned 0812 four-class model."""

import os


# Bind this process to at most three logical CPUs before native libraries load.
CPU_LIMIT = max(1, min(3, int(os.environ.get("D0812_CPU_LIMIT", "3"))))


def limit_cpu_affinity(limit):
    if not hasattr(os, "sched_getaffinity"):
        return ()
    allowed = sorted(os.sched_getaffinity(0))
    selected = tuple(allowed[: min(limit, len(allowed))])
    if selected:
        os.sched_setaffinity(0, selected)
    return selected


CPU_AFFINITY = limit_cpu_affinity(CPU_LIMIT)
os.environ["OMP_NUM_THREADS"] = str(CPU_LIMIT)
os.environ["MKL_NUM_THREADS"] = str(CPU_LIMIT)
os.environ["OPENBLAS_NUM_THREADS"] = "1"
os.environ["NUMEXPR_NUM_THREADS"] = str(CPU_LIMIT)
os.environ["YOLO_OFFLINE"] = "true"

import argparse
import queue
import signal
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import pyrealsense2 as rs
import torch
from ultralytics import YOLO


SCRIPT_DIR = Path(__file__).resolve().parent


DEFAULT_MODEL_PATH = SCRIPT_DIR / "l250v8.pt"
# DEFAULT_MODEL_PATH = SCRIPT_DIR / "v8-200.pt"
# DEFAULT_MODEL_PATH = SCRIPT_DIR / "v11-200.pt"
# DEFAULT_MODEL_PATH = SCRIPT_DIR / "v8-350.pt"
# DEFAULT_MODEL_PATH = SCRIPT_DIR / "v11-350.pt"



EXPECTED_NAMES = {0: "ship", 1: "plane", 2: "car", 3: "house"}
COLOR_WIDTH = 1280
COLOR_HEIGHT = 720
COLOR_FPS = 30
INFERENCE_SIZE = max(320, int(os.environ.get("D0812_IMGSZ", "640")))
CLASS_CONFIDENCE = {
    "ship": 0.25,
    "plane": 0.25,
    "car": 0.25,
    "house": 0.25,
}
MIN_CONFIDENCE = min(CLASS_CONFIDENCE.values())
DETECT_INTERVAL = max(1, int(os.environ.get("D0812_DETECT_INTERVAL", "2")))
RESULT_TTL = max(0.2, float(os.environ.get("D0812_RESULT_TTL", "1.5")))
CONFIRMATIONS_REQUIRED = max(1, int(os.environ.get("D0812_CONFIRMATIONS", "5")))
CONFIRM_IOU = float(os.environ.get("D0812_CONFIRM_IOU", "0.30"))

cv2.setNumThreads(1)
torch.set_num_threads(CPU_LIMIT)
torch.set_num_interop_threads(1)


@dataclass(frozen=True)
class Detection:
    label: str
    confidence: float
    box: tuple


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--model", type=Path, default=DEFAULT_MODEL_PATH, help="四类 best.pt 路径"
    )
    parser.add_argument("--no-display", action="store_true", help="不显示相机窗口")
    parser.add_argument(
        "--self-test", action="store_true", help="预热模型后退出，不连接相机"
    )
    args = parser.parse_args()
    if not args.no_display and not os.environ.get("DISPLAY"):
        args.no_display = True
        print("未检测到 DISPLAY，已自动使用 --no-display 模式")
    return args


def load_model(model_path):
    if not model_path.is_file():
        raise FileNotFoundError(f"找不到训练模型: {model_path}")
    model = YOLO(model_path, task="detect")
    names = {int(key): value for key, value in model.names.items()}
    if names != EXPECTED_NAMES:
        raise ValueError(f"模型类别不符合四类定义: {names}")
    return model


def detect_one(model, frame):
    result = model.predict(
        source=frame,
        imgsz=INFERENCE_SIZE,
        conf=MIN_CONFIDENCE,
        iou=0.50,
        classes=list(EXPECTED_NAMES),
        max_det=10,
        device="cpu",
        verbose=False,
    )[0]
    candidates = []
    for cls, conf, box in zip(result.boxes.cls, result.boxes.conf, result.boxes.xyxy):
        class_id = int(cls)
        if class_id not in EXPECTED_NAMES:
            continue
        label = EXPECTED_NAMES[class_id]
        confidence = float(conf)
        if confidence < CLASS_CONFIDENCE[label]:
            continue
        candidates.append(
            Detection(
                label=label,
                confidence=confidence,
                box=tuple(float(value) for value in box.tolist()),
            )
        )
    return max(candidates, key=lambda item: item.confidence, default=None)


def start_camera():
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(
        rs.stream.color, COLOR_WIDTH, COLOR_HEIGHT, rs.format.bgr8, COLOR_FPS
    )
    try:
        profile = pipeline.start(config)
        stream = (COLOR_WIDTH, COLOR_HEIGHT, COLOR_FPS)
    except RuntimeError as first_error:
        try:
            pipeline.stop()
        except RuntimeError:
            pass
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        try:
            profile = pipeline.start(config)
            stream = (640, 480, 30)
        except RuntimeError:
            raise RuntimeError(
                "D435i 启动失败，请检查 USB 3.0 连接和相机占用情况"
            ) from first_error

    for sensor in profile.get_device().query_sensors():
        if sensor.get_info(rs.camera_info.name) != "RGB Camera":
            continue
        for option, value in (
            (rs.option.enable_auto_exposure, 1.0),
            (rs.option.enable_auto_white_balance, 1.0),
            (rs.option.frames_queue_size, 1.0),
        ):
            try:
                if sensor.supports(option):
                    sensor.set_option(option, value)
            except RuntimeError:
                pass
    return pipeline, stream


def submit_latest(frame_queue, frame):
    try:
        frame_queue.put_nowait(frame.copy())
        return
    except queue.Full:
        pass
    try:
        frame_queue.get_nowait()
        frame_queue.task_done()
    except queue.Empty:
        pass
    try:
        frame_queue.put_nowait(frame.copy())
    except queue.Full:
        pass


def box_iou(first_box, second_box):
    ax1, ay1, ax2, ay2 = first_box
    bx1, by1, bx2, by2 = second_box
    intersection_width = max(0.0, min(ax2, bx2) - max(ax1, bx1))
    intersection_height = max(0.0, min(ay2, by2) - max(ay1, by1))
    intersection = intersection_width * intersection_height
    first_area = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
    second_area = max(0.0, bx2 - bx1) * max(0.0, by2 - by1)
    union = first_area + second_area - intersection
    return intersection / union if union > 0.0 else 0.0


def inference_worker(model, frame_queue, shared, lock, stop_event):
    previous_candidate = None
    confirmations = 0
    while not stop_event.is_set():
        try:
            frame = frame_queue.get(timeout=0.1)
        except queue.Empty:
            continue
        started = time.perf_counter()
        try:
            detection = detect_one(model, frame)
            stable = (
                detection is not None
                and previous_candidate is not None
                and detection.label == previous_candidate.label
                and box_iou(detection.box, previous_candidate.box) >= CONFIRM_IOU
            )
            confirmations = confirmations + 1 if stable else int(detection is not None)
            previous_candidate = detection
            confirmed_detection = (
                detection if confirmations >= CONFIRMATIONS_REQUIRED else None
            )
            with lock:
                shared["detection"] = confirmed_detection
                shared["updated"] = time.monotonic()
                shared["inference_ms"] = (time.perf_counter() - started) * 1000.0
                shared["confirmations"] = confirmations
                shared["error"] = None
        except Exception as exc:
            previous_candidate = None
            confirmations = 0
            with lock:
                shared["detection"] = None
                shared["updated"] = time.monotonic()
                shared["confirmations"] = 0
                shared["error"] = str(exc)
        finally:
            frame_queue.task_done()


def draw_detection(frame, detection):
    x1, y1, x2, y2 = (int(value) for value in detection.box)
    height, width = frame.shape[:2]
    x1, x2 = (int(value) for value in np.clip((x1, x2), 0, width - 1))
    y1, y2 = (int(value) for value in np.clip((y1, y2), 0, height - 1))
    green = (0, 255, 0)
    cv2.rectangle(frame, (x1, y1), (x2, y2), green, 3)
    text = f"{detection.label} {detection.confidence:.2f}"
    (text_width, text_height), _ = cv2.getTextSize(
        text, cv2.FONT_HERSHEY_SIMPLEX, 0.65, 2
    )
    text_top = max(0, y1 - text_height - 10)
    cv2.rectangle(
        frame, (x1, text_top), (min(width - 1, x1 + text_width + 8), y1), green, -1
    )
    cv2.putText(
        frame, text, (x1 + 4, max(text_height + 1, y1 - 5)),
        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0), 2, cv2.LINE_AA,
    )


def main():
    args = parse_args()
    print("正在加载训练后的 0812 四类模型...")
    model = load_model(args.model)
    detect_one(model, np.zeros((COLOR_HEIGHT, COLOR_WIDTH, 3), dtype=np.uint8))
    if args.self_test:
        print(f"自检通过：{args.model} 已完成 CPU 推理")
        return 0

    frame_queue = queue.Queue(maxsize=1)
    lock = threading.Lock()
    stop_event = threading.Event()
    shared = {
        "detection": None,
        "updated": 0.0,
        "inference_ms": 0.0,
        "confirmations": 0,
        "error": None,
    }
    signal.signal(signal.SIGINT, lambda _sig, _frame: stop_event.set())
    pipeline = None
    worker = None

    try:
        pipeline, stream = start_camera()
        worker = threading.Thread(
            target=inference_worker,
            args=(model, frame_queue, shared, lock, stop_event),
            name="0812-inference",
            daemon=True,
        )
        worker.start()
        affinity = ",".join(str(cpu) for cpu in CPU_AFFINITY) or "不支持绑定"
        print(
            f"D435i: {stream[0]}x{stream[1]}@{stream[2]} FPS | "
            f"CPU: {affinity} (上限约 {CPU_LIMIT * 100}%) | ESC/Q 退出"
        )

        frame_id = 0
        fps = 0.0
        fps_count = 0
        fps_started = time.monotonic()
        last_log = 0.0
        while not stop_event.is_set():
            frames = pipeline.wait_for_frames(1000)
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue
            frame = np.asanyarray(color_frame.get_data()).copy()
            frame_id += 1
            if frame_id % DETECT_INTERVAL == 0:
                submit_latest(frame_queue, frame)

            now = time.monotonic()
            fps_count += 1
            if now - fps_started >= 1.0:
                fps = fps_count / (now - fps_started)
                fps_count = 0
                fps_started = now

            with lock:
                detection = shared["detection"]
                age = now - shared["updated"]
                inference_ms = shared["inference_ms"]
                confirmations = shared["confirmations"]
                error = shared["error"]
            if age > RESULT_TTL:
                detection = None
            if detection is not None:
                draw_detection(frame, detection)

            cv2.putText(
                frame, f"Camera {fps:.1f} FPS | CPU <= {CPU_LIMIT * 100}%",
                (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2,
                cv2.LINE_AA,
            )
            if not args.no_display:
                cv2.imshow("D435i - 0812 four-class detector", frame)
                if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                    break

            if now - last_log >= 1.0:
                last_log = now
                target = (
                    f"{detection.label} {detection.confidence:.2f}"
                    if detection else "none"
                )
                suffix = f" | error={error}" if error else ""
                print(
                    f"camera={fps:.1f}fps | inference={inference_ms:.0f}ms | "
                    f"confirm={confirmations}/{CONFIRMATIONS_REQUIRED} | "
                    f"target={target}{suffix}"
                )
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        if worker is not None:
            worker.join(timeout=2.0)
        if pipeline is not None:
            pipeline.stop()
        cv2.destroyAllWindows()
        print("程序已退出")
    return 0


if __name__ == "__main__":
    sys.exit(main())
