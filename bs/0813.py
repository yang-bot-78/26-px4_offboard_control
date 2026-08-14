#!/usr/bin/env python3
"""D435i RGB real-time detector using v8-200.pt."""

import argparse
import os
import queue
import signal
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path


# Three logical CPUs are approximately 300% in top/htop.
CPU_LIMIT = max(1, min(3, int(os.environ.get("D0813_CPU_LIMIT", "3"))))


def limit_cpu_affinity(limit: int) -> tuple[int, ...]:
    if not hasattr(os, "sched_getaffinity"):
        return ()
    available = sorted(os.sched_getaffinity(0))
    selected = tuple(available[: min(limit, len(available))])
    if selected:
        try:
            os.sched_setaffinity(0, selected)
        except OSError:
            pass
    return selected


CPU_AFFINITY = limit_cpu_affinity(CPU_LIMIT)
os.environ["OMP_NUM_THREADS"] = str(CPU_LIMIT)
os.environ["MKL_NUM_THREADS"] = str(CPU_LIMIT)
os.environ["OPENBLAS_NUM_THREADS"] = "1"
os.environ["NUMEXPR_NUM_THREADS"] = str(CPU_LIMIT)
os.environ["YOLO_OFFLINE"] = "true"

import cv2
import numpy as np
import pyrealsense2 as rs
import torch
from ultralytics import YOLO


SCRIPT_DIR = Path(__file__).resolve().parent
# DEFAULT_MODEL = SCRIPT_DIR / "v8-200.pt"
DEFAULT_MODEL = SCRIPT_DIR / "v11-200.pt"



CONFIDENCE_THRESHOLD = 0.55
RGB_WIDTH = 1280
RGB_HEIGHT = 720
RGB_FPS = 30
INFERENCE_SIZE = max(320, int(os.environ.get("D0813_IMGSZ", "640")))
DETECT_INTERVAL = max(1, int(os.environ.get("D0813_DETECT_INTERVAL", "1")))

cv2.setNumThreads(1)
torch.set_num_threads(CPU_LIMIT)
torch.set_num_interop_threads(1)


@dataclass(frozen=True)
class Detection:
    label: str
    confidence: float
    box: tuple[float, float, float, float]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--model", type=Path, default=DEFAULT_MODEL,
        help="模型路径，默认使用 bs/v8-200.pt",
    )
    parser.add_argument(
        "--imgsz", type=int, default=INFERENCE_SIZE,
        help="模型推理尺寸，默认 640；数值越大通常越清晰但 CPU 负载越高",
    )
    parser.add_argument(
        "--no-display", action="store_true",
        help="无窗口运行，仅打印识别结果",
    )
    return parser.parse_args()


def load_model(model_path: Path) -> YOLO:
    if not model_path.is_file():
        raise FileNotFoundError(f"找不到模型: {model_path}")
    model = YOLO(str(model_path), task="detect")
    print(f"模型类别: {model.names}")
    return model


def get_class_name(names: dict, class_id: int) -> str:
    return str(names.get(class_id, names.get(str(class_id), class_id)))


def detect_frame(model: YOLO, frame: np.ndarray, image_size: int) -> list[Detection]:
    result = model.predict(
        source=frame,
        imgsz=max(320, image_size),
        conf=CONFIDENCE_THRESHOLD,
        iou=0.45,
        max_det=100,
        device="cpu",
        verbose=False,
    )[0]
    if result.boxes is None or len(result.boxes) == 0:
        return []

    detections = []
    for box, confidence, cls in zip(result.boxes.xyxy, result.boxes.conf, result.boxes.cls):
        score = float(confidence.item())
        if score < CONFIDENCE_THRESHOLD:
            continue
        detections.append(
            Detection(
                label=get_class_name(model.names, int(cls.item())),
                confidence=score,
                box=tuple(float(value) for value in box.tolist()),
            )
        )
    return detections


def draw_detection(frame: np.ndarray, detection: Detection) -> None:
    height, width = frame.shape[:2]
    x1, y1, x2, y2 = (int(round(value)) for value in detection.box)
    x1 = max(0, min(width - 1, x1))
    y1 = max(0, min(height - 1, y1))
    x2 = max(x1 + 1, min(width - 1, x2))
    y2 = max(y1 + 1, min(height - 1, y2))

    green = (0, 255, 0)
    cv2.rectangle(frame, (x1, y1), (x2, y2), green, 3, cv2.LINE_AA)
    text = f"{detection.label} {detection.confidence:.2f}"
    font = cv2.FONT_HERSHEY_SIMPLEX
    (text_width, text_height), baseline = cv2.getTextSize(text, font, 0.72, 2)
    label_top = max(0, y1 - text_height - baseline - 8)
    label_bottom = min(height - 1, label_top + text_height + baseline + 8)
    label_right = min(width - 1, x1 + text_width + 10)
    cv2.rectangle(frame, (x1, label_top), (label_right, label_bottom), green, -1)
    cv2.putText(
        frame, text, (x1 + 5, label_bottom - baseline - 4), font, 0.72,
        (0, 0, 0), 2, cv2.LINE_AA,
    )


def submit_latest(frame_queue: queue.Queue, frame: np.ndarray) -> None:
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


def print_detections(detections: list[Detection]) -> None:
    timestamp = time.strftime("%H:%M:%S")
    if not detections:
        print(f"[{timestamp}] 未检测到目标", flush=True)
        return
    print(f"[{timestamp}] 识别结果:", flush=True)
    for detection in detections:
        print(
            f"  内容: {detection.label}    置信度: {detection.confidence:.2f}",
            flush=True,
        )


def inference_worker(
    model: YOLO,
    frame_queue: queue.Queue,
    shared: dict,
    lock: threading.Lock,
    stop_event: threading.Event,
    image_size: int,
) -> None:
    while not stop_event.is_set():
        try:
            frame = frame_queue.get(timeout=0.1)
        except queue.Empty:
            continue
        started = time.perf_counter()
        try:
            detections = detect_frame(model, frame, image_size)
            print_detections(detections)
            with lock:
                shared["detections"] = detections
                shared["updated"] = time.monotonic()
                shared["inference_ms"] = (time.perf_counter() - started) * 1000.0
                shared["error"] = None
        except Exception as error:
            with lock:
                shared["detections"] = []
                shared["updated"] = time.monotonic()
                shared["error"] = str(error)
            print(f"推理错误: {error}", file=sys.stderr, flush=True)
        finally:
            frame_queue.task_done()


def start_rgb_stream():
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, RGB_WIDTH, RGB_HEIGHT, rs.format.bgr8, RGB_FPS)
    try:
        profile = pipeline.start(config)
        stream = (RGB_WIDTH, RGB_HEIGHT, RGB_FPS)
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
        except RuntimeError as error:
            raise RuntimeError(
                "D435i RGB 流启动失败，请检查 USB 连接、相机电源以及是否被其他程序占用"
            ) from error

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


def main() -> int:
    args = parse_args()
    if not args.no_display and not os.environ.get("DISPLAY"):
        raise RuntimeError("当前环境没有 DISPLAY，无法打开识别窗口；可使用 --no-display 仅打印结果")

    model = load_model(args.model)
    # Warm up once so the first displayed frame does not appear frozen.
    model.predict(
        source=np.zeros((RGB_HEIGHT, RGB_WIDTH, 3), dtype=np.uint8),
        imgsz=max(320, args.imgsz), conf=CONFIDENCE_THRESHOLD,
        device="cpu", verbose=False,
    )

    frame_queue: queue.Queue = queue.Queue(maxsize=1)
    shared = {"detections": [], "updated": 0.0, "inference_ms": 0.0, "error": None}
    lock = threading.Lock()
    stop_event = threading.Event()
    signal.signal(signal.SIGINT, lambda _signal, _frame: stop_event.set())

    pipeline = None
    worker = None
    try:
        pipeline, stream = start_rgb_stream()
        worker = threading.Thread(
            target=inference_worker,
            args=(model, frame_queue, shared, lock, stop_event, args.imgsz),
            name="0813-d435i-inference",
            daemon=True,
        )
        worker.start()
        affinity = ",".join(str(cpu) for cpu in CPU_AFFINITY) or "系统默认"
        print(
            f"D435i RGB: {stream[0]}x{stream[1]}@{stream[2]} | "
            f"阈值: {CONFIDENCE_THRESHOLD:.2f} | CPU: <= {CPU_LIMIT * 100}% | "
            "按 ESC/Q 退出",
            flush=True,
        )
        print(f"CPU 绑定: {affinity}", flush=True)

        frame_id = 0
        fps = 0.0
        fps_count = 0
        fps_started = time.monotonic()
        last_inference_ms = 0.0

        if not args.no_display:
            cv2.namedWindow("0813 - D435i RGB detection", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("0813 - D435i RGB detection", stream[0], stream[1])

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
                detections = list(shared["detections"])
                last_inference_ms = shared["inference_ms"]
                error = shared["error"]
            for detection in detections:
                draw_detection(frame, detection)
            cv2.putText(
                frame, f"RGB {fps:.1f} FPS | inference {last_inference_ms:.0f} ms",
                (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.72, (0, 255, 0), 2, cv2.LINE_AA,
            )
            if error:
                cv2.putText(
                    frame, "inference error", (12, 60), cv2.FONT_HERSHEY_SIMPLEX,
                    0.72, (0, 0, 255), 2, cv2.LINE_AA,
                )

            if not args.no_display:
                cv2.imshow("0813 - D435i RGB detection", frame)
                if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                    stop_event.set()
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        if worker is not None:
            worker.join(timeout=2.0)
        if pipeline is not None:
            try:
                pipeline.stop()
            except RuntimeError:
                pass
        cv2.destroyAllWindows()
        print("程序已退出", flush=True)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(130)
    except Exception as error:
        print(f"错误: {error}", file=sys.stderr)
        sys.exit(1)
