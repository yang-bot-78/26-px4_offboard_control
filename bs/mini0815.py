#!/usr/bin/env python3
"""D435i RGB detector for the ASUS NUC13ANKi5 (CPU-only, max 500% CPU)."""

import os


# Configure native libraries before importing OpenCV/PyTorch.  Affinity makes
# the process unable to consume more than five logical CPUs on Linux.
CPU_LIMIT = max(1, min(5, int(os.environ.get("MINI0815_CPU_LIMIT", "5"))))
if hasattr(os, "sched_getaffinity"):
    _allowed_cpus = sorted(os.sched_getaffinity(0))
    os.sched_setaffinity(0, _allowed_cpus[: min(CPU_LIMIT, len(_allowed_cpus))])
os.environ["OMP_NUM_THREADS"] = str(CPU_LIMIT)
os.environ["MKL_NUM_THREADS"] = str(CPU_LIMIT)
os.environ["OPENBLAS_NUM_THREADS"] = "1"
os.environ["NUMEXPR_NUM_THREADS"] = str(CPU_LIMIT)

import argparse
import queue
import signal
import threading
import time
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import pyrealsense2 as rs
import torch


SCRIPT_DIR = Path(__file__).resolve().parent
MODEL_PATH = SCRIPT_DIR / "/home/robot/rong_ws/ws_offboard_control/bs/l250v8.pt"
CLASS_NAMES = {0: "ship", 1: "plane", 2: "car", 3: "house"}
RGB_WIDTH = 1920
RGB_HEIGHT = 1080
RGB_FPS = 30
WINDOW_NAME = "D435i RGB - mini0815"

# Training used imgsz=960. The run's F1 curve peaks at confidence=0.515.
DEFAULT_IMGSZ = 960
DEFAULT_CONFIDENCE = 0.515
NMS_IOU = 0.50
RESULT_TTL_SECONDS = 1.5

cv2.setNumThreads(1)
torch.set_num_threads(CPU_LIMIT)
torch.set_num_interop_threads(1)


@dataclass(frozen=True)
class Detection:
    class_id: int
    confidence: float
    box: tuple[float, float, float, float]


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, default=MODEL_PATH)
    parser.add_argument("--imgsz", type=int, default=DEFAULT_IMGSZ)
    parser.add_argument("--conf", type=float, default=DEFAULT_CONFIDENCE)
    parser.add_argument("--no-display", action="store_true")
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.imgsz < 320 or args.imgsz % 32:
        parser.error("--imgsz 必须不小于 320 且为 32 的倍数")
    if not 0.0 <= args.conf <= 1.0:
        parser.error("--conf 必须在 0 到 1 之间")
    if not args.no_display and not os.environ.get("DISPLAY"):
        args.no_display = True
        print("未检测到 DISPLAY，自动关闭窗口显示")
    return args


def load_model(model_path):
    if not model_path.is_file():
        raise FileNotFoundError(f"找不到模型: {model_path}")
    checkpoint = torch.load(model_path, map_location="cpu", weights_only=False)
    model = (checkpoint.get("ema") or checkpoint.get("model"))
    if model is None:
        raise ValueError("不是有效的 Ultralytics 检测权重")
    names = {int(index): name for index, name in model.names.items()}
    if names != CLASS_NAMES:
        raise ValueError(f"模型类别不符合预期: {names}")
    model = model.float().eval()
    if hasattr(model, "fuse"):
        model.fuse()
    return model


def letterbox(frame, size):
    height, width = frame.shape[:2]
    scale = min(size / width, size / height)
    resized_width, resized_height = round(width * scale), round(height * scale)
    resized = cv2.resize(frame, (resized_width, resized_height), interpolation=cv2.INTER_LINEAR)
    pad_x = (size - resized_width) / 2
    pad_y = (size - resized_height) / 2
    left, top = round(pad_x - 0.1), round(pad_y - 0.1)
    right, bottom = round(pad_x + 0.1), round(pad_y + 0.1)
    image = cv2.copyMakeBorder(
        resized, top, bottom, left, right, cv2.BORDER_CONSTANT, value=(114, 114, 114)
    )
    return image, scale, left, top


def detect(model, frame, image_size, confidence_threshold):
    image, scale, pad_x, pad_y = letterbox(frame, image_size)
    tensor = torch.from_numpy(image[:, :, ::-1].copy()).permute(2, 0, 1)
    tensor = tensor.unsqueeze(0).float().div_(255.0)
    with torch.inference_mode():
        prediction = model(tensor)[0][0].transpose(0, 1).cpu().numpy()

    class_scores = prediction[:, 4:]
    class_ids = class_scores.argmax(axis=1)
    scores = class_scores[np.arange(len(prediction)), class_ids]
    selected = scores >= confidence_threshold
    prediction, class_ids, scores = prediction[selected], class_ids[selected], scores[selected]
    candidates = []
    for class_id in CLASS_NAMES:
        indices = np.flatnonzero(class_ids == class_id)
        if not len(indices):
            continue
        boxes_xywh = []
        for center_x, center_y, width, height in prediction[indices, :4]:
            boxes_xywh.append(
                [float(center_x - width / 2), float(center_y - height / 2), float(width), float(height)]
            )
        kept = cv2.dnn.NMSBoxes(boxes_xywh, scores[indices].tolist(), confidence_threshold, NMS_IOU)
        for kept_index in np.asarray(kept).reshape(-1):
            center_x, center_y, width, height = prediction[indices[kept_index], :4]
            x1, y1 = (center_x - width / 2 - pad_x) / scale, (center_y - height / 2 - pad_y) / scale
            x2, y2 = (center_x + width / 2 - pad_x) / scale, (center_y + height / 2 - pad_y) / scale
            candidates.append(Detection(class_id, float(scores[indices[kept_index]]), (x1, y1, x2, y2)))
    return candidates


def start_camera():
    pipeline, config = rs.pipeline(), rs.config()
    config.enable_stream(rs.stream.color, RGB_WIDTH, RGB_HEIGHT, rs.format.bgr8, RGB_FPS)
    try:
        profile = pipeline.start(config)
        stream = (RGB_WIDTH, RGB_HEIGHT, RGB_FPS)
    except RuntimeError as first_error:
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, RGB_FPS)
        try:
            profile, stream = pipeline.start(config), (1280, 720, RGB_FPS)
        except RuntimeError as second_error:
            raise RuntimeError("D435i RGB 启动失败，请检查相机连接或占用") from second_error
    for sensor in profile.get_device().query_sensors():
        if sensor.get_info(rs.camera_info.name) == "RGB Camera":
            for option in (rs.option.enable_auto_exposure, rs.option.enable_auto_white_balance, rs.option.frames_queue_size):
                try:
                    if sensor.supports(option):
                        sensor.set_option(option, 1.0)
                except RuntimeError:
                    pass
    return pipeline, stream


def put_latest(frame_queue, frame):
    try:
        frame_queue.put_nowait(frame)
    except queue.Full:
        try:
            frame_queue.get_nowait()
            frame_queue.task_done()
        except queue.Empty:
            pass
        try:
            frame_queue.put_nowait(frame)
        except queue.Full:
            pass


def worker(model, frame_queue, shared, lock, stop_event, image_size, confidence):
    while not stop_event.is_set():
        try:
            frame = frame_queue.get(timeout=0.1)
        except queue.Empty:
            continue
        started = time.perf_counter()
        try:
            detections = detect(model, frame, image_size, confidence)
            with lock:
                shared.update(detections=detections, updated=time.monotonic(), inference_ms=(time.perf_counter() - started) * 1000, error=None)
        except Exception as exc:
            with lock:
                shared.update(detections=[], updated=time.monotonic(), error=str(exc))
        finally:
            frame_queue.task_done()


def draw_detections(frame, detections):
    height, width = frame.shape[:2]
    for detection in detections:
        x1, y1, x2, y2 = (int(value) for value in detection.box)
        x1, x2 = np.clip((x1, x2), 0, width - 1)
        y1, y2 = np.clip((y1, y2), 0, height - 1)
        color = (0, 255, 0)
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 3)
        label = f"{CLASS_NAMES[detection.class_id]} {detection.confidence:.2f}"
        cv2.putText(frame, label, (x1, max(26, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.75, color, 2, cv2.LINE_AA)


def main():
    args = parse_args()
    print(f"加载模型: {args.model}")
    model = load_model(args.model)
    detect(model, np.zeros((RGB_HEIGHT, RGB_WIDTH, 3), dtype=np.uint8), args.imgsz, args.conf)
    if args.self_test:
        print("自检通过")
        return 0

    pipeline, stop_event, thread = None, threading.Event(), None
    frame_queue, lock = queue.Queue(maxsize=1), threading.Lock()
    shared = {"detections": [], "updated": 0.0, "inference_ms": 0.0, "error": None}
    signal.signal(signal.SIGINT, lambda *_: stop_event.set())
    try:
        pipeline, stream = start_camera()
        thread = threading.Thread(target=worker, args=(model, frame_queue, shared, lock, stop_event, args.imgsz, args.conf), daemon=True)
        thread.start()
        print(f"RGB {stream[0]}x{stream[1]}@{stream[2]} | imgsz={args.imgsz} | conf={args.conf:.3f} | CPU <= {CPU_LIMIT * 100}% | ESC/Q 退出")
        if not args.no_display:
            cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(WINDOW_NAME, 1280, 720)
        count, fps, started = 0, 0.0, time.monotonic()
        while not stop_event.is_set():
            color_frame = pipeline.wait_for_frames(1000).get_color_frame()
            if not color_frame:
                continue
            frame = np.asanyarray(color_frame.get_data()).copy()
            put_latest(frame_queue, frame.copy())
            now, count = time.monotonic(), count + 1
            if now - started >= 1.0:
                fps, count, started = count / (now - started), 0, now
            with lock:
                detections, age, inference_ms = list(shared["detections"]), now - shared["updated"], shared["inference_ms"]
            if age <= RESULT_TTL_SECONDS:
                draw_detections(frame, detections)
            cv2.putText(frame, f"RGB {fps:.1f} FPS | infer {inference_ms:.0f} ms | CPU <= {CPU_LIMIT * 100}%", (16, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 0), 2, cv2.LINE_AA)
            if not args.no_display:
                cv2.imshow(WINDOW_NAME, frame)
                if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                    break
    finally:
        stop_event.set()
        if thread is not None:
            thread.join(timeout=2)
        if pipeline is not None:
            pipeline.stop()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
