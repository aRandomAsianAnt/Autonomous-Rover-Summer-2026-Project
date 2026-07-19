import cv2
import numpy as np
import onnxruntime as ort
import time
import urllib.request
import os
from dotenv import load_dotenv

#Load .env from project root
load_dotenv(dotenv_path="../.env")

#Streaming URL of the Pi camera feed
PI_IP = os.environ.get("ROVER_PI_IP", "YOUR_PI_TAILSCALE_IP")
PI_STREAM_URL = f"http://{PI_IP}:5000/video_feed"

#Depth Anything Model
opts = ort.SessionOptions()
opts.intra_op_num_threads = 6
opts.inter_op_num_threads = 2
opts.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL

session = ort.InferenceSession("model_int8.onnx", sess_options=opts,
                                providers=["CPUExecutionProvider"])
input_name = session.get_inputs()[0].name

#Live settings for depth 
RESOLUTIONS    = [128, 192, 256, 320, 384]
SKIP_OPTIONS   = [2, 3, 4, 5]
COLORMAPS      = [cv2.COLORMAP_INFERNO, cv2.COLORMAP_MAGMA,
                  cv2.COLORMAP_JET,     cv2.COLORMAP_TURBO]
COLORMAP_NAMES = ["INFERNO", "MAGMA", "JET", "TURBO"]

res_idx  = 1   # 192
skip_idx = 0   # every 2nd frame
cmap_idx = 0   # INFERNO

# ── Smoothing ──────────────────────────────────────────────────────────────────
smoothing_on = True
ALPHA        = 0.65
prev_depth   = None

# ── Helpers ────────────────────────────────────────────────────────────────────
MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
STD  = np.array([0.229, 0.224, 0.225], dtype=np.float32)

def preprocess(frame, size):
    img = cv2.resize(frame, (size, size), interpolation=cv2.INTER_NEAREST)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
    img = (img - MEAN) / STD
    return np.transpose(img, (2, 0, 1))[np.newaxis, ...].astype(np.float32)

def colorize(depth, target_size, cmap):
    d8 = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
    d8 = cv2.resize(d8, target_size, interpolation=cv2.INTER_LINEAR)
    return cv2.applyColorMap(d8, cmap)

def draw_overlay(frame, res, skip, cmap_name, inf_ms, net_ms, smooth):
    h, w = frame.shape[:2]
    cv2.rectangle(frame, (0, 0), (w // 2, 130), (0, 0, 0), -1)
    tmp = frame.copy()
    cv2.addWeighted(tmp, 0.4, frame, 0.6, 0, frame)

    inf_color = (0,255,0) if inf_ms < 80 else (0,165,255) if inf_ms < 150 else (0,0,255)
    net_color = (0,255,0) if net_ms < 50 else (0,165,255) if net_ms < 100 else (0,0,255)

    def txt(msg, row, color=(255,255,255)):
        cv2.putText(frame, msg, (10, row),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.52, color, 1)

    txt(f"[W/S] Resolution  : {res}x{res}",            22)
    txt(f"[A/D] Frame Skip  : every {skip} frame(s)",  44)
    txt(f"[I]   Colormap    : {cmap_name}",             66)
    txt(f"      Inference   : {inf_ms:.1f} ms",         88,  inf_color)
    txt(f"      Network     : {net_ms:.1f} ms",         108, net_color)
    txt(f"[T] Smoothing: {'ON' if smooth else 'OFF'}",  128)
    return frame

# ── Connect to Pi Stream ───────────────────────────────────────────────────────
print(f"Connecting to Pi stream: {PI_STREAM_URL}")
stream = urllib.request.urlopen(PI_STREAM_URL, timeout=10)
print("Connected. Starting depth inference — W/S: res | A/D: skip | I: colormap | T: smooth | Q: quit")

# ── MJPEG byte buffer ──────────────────────────────────────────────────────────
byte_buffer  = b""
frame_count  = 0
inference_ms = 0.0
network_ms   = 0.0
depth_colored = None

while True:
    # ── Pull frame from Pi MJPEG stream ───────────────────────────────────────
    t_net = time.perf_counter()
    byte_buffer += stream.read(4096)
    a = byte_buffer.find(b'\xff\xd8')  # JPEG start
    b = byte_buffer.find(b'\xff\xd9')  # JPEG end

    if a == -1 or b == -1:
        continue

    jpg_data    = byte_buffer[a:b+2]
    byte_buffer = byte_buffer[b+2:]
    network_ms  = (time.perf_counter() - t_net) * 1000

    frame = cv2.imdecode(np.frombuffer(jpg_data, dtype=np.uint8), cv2.IMREAD_COLOR)
    if frame is None:
        continue

    frame_count += 1
    res         = RESOLUTIONS[res_idx]
    skip        = SKIP_OPTIONS[skip_idx]
    cmap        = COLORMAPS[cmap_idx]
    cmap_name   = COLORMAP_NAMES[cmap_idx]
    target_size = (frame.shape[1], frame.shape[0])

    """
    # ── Inference ─────────────────────────────────────────────────────────────
    if frame_count % skip == 0:
        t0           = time.perf_counter()
        tensor       = preprocess(frame, res)
        raw          = session.run(None, {input_name: tensor})[0][0]
        inference_ms = (time.perf_counter() - t0) * 1000

        depth_full = cv2.resize(raw.astype(np.float32), target_size,
                                interpolation=cv2.INTER_LINEAR)

        if smoothing_on and prev_depth is not None:
            depth_full = ALPHA * depth_full + (1 - ALPHA) * prev_depth
        prev_depth    = depth_full
        depth_colored = colorize(depth_full, target_size, cmap)
        """

    # ── Display ───────────────────────────────────────────────────────────────
    if depth_colored is not None:
        display  = draw_overlay(frame.copy(), res, skip, cmap_name,
                                inference_ms, network_ms, smoothing_on)
        combined = np.hstack((display, depth_colored))
        #cv2.imshow("Pi Cam | Depth Map  [Q to quit]", combined)
        cv2.imshow("Pi Stream Test [Q to quit]", frame)

    # ── Keys ──────────────────────────────────────────────────────────────────
    key = cv2.waitKey(1) & 0xFF
    if   key == ord('q'): break
    elif key == ord('w'): res_idx  = min(res_idx  + 1, len(RESOLUTIONS)  - 1)
    elif key == ord('s'): res_idx  = max(res_idx  - 1, 0)
    elif key == ord('d'): skip_idx = min(skip_idx + 1, len(SKIP_OPTIONS) - 1)
    elif key == ord('a'): skip_idx = max(skip_idx - 1, 0)
    elif key == ord('i'): cmap_idx = (cmap_idx + 1) % len(COLORMAPS)
    elif key == ord('t'):
        smoothing_on = not smoothing_on
        prev_depth   = None

cv2.destroyAllWindows()