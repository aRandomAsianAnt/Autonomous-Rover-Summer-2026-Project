import cv2
import numpy as np
import onnxruntime as ort

# ── Model ──────────────────────────────────────────────────────────────────────
opts = ort.SessionOptions()
opts.intra_op_num_threads = 4
opts.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL

session = ort.InferenceSession("model_int8.onnx", sess_options=opts,
                                providers=["CPUExecutionProvider"])
input_name = session.get_inputs()[0].name

# ── Config (live tunable) ──────────────────────────────────────────────────────
RESOLUTIONS  = [128, 192, 256, 320, 384, 518]
SKIP_OPTIONS = [1, 2, 3, 4, 5]
COLORMAPS    = [cv2.COLORMAP_INFERNO, cv2.COLORMAP_MAGMA,
                cv2.COLORMAP_JET,     cv2.COLORMAP_TURBO]
COLORMAP_NAMES = ["INFERNO", "MAGMA", "JET", "TURBO"]

res_idx     = 2   # start at 256
skip_idx    = 1   # start at skip 2
cmap_idx    = 0   # start at INFERNO

# ── Helpers ────────────────────────────────────────────────────────────────────
MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
STD  = np.array([0.229, 0.224, 0.225], dtype=np.float32)

def preprocess(frame, size):
    img = cv2.resize(frame, (size, size), interpolation=cv2.INTER_NEAREST)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
    img = (img - MEAN) / STD
    return np.transpose(img, (2, 0, 1))[np.newaxis, ...].astype(np.float32)

def colorize_depth(depth, target_size, cmap):
    depth_norm = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
    depth_resized = cv2.resize(depth_norm, target_size)
    return cv2.applyColorMap(depth_resized, cmap)

def draw_overlay(frame, res, skip, cmap_name):
    overlay = frame.copy()
    cv2.rectangle(overlay, (0, 0), (320, 80), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.5, frame, 0.5, 0, frame)
    cv2.putText(frame, f"[W/S] Resolution : {res}x{res}",  (10, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
    cv2.putText(frame, f"[A/D] Frame Skip : every {skip} frame(s)", (10, 42),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
    cv2.putText(frame, f"[I]   Colormap   : {cmap_name}", (10, 64),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
    return frame

# ── Main loop ──────────────────────────────────────────────────────────────────
cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)  # Use CAP_DSHOW to avoid warnings on Windows

#Set inference resolution lower
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

if not cap.isOpened():
    print("Error: Could not open webcam.")
    exit()

print("Running Depth Anything V2 — W/S: resolution | A/D: skip | I: colormap | Q: quit")

frame_count   = 0
depth_colored = None

while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame_count += 1
    res         = RESOLUTIONS[res_idx]
    skip        = SKIP_OPTIONS[skip_idx]
    cmap        = COLORMAPS[cmap_idx]
    cmap_name   = COLORMAP_NAMES[cmap_idx]
    target_size = (frame.shape[1], frame.shape[0])

    # Run inference on schedule
    if frame_count % skip == 0:
        input_tensor  = preprocess(frame, res)
        outputs       = session.run(None, {input_name: input_tensor})
        depth_colored = colorize_depth(outputs[0][0], target_size, cmap)

    if depth_colored is not None:
        display_frame = draw_overlay(frame.copy(), res, skip, cmap_name)
        combined = np.hstack((display_frame, depth_colored))
        cv2.imshow("Webcam | Depth Map  [Q to quit]", combined)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    elif key == ord('w'):
        res_idx = min(res_idx + 1, len(RESOLUTIONS) - 1)
    elif key == ord('s'):
        res_idx = max(res_idx - 1, 0)
    elif key == ord('d'):
        skip_idx = min(skip_idx + 1, len(SKIP_OPTIONS) - 1)
    elif key == ord('a'):
        skip_idx = max(skip_idx - 1, 0)
    elif key == ord('i'):
        cmap_idx = (cmap_idx + 1) % len(COLORMAPS)

cap.release()
cv2.destroyAllWindows()