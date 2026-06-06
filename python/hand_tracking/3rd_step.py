import cv2
import mediapipe as mp
import math
import time
import socket
import numpy as np
from typing import List, Dict

# ── MediaPipe setup ───────────────────────────────────────────────────────────
mp_hands = mp.solutions.hands
mp_draw  = mp.solutions.drawing_utils

hands = mp_hands.Hands(
    static_image_mode=False,
    max_num_hands=1,
    min_detection_confidence=0.75,
    min_tracking_confidence=0.65,
)

# ── UDP setup ─────────────────────────────────────────────────────────────────
ESP32_IP      = "192.168.4.1"   # ← paste sender ESP32 IP from Serial Monitor
ESP32_PORT    = 5005
SEND_INTERVAL = 0.05             # 50ms = 20 packets/sec

udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
print(f"UDP socket ready → {ESP32_IP}:{ESP32_PORT}")

# ── Landmark indices ──────────────────────────────────────────────────────────
WRIST      = 0
THUMB_CMC  = 1; THUMB_MCP = 2; THUMB_IP  = 3; THUMB_TIP  = 4
INDEX_MCP  = 5; INDEX_PIP = 6; INDEX_DIP = 7; INDEX_TIP  = 8
MID_MCP    = 9; MID_PIP   =10; MID_DIP   =11; MID_TIP    =12
RING_MCP   =13; RING_PIP  =14; RING_DIP  =15; RING_TIP   =16
PINK_MCP   =17; PINK_PIP  =18; PINK_DIP  =19; PINK_TIP   =20

MOTOR_NAMES  = ["MOTOR_THUMB", "MOTOR_INDEX", "MOTOR_MIDDLE", "MOTOR_RING", "MOTOR_PINKY"]
MOTOR_COLORS = [
    (0,   200, 255),
    (0,   220, 120),
    (255, 180,   0),
    (180,   0, 255),
    (0,   120, 255),
]

FINGER_JOINTS = [
    (INDEX_MCP, INDEX_PIP, INDEX_TIP),
    (MID_MCP,   MID_PIP,   MID_TIP),
    (RING_MCP,  RING_PIP,  RING_TIP),
    (PINK_MCP,  PINK_PIP,  PINK_TIP),
]

ANGLE_CALIBRATION = [
    (160.0, 40.0),
    (160.0, 40.0),
    (160.0, 40.0),
    (160.0, 40.0),
]

SMOOTH_N    = 6
curl_buffer: List[List[float]] = [[] for _ in range(5)]

# ── ESP32 monitor log ─────────────────────────────────────────────────────────
esp32_log: List[str] = []
MAX_ESP32_LOG = 6


# ── Math helpers ──────────────────────────────────────────────────────────────
def angle_between_points(a, b, c) -> float:
    ax, ay = a.x - b.x, a.y - b.y
    cx, cy = c.x - b.x, c.y - b.y
    dot = ax * cx + ay * cy
    mag = math.hypot(ax, ay) * math.hypot(cx, cy)
    if mag < 1e-6:
        return 0.0
    return math.degrees(math.acos(max(-1.0, min(1.0, dot / mag))))


def distance_2d(a, b) -> float:
    return math.hypot(a.x - b.x, a.y - b.y)


def angle_to_curl(angle_deg: float, finger_idx: int) -> float:
    straight, curled = ANGLE_CALIBRATION[finger_idx]
    pct = (straight - angle_deg) / (straight - curled)
    return round(max(0.0, min(100.0, pct * 100.0)), 1)


def smooth_curl(finger_idx: int, new_val: float) -> float:
    buf = curl_buffer[finger_idx]
    buf.append(new_val)
    if len(buf) > SMOOTH_N:
        buf.pop(0)
    return round(sum(buf) / len(buf), 1)


# ── Thumb curl ────────────────────────────────────────────────────────────────
THUMB_OPEN_DIST    = 0.38
THUMB_CLOSED_DIST  = 0.10
THUMB_JOINT_OPEN   = 160.0
THUMB_JOINT_CLOSED =  55.0

def compute_thumb_curl(lm) -> float:
    hand_size = distance_2d(lm[WRIST], lm[MID_MCP])
    if hand_size < 1e-4:
        return 0.0
    raw_dist  = distance_2d(lm[THUMB_TIP], lm[INDEX_MCP])
    norm_dist = raw_dist / hand_size
    dist_pct  = (THUMB_OPEN_DIST - norm_dist) / (THUMB_OPEN_DIST - THUMB_CLOSED_DIST)
    dist_pct  = max(0.0, min(1.0, dist_pct))
    joint_ang = angle_between_points(lm[THUMB_CMC], lm[THUMB_MCP], lm[THUMB_IP])
    angle_pct = (THUMB_JOINT_OPEN - joint_ang) / (THUMB_JOINT_OPEN - THUMB_JOINT_CLOSED)
    angle_pct = max(0.0, min(1.0, angle_pct))
    combined  = 0.65 * dist_pct + 0.35 * angle_pct
    return round(combined * 100.0, 1)


# ── All 5 curl percentages ────────────────────────────────────────────────────
def compute_curl_percentages(lm) -> List[float]:
    curls = []
    curls.append(smooth_curl(0, compute_thumb_curl(lm)))
    for i, (base, mid, tip) in enumerate(FINGER_JOINTS):
        ang = angle_between_points(lm[base], lm[mid], lm[tip])
        curls.append(smooth_curl(i + 1, angle_to_curl(ang, i)))
    return curls


# ── UDP: send to ESP32 ────────────────────────────────────────────────────────
last_send_time = 0.0
udp_send_count = 0

def send_to_esp32(curls: List[float]) -> bool:
    global last_send_time, udp_send_count
    now = time.time()
    if now - last_send_time < SEND_INTERVAL:
        return False
    try:
        msg = f"{curls[0]:.1f},{curls[1]:.1f},{curls[2]:.1f},{curls[3]:.1f},{curls[4]:.1f}\n"
        udp_sock.sendto(msg.encode(), (ESP32_IP, ESP32_PORT))
        last_send_time  = now
        udp_send_count += 1
        return True
    except Exception as e:
        print(f"UDP send error: {e}")
        return False


# ── Terminal print ────────────────────────────────────────────────────────────
last_printed_curls: Dict[int, float] = {i: -999.0 for i in range(5)}
last_print_time  = 0.0
PRINT_INTERVAL   = 0.1
CHANGE_THRESHOLD = 2.0

def serial_print(curls: List[float], label: str) -> bool:
    global last_print_time
    now = time.time()
    if now - last_print_time < PRINT_INTERVAL:
        return False
    if not any(abs(curls[i] - last_printed_curls[i]) >= CHANGE_THRESHOLD for i in range(5)):
        return False
    ts = time.strftime("%H:%M:%S")
    print(f"[{ts}] [UDP #{udp_send_count}] {label:5s} | "
          f"THUMB:{curls[0]:5.1f}%  INDEX:{curls[1]:5.1f}%  "
          f"MIDDLE:{curls[2]:5.1f}%  RING:{curls[3]:5.1f}%  PINKY:{curls[4]:5.1f}%")
    for i in range(5):
        last_printed_curls[i] = curls[i]
    last_print_time = now
    return True


# ── Draw: motor panel (top-right) ─────────────────────────────────────────────
def draw_motor_panel(frame, curls, h, w, label):
    panel_w = 280
    panel_x = w - panel_w - 10
    panel_y = 60
    row_h   = 54
    total_h = row_h * 5 + 44
    overlay = frame.copy()
    cv2.rectangle(overlay,
                  (panel_x - 10, panel_y),
                  (panel_x + panel_w, panel_y + total_h),
                  (15, 15, 15), -1)
    frame[:] = cv2.addWeighted(overlay, 0.72, frame, 0.28, 0)

    status       = f"UDP → {ESP32_IP}:{ESP32_PORT}  #{udp_send_count}"
    status_color = (0, 255, 100)
    cv2.putText(frame, status, (panel_x - 6, panel_y - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.40, status_color, 1, cv2.LINE_AA)
    cv2.putText(frame, f"ROBOTIC HAND  [{label}]",
                (panel_x - 6, panel_y + 22),
                cv2.FONT_HERSHEY_SIMPLEX, 0.52, (200, 200, 200), 1, cv2.LINE_AA)

    for i, (name, pct, color) in enumerate(zip(MOTOR_NAMES, curls, MOTOR_COLORS)):
        y0 = panel_y + 38 + i * row_h
        cv2.putText(frame, name, (panel_x, y0),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, color, 1, cv2.LINE_AA)
        bar_x, bar_y = panel_x, y0 + 6
        bar_w, bar_h = panel_w - 10, 14
        cv2.rectangle(frame, (bar_x, bar_y),
                      (bar_x + bar_w, bar_y + bar_h), (50, 50, 50), -1)
        fill_w = int(bar_w * pct / 100.0)
        r = int(pct * 2.55); g = int((100 - pct) * 2.55)
        if fill_w > 0:
            cv2.rectangle(frame, (bar_x, bar_y),
                          (bar_x + fill_w, bar_y + bar_h), (0, g, r), -1)
        cv2.putText(frame, f"{pct:5.1f}%",
                    (bar_x + bar_w + 4, bar_y + bar_h - 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)
        cv2.putText(frame, f"{int(pct*1.8)}deg",
                    (bar_x + bar_w - 48, bar_y - 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (150, 150, 150), 1, cv2.LINE_AA)


# ── Draw: finger tip % labels ─────────────────────────────────────────────────
def draw_finger_labels(frame, lm, curls, w, h):
    tip_indices = [THUMB_TIP, INDEX_TIP, MID_TIP, RING_TIP, PINK_TIP]
    for tip_idx, pct, color in zip(tip_indices, curls, MOTOR_COLORS):
        tx = int(lm[tip_idx].x * w)
        ty = int(lm[tip_idx].y * h) - 16
        lbl = f"{pct:.0f}%"
        (tw, th), _ = cv2.getTextSize(lbl, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        cv2.rectangle(frame, (tx - 4, ty - th - 4), (tx + tw + 4, ty + 4), (0, 0, 0), -1)
        cv2.putText(frame, lbl, (tx, ty),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)


# ── Draw: send log (bottom strip) ─────────────────────────────────────────────
log_history: List[str] = []

def draw_log_panel(frame, h, w):
    if not log_history:
        return
    rows    = log_history[-4:]
    panel_h = len(rows) * 22 + 12
    overlay = frame.copy()
    cv2.rectangle(overlay, (0, h - panel_h), (w, h), (10, 10, 10), -1)
    frame[:] = cv2.addWeighted(overlay, 0.65, frame, 0.35, 0)
    for i, entry in enumerate(rows):
        cv2.putText(frame, entry, (10, h - panel_h + 18 + i * 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (160, 220, 160), 1, cv2.LINE_AA)


# ── Draw: UDP status panel (top-left) ─────────────────────────────────────────
def draw_udp_panel(frame, h, w):
    rows    = [f"UDP → {ESP32_IP}:{ESP32_PORT}", f"Packets sent: {udp_send_count}"]
    panel_h = len(rows) * 22 + 34
    panel_w = 400
    overlay = frame.copy()
    cv2.rectangle(overlay, (0, 0), (panel_w, panel_h), (5, 30, 5), -1)
    frame[:] = cv2.addWeighted(overlay, 0.78, frame, 0.22, 0)
    cv2.putText(frame, "UDP STATUS",
                (10, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (60, 255, 60), 1, cv2.LINE_AA)
    for i, entry in enumerate(rows):
        cv2.putText(frame, entry, (10, 36 + i * 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.40, (120, 255, 120), 1, cv2.LINE_AA)


# ── Camera auto-finder ────────────────────────────────────────────────────────
def find_camera():
    for i in range(5):
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            print(f"Camera found at index {i}")
            return cap
        cap.release()
    print("No camera found!")
    return None


# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    cap = find_camera()
    if cap is None:
        return

    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    WIN_NAME = "Robotic Hand Motor Control"
    cv2.namedWindow(WIN_NAME, cv2.WINDOW_NORMAL)
    cv2.setWindowProperty(WIN_NAME, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

    print("=" * 80)
    print(f"  UDP target: {ESP32_IP}:{ESP32_PORT}")
    print("  THUMB: cross-palm method (tip-to-index-MCP distance + joint angle)")
    print("  SERVOS: MG996R via PCA9685 | format sent: T,I,M,R,P (0-100%)")
    print("  Press Q to quit")
    print("=" * 80)

    fps_time    = time.time()
    frame_count = 0
    fps         = 0.0

    while True:
        ok, frame = cap.read()
        if not ok:
            break

        frame = cv2.flip(frame, 1)
        h, w  = frame.shape[:2]
        rgb   = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        res   = hands.process(rgb)

        if res.multi_hand_landmarks and res.multi_handedness:
            hand_lm   = res.multi_hand_landmarks[0]
            hand_info = res.multi_handedness[0]
            label     = hand_info.classification[0].label

            mp_draw.draw_landmarks(
                frame, hand_lm, mp_hands.HAND_CONNECTIONS,
                mp_draw.DrawingSpec(color=(80, 80, 80), thickness=2, circle_radius=3),
                mp_draw.DrawingSpec(color=(200, 200, 200), thickness=1),
            )

            lm    = hand_lm.landmark
            curls = compute_curl_percentages(lm)

            draw_finger_labels(frame, lm, curls, w, h)
            draw_motor_panel(frame, curls, h, w, label)

            send_to_esp32(curls)

            ts = time.strftime("%H:%M:%S")
            if serial_print(curls, label):
                log_history.append(
                    f"[{ts}] T:{curls[0]:4.0f}% I:{curls[1]:4.0f}% "
                    f"M:{curls[2]:4.0f}% R:{curls[3]:4.0f}% P:{curls[4]:4.0f}%")
                if len(log_history) > 40:
                    log_history.pop(0)
        else:
            cv2.putText(frame, "No hand detected",
                        (w // 2 - 150, h // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (80, 80, 255), 2, cv2.LINE_AA)

        draw_log_panel(frame, h, w)
        draw_udp_panel(frame, h, w)

        frame_count += 1
        elapsed = time.time() - fps_time
        if elapsed >= 1.0:
            fps         = frame_count / elapsed
            frame_count = 0
            fps_time    = time.time()

        cv2.putText(frame, f"FPS: {fps:.1f}", (10, h - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (160, 160, 160), 1, cv2.LINE_AA)
        cv2.putText(frame, "Press Q to quit", (100, h - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (120, 120, 120), 1, cv2.LINE_AA)

        cv2.imshow(WIN_NAME, frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    udp_sock.close()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
