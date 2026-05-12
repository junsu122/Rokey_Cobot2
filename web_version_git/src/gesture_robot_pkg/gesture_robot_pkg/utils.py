"""
utils.py
────────
gesture_robot_control_v6 에서 추출한 공유 유틸리티 함수.
순수 함수만 모아 놓으며, ROS2 / OpenCV 의존성을 최소화한다.
"""

import os
import numpy as np
import cv2
from scipy.spatial.transform import Rotation

from gesture_robot_pkg.constants import (
    DEAD_ZONE, CTRL_DEAD, DEPTH_DEAD,
    MIN_STEP_MM, MAX_STEP_MM, CTRL_MAX_RANGE, DEPTH_MAX_RANGE,
    RANGE_X_MM, RANGE_Y_MM, RANGE_Z_MM,
    LIMIT_X_MM, LIMIT_Y_MM, LIMIT_Z_MM,
    ASPECT_THRESHOLD, FIST_FINGER_COUNT, HAND_CONNECTIONS,
)


# ══════════════════════════════════════════════════════
#  손 제스처 판별
# ══════════════════════════════════════════════════════

def is_fist(landmarks) -> bool:
    """손가락 4개 이상이 접혀 있으면 주먹으로 판정."""
    tips  = [8, 12, 16, 20]
    pips  = [6, 10, 14, 18]
    closed = sum(1 for t, p in zip(tips, pips) if landmarks[t].y > landmarks[p].y)
    return closed >= FIST_FINGER_COUNT


def is_index_pointing(landmarks) -> bool:
    """검지만 펴져 있고 나머지 손가락은 접혀 있으면 포인팅으로 판정."""
    index_up    = landmarks[8].y < landmarks[6].y
    others_down = all(landmarks[t].y > landmarks[p].y
                      for t, p in [(12, 10), (16, 14), (20, 18)])
    return index_up and others_down


def get_palm_center_norm(landmarks):
    """손바닥 중심 (정규화 0-1) 반환."""
    ids = [0, 5, 9, 13, 17]
    px  = float(np.mean([landmarks[i].x for i in ids]))
    py  = float(np.mean([landmarks[i].y for i in ids]))
    return px, -py


def is_in_center(avg_x: float, avg_y: float) -> bool:
    """손이 화면 중앙 dead-zone 안에 있는지 확인."""
    return abs(avg_x - 0.5) <= DEAD_ZONE and abs(avg_y - 0.5) <= DEAD_ZONE


# ══════════════════════════════════════════════════════
#  제어 계산
# ══════════════════════════════════════════════════════

def proportional_step(offset: float, dead: float, max_range: float) -> float:
    if abs(offset) <= dead:
        return 0.0
    ratio = min((abs(offset) - dead) / max_range, 1.0)
    step  = MIN_STEP_MM + ratio * (MAX_STEP_MM - MIN_STEP_MM)
    return -step * float(np.sign(offset))


def compute_delta(avg_x: float, avg_y: float,
                  curr_dist: float, base_dist: float):
    """속도 제어용 delta (mm/step) 계산."""
    ox = avg_x - 0.5
    oy = avg_y - 0.5
    od = curr_dist - base_dist
    dx = proportional_step(ox, CTRL_DEAD, CTRL_MAX_RANGE)
    dz = proportional_step(oy, CTRL_DEAD, CTRL_MAX_RANGE)
    dy = proportional_step(od * (0.5 / DEPTH_MAX_RANGE),
                           DEPTH_DEAD * (0.5 / DEPTH_MAX_RANGE),
                           0.5 - DEPTH_DEAD * (0.5 / DEPTH_MAX_RANGE))
    return dx, dy, dz


def compute_target(avg_x: float, avg_y: float,
                   curr_dist: float, base_dist: float,
                   calib_tcp: list) -> list:
    """위치 제어용 절대 목표 TCP 계산."""
    ox = avg_x - 0.5
    oy = avg_y - 0.5
    od = curr_dist - base_dist

    if abs(ox) < CTRL_DEAD: ox = 0.0
    if abs(oy) < CTRL_DEAD: oy = 0.0
    if abs(od) < DEPTH_DEAD: od = 0.0

    def remap(v, dead):
        if v == 0.0: return 0.0
        sign  = np.sign(v)
        ratio = min((abs(v) - dead) / (0.5 - dead), 1.0)
        return float(sign * ratio)

    rx = remap(ox, CTRL_DEAD)
    ry = remap(oy, CTRL_DEAD)
    rd = float(np.clip(od / DEPTH_MAX_RANGE, -1.0, 1.0))

    tx = calib_tcp[0] + (rx * RANGE_X_MM)
    ty = calib_tcp[1] + (rd * RANGE_Y_MM)
    tz = calib_tcp[2] + (ry * RANGE_Z_MM) # y값 반전을 위한 ry 부호 반전 5.7 수정부

    tx = float(np.clip(-tx, calib_tcp[0] - LIMIT_X_MM, calib_tcp[0] + LIMIT_X_MM))
    ty = float(np.clip( ty, calib_tcp[1] - LIMIT_Y_MM, calib_tcp[1] + LIMIT_Y_MM))
    tz = float(np.clip(-tz, calib_tcp[2] - LIMIT_Z_MM, calib_tcp[2] + LIMIT_Z_MM))

    return [tx, -ty, tz, calib_tcp[3], calib_tcp[4], calib_tcp[5]]


# ══════════════════════════════════════════════════════
#  좌표 변환
# ══════════════════════════════════════════════════════

def get_robot_pose_matrix(x, y, z, rx, ry, rz) -> np.ndarray:
    """로봇 TCP posx [x,y,z,rx,ry,rz] (mm, deg) → 4×4 동차변환행렬."""
    R = Rotation.from_euler('ZYZ', [rx, ry, rz], degrees=True).as_matrix()
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3]  = [x, y, z]
    return T


def transform_camera_to_base(camera_xyz_m: list,
                              gripper2cam_path: str,
                              robot_posx: list) -> np.ndarray:
    """카메라 좌표(m) → 로봇 베이스 좌표(mm) 변환."""
    if not os.path.exists(gripper2cam_path):
        raise FileNotFoundError(
            f'T_gripper2camera.npy 파일 없음: {gripper2cam_path}\n'
            f'GRIPPER2CAM_PATH 를 올바르게 설정하세요.')

    gripper2cam  = np.load(gripper2cam_path)
    coord_mm     = np.append(np.array(camera_xyz_m) * 1000.0, 1.0)
    x, y, z, rx, ry, rz = robot_posx
    base2gripper = get_robot_pose_matrix(x, y, z, rx, ry, rz)
    base2cam     = base2gripper @ gripper2cam
    return (base2cam @ coord_mm)[:3]


def get_orientation_offset(box: list) -> float:
    """물체 bbox 종횡비로 그리퍼 rz 보정값(deg) 반환."""
    x1, y1, x2, y2 = box
    w = x2 - x1
    h = y2 - y1
    if h == 0:
        return 0.0
    ratio = w / h
    if ratio >= ASPECT_THRESHOLD:
        return 0.0
    return 0.0


# ══════════════════════════════════════════════════════
#  OpenCV 드로잉 헬퍼
# ══════════════════════════════════════════════════════

def draw_landmarks_manual(frame, landmarks, color=(0, 200, 100)):
    h, w = frame.shape[:2]
    pts  = [(int(lm.x * w), int(lm.y * h)) for lm in landmarks]
    for a, b in HAND_CONNECTIONS:
        cv2.line(frame, pts[a], pts[b], color, 2, cv2.LINE_AA)
    for x, y in pts:
        cv2.circle(frame, (x, y), 4, (255, 255, 255), -1, cv2.LINE_AA)


def draw_hover_progress(frame, box, progress: float, label: str,
                        color=(0, 220, 255)):
    x1, y1, x2, y2 = box
    bar_w  = x2 - x1
    filled = int(bar_w * progress)
    cv2.rectangle(frame, (x1, y2 + 2), (x2, y2 + 8), (40, 40, 40), -1)
    cv2.rectangle(frame, (x1, y2 + 2), (x1 + filled, y2 + 8), color, -1)
    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 3)
    cv2.putText(frame, f'SELECTING: {label}  {progress * 100:.0f}%',
                (x1, y1 - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2, cv2.LINE_AA)


def draw_selected_label(frame, label: str):
    h, w = frame.shape[:2]
    ov   = frame.copy()
    cv2.rectangle(ov, (0, 0), (w, 60), (0, 40, 0), -1)
    cv2.addWeighted(ov, 0.6, frame, 0.4, 0, frame)
    txt = f'✔ SELECTED:  {label.upper()}'
    (tw, _), _ = cv2.getTextSize(txt, cv2.FONT_HERSHEY_DUPLEX, 1.0, 2)
    cv2.putText(frame, txt, ((w - tw) // 2, 42),
                cv2.FONT_HERSHEY_DUPLEX, 1.0, (0, 255, 120), 2, cv2.LINE_AA)


def draw_picking_overlay(frame, label: str):
    h, w = frame.shape[:2]
    ov   = frame.copy()
    cv2.rectangle(ov, (0, 0), (w, h), (0, 80, 160), -1)
    cv2.addWeighted(ov, 0.15, frame, 0.85, 0, frame)
    txt = f'PICKING: {label.upper()}'
    (tw, th), _ = cv2.getTextSize(txt, cv2.FONT_HERSHEY_DUPLEX, 1.6, 3)
    cv2.putText(frame, txt, ((w - tw) // 2, (h + th) // 2),
                cv2.FONT_HERSHEY_DUPLEX, 1.6, (0, 165, 255), 3, cv2.LINE_AA)
    sub = 'Robot is picking the object...'
    (sw, _), _ = cv2.getTextSize(sub, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 1)
    cv2.putText(frame, sub, ((w - sw) // 2, (h + th) // 2 + 44),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 1, cv2.LINE_AA)


def draw_workspace_grid(frame, avg_x, avg_y, calib_tcp, target_pos, _state):
    h, w   = frame.shape[:2]
    gw, gh = 160, 120
    margin = 15
    x0 = w - gw - margin
    y0 = h - gh - margin - 30
    ov = frame.copy()
    cv2.rectangle(ov, (x0 - 4, y0 - 20), (x0 + gw + 4, y0 + gh + 4), (20, 20, 20), -1)
    cv2.addWeighted(ov, 0.7, frame, 0.3, 0, frame)
    cv2.rectangle(frame, (x0 - 4, y0 - 20), (x0 + gw + 4, y0 + gh + 4), (60, 60, 60), 1)
    cv2.putText(frame, 'WORKSPACE (X-Z)', (x0 - 2, y0 - 6),
                cv2.FONT_HERSHEY_SIMPLEX, 0.3, (100, 100, 100), 1)
    for i in range(5):
        xi = x0 + i * gw // 4
        zi = y0 + i * gh // 4
        cv2.line(frame, (xi, y0), (xi, y0 + gh), (40, 40, 40), 1)
        cv2.line(frame, (x0, zi), (x0 + gw, zi), (40, 40, 40), 1)
    cx, cz = x0 + gw // 2, y0 + gh // 2
    cv2.drawMarker(frame, (cx, cz), (80, 80, 80), cv2.MARKER_CROSS, 12, 1, cv2.LINE_AA)
    if calib_tcp is not None and target_pos is not None:
        tx_n = (target_pos[0] - calib_tcp[0]) / LIMIT_X_MM
        tz_n = (target_pos[2] - calib_tcp[2]) / LIMIT_Z_MM
        tpx  = int(np.clip(cx - tx_n * gw / 2, x0, x0 + gw))
        tpz  = int(np.clip(cz - tz_n * gh / 2, y0, y0 + gh))
        cv2.circle(frame, (tpx, tpz), 7, (0, 220, 80),   -1, cv2.LINE_AA)
        cv2.circle(frame, (tpx, tpz), 7, (255, 255, 255), 1, cv2.LINE_AA)
    if avg_x is not None:
        hpx = int(np.clip(x0 + avg_x * gw, x0, x0 + gw))
        hpz = int(np.clip(y0 + avg_y * gh, y0, y0 + gh))
        cv2.circle(frame, (hpx, hpz), 5, (220, 120, 0), -1, cv2.LINE_AA)
    cv2.circle(frame, (x0, y0 + gh + 14), 4, (0, 220, 80), -1)
    cv2.putText(frame, 'target', (x0 + 7, y0 + gh + 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.3, (80, 80, 80), 1)
    cv2.circle(frame, (x0 + 55, y0 + gh + 14), 4, (220, 120, 0), -1)
    cv2.putText(frame, 'hand', (x0 + 62, y0 + gh + 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.3, (80, 80, 80), 1)
