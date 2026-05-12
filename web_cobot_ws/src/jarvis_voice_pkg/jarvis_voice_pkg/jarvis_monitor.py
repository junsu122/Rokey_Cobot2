#!/usr/bin/env python3
"""
JARVIS TUI Monitor
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
ROS2 토픽을 실시간 구독하여 TUI로 표시

[설치]
    pip install rich

[실행]
    python3 jarvis_monitor.py

[구독 토픽]
    /voice_command
    /intent_result
    /tts_output
    /voice_intent
    /scan_request
    /scan_result
    /object_not_found
    /selected_object   (gesture_robot_interfaces/msg/SelectedObject)
"""

import json
import threading
import time
from datetime import datetime
from collections import deque

from rich.console import Console
from rich.layout import Layout
from rich.live import Live
from rich.panel import Panel
from rich.table import Table
from rich.text import Text
from rich import box

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    from gesture_robot_interfaces.msg import SelectedObject
    HAS_SELECTED_OBJECT = True
except ImportError:
    HAS_SELECTED_OBJECT = False


# ══════════════════════════════════════════════════════════════════════════════
# 상태 저장소
# ══════════════════════════════════════════════════════════════════════════════

class JARVISState:
    def __init__(self):
        self._lock = threading.Lock()

        # STT
        self.stt_text       : str   = "-"
        self.stt_time       : str   = "-"

        # Intent Engine
        self.intent         : str   = "-"
        self.target_object  : str   = "-"
        self.urgency        : str   = "-"
        self.confidence     : float = 0.0
        self.scores         : dict  = {}
        self.reason_log     : list  = []
        self.tts_message    : str   = "-"
        self.intent_time    : str   = "-"

        # 토픽 발행 현황
        self.last_voice_cmd    : str = "-"
        self.last_voice_intent : str = "-"
        self.last_tts          : str = "-"

        # 스캔 상태
        self.scan_status  : str  = "대기 중"
        self.scan_targets : list = []
        self.scan_found   : list = []
        self.scan_time    : str  = "-"

        # Vision 감지 현황
        self.vision_status     : str  = "대기 중"   # "감지됨" | "미감지" | "대기 중"
        self.selected_object   : str  = "-"
        self.selected_conf     : str  = "-"
        self.selected_box      : str  = "-"
        self.selected_time     : str  = "-"
        self.not_found_objects : list = []
        self.not_found_time    : str  = "-"

        # 로그 (최근 20개)
        self.log_entries = deque(maxlen=20)

    def add_log(self, level: str, message: str):
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        with self._lock:
            self.log_entries.append((ts, level, message))

    def update(self, **kwargs):
        with self._lock:
            for k, v in kwargs.items():
                if hasattr(self, k):
                    setattr(self, k, v)


# ══════════════════════════════════════════════════════════════════════════════
# ROS2 모니터 노드
# ══════════════════════════════════════════════════════════════════════════════

class JARVISMonitorNode(Node):
    def __init__(self, state: JARVISState):
        super().__init__("jarvis_monitor_node")
        self.state = state

        # String 토픽 구독
        string_topics = [
            ("/voice_command",    self._cb_voice_command),
            ("/intent_result",    self._cb_intent_result),
            ("/tts_output",       self._cb_tts_output),
            ("/voice_intent",     self._cb_voice_intent),
            ("/scan_request",     self._cb_scan_request),
            ("/scan_result",      self._cb_scan_result),
            ("/object_not_found", self._cb_object_not_found),
        ]
        for topic, cb in string_topics:
            self.create_subscription(String, topic, cb, 10)

        # SelectedObject 커스텀 메시지 구독
        if HAS_SELECTED_OBJECT:
            self.create_subscription(
                SelectedObject, "/selected_object",
                self._cb_selected_object, 10)
        else:
            self.get_logger().warn(
                "⚠️  gesture_robot_interfaces 없음 — /selected_object 구독 불가")

        self.get_logger().info("JARVIS Monitor Node 시작")

    def _parse(self, msg) -> dict:
        try:
            return json.loads(msg.data)
        except Exception:
            return {"raw": msg.data}

    # ── 콜백 ──────────────────────────────────────────────────────────────

    def _cb_voice_command(self, msg):
        data = self._parse(msg)
        text = data.get("text", data.get("raw", "-"))
        ts   = data.get("timestamp", datetime.now().strftime("%H:%M:%S"))
        self.state.update(
            last_voice_cmd=text,
            stt_text=text,
            stt_time=ts,
        )
        self.state.add_log("STT", f'"{text}"')

    def _cb_intent_result(self, msg):
        data    = self._parse(msg)
        intent  = data.get("intent",          "-")
        target  = str(data.get("target_object", "-"))
        urgency = data.get("urgency",          "-")
        conf    = data.get("confidence",       0.0)
        scores  = data.get("scores",           {})
        reasons = data.get("reason_log",       [])
        tts     = data.get("response_message", "-")
        ts      = data.get("timestamp",        datetime.now().strftime("%H:%M:%S"))

        self.state.update(
            intent=intent,
            target_object=target,
            urgency=urgency,
            confidence=conf,
            scores=scores,
            reason_log=reasons,
            tts_message=tts,
            intent_time=ts,
        )
        self.state.add_log("INTENT", f"{intent} / {target} (conf={conf:.2f})")

    def _cb_tts_output(self, msg):
        data = self._parse(msg)
        text = data.get("message", data.get("raw", "-"))
        self.state.update(last_tts=text)
        self.state.add_log("TTS", text)

    def _cb_voice_intent(self, msg):
        data   = self._parse(msg)
        action = data.get("action", "-")
        target = data.get("target_object", [])
        self.state.update(last_voice_intent=f"{action} →{target}")
        self.state.add_log("VISION", f"voice_intent: {action} → {target}")

    def _cb_scan_request(self, msg):
        data    = self._parse(msg)
        action  = data.get("action", "start")
        targets = data.get("target_objects", [])
        ts      = data.get("timestamp", datetime.now().strftime("%H:%M:%S"))

        if action == "cancel":
            self.state.update(scan_status="취소됨", scan_time=ts)
            self.state.add_log("SCAN", "🛑 취소 명령 발행")
        else:
            self.state.update(
                scan_status="탐색 중...",
                scan_targets=targets,
                scan_found=[],
                scan_time=ts,
            )
            self.state.add_log("SCAN", f"🔍 탐색 시작: {targets}")

    def _cb_scan_result(self, msg):
        data    = self._parse(msg)
        status  = data.get("status",         "-")
        targets = data.get("target_objects", [])
        found   = data.get("found_objects",  [])
        ts      = data.get("timestamp",      datetime.now().strftime("%H:%M:%S"))

        status_str = {
            "found"    : "✅ 발견",
            "not_found": "❌ 미발견",
            "cancelled": "🛑 취소됨",
        }.get(status, status)

        self.state.update(
            scan_status=status_str,
            scan_targets=targets,
            scan_found=[obj["label"] for obj in found],
            scan_time=ts,
        )
        self.state.add_log(
            "SCAN", f"{status_str} — {[o['label'] for o in found]}")

    def _cb_object_not_found(self, msg):
        """물체 미감지 → Vision 패널을 미감지 상태로 업데이트"""
        data      = self._parse(msg)
        not_found = data.get("not_found", [])
        ts        = datetime.now().strftime("%H:%M:%S")

        self.state.update(
            vision_status    ="미감지",
            not_found_objects=not_found,
            not_found_time   =ts,
            # 이전 선택 물체 초기화
            selected_object  ="-",
            selected_conf    ="-",
            selected_box     ="-",
            selected_time    ="-",
        )
        self.state.add_log("VISION", f"⚠️  미감지: {not_found}")

    def _cb_selected_object(self, msg):
        """SelectedObject 커스텀 메시지 수신 → Vision 패널 업데이트"""
        try:
            label = msg.label
            conf  = float(msg.confidence)
            bbox  = list(msg.box)
            ts    = datetime.now().strftime("%H:%M:%S")

            self.state.update(
                vision_status    ="감지됨",
                selected_object  =label,
                selected_conf    =f"{conf:.2f}",
                selected_box     =f"[{bbox[0]},{bbox[1]},{bbox[2]},{bbox[3]}]",
                selected_time    =ts,
                # 미감지 목록 초기화
                not_found_objects=[],
                not_found_time   ="-",
            )
            self.state.add_log(
                "VISION",
                f"✅ selected: {label} (conf={conf:.2f}) box={bbox}")

        except Exception as e:
            self.state.add_log("VISION", f"selected_object 오류: {e}")


# ══════════════════════════════════════════════════════════════════════════════
# TUI 렌더러
# ══════════════════════════════════════════════════════════════════════════════

URGENCY_COLOR = {
    "high"  : "bold red",
    "normal": "yellow",
    "low"   : "green",
}

INTENT_COLOR = {
    "bring_water"   : "cyan",
    "bring_medicine": "magenta",
    "bring_food"    : "green",
    "emergency"     : "bold red",
    "cancel"        : "red",
    "weather_query" : "blue",
    "general_query" : "white",
    "unknown"       : "dim white",
}

LOG_COLOR = {
    "STT"   : "cyan",
    "INTENT": "magenta",
    "TTS"   : "yellow",
    "VISION": "green",
    "SCAN"  : "blue",
}


def render(state: JARVISState) -> Layout:
    layout = Layout()
    layout.split_column(
        Layout(name="header", size=3),
        Layout(name="main"),
        Layout(name="log", size=12),
    )
    layout["main"].split_row(
        Layout(name="left"),
        Layout(name="right"),
    )
    layout["left"].split_column(
        Layout(name="stt",    size=6),
        Layout(name="intent", size=16),
    )
    layout["right"].split_column(
        Layout(name="topics", size=7),
        Layout(name="scan",   size=7),
        Layout(name="vision", size=8),
    )

    # ── 헤더 ──────────────────────────────────────────────────────────────
    now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    layout["header"].update(Panel(
        Text(f"🤖  JARVIS TUI Monitor  |  {now}", justify="center",
             style="bold white"),
        style="bold blue",
        box=box.HEAVY,
    ))

    # ── STT 결과 ──────────────────────────────────────────────────────────
    stt_table = Table(box=box.SIMPLE, show_header=False, expand=True)
    stt_table.add_column("Key",   style="dim", width=10)
    stt_table.add_column("Value", style="bold white")
    stt_table.add_row("텍스트", f'"{state.stt_text}"')
    stt_table.add_row("시각",   state.stt_time)
    layout["stt"].update(Panel(
        stt_table, title="🗣️  STT 결과", border_style="cyan"))

    # ── Intent Engine 결과 ────────────────────────────────────────────────
    intent_color  = INTENT_COLOR.get(state.intent, "white")
    urgency_color = URGENCY_COLOR.get(state.urgency, "white")

    intent_table = Table(box=box.SIMPLE, show_header=False, expand=True)
    intent_table.add_column("Key",  style="dim", width=14)
    intent_table.add_column("Value")
    intent_table.add_row(
        "Intent",
        Text(state.intent, style=f"bold {intent_color}"))
    intent_table.add_row(
        "Target",
        Text(str(state.target_object), style="bold white"))
    intent_table.add_row(
        "Urgency",
        Text(state.urgency, style=urgency_color))
    intent_table.add_row(
        "Confidence",
        Text(f"{state.confidence:.2f}", style="bold yellow"))

    if state.scores:
        intent_table.add_row("", "")
        mx = max(state.scores.values(), default=1)
        for act, sc in sorted(state.scores.items(), key=lambda x: -x[1]):
            bar_len = int(sc / max(mx, 1) * 15)
            bar     = "█" * bar_len + "░" * (15 - bar_len)
            color   = intent_color if act == state.intent else "dim"
            mark    = " ◀" if act == state.intent else ""
            intent_table.add_row(
                f"{act[:14]}",
                Text(f"{sc:>3}  {bar}{mark}", style=color))

    if state.reason_log:
        intent_table.add_row("", "")
        for r in state.reason_log[:3]:
            intent_table.add_row("", Text(f"• {r}", style="dim white"))

    intent_table.add_row("", "")
    intent_table.add_row(
        "🔊 TTS",
        Text(state.tts_message, style="bold yellow"))

    layout["intent"].update(Panel(
        intent_table,
        title=f"🧠  Intent Engine  [{state.intent_time}]",
        border_style="magenta"))

    # ── 토픽 발행 현황 ────────────────────────────────────────────────────
    topic_table = Table(box=box.SIMPLE, show_header=False, expand=True)
    topic_table.add_column("토픽",  style="dim", width=18)
    topic_table.add_column("값",    style="white")
    topic_table.add_row("/voice_command",  str(state.last_voice_cmd)[:30])
    topic_table.add_row("/voice_intent",   str(state.last_voice_intent)[:30])
    topic_table.add_row("/tts_output",     str(state.last_tts)[:30])
    layout["topics"].update(Panel(
        topic_table,
        title="📡  토픽 발행 현황",
        border_style="blue"))

    # ── 스캔 상태 ─────────────────────────────────────────────────────────
    scan_color = (
        "green"  if "발견" in state.scan_status else
        "red"    if "미발견" in state.scan_status or
                    "취소"   in state.scan_status else
        "yellow" if "탐색"   in state.scan_status else
        "dim"
    )
    scan_table = Table(box=box.SIMPLE, show_header=False, expand=True)
    scan_table.add_column("Key",  style="dim", width=10)
    scan_table.add_column("Value")
    scan_table.add_row(
        "상태",
        Text(state.scan_status, style=f"bold {scan_color}"))
    scan_table.add_row("탐색 대상", str(state.scan_targets))
    scan_table.add_row("발견 물체", str(state.scan_found))
    scan_table.add_row("시각",     state.scan_time)
    layout["scan"].update(Panel(
        scan_table,
        title="🔍  스캔 상태",
        border_style="blue"))

    # ── Vision 감지 현황 ──────────────────────────────────────────────────
    # vision_status에 따라 색상 결정
    if state.vision_status == "감지됨":
        vision_border = "green"
        status_color  = "bold green"
        status_icon   = "✅"
    elif state.vision_status == "미감지":
        vision_border = "red"
        status_color  = "bold red"
        status_icon   = "❌"
    else:
        vision_border = "dim"
        status_color  = "dim"
        status_icon   = "⏳"

    vision_table = Table(box=box.SIMPLE, show_header=False, expand=True)
    vision_table.add_column("Key",  style="dim", width=12)
    vision_table.add_column("Value")

    # 상태 표시
    vision_table.add_row(
        "상태",
        Text(f"{status_icon} {state.vision_status}", style=status_color))

    if state.vision_status == "감지됨":
        # 감지된 물체 정보
        vision_table.add_row(
            "물체",
            Text(state.selected_object, style="bold green"))
        vision_table.add_row(
            "신뢰도",
            Text(state.selected_conf, style="green"))
        vision_table.add_row(
            "BBox",
            Text(state.selected_box, style="dim white"))
        vision_table.add_row(
            "시각",
            state.selected_time)

    elif state.vision_status == "미감지":
        # 미감지 물체 정보
        vision_table.add_row(
            "미감지",
            Text(str(state.not_found_objects), style="bold red"))
        vision_table.add_row(
            "시각",
            state.not_found_time)
        vision_table.add_row(
            "",
            Text("→ 스캔 요청 발행됨", style="yellow"))

    else:
        vision_table.add_row("", Text("음성 명령 대기 중...", style="dim"))

    layout["vision"].update(Panel(
        vision_table,
        title="👁️  Vision 감지 현황",
        border_style=vision_border))

    # ── 로그 ──────────────────────────────────────────────────────────────
    log_table = Table(
        box=box.SIMPLE, show_header=True, expand=True,
        header_style="bold white")
    log_table.add_column("시각",   width=12, style="dim")
    log_table.add_column("레벨",   width=8)
    log_table.add_column("메시지")

    for ts, level, message in reversed(list(state.log_entries)):
        color = LOG_COLOR.get(level, "white")
        log_table.add_row(
            ts,
            Text(f"[{level}]", style=f"bold {color}"),
            message,
        )

    layout["log"].update(Panel(
        log_table,
        title="📋  실시간 로그",
        border_style="dim white"))

    return layout


# ══════════════════════════════════════════════════════════════════════════════
# 메인
# ══════════════════════════════════════════════════════════════════════════════

def main():
    rclpy.init()
    state = JARVISState()
    node  = JARVISMonitorNode(state)

    spin_thread = threading.Thread(
        target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    console = Console()
    console.print("\n🤖 [bold blue]JARVIS TUI Monitor 시작[/bold blue]")
    console.print("   Ctrl+C 로 종료\n")

    try:
        with Live(render(state), console=console,
                  refresh_per_second=4, screen=True) as live:
            while True:
                live.update(render(state))
                time.sleep(0.25)

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        console.print("\n👋 [bold]종료합니다.[/bold]")


if __name__ == "__main__":
    main()