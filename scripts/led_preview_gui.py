#!/usr/bin/env python3
# /// script
# requires-python = ">=3.11"
# dependencies = ["pyserial>=3.5"]
# ///
"""Button-only LED diagnostics over the tracker's application CDC console."""

from __future__ import annotations

import codecs
import re
import sys
import time

try:
    import tkinter as tk
    from tkinter import font as tkfont, ttk
except ImportError:
    tk = ttk = tkfont = None

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = list_ports = None

SERIAL_HINT = (
    "缺少 pyserial：从仓库根目录运行 uv run scripts/led_preview_gui.py，"
    "或用当前 Python 执行 python -m pip install 'pyserial>=3.5'。"
)
TK_HINT = (
    "需要带 Tk 的 Python 和图形桌面。Linux 可安装 python3-tk，再运行 "
    "uv run --python /usr/bin/python3 scripts/led_preview_gui.py；"
    "WSL 请启用 WSLg（或配置 X server / DISPLAY）；"
    "Windows 请安装 Python 的 Tcl/Tk 组件。"
)
ANSI = re.compile(r"\x1b\[[0-?]*[ -/]*[@-~]|\x1b\][^\x07\x1b]*(?:\x07|\x1b\\)")
CONTROL = re.compile(r"[\x00-\x08\x0b-\x1f\x7f]")
HEADER = re.compile(r"board=(?P<board>\S+) degradation=(?P<degradation>\S+)")
ROW = re.compile(
    r"(?P<name>[a-z][a-z0-9_]{0,63}) extent=(?P<extent>finite|loop|hold) "
    r"style=(?P<style>\S+) role=(?P<role>\S+) level=(?P<level>\S+) "
    r"envelope_ms=(?P<envelope_ms>[0-9]+) board=(?P<board>\S+)"
)
# Human labels only: firmware's led list is the sole behavior catalogue.
LABELS = {
    "input_ack": "输入确认", "accepted": "操作已接受", "stage_ack": "步骤确认",
    "cancelled": "已取消", "success": "成功",
    "rejected": "已拒绝", "failed": "失败", "partial": "部分完成",
    "applied_not_saved": "已应用未保存", "low_battery": "低电量",
    "identify": "寻找设备", "exit_pending": "退出待定", "wait_still": "等待静止",
    "collect_still": "静止采集", "charged": "充电完成",
    "external_power_unknown": "外部供电状态未知", "pairing": "配对",
    "reconnecting": "重新连接", "unpaired_idle": "未配对空闲", "ready": "就绪",
    "wait_move": "等待运动", "collect_move": "运动采集", "processing": "处理中",
    "ota_active": "OTA 外观", "heated_active": "加热外观", "maintenance": "维护",
    "test_active": "测试外观", "initializing": "初始化", "charging": "充电中",
    "sensor_missing": "未检测到传感器", "sensor_fault": "传感器故障",
    "blocking_fault": "系统／阻断故障", "safety_fault": "安全保护",
    "button_hold": "按键长按提示", "manual_exit": "手动退出渐暗",
}
# Describe shared waveform styles, never infer pulse counts from semantic names.
STYLE_LABELS = {
    "short": "短亮", "success": "四短闪", "double": "短双亮",
    "link": "稀疏短亮", "ready": "就绪心跳", "solid": "常亮",
    "move": "短双亮循环", "breathe": "呼吸", "low": "低电量提醒",
    "warning": "长双亮循环", "exit": "退出渐暗",
    "manual_exit": "黑场／退出渐暗", "button_hold": "按下全亮／长按退出提示",
}
COLORS = {"red": "红色", "green": "绿色", "blue": "蓝色", "amber": "琥珀色", "white": "白色"}
GROUPS = {
    "finite": "一次性：播放完整包络，不使用时长",
    "loop": "循环：使用上方时长",
    "hold": "保持：使用上方时长",
}

# Business context only. Firmware discovery supplies every waveform and extent.
# Manual temperature-point acquisition uses the same IMU producer path.
IMU_STAGES = (
    ("accepted", "请求已接受", "accepted", "设备已接受 IMU 零偏校准请求。"),
    ("settle", "放稳设备", "wait_still", "放在稳定表面，等待设备确认静止；不要旋转。"),
    ("sample", "保持静止", "collect_still", "采集加速度计／陀螺仪零偏期间继续保持静止。"),
    ("captured", "采集完成", "stage_ack", "采样阶段已完成；这不是最终成功。"),
    ("apply", "处理／应用", "processing", "等待修整、验证和实际应用完成。"),
    ("saved", "校准成功", "success", "结果已实际应用，并完成持久化。"),
    ("storage", "已应用但未保存", "applied_not_saved", "校准已生效，但存储失败；重启后可能丢失。"),
    ("failed", "校准失败", "failed", "运动、采样、温度、传感器或候选结果检查未通过。"),
    ("rejected", "请求被拒绝", "rejected", "忙碌或无效请求在开始前被拒绝。"),
    ("cancelled", "已取消", "cancelled", "原校准请求已失效或被重置，不是成功。"),
)
FUNCTIONS = {
    "connection": {
        "title": "连接／配对",
        "description": "预览实际连接事实与配对结果；这些是不同条件，不是固定顺序。外部供电时充电背景优先。",
        "stages": (
            ("pairing", "寻找接收器", "pairing", "正在执行配对搜索。"),
            ("unpaired", "未配对空闲", "unpaired_idle", "需要无线连接，但没有已保存的配对地址。"),
            ("reconnect", "重新连接", "reconnecting", "已有配对信息，当前无线连接尚未恢复。"),
            ("waiting", "等待可用输出", "processing", "需要无线连接，但尚未满足就绪条件。"),
            ("ready", "连接／输出就绪", "ready", "无线健康、传感器输出就绪，且无故障、维护或 OTA。"),
            ("paired", "配对已保存", "success", "配对地址及频道已成功存储。"),
            ("storage", "配对成功但保存失败", "partial", "已配对，但地址／频道持久化失败。"),
            ("timeout", "配对超时", "failed", "配对超时，设备请求退出；预览不会真的退出。"),
        ),
    },
    "power": {
        "title": "充电／低电量",
        "description": "预览电源状态，不启动充电，也不改变电池状态。仅电池供电本身没有专用提示。",
        "stages": (
            ("charging", "正在充电", "charging", "充电器报告正在充电。"),
            ("full", "充电完成", "charged", "充电器报告已充满。"),
            ("unknown", "外部供电状态未知", "external_power_unknown", "已接入外部电源，但充电／满电状态未知。"),
            ("low", "低电量警告", "low_battery", "当前低电量条件下的周期提醒；不会真的关机。"),
        ),
    },
    "button": {
        "title": "物理按键／松手退出",
        "description": "按下全亮，满 1 秒后黑场再渐暗；合格松手承接当前阶段，不重播。实际短按 N 次的计数列车须由物理输入触发，不能用单个样式代替。",
        "stages": (
            ("hold", "按下／长按提示", "button_hold", "立即峰值；1 秒后黑 250 毫秒，再渐暗 1 秒。继续按住则在末尾黑场后慢闪。"),
            ("release", "合格松手／承接退出", "manual_exit", "单独预览黑 250 毫秒后从峰值渐暗 1 秒，再留黑；真实松手承接已有阶段，安全等待不变。"),
            ("cancel", "超长按取消", "cancelled", "超过原有长按取消阈值，不执行退出。"),
            ("reject", "退出被拒绝", "rejected", "OTA、测试或退出资格检查拒绝了请求。"),
            ("failed", "退出失败", "failed", "已接受的退出请求在渐暗后失败。"),
        ),
    },
    "exit": {
        "title": "命令退出／引导程序交接",
        "description": "仅预览命令关机及引导程序交接的提示，绝不发送关机、重启或 DFU 命令；串口始终使用 115200。",
        "stages": (
            ("accepted", "请求已接受", "accepted", "命令退出／交接请求已接受时的确认提示。"),
            ("pending", "命令关机待执行", "exit_pending", "关机前仍可撤回阶段的退出提示。"),
            ("handoff", "引导程序交接处理中", "processing", "引导程序交接准备进行中；预览不改变启动模式。"),
            ("rejected", "退出请求被拒绝", "rejected", "退出资格检查未通过。"),
            ("failed", "退出／交接失败", "failed", "退出或设置引导模式失败；不存在退出后的成功灯态。"),
        ),
    },
    "identify": {
        "title": "寻找设备",
        "description": "寻找设备使用现有 6 秒灯光窗口，不是校准准备阶段。",
        "stages": (
            ("identify", "寻找设备提示", "identify", "预览显式寻找追踪器／远程 ping 的灯光提示。"),
        ),
    },
    "imu": {
        "title": "IMU 零偏校准",
        "description": "在稳定表面保持静止，采集并应用加速度计／陀螺仪零偏。各阶段独立预览，不会启动校准。",
        "stages": IMU_STAGES,
    },
    "acc": {
        "title": "ACC 多姿态校准",
        "description": "目标是 18 个不同姿态，不是固定六面。换姿态后静止采样；重复姿态继续等待，不算拒绝。",
        "stages": (
            ("accepted", "请求已接受", "accepted", "设备已接受 ACC 姿态校准请求。"),
            ("initial", "初始等待静止", "wait_still", "请求接受后的初始静止提示。"),
            ("reposition", "换一个新姿态", "wait_move", "转到新的不同方向，再放稳；避免重复或近似姿态。"),
            ("sample", "保持当前姿态", "collect_still", "当前姿态被接受，保持静止完成 500 个样本采集。"),
            ("pose", "本姿态已采集", "stage_ack", "当前姿态记录完成；还不是最终校准成功。"),
            ("fit", "拟合／应用", "processing", "等待姿态数据拟合并应用。"),
            ("saved", "校准成功", "success", "完整校准结果已应用并保存。"),
            ("partial", "部分姿态校准成功", "partial", "等待超时但已有至少 6 个姿态，部分拟合已实际应用。"),
            ("storage", "已应用但未保存", "applied_not_saved", "结果生效，但存储失败。"),
            ("failed", "校准失败", "failed", "采样、拟合或应用检查失败。"),
            ("rejected", "请求被拒绝", "rejected", "忙碌或无效请求在开始前被拒绝。"),
            ("cancelled", "已取消", "cancelled", "原校准请求失效或被重置。"),
        ),
    },
    "mag": {
        "title": "MAG 手动磁校准",
        "description": "向各个方向旋转追踪器采集硬铁／软铁校准数据；在线磁校准没有独立可点选的 LED 流程。",
        "stages": (
            ("accepted", "请求已接受", "accepted", "设备已接受手动磁校准请求。"),
            ("rotate", "向各方向旋转", "collect_move", "通过各个方向收集足够覆盖；不足时继续旋转，不是拒绝。"),
            ("coverage", "覆盖／质量达标", "stage_ack", "采集覆盖及质量达到要求，尚未最终应用。"),
            ("processing", "准备／拟合／应用", "processing", "开始时准备；采集后拟合、验证并应用。内部准备不使用寻找设备提示。"),
            ("saved", "校准成功", "success", "磁校准已实际应用并保存。"),
            ("storage", "已应用但未保存", "applied_not_saved", "磁校准生效，但存储失败。"),
            ("failed", "校准失败", "failed", "候选拟合、验证或应用失败。"),
            ("rejected", "请求被拒绝", "rejected", "忙碌或无效请求在开始前被拒绝。"),
            ("cancelled", "已取消", "cancelled", "原校准请求失效或被重置。"),
        ),
    },
    "sensitivity": {
        "title": "陀螺仪灵敏度（X／Y／Z 轴）",
        "description": "先静止测零偏，再绕所选轴转动指定整圈数后停下。三轴共享相同灯态；不要混成各方向旋转。",
        "stages": (
            ("accepted", "请求已接受", "accepted", "所选 X／Y／Z 轴灵敏度校准请求已接受。"),
            ("settle", "放稳设备", "wait_still", "测量当前陀螺仪零偏前保持静止。"),
            ("bias", "静止测零偏", "collect_still", "零偏采样期间继续保持静止。"),
            ("bias_done", "零偏采集完成", "stage_ack", "零偏测量完成；接下来才进行选定轴旋转。"),
            ("start_spin", "开始绕所选轴旋转", "wait_move", "绕指定 X／Y／Z 轴开始转动。"),
            ("turns", "完成指定整圈后停下", "collect_move", "沿同一轴完成请求的整圈数后停止；两种方向均可，尽量减少偏轴运动。"),
            ("validate", "验证／计算比例", "processing", "录制完成，验证角度、轴向和灵敏度比例。"),
            ("saved", "轴灵敏度已保存", "success", "所选轴比例已实际更新并持久化。"),
            ("storage", "已应用但未保存", "applied_not_saved", "比例已应用，但存储失败。"),
            ("failed", "采样／旋转验证失败", "failed", "静止、采样、起转超时、圈数、偏轴或比例检查未通过。"),
            ("rejected", "请求被拒绝", "rejected", "无效轴、零圈数或忙碌请求在开始前被拒绝。"),
            ("cancelled", "已取消／被替换", "cancelled", "显式设置／重置比例或请求失效，替代了原测量。"),
        ),
    },
    "tcal": {
        "title": "手动温度点采集（IMU 校准）",
        "description": "启用自动温补时，在当前温度下做 IMU 静止校准，可能增加／混合温度点；已有足够覆盖时不会添加点。",
        "stages": IMU_STAGES,
    },
    "heated": {
        "title": "加热温度校准",
        "description": "保持设备静止，由设备控制加热。运动时暂停采样但灯态仍是加热中；无需手动加热或旋转。",
        "stages": (
            ("accepted", "请求已接受", "accepted", "加热校准请求已接受。"),
            ("reserved", "已预约／待运行", "processing", "资源已预约，但加热校准尚未实际运行。"),
            ("running", "加热采集／收尾", "heated_active", "实际加热运行，或最后应用／保存交接；移动会暂停采样。"),
            ("saved", "完整温度校准成功", "success", "完整温度扫描结果已发布并保存。"),
            ("partial", "保留部分温度点", "partial", "用户停止且已有有效温度槽，部分结果已发布。"),
            ("storage", "已应用但未保存", "applied_not_saved", "结果已应用，但存储确认失败。"),
            ("cancelled", "无结果取消", "cancelled", "用户、传感器或电源停止，未发布可用部分结果。"),
            ("failed", "加热校准失败", "failed", "硬件、保护、覆盖或结果检查失败。"),
            ("rejected", "请求被拒绝", "rejected", "目标无效、设备忙碌或功能不可用。"),
            ("safety", "安全保护", "safety_fault", "实际保护记录或关闭加热失败；预览不会启用加热器。"),
        ),
    },
    "ota": {
        "title": "OTA 固件更新",
        "description": "只预览 OTA 生命周期灯态，不传输、验证、写入或重启固件；实际流程没有独立的成功灯态。",
        "stages": (
            ("accepted", "更新已接受", "accepted", "OTA 会话已接受。"),
            ("active", "接收／验证／写入", "ota_active", "OTA 处于接收、验证、写入或等待重启状态。"),
            ("rejected", "更新请求被拒绝", "rejected", "无效或尚未接受的开始／结束／取消请求。"),
            ("failed", "更新错误", "failed", "已接受的 OTA 会话进入错误状态。"),
            ("cancelled", "更新已取消", "cancelled", "已接受的会话完成取消。"),
        ),
    },
    "maintenance": {
        "title": "清除校准／重新初始化",
        "description": "有限维护任务，例如清除 IMU／ACC／MAG 校准或重新初始化；这里只显示提示，不删除任何数据。",
        "stages": (
            ("accepted", "请求已接受", "accepted", "清除／重置请求已接受。"),
            ("active", "维护处理中", "maintenance", "执行有限的清除、重置、传感器或融合重新初始化任务。"),
            ("saved", "重置完成", "success", "请求的重置已生效并完成存储。"),
            ("storage", "已重置但未保存", "applied_not_saved", "重置已生效，但持久化失败。"),
            ("cancelled", "重置已取消", "cancelled", "原请求失效，未完成原维护结果。"),
            ("rejected", "请求被拒绝", "rejected", "请求在开始前被拒绝。"),
        ),
    },
    "diagnostics": {
        "title": "测试／原始数据／诊断模式",
        "description": "测试、原始／批量采集、诊断、无线捕获或无线禁用的持续状态；预览不启用这些模式。",
        "stages": (
            ("active", "测试／诊断活动中", "test_active", "真实持续维护事实使用测试灯态，而不是有限维护任务灯态。"),
            ("changed", "模式切换成功", "success", "有结果提示的原始采集／无线捕获命令已成功切换。"),
            ("rejected", "模式请求被拒绝", "rejected", "对应模式命令的资格检查拒绝了请求。"),
        ),
    },
    "faults": {
        "title": "启动／传感器／系统／安全故障",
        "description": "预览当前条件，不制造故障，不解除保护，也不暗示故障恢复后一定出现成功提示。",
        "stages": (
            ("startup", "传感器初始化", "initializing", "传感器启动和初始化期间的提示。"),
            ("missing", "未检测到传感器", "sensor_missing", "缺少传感器硬件的类型化故障。"),
            ("sensor", "传感器故障", "sensor_fault", "其他活动传感器错误。"),
            ("system", "系统阻断故障", "blocking_fault", "活动系统错误阻断普通状态。"),
            ("safety", "安全保护", "safety_fault", "当前安全条件或已有保护记录。"),
        ),
    },
}


class LedPreviewApp:
    """Single-threaded Tk app; nonblocking reads and session-scoped callbacks."""

    def __init__(self, root):
        if tk is None:
            raise RuntimeError(TK_HINT)
        if serial is None:
            raise RuntimeError(SERIAL_HINT)
        self.root = root
        self.serial_port = None
        self.styles = {}
        self.style_buttons = {}
        self.function_buttons = {}
        self.function_stages = {}
        self.function_metadata_labels = {}
        self.function_action_labels = {}
        self.color_buttons = {}
        self._session = 0
        self._poll_job = self._probe_job = None
        self._ready = False
        self._decoder = codecs.getincrementaldecoder("utf-8")(errors="replace")
        self._partial = ""
        self._discard_line = False
        self.port_var = tk.StringVar()
        self.duration_var = tk.StringVar(value="10000")
        self.status_var = tk.StringVar(value="未连接")
        self.board_var = tk.StringVar(value="板型与样式由设备 led list 返回")
        self.function_var = tk.StringVar(value=next(iter(FUNCTIONS.values()))["title"])
        self.function_hint_var = tk.StringVar()
        root.title("SlimeNRF LED 样式预览")
        root.geometry("1000x780")
        root.minsize(700, 600)
        root.protocol("WM_DELETE_WINDOW", self.close)
        self._build_ui()
        self.refresh_ports()
        self._set_controls()

    def _build_ui(self):
        body = ttk.Frame(self.root, padding=10)
        body.pack(fill="both", expand=True)
        body.columnconfigure(0, weight=1)
        body.rowconfigure(4, weight=3)
        body.rowconfigure(6, weight=2)
        connection = ttk.Frame(body)
        connection.grid(row=0, column=0, sticky="ew")
        connection.columnconfigure(1, weight=1)
        ttk.Label(connection, text="串口（可手动输入 COM / 路径）").grid(row=0, column=0)
        self.port_combo = ttk.Combobox(connection, textvariable=self.port_var, width=32)
        self.port_combo.grid(row=0, column=1, sticky="ew", padx=8)
        self.refresh_button = ttk.Button(connection, text="刷新", command=self.refresh_ports)
        self.refresh_button.grid(row=0, column=2, padx=4)
        self.connect_button = ttk.Button(connection, text="连接", command=self.toggle_connection)
        self.connect_button.grid(row=0, column=3, padx=4)
        ttk.Label(connection, text="115200 baud · DTR 高").grid(row=0, column=4, padx=4)
        ttk.Label(
            body,
            text="固件需要 LED_DEBUG；仅模拟灯光外观，不执行加热、OTA、关机等操作。停止预览恢复真实灯态，不一定熄灯。",
            wraplength=960,
        ).grid(row=1, column=0, sticky="w", pady=(8, 6))
        tools = ttk.Frame(body)
        tools.grid(row=2, column=0, sticky="ew")
        ttk.Label(tools, text="循环 / 保持 / 颜色时长（ms）").pack(side="left")
        self.duration_spinbox = ttk.Spinbox(
            tools, from_=250, to=30000, increment=250, textvariable=self.duration_var, width=8
        )
        self.duration_spinbox.pack(side="left", padx=6)
        self.stop_button = ttk.Button(tools, text="停止预览", command=lambda: self._send("led stop"))
        self.status_button = ttk.Button(tools, text="查询状态", command=lambda: self._send("led status"))
        self.reload_button = ttk.Button(tools, text="重新读取样式", command=self.reload_styles)
        for button in (self.stop_button, self.status_button, self.reload_button):
            button.pack(side="left", padx=3)
        colors = ttk.Frame(body)
        colors.grid(row=3, column=0, sticky="ew", pady=6)
        ttk.Label(colors, text="物理颜色（可能不支持）：").pack(side="left")
        for name, label in COLORS.items():
            button = ttk.Button(colors, text=label, command=lambda n=name: self.play_color(n))
            button.pack(side="left", padx=3)
            self.color_buttons[name] = button
        self.notebook = ttk.Notebook(body)
        self.notebook.grid(row=4, column=0, sticky="nsew")
        self.functions_tab = ttk.Frame(self.notebook, padding=6)
        self.advanced_tab = ttk.Frame(self.notebook, padding=6)
        self.notebook.add(self.functions_tab, text="功能 Functions")
        self.notebook.add(self.advanced_tab, text="高级 Advanced · 全部灯效条目")
        self.functions_tab.columnconfigure(0, weight=1)
        self.functions_tab.rowconfigure(3, weight=1)
        selector = ttk.Frame(self.functions_tab)
        selector.grid(row=0, column=0, sticky="ew", pady=(0, 6))
        selector.columnconfigure(1, weight=1)
        ttk.Label(selector, text="选择要了解的功能：").grid(row=0, column=0, padx=(0, 6))
        self.function_combo = ttk.Combobox(
            selector, textvariable=self.function_var, state="readonly",
            values=[function["title"] for function in FUNCTIONS.values()],
        )
        self.function_combo.grid(row=0, column=1, sticky="ew")
        self.function_combo.bind("<<ComboboxSelected>>", self._show_function)
        ttk.Label(
            self.functions_tab, textvariable=self.function_hint_var, wraplength=930,
        ).grid(row=1, column=0, sticky="w", pady=(0, 6))
        ttk.Label(
            self.functions_tab,
            text="各阶段／结果可独立点选，只预览灯光；不是自动流程，不会执行校准、OTA 或关机。",
            wraplength=930,
        ).grid(row=2, column=0, sticky="w", pady=(0, 6))
        function_scroll = ttk.Frame(self.functions_tab)
        function_scroll.grid(row=3, column=0, sticky="nsew")
        function_scroll.rowconfigure(0, weight=1)
        function_scroll.columnconfigure(0, weight=1)
        self.function_canvas = tk.Canvas(function_scroll, highlightthickness=0)
        self.function_canvas.grid(row=0, column=0, sticky="nsew")
        function_scrollbar = ttk.Scrollbar(
            function_scroll, orient="vertical", command=self.function_canvas.yview,
        )
        function_scrollbar.grid(row=0, column=1, sticky="ns")
        self.function_canvas.configure(yscrollcommand=function_scrollbar.set)
        self.function_frame = ttk.Frame(self.function_canvas)
        self._function_window = self.function_canvas.create_window(
            (0, 0), window=self.function_frame, anchor="nw",
        )
        self.function_frame.columnconfigure(0, weight=1)
        self.function_frame.bind(
            "<Configure>",
            lambda e: self.function_canvas.configure(scrollregion=self.function_canvas.bbox("all")),
        )
        self.function_canvas.bind("<Configure>", self._resize_functions)
        self.advanced_tab.rowconfigure(0, weight=1)
        self.advanced_tab.columnconfigure(0, weight=1)
        scroll_area = ttk.Frame(self.advanced_tab)
        scroll_area.grid(row=0, column=0, sticky="nsew")
        scroll_area.rowconfigure(0, weight=1)
        scroll_area.columnconfigure(0, weight=1)
        self.style_canvas = tk.Canvas(scroll_area, highlightthickness=0)
        self.style_canvas.grid(row=0, column=0, sticky="nsew")
        scrollbar = ttk.Scrollbar(scroll_area, orient="vertical", command=self.style_canvas.yview)
        scrollbar.grid(row=0, column=1, sticky="ns")
        self.style_canvas.configure(yscrollcommand=scrollbar.set)
        self.style_frame = ttk.Frame(self.style_canvas)
        self._style_window = self.style_canvas.create_window((0, 0), window=self.style_frame, anchor="nw")
        self.style_frame.columnconfigure(0, weight=1)
        self._groups = {}
        for row, (extent, title) in enumerate(GROUPS.items()):
            group = ttk.LabelFrame(self.style_frame, text=title, padding=6)
            group.grid(row=row, column=0, sticky="ew", pady=(0, 6))
            self._groups[extent] = group
        self.style_frame.bind("<Configure>", lambda e: self.style_canvas.configure(scrollregion=self.style_canvas.bbox("all")))
        self.style_canvas.bind("<Configure>", self._resize_styles)
        for event in ("<MouseWheel>", "<Button-4>", "<Button-5>"):
            self.root.bind(event, self._scroll_previews, add="+")
        summary = ttk.Frame(body)
        summary.grid(row=5, column=0, sticky="ew", pady=6)
        ttk.Label(summary, textvariable=self.status_var).pack(side="left")
        ttk.Label(summary, textvariable=self.board_var).pack(side="left", padx=12)
        self.clear_button = ttk.Button(summary, text="清空日志", command=self.clear_log)
        self.clear_button.pack(side="right")
        log_area = ttk.Frame(body)
        log_area.grid(row=6, column=0, sticky="nsew")
        log_area.rowconfigure(0, weight=1)
        log_area.columnconfigure(0, weight=1)
        self.log_text = tk.Text(log_area, state="disabled", height=10, wrap="word", font="TkFixedFont")
        self.log_text.grid(row=0, column=0, sticky="nsew")
        log_scroll = ttk.Scrollbar(log_area, command=self.log_text.yview)
        log_scroll.grid(row=0, column=1, sticky="ns")
        self.log_text.configure(yscrollcommand=log_scroll.set)
        self._show_function()

    def _resize_styles(self, event):
        self.style_canvas.itemconfigure(self._style_window, width=event.width)
        self._layout_styles()

    def _resize_functions(self, event):
        self.function_canvas.itemconfigure(self._function_window, width=event.width)
        wraplength = max(180, event.width - 240)
        for label in (*self.function_action_labels.values(), *self.function_metadata_labels.values()):
            label.configure(wraplength=wraplength)

    def _layout_styles(self):
        columns = max(1, min(4, self.style_canvas.winfo_width() // 290))
        for extent, group in self._groups.items():
            for column in range(4):
                group.columnconfigure(column, weight=1 if column < columns else 0)
            buttons = [b for name, b in self.style_buttons.items() if self.styles[name]["extent"] == extent]
            for index, button in enumerate(buttons):
                button.grid(row=index // columns, column=index % columns, sticky="ew", padx=3, pady=3)

    def _scroll_previews(self, event):
        for canvas in (self.function_canvas, self.style_canvas):
            if canvas.winfo_viewable() and (
                canvas.winfo_rooty() <= event.y_root < canvas.winfo_rooty() + canvas.winfo_height()
                and canvas.winfo_rootx() <= event.x_root < canvas.winfo_rootx() + canvas.winfo_width()
            ):
                direction = -1 if event.num == 4 or getattr(event, "delta", 0) > 0 else 1
                canvas.yview_scroll(direction * 3, "units")
                return "break"

    def _show_function(self, event=None):
        self.function_id, function = next(
            (key, value) for key, value in FUNCTIONS.items()
            if value["title"] == self.function_var.get()
        )
        self.function_hint_var.set(function["description"])
        for widget in self.function_frame.winfo_children():
            widget.destroy()
        self.function_buttons.clear()
        self.function_stages.clear()
        self.function_metadata_labels.clear()
        self.function_action_labels.clear()
        for row, (stage_id, label, semantic, action) in enumerate(function["stages"]):
            self.function_stages[stage_id] = {"label": label, "semantic": semantic, "action": action}
            stage = ttk.Frame(self.function_frame, padding=(2, 4))
            stage.grid(row=row, column=0, sticky="ew")
            stage.columnconfigure(1, weight=1)
            button = ttk.Button(stage, text=label, width=22, command=lambda n=semantic: self.play(n))
            button.grid(row=0, column=0, rowspan=2, sticky="ns", padx=(0, 8))
            self.function_buttons[stage_id] = button
            wraplength = max(180, self.function_canvas.winfo_width() - 240)
            action_label = ttk.Label(stage, text=action, wraplength=wraplength)
            action_label.grid(row=0, column=1, sticky="w")
            self.function_action_labels[stage_id] = action_label
            metadata = ttk.Label(stage, wraplength=wraplength)
            metadata.grid(row=1, column=1, sticky="w")
            self.function_metadata_labels[stage_id] = metadata
        self.function_canvas.yview_moveto(0)
        self._update_function_metadata()
        self._set_controls()

    def _update_function_metadata(self):
        for stage_id, stage in self.function_stages.items():
            semantic = stage["semantic"]
            info = self.styles.get(semantic)
            if info is None:
                text = f"{semantic} · 等待设备列表／此固件未提供"
            else:
                style_label = STYLE_LABELS.get(info["style"], info["style"])
                extent_label = "完整一次性包络" if info["extent"] == "finite" else "使用上方时长"
                text = (
                    f"{semantic} · {style_label} ({info['style']}) · {extent_label} · "
                    f"{info['role']} / {info['level']} · {info['envelope_ms']} ms"
                )
            self.function_metadata_labels[stage_id].configure(text=text)

    def _log(self, message):
        self.log_text.configure(state="normal")
        self.log_text.insert("end", f"[{time.strftime('%H:%M:%S')}] {message}\n")
        lines = int(self.log_text.index("end-1c").split(".")[0])
        if lines > 600:
            self.log_text.delete("1.0", f"{lines - 600}.0")
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    def clear_log(self):
        self.log_text.configure(state="normal")
        self.log_text.delete("1.0", "end")
        self.log_text.configure(state="disabled")

    def refresh_ports(self):
        try:
            ports = sorted(list_ports.comports(), key=lambda p: p.device)
            self.port_combo.configure(values=[p.device for p in ports])
            if not self.port_var.get() and ports:
                self.port_var.set(ports[0].device)
            self._log("串口：" + ("；".join(f"{p.device} ({p.description})" for p in ports) or "未发现，可手动输入"))
        except (OSError, serial.SerialException) as exc:
            self._log(f"刷新串口失败：{exc}；仍可手动输入。")

    def _set_controls(self):
        connected = self.serial_port is not None
        self.connect_button.configure(text="断开" if connected else "连接")
        self.port_combo.configure(state="disabled" if connected else "normal")
        for button in (self.stop_button, self.status_button, self.reload_button):
            button.configure(state="normal" if connected else "disabled")
        for button in (*self.color_buttons.values(), *self.style_buttons.values()):
            button.configure(state="normal" if connected and self._ready else "disabled")
        for stage_id, button in self.function_buttons.items():
            available = self.function_stages[stage_id]["semantic"] in self.styles
            button.configure(state="normal" if connected and self._ready and available else "disabled")

    def toggle_connection(self):
        if self.serial_port is None:
            self.connect()
        else:
            self.disconnect()

    def connect(self):
        if self.serial_port is not None:
            return
        port = self.port_var.get().strip()
        if not port:
            self._log("请先选择或输入串口。")
            return
        candidate = None
        try:
            # pySerial defaults DTR high before open; avoid a post-open ioctl
            # (which some virtual ports reject). Never use the 1200-baud DFU touch.
            candidate = serial.Serial(port=None, baudrate=115200, timeout=0, write_timeout=0.2)
            candidate.port = port
            candidate.open()
        except (OSError, ValueError, serial.SerialException) as exc:
            if candidate is not None:
                try:
                    candidate.close()
                except OSError:
                    pass
            self._log(f"连接失败：{exc}。检查串口占用/权限；WSL 需要把 USB 串口接入 Linux。")
            return
        self.serial_port = candidate
        self._session += 1
        session = self._session
        self._log(f"已打开 {port}，115200 baud；等待设备 LED 列表，不代表灯光已显示。")
        self._set_controls()
        self._poll_job = self.root.after(20, lambda: self._poll(session))
        self.reload_styles()

    def _cancel_job(self, attribute):
        job = getattr(self, attribute)
        setattr(self, attribute, None)
        if job is not None:
            try:
                self.root.after_cancel(job)
            except tk.TclError:
                pass

    def _clear_styles(self):
        for button in self.style_buttons.values():
            button.destroy()
        self.style_buttons.clear()
        self.styles.clear()
        self._update_function_metadata()

    def disconnect(self, message="已断开"):
        self._session += 1
        self._cancel_job("_poll_job")
        self._cancel_job("_probe_job")
        port, self.serial_port = self.serial_port, None
        self._ready = False
        self._decoder.reset()
        self._partial = ""
        self._discard_line = False
        self._clear_styles()
        self.board_var.set("板型与样式由设备 led list 返回")
        self.status_var.set("未连接")
        self._set_controls()
        if port is not None:
            try:
                port.close()
            except (OSError, serial.SerialException) as exc:
                self._log(f"关闭串口失败：{exc}")
        if message:
            self._log(message)

    def close(self):
        self.disconnect(message=None)
        self.root.destroy()

    def _send(self, command):
        if self.serial_port is None:
            self._log("未连接，未发送命令。")
            return False
        data = (command + "\n").encode("ascii")
        self._log(f"TX > {command}")
        try:
            if self.serial_port.write(data) != len(data):
                raise OSError("串口未完整写入命令")
        except (OSError, serial.SerialException) as exc:
            self.disconnect(message=f"串口发送失败，已断开：{exc}")
            return False
        return True

    def reload_styles(self):
        if self.serial_port is None:
            return
        self._cancel_job("_probe_job")
        self._clear_styles()
        self._ready = False
        self.board_var.set("等待设备列表…")
        self.status_var.set("已连接 · 等待 LED_DEBUG 响应")
        self._set_controls()
        self._probe_deadline = time.monotonic() + 8.0
        self._probe(self._session)

    def _probe(self, session):
        if session != self._session or self.serial_port is None:
            return
        self._probe_job = None
        if self._ready:
            return
        if time.monotonic() >= self._probe_deadline:
            self.status_var.set("已连接 · LED 列表无响应")
            self._log("8 秒内未收到有效 LED 列表。检查 LED_DEBUG / 应用 CDC 端口；可重新读取样式。")
            return
        if self._send("led list"):
            self._probe_job = self.root.after(500, lambda: self._probe(session))

    def _poll(self, session):
        if session != self._session or self.serial_port is None:
            return
        self._poll_job = None
        try:
            available = self.serial_port.in_waiting
            if available:
                self._receive(self.serial_port.read(min(available, 4096)))
        except (OSError, serial.SerialException) as exc:
            self.disconnect(message=f"串口读取失败，已断开：{exc}")
            return
        if session == self._session and self.serial_port is not None:
            self._poll_job = self.root.after(20, lambda: self._poll(session))

    def _receive(self, data):
        # Decode and strip ANSI only after assembling whole lines, so split
        # UTF-8 / escape sequences remain intact. Drop oversized lines entirely.
        parts = re.split(r"[\r\n]", self._decoder.decode(data))
        for index, part in enumerate(parts):
            if not self._discard_line:
                self._partial += part
                if len(self._partial) > 4096:
                    self._partial = ""
                    self._discard_line = True
            if index < len(parts) - 1:
                if self._discard_line:
                    self._log("RX > [已丢弃超过 4096 字符的行]")
                elif self._partial:
                    self._line(CONTROL.sub("", ANSI.sub("", self._partial)))
                self._partial = ""
                self._discard_line = False

    def _line(self, line):
        self._log(f"RX > {line}")
        header = HEADER.fullmatch(line)
        row = ROW.fullmatch(line)
        if header is None and row is None:
            return
        self._ready = True
        self._cancel_job("_probe_job")
        if header:
            self.board_var.set(f"board={header['board']} · degradation={header['degradation']}")
        if row:
            info = row.groupdict()
            name = info["name"]
            if name not in self.styles and len(self.styles) >= 128:
                self._log("样式数量超过 128，已忽略额外条目。")
                return
            old = self.style_buttons.get(name)
            if old is not None:
                old.destroy()
            self.styles[name] = info
            style_label = STYLE_LABELS.get(info["style"], info["style"])
            text = (
                f"{LABELS.get(name, name)} · {style_label}\n{name} · {info['style']}\n"
                f"{info['role']} / {info['level']} · {info['envelope_ms']} ms"
            )
            self.style_buttons[name] = ttk.Button(
                self._groups[info["extent"]], text=text, command=lambda n=name: self.play(n)
            )
            self._layout_styles()
            self._update_function_metadata()
        self.status_var.set(f"已连接 · LED 接口已响应 · {len(self.styles)} 个灯效条目")
        self._set_controls()

    def _duration(self):
        value = self.duration_var.get().strip()
        if not re.fullmatch(r"[0-9]{1,5}", value) or not 250 <= int(value) <= 30000:
            self._log("时长必须是 250–30000 ms 的整数；一次性样式不使用此项。")
            return None
        return int(value)

    def play(self, name):
        if not self._ready or self.serial_port is None or name not in self.styles:
            return
        if self.styles[name]["extent"] == "finite":
            self._send(f"led play {name}")
        elif (duration := self._duration()) is not None:
            self._send(f"led play {name} {duration}")

    def play_color(self, name):
        if self._ready and self.serial_port is not None and name in COLORS:
            if (duration := self._duration()) is not None:
                self._send(f"led color {name} {duration}")


def main():
    if sys.version_info < (3, 11):
        print("需要 Python 3.11 或更新版本；可用 uv run --python 3.11 scripts/led_preview_gui.py。", file=sys.stderr)
        return 1
    if tk is None or serial is None:
        print(TK_HINT if tk is None else SERIAL_HINT, file=sys.stderr)
        return 1
    try:
        root = tk.Tk()
    except tk.TclError as exc:
        print(f"无法启动 Tk 图形界面：{exc}\n{TK_HINT}", file=sys.stderr)
        return 1
    if sys.platform.startswith("linux") and root.tk.call("tk", "windowingsystem") == "x11":
        families = {family.casefold() for family in tkfont.families(root)}
        if not families or families == {"fixed"}:
            print(
                "Tk/X11 has no usable Unicode font backend (only 'fixed' or no fonts).\n"
                "Use system Python/Tk to display the Chinese UI:\n"
                "  uv run --python /usr/bin/python3 scripts/led_preview_gui.py",
                file=sys.stderr,
            )
            root.destroy()
            return 1
    LedPreviewApp(root)
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
