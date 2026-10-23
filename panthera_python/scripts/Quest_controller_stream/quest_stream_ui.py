import pathlib
import json
import socket
import struct
import sys
import time

from PyQt5 import QtCore, QtGui, QtWidgets


BASE_DIR = pathlib.Path(__file__).resolve().parent
SERVER_SCRIPT = BASE_DIR / "start_stream_server.py"
UI_SETTINGS_FILE = BASE_DIR / ".quest_stream_ui.json"
PACKET_FORMAT = "40f"
PACKET_SIZE = struct.calcsize(PACKET_FORMAT)


def fmt3(values):
    return ", ".join(f"{value:.3f}" for value in values)


class UdpMonitorWorker(QtCore.QObject):
    log_line = QtCore.pyqtSignal(str)
    status_changed = QtCore.pyqtSignal(str)
    packet_changed = QtCore.pyqtSignal(dict)
    finished = QtCore.pyqtSignal()

    def __init__(self, port: int):
        super().__init__()
        self.port = port
        self._stopping = False
        self.sock = None
        self.packet_count = 0

    @QtCore.pyqtSlot()
    def run(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            self.sock.bind(("0.0.0.0", self.port))
        except OSError as exc:
            self.status_changed.emit(f"Bind failed: {exc}")
            self.log_line.emit(f"Bind failed on :{self.port}: {exc}\n")
            self.finished.emit()
            return

        self.sock.settimeout(0.5)
        self.status_changed.emit(f"Listening on UDP :{self.port}")
        self.log_line.emit(f"Listening on UDP :{self.port}\n")

        try:
            while not self._stopping:
                try:
                    data, addr = self.sock.recvfrom(4096)
                except socket.timeout:
                    continue

                if len(data) != PACKET_SIZE:
                    self.log_line.emit(
                        f"Unexpected packet size from {addr[0]}:{addr[1]} -> {len(data)} bytes\n"
                    )
                    continue

                vals = struct.unpack(PACKET_FORMAT, data)
                left = vals[:20]
                right = vals[20:]
                self.packet_count += 1

                payload = {
                    "packets": str(self.packet_count),
                    "last_time": time.strftime("%H:%M:%S"),
                    "source": f"{addr[0]}:{addr[1]}",
                    "left": self._decode_hand(left),
                    "right": self._decode_hand(right),
                }
                self.packet_changed.emit(payload)
                self.log_line.emit(self._format_packet_summary(payload))
        finally:
            if self.sock is not None:
                self.sock.close()
            self.status_changed.emit("Monitor stopped")
            self.finished.emit()

    def stop(self):
        self._stopping = True

    def _decode_hand(self, vals):
        return {
            "seq": str(int(vals[0])),
            "pose_valid": "valid" if int(vals[2]) else "missing",
            "position": fmt3(vals[3:6]),
            "rotation": fmt3(vals[6:10]),
            "thumbstick": fmt3(vals[10:12]),
            "trigger": f"{vals[12]:.3f}",
            "squeeze": f"{vals[13]:.3f}",
            "thumb_click": "ON" if int(vals[14]) else "OFF",
            "primary": "ON" if int(vals[15]) else "OFF",
            "secondary": "ON" if int(vals[16]) else "OFF",
            "trigger_pressed": "ON" if int(vals[17]) else "OFF",
            "squeeze_pressed": "ON" if int(vals[18]) else "OFF",
        }

    def _format_packet_summary(self, payload):
        left = payload["left"]
        right = payload["right"]
        return (
            f"[{payload['last_time']}] {payload['source']} "
            f"L(seq={left['seq']}, pose={left['pose_valid']}, pos={left['position']}) "
            f"R(seq={right['seq']}, pose={right['pose_valid']}, pos={right['position']})\n"
        )


class ValuePill(QtWidgets.QFrame):
    def __init__(self, label: str, value: str = "-"):
        super().__init__()
        self.setObjectName("ValuePill")
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(10, 8, 10, 8)
        layout.setSpacing(2)
        self.label = QtWidgets.QLabel(label)
        self.label.setObjectName("PillLabel")
        self.value = QtWidgets.QLabel(value)
        self.value.setObjectName("PillValue")
        self.value.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        layout.addWidget(self.label)
        layout.addWidget(self.value)

    def set_value(self, value: str):
        self.value.setText(value)


class HandPanel(QtWidgets.QFrame):
    def __init__(self, title: str):
        super().__init__()
        self.setObjectName("HandPanel")
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(6)

        header = QtWidgets.QHBoxLayout()
        self.title = QtWidgets.QLabel(title)
        self.title.setObjectName("HandTitle")
        self.pose_badge = QtWidgets.QLabel("missing")
        self.pose_badge.setObjectName("PoseBadgeMissing")
        header.addWidget(self.title)
        header.addStretch(1)
        header.addWidget(self.pose_badge)
        layout.addLayout(header)

        top_grid = QtWidgets.QGridLayout()
        top_grid.setHorizontalSpacing(6)
        top_grid.setVerticalSpacing(6)
        self.seq_pill = ValuePill("Seq", "-")
        self.pos_pill = ValuePill("Position", "-")
        self.rot_pill = ValuePill("Quaternion", "-")
        self.thumb_pill = ValuePill("Thumbstick", "-")
        self.trigger_pill = ValuePill("Trigger", "-")
        self.squeeze_pill = ValuePill("Squeeze", "-")
        top_grid.addWidget(self.seq_pill, 0, 0)
        top_grid.addWidget(self.pos_pill, 0, 1)
        top_grid.addWidget(self.rot_pill, 1, 0, 1, 2)
        top_grid.addWidget(self.thumb_pill, 2, 0)
        top_grid.addWidget(self.trigger_pill, 2, 1)
        top_grid.addWidget(self.squeeze_pill, 3, 0, 1, 2)
        layout.addLayout(top_grid)

        button_grid = QtWidgets.QGridLayout()
        button_grid.setHorizontalSpacing(6)
        button_grid.setVerticalSpacing(6)
        self.thumb_click_pill = ValuePill("Thumb Click", "-")
        self.primary_pill = ValuePill("Primary", "-")
        self.secondary_pill = ValuePill("Secondary", "-")
        self.trigger_pressed_pill = ValuePill("Trigger Pressed", "-")
        self.squeeze_pressed_pill = ValuePill("Squeeze Pressed", "-")
        button_grid.addWidget(self.thumb_click_pill, 0, 0)
        button_grid.addWidget(self.primary_pill, 0, 1)
        button_grid.addWidget(self.secondary_pill, 1, 0)
        button_grid.addWidget(self.trigger_pressed_pill, 1, 1)
        button_grid.addWidget(self.squeeze_pressed_pill, 2, 0, 1, 2)
        layout.addLayout(button_grid)

    def update_hand(self, hand: dict):
        self.seq_pill.set_value(hand["seq"])
        self.pos_pill.set_value(hand["position"])
        self.rot_pill.set_value(hand["rotation"])
        self.thumb_pill.set_value(hand["thumbstick"])
        self.trigger_pill.set_value(hand["trigger"])
        self.squeeze_pill.set_value(hand["squeeze"])
        self.thumb_click_pill.set_value(hand["thumb_click"])
        self.primary_pill.set_value(hand["primary"])
        self.secondary_pill.set_value(hand["secondary"])
        self.trigger_pressed_pill.set_value(hand["trigger_pressed"])
        self.squeeze_pressed_pill.set_value(hand["squeeze_pressed"])

        if hand["pose_valid"] == "valid":
            self.pose_badge.setText("tracking")
            self.pose_badge.setObjectName("PoseBadgeOk")
        else:
            self.pose_badge.setText("missing")
            self.pose_badge.setObjectName("PoseBadgeMissing")
        self.pose_badge.style().unpolish(self.pose_badge)
        self.pose_badge.style().polish(self.pose_badge)


class QuestStreamWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Quest Stream Console")
        self.resize(1320, 700)

        self.server_process = QtCore.QProcess(self)
        self.server_process.setProgram(sys.executable)
        self.server_process.setWorkingDirectory(str(BASE_DIR))
        self.server_process.readyReadStandardOutput.connect(self._consume_server_output)
        self.server_process.readyReadStandardError.connect(self._consume_server_output)
        self.server_process.stateChanged.connect(self._server_state_changed)
        self.server_process.errorOccurred.connect(self._server_error)

        self.monitor_thread = None
        self.monitor_worker = None
        self.defaults = {
            "mode": "USB",
            "force_https": False,
            "udp_ip": "127.0.0.1",
            "udp_port": "5005",
            "http_host": "0.0.0.0",
            "http_port": "8080",
            "ws_host": "0.0.0.0",
            "ws_port": "8765",
            "cert_file": ".certs/quest-controller.crt",
            "key_file": ".certs/quest-controller.key",
            "verify_port": "5005",
        }

        self.mode_combo = None
        self.force_https_checkbox = None
        self.udp_ip_edit = None
        self.udp_port_edit = None
        self.http_host_edit = None
        self.http_port_edit = None
        self.ws_host_edit = None
        self.ws_port_edit = None
        self.cert_edit = None
        self.key_edit = None
        self.verify_port_edit = None

        self.server_status = QtWidgets.QLabel("Server stopped")
        self.monitor_status = QtWidgets.QLabel("Monitor stopped")
        self.usb_url = QtWidgets.QLabel("-")
        self.wifi_url = QtWidgets.QLabel("-")
        self.ws_url = QtWidgets.QLabel("-")
        self.packet_count = QtWidgets.QLabel("0")
        self.last_time = QtWidgets.QLabel("-")
        self.packet_source = QtWidgets.QLabel("-")
        self.server_log = QtWidgets.QPlainTextEdit()
        self.monitor_log = QtWidgets.QPlainTextEdit()
        self.left_panel = HandPanel("Left Controller")
        self.right_panel = HandPanel("Right Controller")
        self.hand_tabs = None

        self._build_ui()
        self._apply_style()
        self._load_settings()

    def _build_ui(self):
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QVBoxLayout(central)
        root.setContentsMargins(12, 12, 12, 12)
        root.setSpacing(10)

        hero = QtWidgets.QFrame()
        hero.setObjectName("Hero")
        hero_layout = QtWidgets.QHBoxLayout(hero)
        hero_layout.setContentsMargins(18, 14, 18, 14)
        hero_layout.setSpacing(14)
        title_box = QtWidgets.QVBoxLayout()
        title = QtWidgets.QLabel("Quest Stream Console")
        title.setObjectName("HeroTitle")
        subtitle = QtWidgets.QLabel("启动服务、切换 USB/WiFi、验证 UDP 接收，并实时查看手柄位姿。")
        subtitle.setObjectName("HeroSubtitle")
        title_box.addWidget(title)
        title_box.addWidget(subtitle)
        hero_layout.addLayout(title_box, 1)
        summary_box = QtWidgets.QGridLayout()
        summary_box.setHorizontalSpacing(10)
        summary_box.setVerticalSpacing(6)
        summary_box.addWidget(self._summary_card("Server", self.server_status), 0, 0)
        summary_box.addWidget(self._summary_card("Monitor", self.monitor_status), 0, 1)
        summary_box.addWidget(self._summary_card("Packets", self.packet_count), 1, 0)
        summary_box.addWidget(self._summary_card("Last Packet", self.last_time), 1, 1)
        hero_layout.addLayout(summary_box)
        root.addWidget(hero)

        top_split = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        root.addWidget(top_split, 1)

        control_column = QtWidgets.QWidget()
        control_layout = QtWidgets.QVBoxLayout(control_column)
        control_layout.setContentsMargins(0, 0, 0, 0)
        control_layout.setSpacing(10)

        control_card = self._make_card("Launch Controls")
        control_form = QtWidgets.QGridLayout(control_card)
        control_form.setHorizontalSpacing(10)
        control_form.setVerticalSpacing(8)

        self.mode_combo = QtWidgets.QComboBox()
        self.mode_combo.addItems(["USB", "WiFi"])
        self.mode_combo.currentIndexChanged.connect(self._sync_mode_defaults)
        self.force_https_checkbox = QtWidgets.QCheckBox("Force HTTPS/WSS")
        self.udp_ip_edit = QtWidgets.QLineEdit("127.0.0.1")
        self.udp_port_edit = QtWidgets.QLineEdit("5005")
        self.http_host_edit = QtWidgets.QLineEdit("0.0.0.0")
        self.http_port_edit = QtWidgets.QLineEdit("8080")
        self.ws_host_edit = QtWidgets.QLineEdit("0.0.0.0")
        self.ws_port_edit = QtWidgets.QLineEdit("8765")
        self.cert_edit = QtWidgets.QLineEdit(".certs/quest-controller.crt")
        self.key_edit = QtWidgets.QLineEdit(".certs/quest-controller.key")
        self.verify_port_edit = QtWidgets.QLineEdit("5005")

        fields = [
            ("Mode", self.mode_combo, 0, 0),
            ("UDP IP", self.udp_ip_edit, 1, 0),
            ("UDP Port", self.udp_port_edit, 1, 2),
            ("HTTP Host", self.http_host_edit, 2, 0),
            ("HTTP Port", self.http_port_edit, 2, 2),
            ("WS Host", self.ws_host_edit, 3, 0),
            ("WS Port", self.ws_port_edit, 3, 2),
            ("Cert File", self.cert_edit, 4, 0),
            ("Key File", self.key_edit, 5, 0),
            ("Verify Port", self.verify_port_edit, 5, 2),
        ]
        for text, widget, row, col in fields:
            control_form.addWidget(QtWidgets.QLabel(text), row, col)
            if col == 0:
                span = 1 if row == 0 else 1
                control_form.addWidget(widget, row, col + 1)
            else:
                control_form.addWidget(widget, row, col + 1)

        control_form.addWidget(self.force_https_checkbox, 0, 2, 1, 2)

        action_row = QtWidgets.QHBoxLayout()
        start_btn = QtWidgets.QPushButton("Start Server")
        start_btn.setObjectName("PrimaryButton")
        stop_btn = QtWidgets.QPushButton("Stop Server")
        verify_btn = QtWidgets.QPushButton("Start Verify")
        stop_verify_btn = QtWidgets.QPushButton("Stop Verify")
        save_btn = QtWidgets.QPushButton("Save Settings")
        reset_btn = QtWidgets.QPushButton("Reset Defaults")
        start_btn.clicked.connect(self.start_server)
        stop_btn.clicked.connect(self.stop_server)
        verify_btn.clicked.connect(self.start_monitor)
        stop_verify_btn.clicked.connect(self.stop_monitor)
        save_btn.clicked.connect(self._save_settings)
        reset_btn.clicked.connect(self._reset_defaults)
        action_row.addWidget(start_btn)
        action_row.addWidget(stop_btn)
        action_row.addSpacing(10)
        action_row.addWidget(verify_btn)
        action_row.addWidget(stop_verify_btn)
        action_row.addSpacing(10)
        action_row.addWidget(save_btn)
        action_row.addWidget(reset_btn)
        action_row.addStretch(1)
        control_layout.addWidget(control_card)
        control_layout.addLayout(action_row)

        endpoint_card = self._make_card("Resolved Endpoints")
        endpoint_layout = QtWidgets.QFormLayout(endpoint_card)
        for label in [self.usb_url, self.wifi_url, self.ws_url, self.packet_source]:
            label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
            label.setWordWrap(True)
        endpoint_layout.addRow("USB URL", self.usb_url)
        endpoint_layout.addRow("WiFi URL", self.wifi_url)
        endpoint_layout.addRow("WS URL", self.ws_url)
        endpoint_layout.addRow("Last Source", self.packet_source)
        control_layout.addWidget(endpoint_card)

        log_split = QtWidgets.QSplitter(QtCore.Qt.Vertical)
        self.server_log.setReadOnly(True)
        self.monitor_log.setReadOnly(True)
        self.server_log.setPlaceholderText("Server output will appear here.")
        self.monitor_log.setPlaceholderText("UDP verification log will appear here.")
        self.server_log.setMaximumBlockCount(1500)
        self.monitor_log.setMaximumBlockCount(1500)
        server_log_card = self._log_card("Server Log", self.server_log)
        monitor_log_card = self._log_card("Verify Log", self.monitor_log)
        log_split.addWidget(server_log_card)
        log_split.addWidget(monitor_log_card)
        log_split.setSizes([190, 110])
        control_layout.addWidget(log_split, 1)
        top_split.addWidget(control_column)

        telemetry_column = QtWidgets.QWidget()
        telemetry_layout = QtWidgets.QVBoxLayout(telemetry_column)
        telemetry_layout.setContentsMargins(0, 0, 0, 0)
        telemetry_layout.setSpacing(10)
        self.hand_tabs = QtWidgets.QTabWidget()
        self.hand_tabs.addTab(self.left_panel, "Left")
        self.hand_tabs.addTab(self.right_panel, "Right")
        telemetry_layout.addWidget(self.hand_tabs)
        top_split.addWidget(telemetry_column)
        top_split.setSizes([640, 420])

    def _summary_card(self, title: str, value_widget: QtWidgets.QLabel):
        card = QtWidgets.QFrame()
        card.setObjectName("SummaryCard")
        layout = QtWidgets.QVBoxLayout(card)
        layout.setContentsMargins(14, 12, 14, 12)
        layout.setSpacing(4)
        label = QtWidgets.QLabel(title)
        label.setObjectName("SummaryLabel")
        value_widget.setObjectName("SummaryValue")
        layout.addWidget(label)
        layout.addWidget(value_widget)
        return card

    def _make_card(self, title: str):
        card = QtWidgets.QGroupBox(title)
        card.setObjectName("Card")
        return card

    def _log_card(self, title: str, editor: QtWidgets.QPlainTextEdit):
        card = self._make_card(title)
        layout = QtWidgets.QVBoxLayout(card)
        layout.addWidget(editor)
        return card

    def _apply_style(self):
        self.setStyleSheet(
            """
            QWidget {
                background: #f3efe7;
                color: #2a241b;
                font-family: "DejaVu Sans", "Noto Sans", sans-serif;
                font-size: 12px;
            }
            QMainWindow {
                background: #f3efe7;
            }
            #Hero {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:1,
                                            stop:0 #fff8eb, stop:1 #e8ddc7);
                border: 1px solid #d7c6a8;
                border-radius: 18px;
            }
            #HeroTitle {
                font-size: 21px;
                font-weight: 700;
            }
            #HeroSubtitle {
                color: #6b5f4d;
                font-size: 12px;
            }
            #SummaryCard {
                background: rgba(255, 250, 241, 0.96);
                border: 1px solid #d7c6a8;
                border-radius: 14px;
            }
            #SummaryLabel {
                color: #7a6f5d;
                font-size: 12px;
                text-transform: uppercase;
            }
            #SummaryValue {
                font-size: 16px;
                font-weight: 700;
            }
            #Card, #HandPanel {
                background: #fffaf1;
                border: 1px solid #d7c6a8;
                border-radius: 18px;
                margin-top: 8px;
                padding-top: 8px;
            }
            #Card::title {
                subcontrol-origin: margin;
                left: 14px;
                padding: 0 8px;
                color: #7a6f5d;
                font-weight: 600;
            }
            #HandPanel {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:1,
                                            stop:0 #fffef9, stop:1 #f7eddc);
            }
            #HandTitle {
                font-size: 16px;
                font-weight: 700;
            }
            #PoseBadgeOk, #PoseBadgeMissing {
                padding: 6px 12px;
                border-radius: 999px;
                font-weight: 700;
                color: white;
            }
            #PoseBadgeOk {
                background: #317d48;
            }
            #PoseBadgeMissing {
                background: #a64a2a;
            }
            #ValuePill {
                background: rgba(255, 255, 255, 0.92);
                border: 1px solid #dfd1b8;
                border-radius: 14px;
            }
            #PillLabel {
                color: #7a6f5d;
                font-size: 12px;
            }
            #PillValue {
                font-size: 13px;
                font-weight: 600;
            }
            QLineEdit, QComboBox, QPlainTextEdit {
                background: white;
                border: 1px solid #d7c6a8;
                border-radius: 12px;
                padding: 6px 8px;
            }
            QComboBox::drop-down {
                border: none;
            }
            QPushButton {
                background: #ead8b7;
                border: 1px solid #c7b089;
                border-radius: 12px;
                padding: 8px 14px;
                font-weight: 600;
            }
            QPushButton:hover {
                background: #e3cfab;
            }
            #PrimaryButton {
                background: #2a241b;
                color: #fff8eb;
                border: 1px solid #2a241b;
            }
            #PrimaryButton:hover {
                background: #3a3126;
            }
            QTabWidget::pane {
                border: 1px solid #d7c6a8;
                border-radius: 16px;
                background: #fffaf1;
                top: -1px;
            }
            QTabBar::tab {
                background: #ead8b7;
                border: 1px solid #c7b089;
                padding: 7px 16px;
                margin-right: 4px;
                border-top-left-radius: 10px;
                border-top-right-radius: 10px;
            }
            QTabBar::tab:selected {
                background: #2a241b;
                color: #fff8eb;
                border-color: #2a241b;
            }
            QSplitter::handle {
                background: transparent;
            }
            """
        )

    def _sync_mode_defaults(self):
        is_wifi = self.mode_combo.currentText() == "WiFi"
        self.force_https_checkbox.setChecked(is_wifi)

    def _collect_settings(self):
        return {
            "mode": self.mode_combo.currentText(),
            "force_https": self.force_https_checkbox.isChecked(),
            "udp_ip": self.udp_ip_edit.text().strip(),
            "udp_port": self.udp_port_edit.text().strip(),
            "http_host": self.http_host_edit.text().strip(),
            "http_port": self.http_port_edit.text().strip(),
            "ws_host": self.ws_host_edit.text().strip(),
            "ws_port": self.ws_port_edit.text().strip(),
            "cert_file": self.cert_edit.text().strip(),
            "key_file": self.key_edit.text().strip(),
            "verify_port": self.verify_port_edit.text().strip(),
        }

    def _apply_settings(self, settings: dict):
        merged = dict(self.defaults)
        merged.update(settings or {})
        mode = merged.get("mode", "USB")
        self.mode_combo.setCurrentText(mode if mode in {"USB", "WiFi"} else "USB")
        self.udp_ip_edit.setText(merged["udp_ip"])
        self.udp_port_edit.setText(merged["udp_port"])
        self.http_host_edit.setText(merged["http_host"])
        self.http_port_edit.setText(merged["http_port"])
        self.ws_host_edit.setText(merged["ws_host"])
        self.ws_port_edit.setText(merged["ws_port"])
        self.cert_edit.setText(merged["cert_file"])
        self.key_edit.setText(merged["key_file"])
        self.verify_port_edit.setText(merged["verify_port"])
        if self.mode_combo.currentText() == "WiFi":
            self.force_https_checkbox.setChecked(True)
        else:
            self.force_https_checkbox.setChecked(bool(merged["force_https"]))

    def _load_settings(self):
        settings = None
        if UI_SETTINGS_FILE.exists():
            try:
                settings = json.loads(UI_SETTINGS_FILE.read_text(encoding="utf-8"))
            except (OSError, json.JSONDecodeError):
                settings = None
        self._apply_settings(settings or self.defaults)

    def _save_settings(self):
        settings = self._collect_settings()
        try:
            UI_SETTINGS_FILE.write_text(
                json.dumps(settings, ensure_ascii=False, indent=2),
                encoding="utf-8",
            )
            self.server_status.setText("Settings saved")
        except OSError as exc:
            self.server_status.setText(f"Save failed: {exc}")

    def _reset_defaults(self):
        self._apply_settings(self.defaults)
        self.server_status.setText("Defaults restored")

    def _build_server_args(self):
        args = [
            str(SERVER_SCRIPT),
            "--udp-ip",
            self.udp_ip_edit.text().strip(),
            "--udp-port",
            self.udp_port_edit.text().strip(),
            "--http-host",
            self.http_host_edit.text().strip(),
            "--http-port",
            self.http_port_edit.text().strip(),
            "--ws-host",
            self.ws_host_edit.text().strip(),
            "--ws-port",
            self.ws_port_edit.text().strip(),
            "--cert-file",
            self.cert_edit.text().strip(),
            "--key-file",
            self.key_edit.text().strip(),
        ]
        if self.mode_combo.currentText() == "WiFi":
            args.extend(["--adb", "never"])
        else:
            args.extend(["--adb", "auto"])
        if self.force_https_checkbox.isChecked():
            args.append("--https")
        return args

    def start_server(self):
        if self.server_process.state() != QtCore.QProcess.NotRunning:
            self.server_status.setText("Server already running")
            return
        self.server_log.appendPlainText(f"$ {sys.executable} {' '.join(self._build_server_args())}")
        self.server_process.start(sys.executable, self._build_server_args())

    def stop_server(self):
        if self.server_process.state() == QtCore.QProcess.NotRunning:
            self.server_status.setText("Server already stopped")
            return
        self.server_status.setText("Stopping server...")
        self.server_process.terminate()
        if not self.server_process.waitForFinished(3000):
            self.server_process.kill()

    def _consume_server_output(self):
        text = bytes(self.server_process.readAllStandardOutput()).decode(errors="replace")
        if not text:
            text = bytes(self.server_process.readAllStandardError()).decode(errors="replace")
        if not text:
            return
        self.server_log.moveCursor(QtGui.QTextCursor.End)
        self.server_log.insertPlainText(text)
        self.server_log.moveCursor(QtGui.QTextCursor.End)
        for line in text.splitlines():
            self._extract_urls(line.strip())

    def _extract_urls(self, line: str):
        if line.startswith("USB access with ws param:"):
            self.usb_url.setText(line.split(":", 1)[1].strip())
        elif line.startswith("USB access:") and self.usb_url.text() == "-":
            self.usb_url.setText(line.split(":", 1)[1].strip())
        elif line.startswith("WiFi access with ws param:"):
            self.wifi_url.setText(line.split(":", 1)[1].strip())
        elif line.startswith("WS relay listening on"):
            self.ws_url.setText(line.replace("WS relay listening on", "").strip())

    def _server_state_changed(self, state):
        mapping = {
            QtCore.QProcess.NotRunning: "Server stopped",
            QtCore.QProcess.Starting: "Server starting...",
            QtCore.QProcess.Running: "Server running",
        }
        self.server_status.setText(mapping.get(state, "Server unknown"))

    def _server_error(self, _error):
        self.server_status.setText(self.server_process.errorString())

    def start_monitor(self):
        if self.monitor_thread and self.monitor_thread.isRunning():
            self.monitor_status.setText("Monitor already running")
            return
        try:
            port = int(self.verify_port_edit.text().strip())
        except ValueError:
            self.monitor_status.setText("Verify port invalid")
            return

        self.packet_count.setText("0")
        self.last_time.setText("-")
        self.packet_source.setText("-")
        self.left_panel.update_hand(
            {
                "seq": "-",
                "pose_valid": "missing",
                "position": "-",
                "rotation": "-",
                "thumbstick": "-",
                "trigger": "-",
                "squeeze": "-",
                "thumb_click": "-",
                "primary": "-",
                "secondary": "-",
                "trigger_pressed": "-",
                "squeeze_pressed": "-",
            }
        )
        self.right_panel.update_hand(
            {
                "seq": "-",
                "pose_valid": "missing",
                "position": "-",
                "rotation": "-",
                "thumbstick": "-",
                "trigger": "-",
                "squeeze": "-",
                "thumb_click": "-",
                "primary": "-",
                "secondary": "-",
                "trigger_pressed": "-",
                "squeeze_pressed": "-",
            }
        )

        self.monitor_worker = UdpMonitorWorker(port)
        self.monitor_thread = QtCore.QThread()
        self.monitor_worker.moveToThread(self.monitor_thread)
        self.monitor_thread.started.connect(self.monitor_worker.run)
        self.monitor_worker.log_line.connect(self._append_monitor_log)
        self.monitor_worker.status_changed.connect(self.monitor_status.setText)
        self.monitor_worker.packet_changed.connect(self._update_packet_view)
        self.monitor_worker.finished.connect(self.monitor_thread.quit)
        self.monitor_worker.finished.connect(self.monitor_worker.deleteLater)
        self.monitor_thread.finished.connect(self._clear_monitor_refs)
        self.monitor_thread.finished.connect(self.monitor_thread.deleteLater)
        self.monitor_thread.start()

    def stop_monitor(self):
        if not self.monitor_worker:
            self.monitor_status.setText("Monitor already stopped")
            return
        self.monitor_status.setText("Stopping monitor...")
        self.monitor_worker.stop()

    def _append_monitor_log(self, text: str):
        self.monitor_log.moveCursor(QtGui.QTextCursor.End)
        self.monitor_log.insertPlainText(text)
        self.monitor_log.moveCursor(QtGui.QTextCursor.End)

    def _update_packet_view(self, payload: dict):
        self.packet_count.setText(payload["packets"])
        self.last_time.setText(payload["last_time"])
        self.packet_source.setText(payload["source"])
        self.left_panel.update_hand(payload["left"])
        self.right_panel.update_hand(payload["right"])

    def _clear_monitor_refs(self):
        self.monitor_worker = None
        self.monitor_thread = None

    def closeEvent(self, event):
        self._save_settings()
        self.stop_monitor()
        if self.monitor_thread:
            self.monitor_thread.quit()
            self.monitor_thread.wait(1500)
        self.stop_server()
        super().closeEvent(event)


def main():
    app = QtWidgets.QApplication(sys.argv)
    app.setApplicationName("Quest Stream Console")
    window = QuestStreamWindow()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
