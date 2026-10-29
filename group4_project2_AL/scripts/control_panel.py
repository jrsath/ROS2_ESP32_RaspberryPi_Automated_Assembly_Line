#!/usr/bin/env python3
# Virtual switches go through the micro-ROS board. Lamps show returned outputs.
import json
import signal
import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String, Bool, UInt32MultiArray, UInt64MultiArray
from std_srvs.srv import SetBool, Trigger
from PyQt5.QtCore import QTimer, Qt, QRectF, QProcess
from PyQt5.QtGui import QPainter, QColor, QPen
from PyQt5.QtWidgets import (
    QApplication,
    QWidget,
    QLabel,
    QPushButton,
    QComboBox,
    QCheckBox,
    QVBoxLayout,
    QHBoxLayout,
    QGridLayout,
    QGroupBox,
    QTextEdit,
    QDialog,
    QLineEdit,
    QSizePolicy,
)


from group4_project2_al.path_editor import PathEditor
from group4_project2_al.environment_widget import EnvironmentWidget


class ConveyorView(QWidget):
    def __init__(self):
        super().__init__()
        self.setFixedHeight(82)
        self.data = {}
        self.running = False

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.scale(1.0, 0.74)
        width = self.width() - 80
        painter.setPen(QPen(QColor("#667b8f"), 2))
        painter.setBrush(QColor("#d3dce5"))
        painter.drawRoundedRect(QRectF(40, 32, width, 38), 18, 18)
        painter.drawText(40, 22, "Assembly input")
        painter.drawText(self.width() - 175, 22, "Inspection / output")
        painter.drawText(40, 98, "Moving →" if self.running else "Stopped")
        colour = "#ef5350" if self.data.get("occupied") else "#42a568"
        if not self.data.get("fresh"):
            colour = "#9cabb9"
        painter.setBrush(QColor(colour))
        painter.drawEllipse(QRectF(self.width() - 58, 72, 12, 12))
        label = "Sensor occupied" if self.data.get("occupied") else "Sensor clear"
        if not self.data.get("fresh"):
            label = "Sensor offline"
        painter.drawText(self.width() - 195, 99, label)
        for item in self.data.get("toys", []):
            x = 46 + float(item["position"]) * (width - 32)
            painter.setBrush(QColor("#3687ba"))
            painter.drawRoundedRect(QRectF(x, 38, 25, 26), 4, 4)
            painter.setPen(QColor("white"))
            painter.drawText(QRectF(x, 38, 25, 26), Qt.AlignCenter, str(item["toy"]))
            painter.setPen(QColor("#192e43"))


class Panel(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setWindowTitle("Project 2 · Control box")
        self.resize(960, 690)
        self.inputs = [0, 0, 1, 0, 0]
        self.run_widgets = []
        self.io = None
        self.io_time = 0.0
        self.cell = {}
        self.power = None
        self.arm = {}
        self.previous = None
        self.offset = 0
        self.panel_pub = node.create_publisher(
            UInt32MultiArray, "/project2/sim/panel", 10
        )
        self.event_pub = node.create_publisher(String, "/project2/sim/event", 10)
        self.link_pub = node.create_publisher(Bool, "/project2/sim/command_link", 10)
        self.camera_pub = node.create_publisher(Bool, "/project2/camera/open", 10)
        self.camera_open = False
        self.camera_seen = 0.0
        self.camera_requested = None
        self.camera_process = QProcess(self)
        self.camera_process.setProcessChannelMode(QProcess.MergedChannels)
        self.camera_process.readyReadStandardOutput.connect(self.camera_output)
        self.camera_process.errorOccurred.connect(self.camera_failed)
        self.cad_camera = node.declare_parameter("cad_camera", False).value
        node.create_subscription(
            Bool, "/project2/camera/window_open", self.on_camera, 10
        )
        node.create_subscription(
            UInt64MultiArray,
            "/project2/sim/io/state",
            self.on_io,
            qos_profile_sensor_data,
        )
        node.create_subscription(
            Bool,
            "/project2/sim/actuator_power",
            lambda msg: setattr(self, "power", msg.data),
            10,
        )
        node.create_subscription(String, "/project2/sim/state", self.on_cell, 10)
        node.create_subscription(String, "/project2/sim/arm_status", self.on_arm, 10)
        self.follow_client = node.create_client(SetBool, "/project2/sim/master_slave")
        self.home_client = node.create_client(Trigger, "/project2/sim/home")
        self.setStyleSheet("""
            QWidget { background: #eef2f6; color: #192e43; font-size: 13px; }
            QGroupBox { border: 1px solid #b8c7d5; border-radius: 8px; margin-top: 10px; padding: 6px; }
            QGroupBox::title { subcontrol-origin: margin; left: 15px; font-weight: bold; }
            QPushButton { background: white; border: 1px solid #9cabb9; border-radius: 6px; padding: 6px; }
            QPushButton:pressed { background: #b9d5e7; }
            QComboBox, QTextEdit { background: white; padding: 6px; }
        """)
        layout = QVBoxLayout(self)
        layout.setSpacing(5)
        title = QLabel("PROJECT 2  /  CONTROL BOX")
        title.setStyleSheet("font-size: 19px; font-weight: bold;")
        layout.addWidget(title)
        self.link = QLabel("Waiting for micro-ROS board…")
        layout.addWidget(self.link)
        self.environment = EnvironmentWidget(self)
        layout.addWidget(self.environment)
        camera_row = QHBoxLayout()
        self.camera_button = QPushButton("Open camera")
        self.camera_button.clicked.connect(self.toggle_camera)
        camera_row.addWidget(self.camera_button)
        layout.addLayout(camera_row)
        self.activity = QComboBox()
        self.activity.addItems(["Run line", "Plan a path"])
        camera_row.addWidget(self.activity)
        self.editor = PathEditor(self)
        self.editor_window = QDialog(self)
        self.editor_window.setModal(False)
        self.editor_window.setWindowTitle("Project 2 · Plan a path")
        editor_layout = QVBoxLayout(self.editor_window)
        editor_layout.addWidget(self.editor)
        self.editor_window.resize(1000, 650)
        self.editor_window.finished.connect(lambda _: self.activity.setCurrentIndex(0))
        self.activity.currentIndexChanged.connect(self.activity_changed)
        controls = QGroupBox("Operator controls")
        row = QGridLayout(controls)
        self.settings_seen = False
        self.settings_edited = False
        self.selector_revision = 0
        row.addWidget(QLabel("Task"), 0, 0)
        self.task_choice = QComboBox()
        for label, mode in (
            ("Full task", "full"),
            ("Assembly", "assembly"),
            ("Inspection", "inspection"),
            ("Packaging", "packaging"),
        ):
            self.task_choice.addItem(label, mode)
        row.addWidget(self.task_choice, 0, 1)
        row.addWidget(QLabel("No. of Products"), 0, 2)
        self.product_choice = QComboBox()
        self.product_choice.addItem("Select…", 0)
        for count in range(1, 6):
            self.product_choice.addItem(str(count), count)
        row.addWidget(self.product_choice, 0, 3)
        self.task_choice.currentIndexChanged.connect(self.settings_changed)
        self.product_choice.currentIndexChanged.connect(self.settings_changed)
        row.addWidget(QLabel("Faulty products"), 1, 0)
        self.fault_count = QComboBox()
        self.fault_count.addItem("0", 0)
        row.addWidget(self.fault_count, 1, 1)
        self.fault_position = QComboBox()
        for label, value in (
            ("Random positions", "random"),
            ("First products", "first"),
            ("Last products", "last"),
            ("Choose positions", "custom"),
        ):
            self.fault_position.addItem(label, value)
        row.addWidget(self.fault_position, 1, 2)
        self.fault_indices = QLineEdit()
        self.fault_indices.setPlaceholderText("e.g. 2,4")
        self.fault_indices.setEnabled(False)
        row.addWidget(self.fault_indices, 1, 3)
        self.fault_count.currentIndexChanged.connect(self.settings_changed)
        self.fault_position.currentIndexChanged.connect(self.settings_changed)
        self.fault_indices.editingFinished.connect(self.settings_changed)
        start = QPushButton("START")
        self.start_button = start
        start.setStyleSheet(
            "background: #218c45; color: white; font-weight: bold; border-radius: 22px;"
        )
        start.setToolTip("Start or resume the line")
        start.clicked.connect(lambda: self.pulse(0))
        pause = QPushButton("PAUSE")
        self.pause_button = pause
        pause.setStyleSheet(
            "background: #d12e34; color: white; font-weight: bold; border-radius: 22px;"
        )
        pause.setToolTip("Stop motion and hold position; actuator power stays enabled")
        pause.clicked.connect(lambda: self.pulse(1))
        reset = QPushButton("RESET")
        reset.clicked.connect(lambda: self.send_event("reset"))
        self.estop = QPushButton("E-STOP")
        self.estop.setFixedSize(74, 74)
        self.estop.setCheckable(True)
        self.estop.setStyleSheet(
            "background: #a71920; color: white; border: 5px solid #ffd239; border-radius: 37px; font-weight: bold; padding: 0px;"
        )
        self.estop.toggled.connect(self.stop_changed)
        for col, widget in enumerate((start, pause, reset, self.estop)):
            row.addWidget(
                widget,
                2,
                col,
                alignment=Qt.AlignCenter if widget is self.estop else Qt.Alignment(),
            )
        self.estop.setToolTip("Emergency stop. Release, then Reset before restarting.")
        layout.addWidget(controls)
        outputs = QGroupBox("Board outputs")
        row = QHBoxLayout(outputs)
        row.setContentsMargins(6, 6, 6, 6)
        self.lamps = []
        for name in ("RED", "YELLOW", "GREEN", "ALARM"):
            lamp = QLabel(name)
            lamp.setAlignment(Qt.AlignCenter)
            lamp.setFixedHeight(28)
            self.lamps.append(lamp)
            row.addWidget(lamp)
        layout.addWidget(outputs)
        self.run_widgets.append(outputs)
        self.status = QLabel("Cell: waiting")
        self.status.setFixedHeight(54)
        self.status.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Fixed)
        layout.addWidget(self.status)
        self.conveyor = ConveyorView()
        node.create_subscription(String, "/project2/sim/conveyor", self.on_conveyor, 10)
        layout.addWidget(self.conveyor)
        self.run_widgets.append(self.conveyor)
        self.results_row = QHBoxLayout()
        self.results = []
        layout.addLayout(self.results_row)
        test = QWidget()
        test_row = QVBoxLayout(test)
        test_row.setContentsMargins(4, 0, 0, 0)
        occupied = QCheckBox("Sensor occupied")
        occupied.setToolTip(
            "Simulation test: force the output sensor occupied. Clear this to let the conveyor run."
        )
        occupied.toggled.connect(lambda value: self.set_input(4, int(value)))
        self.follow = QCheckBox("Master/slave: arm 1 leads")
        self.follow.setToolTip(
            "Manual test only: copy arm 1 joints to the other arms. Off during production."
        )
        self.follow.toggled.connect(self.set_follow)
        home = QPushButton("Home arms")
        home.clicked.connect(self.home)
        fault = QPushButton("Inject fault")
        fault.clicked.connect(lambda: self.send_event("fault"))
        lost_link = QCheckBox("Interrupt command heartbeat")
        lost_link.setToolTip(
            "Simulation test: stop controller messages to test the watchdog. Motion stops until restored and reset."
        )
        lost_link.toggled.connect(
            lambda lost: self.link_pub.publish(Bool(data=not lost))
        )
        for widget in (occupied, self.follow, lost_link):
            test_row.addWidget(widget)
        controls.layout().addWidget(test, 2, 4)
        self.sensor_test = occupied
        self.heartbeat_test = lost_link
        self.test_window = QDialog(self)
        self.test_window.setModal(False)
        self.test_window.setWindowTitle("Project 2 · Log")
        test_layout = QVBoxLayout(self.test_window)
        # Keep optional manual actions with the log, away from normal Start/Pause.
        actions = QHBoxLayout()
        actions.addWidget(home)
        actions.addWidget(fault)
        test_layout.addLayout(actions)
        advanced = QPushButton("Open log")
        advanced.clicked.connect(self.test_window.show)
        camera_row.addWidget(advanced)
        self.log = QTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumHeight(120)
        self.log.document().setMaximumBlockCount(100)
        test_layout.addWidget(self.log)
        note = QLabel(
            "SIMULATION · Micro-ROS messages are real. GPIO, sensor inputs and actuator loads are simulated."
        )
        note.setWordWrap(True)
        note.setStyleSheet("font-size: 12px; color: #52677b;")
        layout.addWidget(note)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(50)

    def on_camera(self, message):
        self.camera_seen = time.monotonic()
        self.camera_open = message.data
        self.camera_button.setText("Close camera" if message.data else "Open camera")
        if self.camera_requested is not None:
            if message.data == self.camera_requested:
                self.camera_requested = None
            else:
                # The viewer may have subscribed after the original click.
                self.camera_pub.publish(Bool(data=self.camera_requested))

    def camera_output(self):
        text = (
            bytes(self.camera_process.readAllStandardOutput())
            .decode(errors="replace")
            .strip()
        )
        if text:
            self.log.append(text)

    def camera_failed(self, _error):
        self.camera_open = False
        self.camera_requested = None
        self.camera_button.setText("Open camera")
        self.log.append("Camera could not start: " + self.camera_process.errorString())

    def toggle_camera(self):
        opened = not self.camera_open
        self.camera_requested = opened
        # Start a replacement viewer if its previous process exited.
        if (
            opened
            and time.monotonic() - self.camera_seen > 2.0
            and self.camera_process.state() == QProcess.NotRunning
            and "project2_camera_inspection" not in self.node.get_node_names()
        ):
            self.camera_process.start(
                "ros2",
                [
                    "run",
                    "group4_project2_al",
                    "camera_inspection.py",
                    "--ros-args",
                    "-p",
                    f"cad_camera:={str(self.cad_camera).lower()}",
                ],
            )
        self.camera_pub.publish(Bool(data=opened))
        self.camera_open = opened
        self.camera_button.setText("Close camera" if opened else "Open camera")

    def activity_changed(self, index):
        self.editor_window.setVisible(index == 1)
        for widget in (self.start_button, self.task_choice, self.product_choice):
            widget.setEnabled(index == 0)
        if index == 1:
            self.send_event("teach")
            self.editor.refresh()
            self.editor_window.raise_()
        else:
            self.send_event("pause")
            self.log.append("Press Reset to apply path edits before starting.")

    def settings_changed(self):
        self.settings_edited = True
        mode = self.task_choice.currentData()
        if not hasattr(self, "fault_count"):
            return
        count = self.product_choice.currentData()
        self.update_fault_choices(count)
        selected = self.fault_count.currentData()
        self.fault_indices.setEnabled(self.fault_position.currentData() == "custom")
        self.send_event(
            json.dumps(
                {
                    "mode": mode,
                    "products": count,
                    "faults": selected,
                    "fault_position": self.fault_position.currentData(),
                    "fault_indices": self.fault_indices.text(),
                }
            )
        )

    def update_fault_choices(self, count):
        # Keep the chosen fault count valid when the product count changes.
        selected = min(self.fault_count.currentData() or 0, count)
        self.fault_count.blockSignals(True)
        self.fault_count.clear()
        for value in range(count + 1):
            self.fault_count.addItem(str(value), value)
        self.fault_count.setCurrentIndex(selected)
        self.fault_count.blockSignals(False)

    def send_event(self, name):
        self.event_pub.publish(String(data=name))

    def set_input(self, index, value):
        self.inputs[index] = value
        self.panel_pub.publish(UInt32MultiArray(data=self.inputs))

    def pulse(self, index):
        # A short click must survive the board's 20Hz state publication.
        # Cancel the other virtual button; physical button state is kept by the board.
        self.inputs[1 - index] = 0
        self.set_input(index, 1)
        QTimer.singleShot(300, lambda: self.set_input(index, 0))

    def stop_changed(self, pressed):
        self.set_input(2, int(not pressed))
        if pressed:
            self.send_event("estop")
        self.estop.setText("RELEASE" if pressed else "E-STOP")

    def on_io(self, msg):
        if len(msg.data) == 14:
            self.io, self.io_time = list(msg.data), time.monotonic()

    def on_cell(self, msg):
        self.cell = json.loads(msg.data)
        if (not self.settings_seen and not self.settings_edited) or self.cell.get(
            "selector_revision", 0
        ) != self.selector_revision:
            self.settings_seen = True
            for widget, value in (
                (self.task_choice, self.cell.get("requested_mode", "full")),
                (self.product_choice, self.cell.get("requested_count", 0)),
            ):
                widget.blockSignals(True)
                widget.setCurrentIndex(max(0, widget.findData(value)))
                widget.blockSignals(False)
            self.update_fault_choices(self.product_choice.currentData())
            if self.cell.get("qa_trial"):
                self.task_choice.setEnabled(False)
                self.product_choice.setEnabled(False)
        self.selector_revision = self.cell.get("selector_revision", 0)
        key = (
            self.cell["state"],
            self.cell["step"],
            self.cell["cycle"],
            self.cell["mode"],
            self.cell.get("reason", ""),
        )
        if key != self.previous:
            self.log.append(
                f"{time.strftime('%H:%M:%S')}   {key[0]} · {key[3]} · {key[1]} · cycle {key[2]}\n{key[4]}"
            )
            self.previous = key

    def on_conveyor(self, msg):
        self.conveyor.data = json.loads(msg.data)
        self.conveyor.update()

    def on_arm(self, msg):
        self.arm = json.loads(msg.data)
        self.follow.blockSignals(True)
        self.follow.setChecked(self.arm["master_slave"])
        self.follow.setEnabled(self.arm["control"] == "manual")
        self.follow.blockSignals(False)

    def set_follow(self, enabled):
        if self.follow_client.service_is_ready():
            self.follow_client.call_async(SetBool.Request(data=enabled))

    def home(self):
        if self.home_client.service_is_ready():
            self.home_client.call_async(Trigger.Request())

    def tick(self):
        if not rclpy.ok():
            QApplication.quit()
            return
        for _ in range(5):
            rclpy.spin_once(self.node, timeout_sec=0)
        self.panel_pub.publish(UInt32MultiArray(data=self.inputs))
        fresh = self.io is not None and time.monotonic() - self.io_time < 0.75
        # Show the actual returned button state, plus the latched run/pause state.
        for button, colour, active in (
            (
                self.start_button,
                "#218c45",
                bool(self.inputs[0])
                or (fresh and bool(self.io[3]))
                or self.cell.get("state") == "RUNNING"
                or self.cell.get("start_pending", False),
            ),
            (
                self.pause_button,
                "#d12e34",
                bool(self.inputs[1])
                or (fresh and bool(self.io[4]))
                or self.cell.get("state") == "PAUSED",
            ),
        ):
            button.setStyleSheet(
                "QPushButton {"
                f"background: {('#39ff70' if button is self.start_button else '#ff424b') if active else ('#245d38' if button is self.start_button else '#742b31')}; color: white; font-weight: bold; border-radius: 18px;"
                f"border: {4 if active else 1}px solid {'#d8ffe2' if active else '#627585'};"
                "padding: 10px; } QPushButton:pressed { background: #49687a; }"
            )
        emergency = self.estop.isChecked() or self.cell.get("state") == "ESTOP"
        self.estop.setStyleSheet(
            f"background: {'#ff1729' if emergency else '#a71920'}; color: white;"
            "border: 5px solid #ffd239; border-radius: 37px; font-weight: bold; padding: 0px;"
        )
        self.link.setText(
            "Micro-ROS board connected"
            if fresh
            else "Micro-ROS board OFFLINE · outputs unknown"
        )
        colours = ("#ef5350", "#ffd54f", "#66bb6a", "#f28d45")
        for i, lamp in enumerate(self.lamps):
            on = fresh and (self.io[10] == i if i < 3 else bool(self.io[11]))
            lamp.setStyleSheet(
                f"background: {colours[i] if on else '#d8dfe6'}; border-radius: 9px; font-weight: bold;"
            )
        hz = self.io[9] if fresh else 0
        self.conveyor.running = bool(hz)
        self.conveyor.update()
        count = self.cell.get("cycles", 1)
        if len(self.results) != count:
            for item in self.results:
                self.results_row.removeWidget(item)
                item.deleteLater()
            self.results = []
            for toy in range(1, count + 1):
                item = QLabel(str(toy))
                item.setAlignment(Qt.AlignCenter)
                item.setFixedSize(28, 28)
                self.results_row.addWidget(item)
                self.results.append(item)
        for toy, item in enumerate(self.results, 1):
            outcome = self.cell.get("outcomes", {}).get(str(toy), "waiting")
            colour = {
                "pass": "#42a568",
                "assembled": "#3687ba",
                "fail": "#ef5350",
                "bypass": "#e8b443",
            }.get(outcome, "#d8dfe6")
            item.setStyleSheet(
                f"background: {colour}; border: 1px solid #9cabb9; font-weight: bold;"
            )
            item.setToolTip(f"Toy {toy}: {outcome}")
        c = self.cell
        watchdog = "expired" if not fresh or self.io[12] else "healthy"
        power_state = (
            "ON" if self.power else ("OFF" if self.power is False else "waiting")
        )
        arm_state = "ready" if self.arm.get("feedback_ready") else "waiting"
        reason = c.get("reason", "")
        if not reason and not c.get("paths_ready", True) and c.get("planning_task"):
            reason = "Planning: " + c["planning_task"].replace("_", " ")
        detail = (
            f"Cell: {c.get('state', 'waiting')}   |   {c.get('step_label', c.get('step', ''))}\n"
            f"Completed: {c.get('completed', 0)}   Packed: {c.get('packed', 0)}   Rejected: {c.get('rejected', 0)}"
            f"   |   Watchdog: {watchdog}   |   Arms: {arm_state}   |   Actuator power: {power_state}\n{reason}"
        )
        self.status.setToolTip(detail)
        metrics = self.status.fontMetrics()
        self.status.setText(
            "\n".join(
                metrics.elidedText(line, Qt.ElideRight, max(100, self.status.width()))
                for line in detail.splitlines()
            )
        )

    def closeEvent(self, event):
        # Closing the control box removes its healthy feedback immediately.
        self.set_input(2, 0)
        self.timer.stop()
        self.environment.stop()
        self.editor_window.close()
        self.test_window.close()
        self.camera_pub.publish(Bool(data=False))
        if self.camera_process.state() != QProcess.NotRunning:
            self.camera_process.terminate()
            self.camera_process.waitForFinished(3000)
        event.accept()


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    app = QApplication(sys.argv)
    signal.signal(signal.SIGINT, lambda *_: app.quit())
    signal.signal(signal.SIGTERM, lambda *_: app.quit())
    node = Node("project2_control_panel")
    panel = Panel(node)
    panel.show()
    result = app.exec_()
    panel.close()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return result


if __name__ == "__main__":
    sys.exit(main())
