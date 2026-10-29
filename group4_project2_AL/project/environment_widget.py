# Physical ROS feedback stays separate from simulated controls and joint commands.
import json
import os
from pathlib import Path
import time
import socket

import yaml
from PyQt5.QtCore import QProcess, QTimer
from PyQt5.QtWidgets import (
    QGroupBox,
    QVBoxLayout,
    QHBoxLayout,
    QComboBox,
    QPushButton,
    QLabel,
)
from .environment_profile import PROFILES, profile_settings
from .path_store import write_sections


class EnvironmentWidget(QGroupBox):
    def __init__(self, panel):
        super().__init__("Physical ROS connection")
        self.panel = panel
        self.root = Path(os.environ.get("PROJECT2_PROJECT_DIR", "."))
        self.identity_path = self.root / "ID.yaml"
        self.process = QProcess(self)
        self.process.setProcessChannelMode(QProcess.MergedChannels)
        self.process.readyReadStandardOutput.connect(self.read_output)
        self.process.finished.connect(self.finished)
        self.process.errorOccurred.connect(self.failed)
        self.agent = QProcess(self)
        self.agent.setProcessChannelMode(QProcess.MergedChannels)
        self.agent.readyReadStandardOutput.connect(self.read_agent)
        self.buffer = ""
        self.seen = 0.0
        self.last_state = {}
        layout = QVBoxLayout(self)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(4)
        row = QHBoxLayout()
        self.choice = QComboBox()
        for key, (label, _, _) in PROFILES.items():
            self.choice.addItem(label, key)
        self.button = QPushButton("Connect feedback")
        self.button.clicked.connect(self.toggle)
        row.addWidget(self.choice)
        row.addWidget(self.button)
        layout.addLayout(row)
        self.help = QLabel()
        self.help.setWordWrap(True)
        self.help.hide()
        self.status = QLabel(
            "Physical ROS: disconnected · simulation remains independent"
        )
        self.status.setWordWrap(True)
        layout.addWidget(self.status)
        try:
            identity = yaml.safe_load(self.identity_path.read_text())
            key = identity.get("panel_environment", "usb_lab")
            self.choice.setCurrentIndex(max(0, self.choice.findData(key)))
        except (OSError, ValueError, yaml.YAMLError):
            pass
        self.choice.currentIndexChanged.connect(self.selection_changed)
        self.selection_changed(save=False)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.check_fresh)
        self.timer.start(500)

    def selection_changed(self, *_, save=True):
        key = self.choice.currentData()
        location = (
            "ESP32 Wi-Fi → laptop agent. Connect starts the laptop agent if its UDP port is free."
            if key == "wifi_laptop"
            else "ESP32 USB → lab PC agent. On the lab PC: bash ROS_Environment/run_agent.sh usb"
        )
        self.help.setText(
            location
            + "\nBoth computers need the same hardware ROS domain and network. Arm command mapping is pending the lab check."
        )
        self.choice.setToolTip(self.help.text())
        if save:
            try:
                write_sections(self.identity_path, {"panel_environment": key})
            except (OSError, ValueError, yaml.YAMLError) as error:
                self.status.setText(str(error))

    def toggle(self):
        if self.process.state() != QProcess.NotRunning:
            self.stop()
            return
        try:
            settings, local_agent = profile_settings(
                yaml.safe_load(self.identity_path.read_text()),
                self.choice.currentData(),
            )
            if local_agent:
                # Reuse an already-running agent instead of competing for its port.
                with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
                    try:
                        probe.bind(("0.0.0.0", int(settings["network"]["agent_port"])))
                    except OSError:
                        start_agent = False
                    else:
                        start_agent = True
                if start_agent:
                    self.agent.start(
                        "bash",
                        [str(self.root / "ROS_Environment/run_agent.sh"), "wifi"],
                    )
            self.buffer = ""
            self.seen = 0.0
            self.last_state = {}
            self.choice.setEnabled(False)
            self.button.setText("Disconnect feedback")
            self.status.setText(
                "Connecting to physical ROS; waiting for ESP32 telemetry…"
            )
            self.process.start("bash", [str(self.root / "tools/environment_link.sh")])
        except (OSError, ValueError, yaml.YAMLError) as error:
            self.status.setText(str(error))

    def read_output(self):
        self.buffer += bytes(self.process.readAllStandardOutput()).decode(
            errors="replace"
        )
        while "\n" in self.buffer:
            line, self.buffer = self.buffer.split("\n", 1)
            try:
                data = json.loads(line)
            except ValueError:
                if line.strip():
                    self.panel.log.append(line[-500:])
                continue
            if data.get("environment_status"):
                self.last_state = data
                self.seen = time.monotonic()
                self.check_fresh()

    def read_agent(self):
        text = bytes(self.agent.readAllStandardOutput()).decode(errors="replace")
        if text.strip():
            self.panel.log.append("Laptop agent: " + text[-1500:])

    def check_fresh(self):
        if self.process.state() == QProcess.NotRunning:
            return
        data = self.last_state if time.monotonic() - self.seen < 1.5 else {}
        io = data.get("io")
        details = (
            f"ESP32 connected · Start={io[3]} Pause={io[4]} E-stop healthy={io[5]} selector={io[6]}"
            if io
            else "ESP32 offline · check the selected host's agent, firmware transport and network"
        )
        self.status.setText(
            f"{details}\nArm feedback: {len(data.get('arms', {}))}/4 · physical motor commands disabled until driver mapping is checked"
        )

    def failed(self, error):
        self.status.setText("Connection process failed: " + self.process.errorString())
        self.choice.setEnabled(True)
        self.button.setText("Connect feedback")

    def finished(self, code, status):
        self.choice.setEnabled(True)
        self.button.setText("Connect feedback")
        self.status.setText(
            f"Physical ROS disconnected (exit {code}); simulation remains independent"
        )

    def stop(self):
        # End only our observer. Do not stop the remote lab agent or any arm driver.
        self.process.terminate()
        if not self.process.waitForFinished(1500):
            self.process.kill()
            self.process.waitForFinished(1000)
        # Only the Wi-Fi agent launched by this panel belongs to this panel.
        if self.agent.state() != QProcess.NotRunning:
            self.agent.terminate()
            if not self.agent.waitForFinished(1500):
                self.agent.kill()
                self.agent.waitForFinished(1000)
