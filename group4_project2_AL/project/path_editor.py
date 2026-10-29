# Teach measured joint paths while automatic production is paused.
import json
import os
import time
import uuid
from pathlib import Path
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QGroupBox,
    QVBoxLayout,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QListWidget,
    QListWidgetItem,
    QDoubleSpinBox,
    QStackedWidget,
    QWidget,
    QMenu,
    QInputDialog,
    QMessageBox,
    QAbstractItemView,
)
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from .arm_model import JOINTS
from .path_store import angles, save_point, remove_point, load_solved_paths
from .teaching_tasks import ARMS, load_tasks, save_tasks, resolve


class PathEditor(QGroupBox):
    def __init__(self, panel):
        super().__init__("Plan a path · simulation")
        self.panel = panel
        self.feedback, self.solved = {}, {}
        self.path = (
            Path(os.environ.get("PROJECT2_PROJECT_DIR", ".")) / "config/production.yaml"
        )
        self.solved = load_solved_paths(self.path.parent.parent)
        self.tasks = load_tasks(self.path)
        self.arm, self.task = ARMS[0], None
        self.updating = False
        layout = QVBoxLayout(self)
        self.pages = QStackedWidget()
        layout.addWidget(self.pages)
        overview = QWidget()
        columns = QHBoxLayout(overview)
        self.columns = {}
        for index, arm in enumerate(ARMS, 1):
            group = QGroupBox(f"Arm {index}")
            column = QVBoxLayout(group)
            items = QListWidget()
            items.setWordWrap(True)
            items.setDragDropMode(QAbstractItemView.InternalMove)
            items.setContextMenuPolicy(Qt.CustomContextMenu)
            items.customContextMenuRequested.connect(
                lambda pos, a=arm: self.task_menu(a, pos)
            )
            items.itemClicked.connect(
                lambda item, a=arm: self.open_task(a, item.data(Qt.UserRole))
            )
            items.model().rowsMoved.connect(lambda *_, a=arm: self.reorder_tasks(a))
            column.addWidget(items)
            columns.addWidget(group)
            self.columns[arm] = items
        self.pages.addWidget(overview)
        detail = QWidget()
        body = QVBoxLayout(detail)
        heading = QHBoxLayout()
        self.title = QLabel()
        back = QPushButton("▼ Tasks")
        back.clicked.connect(lambda: self.pages.setCurrentIndex(0))
        heading.addWidget(self.title, 1)
        heading.addWidget(back)
        body.addLayout(heading)
        self.live = QLabel("Waiting for joint feedback")
        self.live.setWordWrap(True)
        body.addWidget(self.live)
        self.movements = QListWidget()
        self.movements.setDragDropMode(QAbstractItemView.InternalMove)
        self.movements.setContextMenuPolicy(Qt.CustomContextMenu)
        self.movements.customContextMenuRequested.connect(self.movement_menu)
        self.movements.model().rowsMoved.connect(self.reorder_movements)
        body.addWidget(self.movements)
        row = QHBoxLayout()
        save = QPushButton("Save coordinate")
        save.clicked.connect(self.capture)
        one = QPushButton("Complete selected movement")
        one.clicked.connect(self.complete_one)
        all_moves = QPushButton("Complete selected task")
        all_moves.clicked.connect(self.complete_task)
        for button in (save, one, all_moves):
            row.addWidget(button)
        body.addLayout(row)
        body.addWidget(QLabel("Manual joint input (rad)"))
        row = QHBoxLayout()
        self.joints = []
        for name in JOINTS:
            box = QDoubleSpinBox()
            box.setRange(-6.3, 6.3)
            box.setDecimals(4)
            box.setSingleStep(0.02)
            box.setToolTip(name + " (rad)")
            self.joints.append(box)
            row.addWidget(box)
        body.addLayout(row)
        move = QPushButton("Complete input movement")
        move.clicked.connect(
            lambda: self.request_moves([[box.value() for box in self.joints]])
        )
        body.addWidget(move)
        self.pages.addWidget(detail)
        self.message = QLabel(
            "Right-click a task column to create a path. Drag tasks or movements to reorder replay. Production dependencies are separate."
        )
        self.message.setWordWrap(True)
        layout.addWidget(self.message)
        for arm in ARMS:
            panel.node.create_subscription(
                JointState,
                f"/{arm}/joint_state_broadcaster/joint_states",
                lambda msg, a=arm: self.receive(a, msg),
                qos_profile_sensor_data,
            )
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        panel.node.create_subscription(String, "/project2/sim/paths", self.plans, qos)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_live)
        self.timer.start(100)
        self.refresh()

    def receive(self, arm, message):
        values = dict(zip(message.name, message.position))
        if not all(name in values for name in JOINTS):
            return
        try:
            q = angles([values[name] for name in JOINTS])
        except ValueError:
            return
        now = time.monotonic()
        old = self.feedback.get(arm)
        stable = (
            old[2]
            if old and max(abs(a - b) for a, b in zip(old[3], q)) < 0.003
            else now
        )
        anchor = old[3] if old and stable == old[2] else q
        self.feedback[arm] = (q, now, stable, anchor)

    def update_live(self):
        value = self.feedback.get(self.arm)
        if value and time.monotonic() - value[1] < 0.5:
            self.live.setText(
                "Live joints (rad): " + " · ".join(f"{q:.4f}" for q in value[0])
            )
        else:
            self.live.setText("Joint feedback offline · coordinate capture disabled")

    def plans(self, message):
        self.solved = json.loads(message.data)
        self.refresh()

    def current(self, settled=False):
        if self.panel.cell.get("state") != "TEACH":
            raise ValueError("Wait for Plan a path mode")
        value = self.feedback.get(self.arm)
        now = time.monotonic()
        if not value or now - value[1] > 0.5:
            raise ValueError("No fresh feedback for the selected arm")
        if settled and now - value[2] < 0.5:
            raise ValueError("Wait for the arm to stop before saving")
        return value[0]

    def refresh(self):
        self.updating = True
        for arm, items in self.columns.items():
            items.clear()
            for index, task in enumerate(self.tasks[arm], 1):
                item = QListWidgetItem(f"{index}. {task['label']}")
                item.setData(Qt.UserRole, task["id"])
                items.addItem(item)
        self.movements.clear()
        if self.task:
            self.title.setText(f"Arm {ARMS.index(self.arm)+1} · {self.task['label']}")
            for index, (movement, q) in enumerate(
                zip(self.task["movements"], resolve(self.task, self.solved)), 1
            ):
                text = (
                    ", ".join(f"{v:.4f}" for v in q)
                    if q is not None
                    else "Needs a valid path or measured capture"
                )
                item = QListWidgetItem(f"{index}. {movement['label']}\n{text}")
                item.setData(Qt.UserRole, movement["id"])
                self.movements.addItem(item)
        self.updating = False

    def persist(self):
        save_tasks(self.path, self.tasks)
        self.refresh()

    def open_task(self, arm, key):
        self.arm = arm
        self.task = next(task for task in self.tasks[arm] if task["id"] == key)
        self.refresh()
        self.pages.setCurrentIndex(1)
        self.update_live()

    def task_menu(self, arm, pos):
        items = self.columns[arm]
        item = items.itemAt(pos)
        menu = QMenu(self)
        create = menu.addAction("Create new task")
        delete = menu.addAction("Delete task") if item else None
        selected = menu.exec_(items.mapToGlobal(pos))
        try:
            if selected == create:
                name, ok = QInputDialog.getText(self, "New task", "Task name")
                if ok and name.strip():
                    self.tasks[arm].append(
                        dict(id=uuid.uuid4().hex, label=name.strip(), movements=[])
                    )
                    self.persist()
            elif delete is not None and selected == delete:
                if (
                    QMessageBox.question(
                        self,
                        "Delete task",
                        "Delete this taught task? Automatic production actions are edited in production_run.py.",
                    )
                    == QMessageBox.Yes
                ):
                    self.tasks[arm] = [
                        task
                        for task in self.tasks[arm]
                        if task["id"] != item.data(Qt.UserRole)
                    ]
                    self.task = None
                    self.persist()
        except (OSError, ValueError) as error:
            self.message.setText(str(error))

    def reorder_tasks(self, arm):
        if self.updating:
            return
        items = self.columns[arm]
        by_id = {task["id"]: task for task in self.tasks[arm]}
        self.tasks[arm] = [
            by_id[items.item(i).data(Qt.UserRole)] for i in range(items.count())
        ]
        try:
            self.persist()
        except (OSError, ValueError) as error:
            self.message.setText(str(error))

    def reorder_movements(self, *_):
        if self.updating or self.task is None:
            return
        by_id = {point["id"]: point for point in self.task["movements"]}
        self.task["movements"] = [
            by_id[self.movements.item(i).data(Qt.UserRole)]
            for i in range(self.movements.count())
        ]
        try:
            self.persist()
        except (OSError, ValueError) as error:
            self.message.setText(str(error))

    def capture(self):
        try:
            q = self.current(True)
            name, ok = QInputDialog.getText(self, "Save coordinate", "Movement name")
            if ok and name.strip():
                self.task["movements"].append(
                    dict(id=uuid.uuid4().hex, label=name.strip(), joints=q)
                )
                self.persist()
                self.message.setText(
                    "Saved measured joints to config/production.yaml · teaching_tasks"
                )
        except (OSError, ValueError) as error:
            self.message.setText(str(error))

    def movement_menu(self, pos):
        item = self.movements.itemAt(pos)
        if item is None:
            return
        movement = next(
            point
            for point in self.task["movements"]
            if point["id"] == item.data(Qt.UserRole)
        )
        menu = QMenu(self)
        overwrite = menu.addAction("Overwrite with current arm joints")
        delete = menu.addAction("Delete movement")
        selected = menu.exec_(self.movements.mapToGlobal(pos))
        try:
            if selected == overwrite:
                q = self.current(True)
                movement["joints"] = q
                if self.task.get("action") and movement.get("point"):
                    save_point(self.path, self.task["action"], movement["point"], q)
                self.persist()
                self.message.setText(
                    "Saved. A named production target also updates joint_overrides; Reset applies it."
                )
            elif selected == delete:
                if (
                    QMessageBox.question(
                        self,
                        "Delete movement",
                        "Delete this movement? A saved production override will revert to its calculated target.",
                    )
                    == QMessageBox.Yes
                ):
                    if self.task.get("action") and movement.get("point"):
                        remove_point(self.path, self.task["action"], movement["point"])
                    self.task["movements"].remove(movement)
                    self.persist()
        except (OSError, ValueError) as error:
            self.message.setText(str(error))

    def complete_one(self):
        row = self.movements.currentRow()
        if row < 0:
            self.message.setText("Select a movement first")
            return
        self.request_moves([resolve(self.task, self.solved)[row]])

    def complete_task(self):
        self.request_moves(resolve(self.task, self.solved))

    def request_moves(self, sequence):
        try:
            self.current(True)
            if not sequence or any(q is None for q in sequence):
                raise ValueError("Capture or solve every selected movement first")
            self.panel.send_event(
                json.dumps(
                    dict(
                        teach_move=True,
                        arm=self.arm,
                        sequence=[angles(q) for q in sequence],
                    )
                )
            )
            self.message.setText(
                "Path submitted. Each move waits for joint feedback and checks other-arm clearance; no part attachments are added by replay."
            )
        except ValueError as error:
            self.message.setText(str(error))
