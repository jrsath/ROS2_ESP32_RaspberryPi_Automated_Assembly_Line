# Product-independent process state for the virtual control box.
import random


ASSEMBLY = [
    "pickup_components",
    "assembly_task_1",
    "assembly_task_2",
    "assembly_task_3",
    "place_on_conveyor",
]
INSPECTION = ["transfer_to_inspection", "inspect"]
PACKAGING = [
    "pickup_box",
    "pickup_assembly",
    "place_in_box",
    "pickup_lid",
    "place_lid",
    "pickup_package",
    "stack_package",
]
MODES = ("full", "assembly", "inspection", "packaging")
TASK_LABELS = {
    "pickup_components": "Arm 1 picks 2V4; arm 2 picks 1V4",
    "assembly_task_1": "Align 2V4 over the top pin",
    "assembly_task_2": "Seat 2V4 while arm 2 holds the base",
    "assembly_task_3": "Insert 3V4 from the open side of 1V4",
}


class Cell:
    # One product at a time; conveyor and packaging handshakes are separate.

    def __init__(self, mode="full", cycles=5, step_seconds=0.5, seed=4):
        if mode not in MODES or cycles < 1 or not 0 < step_seconds <= 60:
            raise ValueError("Invalid mode, cycle count or simulation step time")
        self.mode, self.cycles, self.step_seconds = mode, cycles, step_seconds
        self.rng = random.Random(seed)
        self.quality = ["good"] * cycles
        # The handout QA trial mixes two different defects with four good items.
        if mode == "inspection":
            self.quality = ["good"] * 4 + ["flaw_1", "flaw_2"]
            self.rng.shuffle(self.quality)
            self.cycles = 6
        self.state = "READY"
        self.estop_pressed = False
        self.packaging_occupied = False
        self.index = 0
        self.step_index = 0
        self.elapsed = 0.0
        self.cycle_elapsed = 0.0
        self.packed = 0
        self.rejected = 0
        self.completed = 0
        self.events = []
        self.reason = ""
        self.plan = []
        self._build_plan()

    def _build_plan(self):
        if self.index >= self.cycles:
            self.plan = []
            return
        q = self.quality[self.index]
        if self.mode == "assembly":
            self.plan = ASSEMBLY[:]
        elif self.mode == "packaging":
            self.plan = PACKAGING[:]
        else:
            self.plan = (ASSEMBLY[:] if self.mode == "full" else []) + INSPECTION
            self.plan += (
                ["reject_product"] if q != "good" else ["transfer_to_packaging"]
            )
            if self.mode == "full" and q == "good":
                self.plan += PACKAGING

    @property
    def step(self):
        return self.plan[self.step_index] if self.plan else "none"

    def event(self, name):
        # E-stop is latched: release alone never restarts the simulation.
        if name == "estop":
            self.estop_pressed = True
            self.state, self.reason = "ESTOP", "Emergency stop: reset required"
        elif name == "release_estop":
            self.estop_pressed = False
        elif name == "fault":
            if self.state != "ESTOP":
                self.state, self.reason = "FAULT", "Simulated subsystem fault"
        elif name == "occupy":
            self.packaging_occupied = True
        elif name == "clear":
            self.packaging_occupied = False
        elif name == "pause" and self.state in ("RUNNING", "WAIT_CLEAR"):
            self.state = "PAUSED"
        elif name == "start" and self.state in ("READY", "PAUSED"):
            if not self.estop_pressed:
                self.state = "RUNNING"
        elif name == "reset":
            if self.estop_pressed:
                return
            # Restart this unfinished product; do not count it as a success.
            if self.state in ("FAULT", "ESTOP"):
                self.step_index = 0
                self.elapsed = self.cycle_elapsed = 0.0
                self.state, self.reason = "READY", ""
        elif name not in ("start", "pause"):
            if name not in ("reset",):
                raise ValueError("Unknown event: " + name)

    def tick(self, dt, motion_ready=True):
        if dt < 0:
            raise ValueError("Time cannot run backwards")
        if self.state not in ("RUNNING", "WAIT_CLEAR"):
            return
        # A blocked packaging wait area prevents conveyor motion or new deposit.
        blocked = self.packaging_occupied and self.step in (
            "place_on_conveyor",
            "transfer_to_inspection",
            "transfer_to_packaging",
        )
        if blocked:
            self.state = "WAIT_CLEAR"
            self.cycle_elapsed += dt
            return
        self.state = "RUNNING"
        self.elapsed += dt
        self.cycle_elapsed += dt
        if self.elapsed < self.step_seconds or not motion_ready:
            return
        self.events.append(
            {
                "event": "step_complete",
                "cycle": self.index + 1,
                "step": self.step,
                "quality": self.quality[self.index],
                "duration_s": round(self.elapsed, 6),
            }
        )
        self.elapsed = 0.0
        self.step_index += 1
        if self.step_index < len(self.plan):
            return
        q = self.quality[self.index]
        self.completed += 1
        if q != "good" and self.mode in ("inspection", "full"):
            self.rejected += 1
        elif self.mode in ("packaging", "full"):
            self.packed += 1
        self.events.append(
            {
                "event": "cycle_complete",
                "cycle": self.index + 1,
                "step": self.plan[-1],
                "quality": q,
                "duration_s": round(self.cycle_elapsed, 6),
            }
        )
        self.index += 1
        self.step_index = 0
        self.cycle_elapsed = 0.0
        self._build_plan()
        if self.index == self.cycles:
            self.state = "COMPLETE"

    def snapshot(self):
        active = self.state == "RUNNING"
        return {
            "simulation": True,
            "product_selected": True,
            "mode": self.mode,
            "state": self.state,
            "step": self.step,
            "reason": self.reason,
            "step_label": TASK_LABELS.get(self.step, self.step.replace("_", " ")),
            "contact_assembly_verified": False,
            "cycle": min(self.index + 1, self.cycles),
            "cycles": self.cycles,
            "completed": self.completed,
            "packed": self.packed,
            "rejected": self.rejected,
            "packaging_occupied": self.packaging_occupied,
            "conveyor_running": active
            and not self.packaging_occupied
            and self.step in ("transfer_to_inspection", "transfer_to_packaging"),
            "tower": (
                "red"
                if self.state in ("ESTOP", "FAULT", "PAUSED")
                else "yellow" if self.state in ("RUNNING", "WAIT_CLEAR") else "green"
            ),
            "alarm": self.state == "ESTOP",
        }
