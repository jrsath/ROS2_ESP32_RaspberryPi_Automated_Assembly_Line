# Check the desktop controls without starting Gazebo or connecting to hardware.
import importlib.util
import os
from pathlib import Path
import shutil

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
import pytest
import rclpy
from PyQt5.QtWidgets import QApplication
from std_msgs.msg import Bool

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("control_panel_test", ROOT / "scripts/control_panel.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


@pytest.fixture
def panel(tmp_path, monkeypatch):
    monkeypatch.setenv("ROS_DOMAIN_ID", "96")
    monkeypatch.setenv("PROJECT2_PROJECT_DIR", str(tmp_path))
    (tmp_path / "config").mkdir()
    shutil.copy(ROOT / "config/production.yaml", tmp_path / "config/production.yaml")
    app = QApplication.instance() or QApplication([])
    rclpy.init()
    node = rclpy.create_node("panel_unit_test")
    widget = module.Panel(node)
    widget.show()
    app.processEvents()
    yield widget
    widget.close()
    node.destroy_node()
    rclpy.shutdown()


def test_five_products_fit_and_dialogs_stay_nonmodal(panel):
    panel.product_choice.setCurrentIndex(5)
    panel.cell = dict(state="PAUSED", cycles=5, step_label="Insert seating", reason="Paused")
    panel.tick()
    QApplication.processEvents()
    assert panel.height() <= 700
    assert len(panel.results) == 5
    assert panel.results[0].y() > panel.conveyor.y()
    panel.test_window.show()
    panel.editor_window.show()
    assert not panel.test_window.isModal() and not panel.editor_window.isModal()
    assert panel.pause_button.isEnabled() and panel.estop.isEnabled()
    assert "border-radius: 37px" in panel.estop.styleSheet()


def test_fault_count_tracks_product_count(panel):
    panel.product_choice.setCurrentIndex(5)
    panel.fault_count.setCurrentIndex(4)
    panel.product_choice.setCurrentIndex(2)
    assert panel.fault_count.count() == 3
    assert panel.fault_count.currentData() == 2
    panel.fault_position.setCurrentIndex(panel.fault_position.findData("custom"))
    assert panel.fault_indices.isEnabled()


def test_start_cancels_virtual_pause_without_touching_estop(panel):
    panel.pulse(1)
    assert panel.inputs[1] == 1
    panel.pulse(0)
    assert panel.inputs[:3] == [1, 0, 1]
    panel.cell = {"state": "RUNNING"}
    panel.tick()
    assert "#39ff70" in panel.start_button.styleSheet()
    assert "#742b31" in panel.pause_button.styleSheet()


def test_camera_caption_follows_viewer_acknowledgement(panel):
    panel.on_camera(Bool(data=True))
    assert panel.camera_button.text() == "Close camera"
    panel.toggle_camera()
    assert panel.camera_requested is False
    panel.on_camera(Bool(data=False))
    assert panel.camera_requested is None
    assert panel.camera_button.text() == "Open camera"
