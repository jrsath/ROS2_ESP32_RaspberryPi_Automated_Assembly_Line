# A delayed load reply must not trigger a duplicate load of the same controller.
from pathlib import Path
from types import SimpleNamespace
import importlib.util


def test_controller_load_recovers_existing_state(monkeypatch):
    path = (
        Path(__file__).resolve().parents[1]
        / "vendor/group4_assembly_line/scripts/start_controllers.py"
    )
    spec = importlib.util.spec_from_file_location("controller_start", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    state, loads = {}, []

    def listing(*args, **kwargs):
        return SimpleNamespace(
            controller=[SimpleNamespace(name=n, state=s) for n, s in state.items()]
        )

    def load(node, manager, name, **kwargs):
        assert kwargs["call_timeout"] == 90.0
        assert name not in state
        loads.append(name)
        state[name] = "unconfigured"
        return SimpleNamespace(ok=False)

    def configure(node, manager, name, **kwargs):
        state[name] = "inactive"
        return SimpleNamespace(ok=True)

    def switch(node, manager, stop, names, *args, **kwargs):
        state.update({name: "active" for name in names})
        return SimpleNamespace(ok=True)

    monkeypatch.setattr(module, "list_controllers", listing)
    monkeypatch.setattr(module, "load_controller", load)
    monkeypatch.setattr(module, "configure_controller", configure)
    monkeypatch.setattr(module, "switch_controllers", switch)
    node = SimpleNamespace(get_logger=lambda: SimpleNamespace(info=lambda text: None))
    module.start(node, "/test/controller_manager")
    module.start(node, "/test/controller_manager")
    assert len(loads) == 3
    assert set(state.values()) == {"active"}
