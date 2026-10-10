"""Flying-Dalgu-in-ISS flight controls.

Construct the visible panel after SimulationApp starts. Headless dispatch does
not import Kit modules, so automation uses the same actions as the GUI.
"""

from __future__ import annotations

import math
from collections.abc import Callable, Mapping
from typing import Any


class ISSControlPanel:
    """Latched flight commands, camera selection, and live telemetry.

    ``on_command(forward_mps, yaw_radps, vertical_mps)`` controls local forward
    motion, left turns around world Z, and vertical motion along world Z.
    ``on_coast()`` releases control while preserving the body's velocity.
    ``on_camera(mode)`` accepts follow, overview, rover, or interior. The rover
    identifier preserves the source panel's camera automation interface.

    Call ``update(status)`` on the Kit main thread and ``shutdown()`` before
    closing SimulationApp. Painting and telemetry updates never send commands.
    """

    TITLE = "ISS Flight Controller"
    ACTIONS = (
        "forward", "reverse", "left", "right", "straight",
        "up", "level", "down", "stop", "coast",
        "pause", "reset", "follow", "overview", "rover", "interior",
    )

    def __init__(
        self,
        on_command: Callable[[float, float, float], Any],
        on_pause: Callable[[], Any],
        on_reset: Callable[[], Any],
        on_camera: Callable[[str], Any],
        *,
        on_coast: Callable[[], Any],
        build_ui: bool = True,
        position: tuple[float, float] = (1020, 80),
    ) -> None:
        self._on_command = on_command
        self._on_pause = on_pause
        self._on_reset = on_reset
        self._on_camera = on_camera
        self._on_coast = on_coast
        self.requested_v = 0.0
        self.requested_w = 0.0
        self.requested_z = 0.0
        self.speed_magnitude = 0.25
        self.turn_magnitude = 0.12
        self._closed = False
        self._syncing_models = False
        self._labels: dict[str, Any] = {}
        self._speed_model = None
        self._turn_model = None
        self._pause_button = None
        self._model_callbacks: list[tuple[Any, Any]] = []
        self.window = None
        if build_ui:
            self._build_ui(position)

    def _build_ui(self, position: tuple[float, float]) -> None:
        # omni.ui is only available after SimulationApp creates the Kit runtime.
        import omni.ui as ui

        self.window = ui.Window(
            self.TITLE,
            width=380,
            height=650,
            position_x=position[0],
            position_y=position[1],
            dockPreference=ui.DockPreference.DISABLED,
        )
        with self.window.frame:
            with ui.VStack(spacing=4, margin=12):
                ui.Label("ISS / Dalgu", height=25, style={"font_size": 20})
                self._labels["state"] = ui.Label("State: Ready", height=20)

                ui.Label("Flight speed (m/s)", height=18)
                self._speed_model = ui.FloatSlider(
                    min=0.0, max=0.6, step=0.01, precision=2, height=24,
                    identifier="iss_speed_magnitude",
                ).model
                self._speed_model.set_value(self.speed_magnitude)
                ui.Label("Turn rate (rad/s)", height=18)
                self._turn_model = ui.FloatSlider(
                    min=0.0, max=0.3, step=0.01, precision=2, height=24,
                    identifier="iss_turn_magnitude",
                ).model
                self._turn_model.set_value(self.turn_magnitude)

                with ui.HStack(height=36, spacing=6):
                    self._button(ui, "Forward", "forward")
                    self._button(ui, "Reverse", "reverse")
                with ui.HStack(height=36, spacing=6):
                    self._button(ui, "Left", "left")
                    self._button(ui, "Straight", "straight")
                    self._button(ui, "Right", "right")
                with ui.HStack(height=36, spacing=6):
                    self._button(ui, "Up", "up")
                    self._button(ui, "Level", "level")
                    self._button(ui, "Down", "down")
                self._button(
                    ui, "STOP", "stop", height=42,
                    style={
                        "Button": {"background_color": 0xFF3434A3, "border_radius": 5},
                        "Button:hovered": {"background_color": 0xFF4545C0},
                        "Button:pressed": {"background_color": 0xFF24247F},
                        "Button.Label": {"font_size": 18, "color": 0xFFFFFFFF},
                    },
                )
                self._button(ui, "Coast / Free drift", "coast", height=30)
                self._labels["command"] = ui.Label("", height=22)
                ui.Separator(height=2)
                with ui.HStack(height=30, spacing=6):
                    self._pause_button = self._button(ui, "Pause / Resume", "pause")
                    self._button(ui, "Reset spawn", "reset")
                with ui.HStack(height=30, spacing=6):
                    self._button(ui, "Follow view", "follow")
                    self._button(ui, "Overview", "overview")
                with ui.HStack(height=30, spacing=6):
                    self._button(ui, "Dalgu view", "rover")
                    self._button(ui, "Interior view", "interior")
                self._labels["position"] = ui.Label("Position: --", height=20)
                self._labels["motion"] = ui.Label("Speed: --   Turn rate: --", height=20)
                self._labels["support"] = ui.Label(
                    "Zero gravity. Static ISS surface collision is active.",
                    height=32, word_wrap=True,
                )
                ui.Label(
                    "Commands stay active until STOP or Coast.\n"
                    "Up/Down use world Z; Coast preserves momentum.",
                    height=34, word_wrap=True, style={"color": 0xFFBFC9D3},
                )

        self._model_callbacks = [
            (self._speed_model, self._speed_model.add_value_changed_fn(self._speed_changed)),
            (self._turn_model, self._turn_model.add_value_changed_fn(self._turn_changed)),
        ]
        self.window.set_visibility_changed_fn(self._visibility_changed)
        self._refresh_command_label()

    def _button(self, ui: Any, label: str, action: str, **kwargs: Any) -> Any:
        return ui.Button(
            label, identifier=f"iss_action_{action}",
            clicked_fn=lambda: self.press(action), **kwargs,
        )

    def press(self, action: str) -> Any:
        """Dispatch an action exactly as its corresponding UI button does."""
        if self._closed:
            return None
        action = action.strip().lower()
        if action == "forward":
            return self._send(self.speed_magnitude, self.requested_w, self.requested_z)
        if action == "reverse":
            return self._send(-self.speed_magnitude, self.requested_w, self.requested_z)
        if action == "left":
            return self._send(self.requested_v, self.turn_magnitude, self.requested_z)
        if action == "right":
            return self._send(self.requested_v, -self.turn_magnitude, self.requested_z)
        if action == "straight":
            return self._send(self.requested_v, 0.0, self.requested_z)
        if action == "up":
            return self._send(self.requested_v, self.requested_w, self.speed_magnitude)
        if action == "level":
            return self._send(self.requested_v, self.requested_w, 0.0)
        if action == "down":
            return self._send(self.requested_v, self.requested_w, -self.speed_magnitude)
        if action == "stop":
            return self._send(0.0, 0.0, 0.0)
        if action == "coast":
            # Clearing the display must not brake the body before releasing it.
            self.sync_command(0.0, 0.0, 0.0)
            return self._on_coast()
        if action == "pause":
            self._send(0.0, 0.0, 0.0)
            return self._on_pause()
        if action == "reset":
            self._send(0.0, 0.0, 0.0)
            return self._on_reset()
        if action in ("follow", "overview", "rover", "interior"):
            return self._on_camera(action)
        raise ValueError(f"Unknown ISS action {action!r}; choose from {self.ACTIONS}")

    def _send(self, v_mps: float, w_radps: float, z_mps: float) -> Any:
        self.sync_command(v_mps, w_radps, z_mps)
        return self._on_command(self.requested_v, self.requested_w, self.requested_z)

    def sync_command(self, v_mps: float, w_radps: float, z_mps: float) -> None:
        """Reflect a runtime command change without invoking control callbacks."""
        if self._closed:
            return
        self.requested_v = float(v_mps)
        self.requested_w = float(w_radps)
        self.requested_z = float(z_mps)
        self._refresh_command_label()

    def set_magnitudes(self, speed: float | None = None, turn: float | None = None) -> None:
        """Set slider magnitudes; preserve signs of active motion commands."""
        if self._closed:
            return
        new_speed = self.speed_magnitude if speed is None else self._bounded(speed, 0.6)
        new_turn = self.turn_magnitude if turn is None else self._bounded(turn, 0.3)
        v = math.copysign(new_speed, self.requested_v) if self.requested_v else 0.0
        w = math.copysign(new_turn, self.requested_w) if self.requested_w else 0.0
        z = math.copysign(new_speed, self.requested_z) if self.requested_z else 0.0
        self.speed_magnitude, self.turn_magnitude = new_speed, new_turn
        self._syncing_models = True
        try:
            if self._speed_model is not None:
                self._speed_model.set_value(new_speed)
                self._turn_model.set_value(new_turn)
        finally:
            self._syncing_models = False
        if (v, w, z) != (self.requested_v, self.requested_w, self.requested_z):
            self._send(v, w, z)

    @staticmethod
    def _bounded(value: float, maximum: float) -> float:
        value = float(value)
        if not math.isfinite(value):
            raise ValueError("Control magnitudes must be finite")
        return min(max(value, 0.0), maximum)

    def _speed_changed(self, model: Any) -> None:
        if not self._syncing_models:
            self.set_magnitudes(speed=model.get_value_as_float())

    def _turn_changed(self, model: Any) -> None:
        if not self._syncing_models:
            self.set_magnitudes(turn=model.get_value_as_float())

    def _refresh_command_label(self) -> None:
        self._set_label(
            "command",
            f"Cmd: {self.requested_v:+.2f} m/s   "
            f"Yaw: {self.requested_w:+.2f}   Z: {self.requested_z:+.2f}",
        )

    def _set_label(self, key: str, text: str) -> None:
        label = self._labels.get(key)
        if label is not None and label.text != text:
            label.text = text

    def update(self, status: Mapping[str, Any]) -> None:
        """Refresh telemetry without sending a flight command.

        Accepted keys: position (xyz), speed (m/s), yawrate (rad/s), state,
        timeline (string or bool), support (text). ``playing`` optionally sets
        the Pause/Resume label directly. Call this on the Kit main thread.
        """
        if self._closed:
            return
        state = status.get("state", status.get("timeline", "Ready"))
        if isinstance(state, bool):
            state = "Running" if state else "Paused"
        self._set_label("state", f"State: {state}")
        position = status.get("position")
        if position is not None:
            x, y, z = (float(c) for c in position)
            self._set_label("position", f"Position: X {x:.2f}   Y {y:.2f}   Z {z:.2f} m")
        if "speed" in status or "yawrate" in status:
            speed = self._number(status.get("speed"))
            yawrate = self._number(status.get("yawrate"))
            self._set_label("motion", f"Speed: {speed} m/s   Turn: {yawrate} rad/s")
        if "support" in status:
            self._set_label("support", str(status["support"]))
        playing = status.get("playing")
        timeline = status.get("timeline")
        if playing is None and isinstance(timeline, bool):
            playing = timeline
        elif playing is None and isinstance(timeline, str):
            lowered = timeline.lower()
            if lowered in ("running", "playing", "play"):
                playing = True
            elif lowered in ("paused", "stopped", "pause", "stop"):
                playing = False
        if self._pause_button is not None and playing is not None:
            text = "Pause" if playing else "Resume"
            if self._pause_button.text != text:
                self._pause_button.text = text

    @staticmethod
    def _number(value: Any) -> str:
        return "--" if value is None else f"{float(value):+.2f}"

    def _visibility_changed(self, visible: bool) -> None:
        if not visible and not self._closed:
            self.press("stop")

    def show(self) -> None:
        if self.window is not None and not self._closed:
            self.window.visible = True

    def shutdown(self) -> None:
        """Stop motion and release UI callbacks before the runtime closes."""
        if self._closed:
            return
        try:
            self._send(0.0, 0.0, 0.0)
        finally:
            self._closed = True
            for model, callback in self._model_callbacks:
                model.remove_value_changed_fn(callback)
            self._model_callbacks.clear()
            if self.window is not None:
                self.window.set_visibility_changed_fn(None)
                self.window.destroy()
                self.window = None
            self._labels.clear()
            self._speed_model = self._turn_model = self._pause_button = None

    close = shutdown
