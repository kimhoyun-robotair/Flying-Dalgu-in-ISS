"""Flying-Dalgu-in-ISS flight runtime.

Create ISSRuntime after SimulationApp opens iss_sim.usda. The runtime adds its
cameras and transient state to the session layer only. Manual flight uses
bounded PhysX forces and torques; only an explicit reset changes the body pose.
"""

from __future__ import annotations

import json
import math
import os
from pathlib import Path
import time


class ISSRuntime:
    DT = 1.0 / 120.0
    MAX_SPEED = 0.6
    MAX_YAW_RATE = 0.3
    MAX_ACCELERATION = 0.8
    MAX_ANGULAR_ACCELERATION = 0.9
    BODY_PATH = "/World/Dalgu"

    def __init__(self, app, headless=False, output=None):
        import numpy as np
        import omni.timeline
        import omni.usd
        from isaacsim.core.api import World
        from isaacsim.core.prims import RigidPrim
        from omni.physx import get_physx_scene_query_interface
        from pxr import Gf, Usd, UsdGeom, UsdPhysics
        from control_panel import ISSControlPanel

        self.app = app
        self.headless = bool(headless)
        self.output = Path(output or Path(__file__).resolve().parents[1] / "outputs/controller")
        self.output.mkdir(parents=True, exist_ok=True)
        self.stage = omni.usd.get_context().get_stage()
        if self.stage is None or not self.stage.GetPrimAtPath(self.BODY_PATH):
            raise RuntimeError("Open iss_sim.usda before constructing ISSRuntime")
        self.stage.SetEditTarget(self.stage.GetSessionLayer())
        self.timeline = omni.timeline.get_timeline_interface()
        self.events = []
        self.sim_seconds = 0.0
        self.episode_time = 0.0
        self.episode = 0
        self.steps = 0
        self.request = (0.0, 0.0, 0.0)
        self.control_active = False
        self.initializing = True
        self.restart_pending = False
        self.reset_pending = False
        self.camera_mode = "interior"
        self.last_status_wall = 0.0
        self._closed = False
        self.last_force = np.zeros(3)
        self.last_torque = np.zeros(3)

        prim = self.stage.GetPrimAtPath(self.BODY_PATH)
        transform = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        # Asset height is a uniform root scale. Extract a pure unit rotation;
        # PhysX rejects quaternions taken directly from a scaled USD matrix.
        rotation = transform.RemoveScaleShear().ExtractRotationQuat().GetNormalized()
        self.spawn = np.asarray(transform.ExtractTranslation(), dtype=np.float32)
        self.spawn_orientation = np.asarray(
            [rotation.GetReal(), *rotation.GetImaginary()], dtype=np.float32
        )
        physics = UsdPhysics.RigidBodyAPI(prim)
        # USD angular velocity is degrees/s; the tensor API uses radians/s.
        self.initial_velocity = np.concatenate((
            np.asarray(physics.GetVelocityAttr().Get() or (0, 0, 0), dtype=float),
            np.radians(physics.GetAngularVelocityAttr().Get() or (0, 0, 0)),
        )).astype(np.float32)
        self.last_pose = (self.spawn.copy(), self.spawn_orientation.copy())
        self.last_velocity = self.initial_velocity.copy()
        bounds = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_]).ComputeUntransformedBound(prim).ComputeAlignedRange()
        minimum, maximum = bounds.GetMin(), bounds.GetMax()
        self.onboard_offset = np.asarray((
            0.0, float(minimum[1]) - 0.08,
            float(minimum[2] + 0.58 * (maximum[2] - minimum[2])),
        ))

        self.world = World(
            stage_units_in_meters=1.0, physics_dt=self.DT,
            rendering_dt=self.DT, physics_prim_path="/World/PhysicsScene",
            backend="numpy", device="cpu",
        )
        self.world.get_physics_context().set_gravity(0.0)
        self.body = RigidPrim(self.BODY_PATH, name="iss_dalgu", reset_xform_properties=False)
        self.world.scene.add(self.body)
        self.body.set_default_state(
            positions=self.spawn[None, :], orientations=self.spawn_orientation[None, :],
            linear_velocities=np.zeros((1, 3), dtype=np.float32),
            angular_velocities=np.zeros((1, 3), dtype=np.float32),
        )
        self.scene_query = get_physx_scene_query_interface()
        self.cameras = {"interior": UsdGeom.Camera.Get(self.stage, "/World/InteriorCamera")}
        for mode in ("follow", "overview", "rover"):
            camera_name = "Dalgu" if mode == "rover" else mode.title()
            camera = UsdGeom.Camera.Define(self.stage, "/World/FlightViews/" + camera_name)
            camera.CreateFocalLengthAttr(18.0 if mode == "rover" else 20.0)
            camera.CreateHorizontalApertureAttr(36.0)
            camera.CreateVerticalApertureAttr(20.25)
            camera.CreateClippingRangeAttr(Gf.Vec2f(0.015, 150.0))
            self.cameras[mode] = camera
        self.position_camera("overview", (1.3, 0.65, 5.35), (4.0, 0.0, 4.85))
        self.timeline_subscription = self.timeline.get_timeline_event_stream().create_subscription_to_pop(self.on_timeline)
        # Initialization leaves the timeline paused so the launcher can warm up
        # rendering without consuming any of the initial free-flight motion.
        self._reset(initial=True, resume=False)
        panel_x = 20
        if not self.headless:
            import omni.appwindow
            panel_x = max(20, omni.appwindow.get_default_app_window().get_width() - 405)
        self.panel = ISSControlPanel(
            self.command, self.pause, self.queue_reset, self.camera,
            on_coast=self.coast, build_ui=not self.headless, position=(panel_x, 80),
        )
        self.camera("interior")
        self._update_status()
        self.event("ready", pid=os.getpid(), spawn=self.spawn.tolist(), dt=self.DT)

    def event(self, kind, **values):
        record = {"event": kind, "time": self.sim_seconds, "episode": self.episode,
                  "wall_time": time.time(), **values}
        self.events.append(record)
        with (self.output / "events.jsonl").open("a") as stream:
            stream.write(json.dumps(record) + "\n")
        print("ISS_EVENT " + json.dumps(record), flush=True)

    @staticmethod
    def _rotation(q):
        import numpy as np
        w, x, y, z = np.asarray(q, dtype=float) / np.linalg.norm(q)
        return np.array([
            [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
            [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
            [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)],
        ])

    @staticmethod
    def _limit(vector, maximum):
        import numpy as np
        magnitude = float(np.linalg.norm(vector))
        return vector * min(1.0, maximum / max(magnitude, 1e-12))

    def _sync_panel(self):
        if hasattr(self, "panel"):
            self.panel.sync_command(*self.request)

    def command(self, v, w, z=0.0):
        values = tuple(map(float, (v, w, z)))
        if not all(map(math.isfinite, values)):
            raise ValueError("Flight commands must be finite")
        self.request = tuple(max(-limit, min(limit, value)) for value, limit in
                             zip(values, (self.MAX_SPEED, self.MAX_YAW_RATE, self.MAX_SPEED)))
        self.control_active = True
        self._sync_panel()
        self.event("command", v=self.request[0], w=self.request[1], z=self.request[2])

    def coast(self):
        self.request = (0.0, 0.0, 0.0)
        self.control_active = False
        self.last_force[:] = self.last_torque[:] = 0.0
        self._sync_panel()
        self.event("coast")

    def pause(self):
        self.request = (0.0, 0.0, 0.0)
        self.control_active = True
        self._sync_panel()
        if self.timeline.is_playing():
            self.timeline.pause()
        else:
            self.timeline.play()
        self.timeline.commit()
        self.event("pause_toggle", playing=self.timeline.is_playing())

    def on_timeline(self, event):
        import omni.timeline
        if self.initializing or self._closed:
            return
        if event.type == int(omni.timeline.TimelineEventType.STOP):
            self.request = (0.0, 0.0, 0.0)
            self.control_active = True
            self.restart_pending = True
            self._sync_panel()
            self.event("timeline_stop")

    def queue_reset(self):
        self.reset_pending = True

    def reset(self):
        """Restore the spawn pose with zero velocity; preserve pause state."""
        self._reset(initial=False, resume=self.timeline.is_playing())

    def _reset(self, initial, resume):
        import numpy as np
        self.initializing = True
        try:
            self.request = (0.0, 0.0, 0.0)
            self.control_active = not initial
            self.world.reset()
            if not self.body.is_physics_handle_valid():
                self.body.initialize()
            self.body.set_world_poses(self.spawn[None, :], self.spawn_orientation[None, :])
            velocity = self.initial_velocity if initial else np.zeros(6, dtype=np.float32)
            self.body.set_velocities(velocity[None, :])
            self.mass = float(self.body.get_masses()[0])
            self.inertia = np.asarray(self.body.get_inertias()[0], dtype=float).reshape(3, 3)
            self.last_pose = (self.spawn.copy(), self.spawn_orientation.copy())
            self.last_velocity = velocity.copy()
            self.last_force[:] = self.last_torque[:] = 0.0
            self.episode += 1
            self.episode_time = 0.0
            self.reset_pending = self.restart_pending = False
            if not resume:
                self.timeline.pause()
                self.timeline.commit()
            self._sync_panel()
            self.update_camera(*self.last_pose, all_cameras=True)
        finally:
            self.initializing = False
        self.event("initial_drift" if initial else "reset", position=self.spawn.tolist(), playing=self.timeline.is_playing())

    def pose(self):
        if self.body.is_physics_handle_valid() and not self.timeline.is_stopped():
            position, orientation = self.body.get_world_poses()
            self.last_pose = (position[0].copy(), orientation[0].copy())
        return tuple(value.copy() for value in self.last_pose)

    def _apply_control(self):
        import numpy as np
        self.last_force[:] = self.last_torque[:] = 0.0
        if not self.control_active:
            return
        _, q = self.pose()
        rotation = self._rotation(q)
        forward = rotation @ np.array((0.0, -1.0, 0.0))
        # Keep Up/Down aligned with the cabin Z axis, as in the panel. Pitch and
        # roll from collisions remain physical; no attitude teleport is used.
        forward[2] = 0.0
        norm = np.linalg.norm(forward)
        if norm > 1e-6:
            forward /= norm
        v, w, z = self.request
        target_velocity = self._limit(v * forward + np.array((0.0, 0.0, z)), self.MAX_SPEED)
        velocity = self.body.get_velocities()[0]
        acceleration = self._limit((target_velocity - velocity[:3]) / 0.32, self.MAX_ACCELERATION)
        angular_acceleration = self._limit((np.array((0.0, 0.0, w)) - velocity[3:]) / 0.28, self.MAX_ANGULAR_ACCELERATION)
        world_inertia = rotation @ self.inertia @ rotation.T
        self.last_force = self.mass * acceleration
        self.last_torque = self._limit(world_inertia @ angular_acceleration, 0.12)
        self.body.apply_forces_and_torques_at_pos(
            forces=self.last_force.astype(np.float32)[None, :],
            torques=self.last_torque.astype(np.float32)[None, :], is_global=True,
        )

    def position_camera(self, mode, eye, target):
        import numpy as np
        from pxr import Gf, UsdGeom
        direction = np.asarray(target) - np.asarray(eye)
        up = Gf.Vec3d(0, 1, 0) if abs(direction[2]) > 0.98 * np.linalg.norm(direction) else Gf.Vec3d(0, 0, 1)
        transform = Gf.Matrix4d().SetLookAt(Gf.Vec3d(*map(float, eye)), Gf.Vec3d(*map(float, target)), up).GetInverse()
        UsdGeom.Xformable(self.cameras[mode]).MakeMatrixXform().Set(transform)

    def _camera_clearance(self, origin, desired):
        import numpy as np
        from pxr import Gf
        delta = np.asarray(desired) - np.asarray(origin)
        distance = float(np.linalg.norm(delta))
        if distance < 1e-6 or not self.body.is_physics_handle_valid():
            return desired
        nearest = distance

        def hit_wall(hit):
            nonlocal nearest
            if str(hit.collision).startswith("/World/ISS/"):
                nearest = min(nearest, float(hit.distance))
            return True

        self.scene_query.raycast_all(
            Gf.Vec3f(*map(float, origin)), Gf.Vec3f(*map(float, delta / distance)), distance, hit_wall,
        )
        if nearest < distance:
            return np.asarray(origin) + delta / distance * max(0.02, nearest - 0.1)
        return desired

    def update_camera(self, p, q, all_cameras=False):
        import numpy as np
        rotation = self._rotation(q)
        forward = rotation @ np.array((0.0, -1.0, 0.0))
        if all_cameras or self.camera_mode == "follow":
            heading = forward.copy()
            heading[2] = 0.0
            heading /= max(float(np.linalg.norm(heading)), 1e-6)
            target = p + np.array((0.0, 0.0, 0.08))
            eye = self._camera_clearance(p, p - 1.3 * heading + np.array((0.0, 0.0, 0.25)))
            self.position_camera("follow", eye, target)
        if all_cameras or self.camera_mode == "rover":
            eye = self._camera_clearance(p, p + rotation @ self.onboard_offset)
            self.position_camera("rover", eye, eye + 2.0 * forward)

    def camera(self, mode):
        from omni.kit.viewport.utility import get_active_viewport
        from pxr import UsdRender
        if mode not in self.cameras:
            raise ValueError(f"Unknown ISS camera: {mode}")
        self.camera_mode = mode
        self.update_camera(*self.pose())
        path = self.cameras[mode].GetPath()
        viewport = get_active_viewport()
        if viewport is not None:
            viewport.camera_path = str(path)
        for prim in self.stage.Traverse():
            if prim.IsA(UsdRender.Product):
                UsdRender.Product(prim).GetCameraRel().SetTargets([path])
        self.event("camera", mode=mode)

    def snapshot(self):
        p, q = self.pose()
        if self.body.is_physics_handle_valid() and not self.timeline.is_stopped():
            self.last_velocity = self.body.get_velocities()[0].copy()
        w, x, y, z = map(float, q)
        return {
            "time": self.sim_seconds, "episode": self.episode,
            "position": p.tolist(), "quaternion_wxyz": q.tolist(),
            "velocity": self.last_velocity.tolist(),
            "yaw": math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z)),
            "request": list(self.request), "playing": self.timeline.is_playing(),
            "camera": self.camera_mode, "control_active": self.control_active,
            "force_N": self.last_force.tolist(), "torque_Nm": self.last_torque.tolist(),
        }

    def _update_status(self):
        import numpy as np
        state = self.snapshot()
        if not state["playing"]:
            phase = "Stopped" if self.timeline.is_stopped() else "Paused"
        elif not self.control_active:
            phase = "Free drift"
        elif any(self.request):
            phase = "Flying"
        else:
            phase = "Braking / holding"
        self.panel.update({
            "position": state["position"], "speed": float(np.linalg.norm(state["velocity"][:3])),
            "yawrate": state["velocity"][5], "vertical_speed": state["velocity"][2],
            "playing": state["playing"], "state": phase, "camera": self.camera_mode,
            "support": "Zero gravity. Commands use bounded thruster forces. STOP brakes; Coast preserves momentum.",
        })
        if time.monotonic() - self.last_status_wall > 0.5:
            temporary = self.output / "live_status.json.tmp"
            temporary.write_text(json.dumps({**state, "pid": os.getpid(), "event_count": len(self.events)}, indent=2) + "\n")
            temporary.replace(self.output / "live_status.json")
            self.last_status_wall = time.monotonic()

    def tick(self, force_render=False):
        if self._closed:
            return
        if self.reset_pending or (self.restart_pending and self.timeline.is_playing()):
            self.reset()
        if self.timeline.is_playing():
            self._apply_control()
            # A physics-only step followed by render() keeps camera/UI updates
            # from advancing physics twice or depending on desktop frame rate.
            self.world.step(render=False)
            self.sim_seconds += self.DT
            self.episode_time += self.DT
            self.steps += 1
            if force_render or self.steps % (12 if self.headless else 2) == 0:
                self.update_camera(*self.pose())
                self.world.render()
        else:
            self.app.update()
            self.update_camera(*self.pose())
        if self.steps % 12 == 0 or not self.timeline.is_playing():
            self._update_status()

    def run_seconds(self, duration):
        for _ in range(round(float(duration) / self.DT)):
            self.tick()

    def shutdown(self):
        if self._closed:
            return
        self.panel.shutdown()
        self._closed = True
        self.timeline_subscription = None
