#!/usr/bin/env python3
"""Open the ISS scene in Isaac Sim 5.1, select the interior camera and play."""
import argparse
import asyncio
import sys
import time
from pathlib import Path

# Bytecode caches embed the absolute source filename, including the local login.
sys.dont_write_bytecode = True

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--headless", action="store_true")
parser.add_argument("--paused", action="store_true", help="Open without playing")
parser.add_argument("--frames", type=int, default=0, help="Exit after N runtime ticks; 0 runs until closed")
parser.add_argument("--screenshot", type=Path, help="Save the initial interior view")
parser.add_argument("--output", type=Path, default=Path(__file__).resolve().parents[1] / "outputs",
                    help="Directory for controller verification reports")
parser.add_argument("--verify-controls", action="store_true", help="Exercise control callbacks and physical motion, then exit")
parser.add_argument("--verify-gui", action="store_true", help="Exercise visible controls with real mouse clicks, then exit")
args, kit_args = parser.parse_known_args()
if args.verify_gui and args.headless:
    parser.error("--verify-gui requires a visible window (remove --headless)")
if args.paused and (args.verify_controls or args.verify_gui):
    parser.error("Verification starts from running simulation; remove --paused")
sys.argv = [sys.argv[0], *kit_args]
ROOT = Path(__file__).resolve().parents[1]
if not (ROOT / "iss_sim.usda").is_file():
    parser.error("Missing iss_sim.usda; run ./build_iss.sh first")

from isaacsim import SimulationApp

app = SimulationApp({"headless": args.headless, "width": 1600, "height": 900,
                     "window_width": 1600, "window_height": 1000,
                     "renderer": "RaytracedLighting", "anti_aliasing": 3,
                     "open_usd": str(ROOT / "iss_sim.usda"),
                     "create_new_stage": False})
runtime = None
exit_code = 0
try:
    import carb
    import omni.usd
    import omni.timeline
    from omni.kit.viewport.utility import get_active_viewport, capture_viewport_to_file
    from flight_controller import ISSRuntime

    settings = carb.settings.get_settings()
    settings.set("/app/player/useFixedTimeStepping", True)
    settings.set("/app/player/playSimulations", True)
    settings.set("/app/asyncRendering", False)
    settings.set("/app/viewport/grid/enabled", False)
    for _ in range(30):
        app.update()
    viewport = get_active_viewport()
    if viewport is None:
        raise RuntimeError("No viewport available")
    viewport.camera_path = "/World/InteriorCamera"
    viewport.set_texture_resolution((1600, 900))
    runtime = ISSRuntime(app, headless=args.headless, output=args.output)
    timeline = runtime.timeline
    timeline.pause()
    timeline.commit()
    # Let RTX finish asynchronous texture/shader preparation before the first
    # capture or Play; update calls alone can finish before GPU uploads do.
    for _ in range(180):
        runtime.tick()
        time.sleep(0.02)
    if args.screenshot:
        args.screenshot.parent.mkdir(parents=True, exist_ok=True)
        capture = capture_viewport_to_file(viewport, str(args.screenshot.resolve()))
        task = asyncio.ensure_future(capture.wait_for_result())
        while not task.done():
            app.update()
        task.result()
        print(f"Screenshot: {args.screenshot.resolve()}", flush=True)
    if not args.paused:
        timeline.play()
        timeline.commit()
    verification = None
    if args.verify_controls or args.verify_gui:
        from verify_controls import verify_controls
        verification = asyncio.ensure_future(verify_controls(runtime, visible=args.verify_gui))
    frames = 0
    while app.is_running() and (args.frames <= 0 or frames < args.frames):
        runtime.tick()
        frames += 1
        if verification is not None and verification.done():
            result = verification.result()
            exit_code = 0 if result.get("status") == "PASS" else 1
            print(f"Controller verification: {result.get('status')}", flush=True)
            break
    if verification is not None and not verification.done():
        verification.cancel()
        exit_code = 1
finally:
    if runtime is not None:
        runtime.shutdown()
    app.close()
if exit_code:
    raise SystemExit(exit_code)
