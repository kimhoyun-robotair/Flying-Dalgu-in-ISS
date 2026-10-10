# Flying-Dalgu-in-ISS

## Running the Simulation

Tested on Ubuntu 24.04 LTS with Isaac Sim 5.1.
Run the following command from the project root:

```bash
./run_iss.sh
```

By default, Isaac Sim is assumed to be installed in `~/isaacsim`. If it is installed elsewhere, set `ISAAC_SIM_DIR` as follows:

```bash
ISAAC_SIM_DIR=../isaacsim ./run_iss.sh
```

Available launch options include:

```bash
# Open the scene in a paused state
./run_iss.sh --paused

# Run without a window for 120 runtime ticks, then exit
./run_iss.sh --headless --frames 120

# Save the initial interior view, run briefly, then exit
./run_iss.sh --headless --frames 120 --screenshot outputs/iss.png

# Specify where to save session logs
./run_iss.sh --output outputs/session
```

If `--frames` is omitted or set to `0`, the simulation runs until closed. `--screenshot` saves the initial view before simulation playback begins.

## Control Panel

Use the `ISS Flight Controller` panel displayed when running with the GUI.

| Control | Action |
| --- | --- |
| Forward / Reverse | Move forward / backward relative to Dalgu's facing direction |
| Left / Straight / Right | Turn left / clear the turn command / turn right |
| Up / Level / Down | Move up along the world Z axis / clear the vertical movement command / move down |
| STOP | Use thrust to slow down and stop |
| Coast | Drift without thrust |
| Pause / Resume | Pause / resume the simulation |
| Reset spawn | Return to the initial position and orientation, then stop |
| Follow / Overview / Dalgu / Interior view | Follow / wide-angle / Dalgu's forward-facing / initial interior camera view |
| Flight speed / Turn rate | Set movement speed / turning speed |

Movement commands remain active until the next control input. Motion is driven by PhysX forces and torques; resetting restores the initial position and orientation. At startup, Dalgu drifts freely with an initial velocity.

## Sources and Licensing

The ISS assets were converted from the `astrobee_iss` package in [NASA Astrobee Multimedia](https://github.com/nasa/astrobee_media). The upstream README credits IGOAL as the source of the ISS interior model.

[LICENSE](LICENSE) includes the original package's **Apache License 2.0 notice and full license text**, the NASA copyright notice, media usage guidelines, and a record of the conversion changes.The NASA-related notices apply to materials derived from the ISS assets. Rights to the Dalgu/DGIST characters and branding, as well as the project's own code and models, are separate; this file does not grant a NASA license for those materials.
