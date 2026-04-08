# Lab 5 — Real Car Testing

**The car must be in the Stata basement.**

---

## 1. Deploy (from your laptop, on any wifi)

```bash
cd ~/Documents/GitHub/localization
./scripts/deploy.sh
```

To deploy to a different car: `./scripts/deploy.sh 104`

---

## 2. Switch to car wifi, then open 5 tabs

Run each command **after** you see the shell prompt — do not paste multiple commands before SSH connects.

### Tab 1 — Docker (start container)

```bash
ssh racecar@192.168.1.102
sudo docker rm -f racecar 2>/dev/null
cd && ./run_rostorch.sh
```

Password: `racecar@mit`. Leave this running — it is the container's main process.

Docker details:
- Image: `staffmitrss/racecar2026:latest`
- Mounts `~/racecar_ws` into the container at `/root/racecar_ws`
- Container is ephemeral (`--rm`), so anything saved outside the mount is lost on stop
- VNC display inside the container is `:10`

### Tab 2 — Build + teleop check

```bash
ssh racecar@192.168.1.102
connect
export SIM_WS=/root/sim_ws
cd ~/racecar_ws && colcon build --packages-select localization --symlink-install && source install/setup.bash
```

Wait for `Finished <<< localization`. Warnings about `np.bool` and `setuptools` are harmless.

Verify teleop: hold **LB + left stick** = car moves. If teleop isn't running: open a new terminal, `connect`, then `teleop`.

### Tab 3 — VNC tunnel

```bash
ssh -L 6081:localhost:6081 racecar@192.168.1.102
```

Open browser: `http://localhost:6081/vnc.html?resize=remote`

### Tab 4 — Particle filter

```bash
ssh racecar@192.168.1.102
connect
cd ~/racecar_ws && source install/setup.bash
ros2 launch localization localize_real.launch.xml
```

Wait for `=============+READY+=============` and `Map initialized`.

This launches three things:
- **map_server** — publishes the Stata basement map on `/map` (Transient Local QoS)
- **static_transform_publisher** — publishes `base_link → laser` TF (lidar mount offset)
- **particle_filter** — subscribes to `/vesc/odom` and `/scan`, publishes `/pf/pose/odom`, `/pf/particles`, and `map → base_link` TF

### Tab 5 — RViz

```bash
ssh racecar@192.168.1.102
connect
export DISPLAY=:10
source ~/racecar_ws/install/setup.bash
rviz2 -d $(ros2 pkg prefix localization)/share/localization/rviz_config.rviz
```

The map should appear immediately in a top-down view (the particle filter publishes a default `map → base_link` TF on startup so the map frame exists before initialization).

**If the map doesn't appear:**

1. If it says `Frame [map] does not exist` — click **2D Pose Estimate** and click-drag anywhere on the viewport to initialize the TF, then the map will render.
2. If the map frame is OK but the map is blank — force a re-publish in a spare terminal:
   ```bash
   ros2 lifecycle set /map_server deactivate && ros2 lifecycle set /map_server activate
   ```
3. If the config file doesn't load, run `rviz2` bare and manually add:
   - `/map` by topic (Map display)
   - `/scan` by topic (LaserScan display)
   - `/pf/particles` by topic (PoseArray display)
   - Set **Fixed Frame** to `map`
   - Set view: Pitch = `1.5708`, Distance = `80`

**RViz displays:**

| Display | Topic | Color | What it shows |
|---------|-------|-------|---------------|
| Map | `/map` | gray/white floor plan | Stata basement occupancy grid |
| LaserScan | `/scan` | red dots | Actual LIDAR data in map frame |
| Particles | `/pf/particles` | green arrows | Particle filter hypotheses |
| Estimated Pose | `/pf/pose/odom` | orange arrow | Mean estimated position |

When localization is working: red laser dots align with black wall edges on the map.

---

## 3. Test runs

For each run, open a **new terminal**:

```bash
ssh racecar@192.168.1.102
connect
source ~/racecar_ws/install/setup.bash
cd ~/racecar_ws/src/localization
```

### Run 1

```bash
./scripts/record_test.sh convergence_run_1
```

1. Start **screen recording** on laptop (QuickTime/OBS on VNC window)
2. In RViz: **2D Pose Estimate** → click where car is → drag facing direction
3. Hold **RB**, drive slowly 30–60 sec, watch particles converge + laser align with walls
4. Stop screen recording, save it
5. **Ctrl-C** in rosbag terminal

**Hz check** (once, after Run 1):

```bash
ros2 topic hz /pf/pose/odom
```

Wait 5 sec, **screenshot it** (should show >20 Hz — we measured 71 Hz), Ctrl-C.

### Run 2

```bash
./scripts/record_test.sh convergence_run_2
```

New screen recording. Different start location, sharp turns.

### Run 3

```bash
./scripts/record_test.sh convergence_run_3
```

New screen recording. Longer drive (~90 sec), varied speed. Best briefing video.

Leave the particle filter running between runs — just set a new initial pose for each.

---

## 4. Copy bags to laptop

Switch to car wifi if not already connected:

```bash
cd ~/Documents/GitHub/localization
scp -r racecar@192.168.1.102:~/racecar_ws/src/localization/bag_files/ ./bag_files/
```

Note: this creates `bag_files/bag_files/` (nested). Move the bags up:

```bash
mv bag_files/bag_files/* bag_files/ && rm -rf bag_files/bag_files
```

Bags are saved inside the workspace mount so they survive container restarts. The `deploy.sh` script excludes `bag_files/` and `*.db3` from `--delete`, so redeploying won't wipe recorded bags.

---

## 5. Replay bags offline (on your laptop)

Bags can be replayed for parameter tuning without the car:

```bash
ros2 bag play bag_files/localization_run_1/
```

Replay with the particle filter running under different parameters to generate convergence charts and runtime analysis for the report.

---

## Troubleshooting

| Problem | Fix |
|---------|-----|
| `connect` says `bash: /root/tempBashRC: No such file or directory` | Harmless warning, ignore it |
| RViz: `could not connect to display` | Set `export DISPLAY=:10` before running `rviz2` |
| RViz: `Frame [map] does not exist` | Click 2D Pose Estimate and click anywhere to initialize TF |
| Map doesn't appear in RViz | Run lifecycle re-publish (see Tab 5 instructions above) |
| Laser scan not visible | Check that teleop is running (provides `base_link → laser` TF). The launch file also publishes this statically. |
| SSH connection drops | Car wifi router may be out of range. Move closer. |
| `ros2: command not found` | You're on the host, not inside Docker. Run `connect` first. |
| Build fails with `SIM_WS` error | Run `export SIM_WS=/root/sim_ws` before building |

---

## What you walk away with

- 3 screen recordings (particles converging + laser aligning with walls)
- 1 Hz screenshot (>20 Hz)
- 3 rosbags in `bag_files/`
- Videos pushed to `videos/` in the repo

---

## Key parameters (real_params.yaml)

| Parameter | Value | Why |
|-----------|-------|-----|
| `num_particles` | 100 | Enough for convergence, fast enough for 71 Hz |
| `num_beams_per_particle` | 99 | ~100 beams from 1080 raw (11x downsample) |
| `angle_step` | 11 | Take every 11th beam for uniform angular coverage |
| `odom_topic` | `/vesc/odom` | Real car wheel odometry (sim uses `/odom`) |
| `scan_topic` | `/scan` | Hokuyo LIDAR |
| `particle_filter_frame` | `base_link` | TF child frame for map → car transform |
| `scan_field_of_view` | 4.71 rad (270°) | Hokuyo UST-10LX field of view |
| `lidar_scale_to_map_scale` | 1.0 | Both in meters |
