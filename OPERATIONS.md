# Lab 5 — Real Car Testing

**The car must be in the Stata basement.**

---

## 1. Deploy (from your laptop, on any wifi)

```bash
cd ~/Documents/GitHub/localization
./scripts/deploy.sh
```

---

## 2. Switch to car wifi, then open 5 tabs

### Tab 1 — Docker (start container)

```bash
ssh racecar@192.168.1.102
sudo docker rm -f racecar 2>/dev/null
cd && ./run_rostorch.sh
```

Password: `racecar@mit`. Leave this running.

### Tab 2 — Build + teleop check

```bash
ssh racecar@192.168.1.102
connect
export SIM_WS=/root/sim_ws
cd ~/racecar_ws && colcon build --packages-select localization --symlink-install && source install/setup.bash
```

Wait for `Finished <<< localization`. Then verify teleop: hold **LB + left stick** = car moves.

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

### Tab 5 — RViz

```bash
ssh racecar@192.168.1.102
connect
export DISPLAY=:10
source ~/racecar_ws/install/setup.bash
rviz2 -d $(ros2 pkg prefix localization)/share/localization/rviz_config.rviz
```

The map should appear immediately (top-down view, Stata basement floor plan).
If it doesn't load the config, run `rviz2` bare and add `/map`, `/scan`, `/pf/particles` by topic. Set Fixed Frame to `map`.

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

Wait 5 sec, **screenshot it** (should show >20 Hz), Ctrl-C.

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

---

## 4. Copy bags to laptop

```bash
cd ~/Documents/GitHub/localization
scp -r racecar@192.168.1.102:~/racecar_ws/src/localization/bag_files/ ./bag_files/
```

---

## What you walk away with

- 3 screen recordings (particles converging + laser aligning with walls)
- 1 Hz screenshot (>20 Hz)
- 3 rosbags in `bag_files/`
