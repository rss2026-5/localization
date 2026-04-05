# Lab 5 — Real Car Testing

**The car must be in the Stata basement.**

---

## Deploy (from your laptop)

```bash
cd ~/Documents/GitHub/localization
./scripts/deploy.sh
```

---

## Build (on the car, inside Docker)

```bash
ssh racecar@192.168.1.102
connect
export SIM_WS=/root/sim_ws
cd ~/racecar_ws && colcon build --packages-select localization --symlink-install && source install/setup.bash
```

Password: `racecar@mit`

If `connect` fails, start Docker first: `cd && ./run_rostorch.sh` then `connect` in a new tab.

Wait for `Finished <<< localization`.

---

## Run

Open **4 terminals** on your laptop.

### Tab 1 — VNC port forward

```bash
ssh -L 6081:localhost:6081 racecar@192.168.1.102
```

Keep this open. Open your browser to: `http://localhost:6081/vnc.html?resize=remote`

### Tab 2 — Particle filter

```bash
ssh racecar@192.168.1.102
connect
cd ~/racecar_ws && source install/setup.bash
ros2 launch localization localize_real.launch.xml
```

Wait for `=============+READY+=============` and `Map initialized`.

### Tab 3 — RViz

```bash
ssh racecar@192.168.1.102
connect
source ~/racecar_ws/install/setup.bash
rviz2 -d $(ros2 pkg prefix localization)/share/localization/rviz_config.rviz
```

If that errors, run `rviz2` and manually add `/map`, `/scan`, `/pf/particles`. Set Fixed Frame to `map`.

### Tab 4 — Rosbag + Hz

```bash
ssh racecar@192.168.1.102
connect
source ~/racecar_ws/install/setup.bash
cd ~/racecar_ws/src/localization
./scripts/record_test.sh convergence_run_1
```

Teleop should already be running. Verify: hold **LB** + move left stick = car responds.
If teleop isn't running: `connect` in a new tab, then `teleop`.

---

## Test

1. **Start screen recording** (QuickTime or OBS on the VNC browser window)
2. In RViz, click **2D Pose Estimate** in the toolbar, click and drag on the map where the car is
3. Hold **RB** and drive slowly
4. Watch particles converge. Red laser scan should align with walls
5. Drive a loop back to start (~30-60 seconds)
6. **Ctrl-C** in Tab 4 to stop recording
7. Stop screen recording

Check Hz (in Tab 4 after stopping the bag):

```bash
ros2 topic hz /pf/pose/odom
```

Should read **>20 Hz**. Screenshot it.

For more runs:

```bash
./scripts/record_test.sh convergence_run_2
./scripts/record_test.sh convergence_run_3
```

Leave the particle filter running between runs.

---

## Copy Bags to Laptop

```bash
cd ~/Documents/GitHub/localization
scp -r racecar@192.168.1.102:~/racecar_ws/src/localization/bag_files/ ./bag_files/
```

---

## What to Walk Away With

- Screen recordings of RViz showing convergence + laser aligning with walls
- Screenshot of Hz monitor showing >20 Hz
- 3+ rosbags for offline parameter tuning and report charts
