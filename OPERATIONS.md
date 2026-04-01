# Lab 5 Operations Guide

## Prerequisites
- SSH access to the car (`ssh racecar@192.168.1.<CAR_NUMBER>`)
- Docker running on the car (`sudo docker start racecar` if needed)
- Teleop already running on the car (handles joystick and deadman switch)

---

## 1. Deploy Code to the Car

From your laptop, run:
```bash
./scripts/deploy.sh          # defaults to car 102
./scripts/deploy.sh 104      # specify car number
```

Then SSH into the car, connect to Docker, and build:
```bash
ssh racecar@192.168.1.102
sudo docker exec -it racecar bash
cd ~/racecar_ws && colcon build --symlink-install && source install/setup.bash
```

---

## 2. Run the Particle Filter (Terminal 1)

```bash
ros2 launch localization localize_real.launch.xml
```

This starts the Stata basement map server and the particle filter. The estimated pose is published on `/pf/pose/odom` and the particle cloud on `/pf/particles`.

**Drive the car with the joystick — hold RB as the deadman switch.**

---

## 3. Record a Rosbag (Terminal 2)

Start recording before you set the initial pose so the bag captures everything:
```bash
./src/localization/scripts/record_test.sh convergence_run_1
```

This records `/vesc/odom`, `/scan`, `/pf/pose/odom`, `/pf/particles`, `/map`, `/initialpose`, `/tf`, and `/tf_static` to `~/rosbags/convergence_run_1/`.

Press **Ctrl-C** to stop recording when the run is done.

Suggested runs to collect:
1. `convergence_run_1` — straight corridor, initialize pose, drive a loop
2. `convergence_run_2` — sharper turns, different starting location
3. `convergence_run_3` — longer drive with varied speed

You can replay bags later to re-tune parameters without driving again:
```bash
ros2 bag play ~/rosbags/convergence_run_1
```

To copy bags back to your laptop:
```bash
scp -r racecar@192.168.1.102:~/rosbags/ ./bag_files/
```

---

## 4. Monitor Hz (Terminal 3)

```bash
ros2 topic hz /pf/pose/odom
```

This should read **>20 Hz** for real-time performance. Record the number for the report.

---

## 5. Set the Initial Pose

The particle filter starts with all particles at (0, 0) and will not converge until initialized.

**In Foxglove** (`ws://192.168.1.<CAR>:8765`):
1. Open a 3D panel and add `/pf/particles` (type: `PoseArray`) and `/map`
2. Use **Publish → Pose Estimate**, click and drag on the map to set the car's starting position and heading
3. Drive around — the particle cloud will converge to the correct location

---

## 6. Verify Localization is Working

- **Good**: tight particle cluster that follows the car as it moves
- **Bad**: particles spread all over the map — re-initialize the pose and try again

---

## 7. Data to Collect

For the lab report, collect at minimum:
1. **Convergence demo** — initialize pose, drive a loop, show particles converging
2. **Localization Hz** — `ros2 topic hz /pf/pose/odom` output while driving
3. **Error analysis** — compare `/pf/pose/odom` to known ground truth points in the building
4. **Parameter sweep** — replay bags with different `num_particles` and `num_beams_per_particle` values, record Hz and convergence for each

Screen-record your Foxglove session for the briefing video.
