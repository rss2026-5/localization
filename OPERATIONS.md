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
```

Then SSH into the car, connect to Docker, and build:
```bash
ssh racecar@192.168.1.102
sudo docker exec -it racecar bash
cd ~/racecar_ws && colcon build --symlink-install && source install/setup.bash
```

---

## 2. Run the Particle Filter

```bash
ros2 launch localization localize_real.launch.xml
```

This starts the stata basement map server and the particle filter. The estimated pose is published on `/pf/pose/odom` and the particle cloud on `/pf/particles`.

**Drive the car with the joystick — hold RB as the deadman switch.**

---

## 3. Set the Initial Pose

The particle filter starts with all particles at (0, 0) and will not converge until initialized.

**In Foxglove** (`ws://192.168.1.<CAR>:8765`):
1. Open a 3D panel and add `/pf/particles` (type: `PoseArray`) and `/map`
2. Use **Publish → Pose Estimate**, click and drag on the map to set the car's starting position and heading
3. Drive around — the particle cloud will converge to the correct location

---

## 4. Verify Localization is Working

- **Good**: tight particle cluster that follows the car as it moves
- **Bad**: particles spread all over the map — re-initialize the pose and try again

---

## 5. Data to Collect

For the lab report, collect at minimum:
1. **Convergence demo** — initialize pose, drive a loop, show particles converging
2. **Localization Hz** — run `ros2 topic hz /pf/pose/odom` while driving
3. **Error analysis** — compare `/pf/pose/odom` to known ground truth points in the building

Screen-record your Foxglove session for the briefing.
