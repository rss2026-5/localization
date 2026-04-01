# Lab 5 — Real Car Testing

**The car must be in the Stata basement. The particle filter uses the Stata basement map.**

---

## One-Time Setup (skip if already done)

From the car **host** (SSH in, but NOT inside Docker):

```bash
ssh racecar@192.168.1.102
sudo docker pull staffmitrss/racecar2026
sudo docker run -it --name racecar --privileged --net=host \
    -v /dev:/dev \
    -v /home/racecar/racecar_ws:/root/racecar_ws \
    staffmitrss/racecar2026 bash
exit
```

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
sudo docker exec -it racecar bash
export SIM_WS=/root/sim_ws
cd ~/racecar_ws && colcon build --packages-select localization --symlink-install && source install/setup.bash
```

Wait for `Finished <<< localization`.

---

## Run

Open **5 terminals** on your laptop. Each one SSHs into the car and enters Docker.

### Tab 1 — Foxglove bridge

```bash
ssh -L 8765:localhost:8765 racecar@192.168.1.102
sudo docker exec -it racecar bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
```

### Tab 2 — Particle filter

```bash
ssh racecar@192.168.1.102
sudo docker exec -it racecar bash
export SIM_WS=/root/sim_ws
cd ~/racecar_ws && source install/setup.bash
ros2 launch localization localize_real.launch.xml
```

Wait for `=============+READY+=============` and `Map initialized`.

### Tab 3 — Rosbag recording

```bash
ssh racecar@192.168.1.102
sudo docker exec -it racecar bash
source ~/racecar_ws/install/setup.bash
cd ~/racecar_ws/src/localization
./scripts/record_test.sh convergence_run_1
```

### Tab 4 — Hz monitor

```bash
ssh racecar@192.168.1.102
sudo docker exec -it racecar bash
source ~/racecar_ws/install/setup.bash
ros2 topic hz /pf/pose/odom
```

### Tab 5 — Teleop

Start teleop per your team's usual process. Verify the car responds to the joystick (hold RB + left stick).

---

## Foxglove

1. Open [app.foxglove.dev](https://app.foxglove.dev)
2. **Open Connection** > `ws://localhost:8765` > **Open**
3. Add a **3D panel**, enable `/map`, `/pf/particles`, `/scan`
4. Scroll down to **Publish > 2D pose estimate**

---

## Test

1. **Start screen recording** (QuickTime or OBS)
2. In Foxglove, **click and drag** on the map where the car physically is to set the initial pose
3. **Hold RB** and drive slowly with the joystick
4. Watch particles converge. Laser scan should align with walls on the map
5. Hz monitor (Tab 4) should read **>20 Hz** — screenshot it
6. Drive a loop back to start (~30-60 seconds)
7. **Ctrl-C** in Tab 3 to stop recording
8. Stop screen recording

For more runs, start a new recording in Tab 3:

```bash
./scripts/record_test.sh convergence_run_2
./scripts/record_test.sh convergence_run_3
```

---

## Copy Bags to Laptop

From your laptop:

```bash
cd ~/Documents/GitHub/localization
scp -r racecar@192.168.1.102:~/rosbags/ ./bag_files/
```

---

## What to Walk Away With

- Screen recordings of Foxglove showing convergence + laser aligning with walls
- Screenshot of Hz monitor showing >20 Hz
- 3+ rosbags for offline parameter tuning and report charts
