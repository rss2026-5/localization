import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
from pathlib import Path
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore  # Import Stores

def extract_data(bag_path, topic_name):
    posts = []
    # Use the Enum directly instead of the string 'ros2'
    typestore = get_typestore(Stores.ROS2_HUMBLE)

    # Pass the typestore to the reader
    with AnyReader([Path(bag_path)], default_typestore=typestore) as reader:
        connections = [c for c in reader.connections if c.topic == topic_name]

        if not connections:
            print(f"Warning: Topic {topic_name} not found in bag!")
            return pd.DataFrame()

        for conn, timestamp, rawdata in reader.messages(connections=connections):
            msg = reader.deserialize(rawdata, conn.msgtype)

            # --- HANDLING FOR ODOMETRY ---
            posts.append({
                'time': timestamp / 1e9,
                'x': msg.pose.pose.position.x,
                'y': msg.pose.pose.position.y
            })

    return pd.DataFrame(posts)


def extract_tf_frame(bag_path, parent_frame, child_frame):
    """Extract a specific transform from /tf.
    Returns a DataFrame with columns: time, x, y, yaw.
    """
    records = []
    typestore = get_typestore(Stores.ROS2_HUMBLE)
    with AnyReader([Path(bag_path)], default_typestore=typestore) as reader:
        connections = [c for c in reader.connections if c.topic == '/tf']
        for conn, timestamp, rawdata in reader.messages(connections=connections):
            msg = reader.deserialize(rawdata, conn.msgtype)
            for tf in msg.transforms:
                if tf.header.frame_id == parent_frame and tf.child_frame_id == child_frame:
                    q = tf.transform.rotation
                    yaw = np.arctan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y**2 + q.z**2))
                    records.append({
                        'time': timestamp / 1e9,
                        'x': tf.transform.translation.x,
                        'y': tf.transform.translation.y,
                        'yaw': yaw
                    })
    return pd.DataFrame(records)

def extract_convergence_data(bag_path, topic_name):
    data = []
    typestore = get_typestore(Stores.ROS2_HUMBLE) 
    
    with AnyReader([Path(bag_path)], default_typestore=typestore) as reader:
        connections = [c for c in reader.connections if c.topic == topic_name]
        for conn, timestamp, rawdata in reader.messages(connections=connections):
            msg = reader.deserialize(rawdata, conn.msgtype)
            
            # Accessing covariance: msg.pose.covariance for Odometry or PoseWithCovarianceStamped
            cov = msg.pose.covariance 
            
            # Sum of X and Y variance
            variance_sum = cov[0] + cov[7]
            
            # To avoid log(0) errors, we ensure the value is at least a tiny number
            std_dev = np.sqrt(max(variance_sum, 1e-9)) 
            
            data.append({
                'time': timestamp / 1e9,
                'std_dev': std_dev
            })
    return pd.DataFrame(data)

def extract_odom(bag_path, topic_name):
    posts = []
    typestore = get_typestore(Stores.ROS2_HUMBLE) 
    with AnyReader([Path(bag_path)], default_typestore=typestore) as reader:
        connections = [c for c in reader.connections if c.topic == topic_name]
        if not connections:
            return pd.DataFrame()
        for conn, timestamp, rawdata in reader.messages(connections=connections):
            msg = reader.deserialize(rawdata, conn.msgtype)
            # Both topics are likely nav_msgs/msg/Odometry
            posts.append({
                'time': timestamp / 1e9,
                'x': msg.pose.pose.position.x,
                'y': msg.pose.pose.position.y
            })
    return pd.DataFrame(posts)

def calculate_neff(bag_path, topic_name):
    neff_data = []
    typestore = get_typestore(Stores.ROS2_HUMBLE)
    
    with AnyReader([Path(bag_path)], default_typestore=typestore) as reader:
        connections = [c for c in reader.connections if c.topic == topic_name]
        
        for conn, timestamp, rawdata in reader.messages(connections=connections):
            msg = reader.deserialize(rawdata, conn.msgtype)
            
            # 1. Extract weights from the PoseArray (stored in orientation.w in many PF labs)
            # If your weights are in a different field, adjust this line:
            weights = np.array([p.orientation.w for p in msg.poses])
            
            # 2. Normalize weights (Sum must equal 1.0)
            if np.sum(weights) == 0:
                continue
            weights /= np.sum(weights)
            
            # 3. Calculate Neff = 1 / sum(w^2)
            neff = 1.0 / np.sum(np.square(weights))
            
            neff_data.append({
                'time': timestamp / 1e9,
                'neff': neff,
                'percent_healthy': (neff / len(weights)) * 100
            })
            
    return pd.DataFrame(neff_data)

def extract_innovation(bag_path, topic_name):
    inv_data = []
    typestore = get_typestore(Stores.ROS2_HUMBLE)
    
    with AnyReader([Path(bag_path)], default_typestore=typestore) as reader:
        connections = [c for c in reader.connections if c.topic == topic_name]
        
        for conn, timestamp, rawdata in reader.messages(connections=connections):
            msg = reader.deserialize(rawdata, conn.msgtype)
            
            # 1. Get raw weights (before normalization)
            # Higher weight = Higher Likelihood = LOWER Innovation
            raw_weights = np.array([p.orientation.w for p in msg.poses])
            
            # 2. Innovation is often represented as -log(Average Likelihood)
            avg_likelihood = np.mean(raw_weights)
            # Use a floor to prevent log(0)
            innovation = -np.log(max(avg_likelihood, 1e-12))
            
            inv_data.append({
                'time': timestamp / 1e9,
                'innovation': innovation
            })
            
    return pd.DataFrame(inv_data)


# -- file selection ---
# Set sim=True for simulation bags (/odom and /pf/pose/odom both in map frame)
# Set sim=False for real-car bags (uses /tf map->base_link vs odom->base_link)
bag_folder = 'bag_files/localization_run_3'
sim = False

# # --- convergence rate plot ---
# df_conv = extract_convergence_data(bag_folder, '/pf/pose/odom')

# # Plotting
# plt.figure(figsize=(10, 5))
# plt.plot(df_conv['time'] - df_conv['time'].iloc[0], df_conv['std_dev'], color='green', lw=2)
# plt.yscale('log') # Log scale helps see the "drop" more clearly
# plt.title('Filter Convergence Rate (Uncertainty over Time)')
# plt.xlabel('Time (seconds)')
# plt.ylabel('Total Standard Deviation (m) [Log Scale]')
# plt.grid(True, which="both", ls="-", alpha=0.5)
# plt.show()


# # --- localization vs ground truth ---
# # 1. Load data
# df_pf = extract_data(bag_folder, '/pf/pose/odom')
# df_gt = extract_data(bag_folder, '/tf')

# # 2. Synchronize data
# # We interpolate Ground Truth to match the exact timestamps of the PF estimate
# df_gt_sync = pd.DataFrame()
# df_gt_sync['time'] = df_pf['time']
# df_gt_sync['x'] = np.interp(df_pf['time'], df_gt['time'], df_gt['x'])
# df_gt_sync['y'] = np.interp(df_pf['time'], df_gt['time'], df_gt['y'])

# # 3. Calculate Euclidean Error
# error = np.sqrt((df_pf['x'] - df_gt_sync['x'])**2 + (df_pf['y'] - df_gt_sync['y'])**2)

# # 4. Create the Plot
# plt.figure(figsize=(10, 5))
# plt.plot(df_pf['time'] - df_pf['time'].iloc[0], error, label='Localization Error', color='crimson')
# plt.axhline(y=np.mean(error), color='black', linestyle='--', label=f'Mean Error: {np.mean(error):.3f}m')

# plt.title('Particle Filter Localization Error vs. Ground Truth')
# plt.xlabel('Time (seconds)')
# plt.ylabel('Error (meters)')
# plt.grid(True, alpha=0.3)
# plt.legend()
# plt.show()

if sim:
    # --- SIMULATION ---
    # Both /odom and /pf/pose/odom are already in the map frame.
    # /odom = noisy simulator odometry (dead reckoning baseline)
    # /pf/pose/odom = PF estimate
    df_pf_data = extract_data(bag_folder, '/pf/pose/odom')
    df_odom_data = extract_data(bag_folder, '/odom')

    pf_times = df_pf_data['time'].values
    odom_x_sync = np.interp(pf_times, df_odom_data['time'].values, df_odom_data['x'].values)
    odom_y_sync = np.interp(pf_times, df_odom_data['time'].values, df_odom_data['y'].values)

    pf_x, pf_y = df_pf_data['x'].values, df_pf_data['y'].values
    odom_label = 'Noisy Odometry (/odom)'
    pf_label = 'PF Estimate (/pf/pose/odom)'
    title_suffix = 'Simulation'
else:
    # --- REAL CAR ---
    df_pf_tf = extract_tf_frame(bag_folder, 'map', 'base_link')
    df_odom_tf = extract_tf_frame(bag_folder, 'odom', 'base_link')

    t0_map = df_pf_tf.iloc[0]
    t0_odom = df_odom_tf.iloc[0]

    pf_times = df_pf_tf['time'].values
    odom_x_t = np.interp(pf_times, df_odom_tf['time'].values, df_odom_tf['x'].values)
    odom_y_t = np.interp(pf_times, df_odom_tf['time'].values, df_odom_tf['y'].values)

    # Frozen rotation (t=0 only)
    delta_yaw_0 = t0_map['yaw'] - t0_odom['yaw']
    dx0 = odom_x_t - t0_odom['x']
    dy0 = odom_y_t - t0_odom['y']
    odom_x_frozen = np.cos(delta_yaw_0)*dx0 - np.sin(delta_yaw_0)*dy0 + t0_map['x']
    odom_y_frozen = np.sin(delta_yaw_0)*dx0 + np.cos(delta_yaw_0)*dy0 + t0_map['y']

    # Continuous rotation: integrate incremental VESC steps using PF heading at each step.
    # At each timestep we rotate the small displacement (dx, dy) by the current PF yaw,
    # then accumulate. This avoids the wild swings from rotating the full displacement vector.
    pf_yaw_t = np.interp(pf_times, df_pf_tf['time'].values, df_pf_tf['yaw'].values)
    odom_yaw_t = np.interp(pf_times, df_odom_tf['time'].values, df_odom_tf['yaw'].values)
    delta_yaw_t = pf_yaw_t - odom_yaw_t

    odom_x_cont = np.zeros(len(pf_times))
    odom_y_cont = np.zeros(len(pf_times))
    odom_x_cont[0] = t0_map['x']
    odom_y_cont[0] = t0_map['y']
    for i in range(1, len(pf_times)):
        ddx = odom_x_t[i] - odom_x_t[i-1]
        ddy = odom_y_t[i] - odom_y_t[i-1]
        c, s = np.cos(delta_yaw_t[i]), np.sin(delta_yaw_t[i])
        odom_x_cont[i] = odom_x_cont[i-1] + c*ddx - s*ddy
        odom_y_cont[i] = odom_y_cont[i-1] + s*ddx + c*ddy

    odom_x_sync = odom_x_frozen  # used for single-series plots below
    odom_y_sync = odom_y_frozen
    pf_x, pf_y = df_pf_tf['x'].values, df_pf_tf['y'].values
    title_suffix = 'Real Car'

# --- Euclidean divergence ---
pos_error_frozen = np.sqrt((pf_x - odom_x_frozen)**2 + (pf_y - odom_y_frozen)**2)
pos_error_cont   = np.sqrt((pf_x - odom_x_cont)**2   + (pf_y - odom_y_cont)**2)

# Divergence over time — continuous rotation only
plt.figure(figsize=(12, 6))
time_axis = pf_times - pf_times[0]
plt.plot(time_axis, pos_error_cont, color='tab:orange', lw=1.5)
plt.axhline(y=np.mean(pos_error_cont), color='black', linestyle='--',
            label=f'Mean Error: {np.mean(pos_error_cont):.3f}m')
plt.title(f'PF vs Dead Reckoning Divergence — {title_suffix}')
plt.xlabel('Time (seconds)')
plt.ylabel('Euclidean Distance (meters)')
plt.legend(loc='upper right')
plt.annotate(
    'Euclidean distance between\nPF Estimate (A) and\nVESC Odometry (B) over time',
    xy=(0.98, 0.92), xycoords='axes fraction',
    ha='right', va='top', fontsize=9,
    bbox=dict(boxstyle='round,pad=0.4', facecolor='white', edgecolor='gray', alpha=0.8)
)
plt.grid(True, alpha=0.3)
plt.savefig('pos_divergence.png', dpi=150, bbox_inches='tight')

# XY trajectory — PF vs continuous rotation only
plt.figure(figsize=(8, 8))
plt.plot(odom_x_cont, odom_y_cont, label='(B) VESC Odometry', alpha=0.7, color='tab:blue', lw=1.5)
plt.plot(pf_x, pf_y,               label='(A) PF Estimate',           color='red', lw=1.5)
plt.axis('equal')
plt.legend(loc='upper right')
plt.title(f'Trajectory Comparison — {title_suffix}')
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.grid(True, alpha=0.3)
plt.savefig('trajectory_comparison.png', dpi=150, bbox_inches='tight')

plt.show()

# # -- Neff ---
# # Execution
# df_neff = calculate_neff(bag_folder, '/pf/particles')

# # 1. Create the figure
# plt.figure(figsize=(12, 6))

# # 2. Plot the Neff values
# # We subtract the first timestamp to start the X-axis at 0
# time_zeroed = df_neff['time'] - df_neff['time'].iloc[0]
# plt.plot(time_zeroed, df_neff['neff'], label='Effective Particles ($N_{eff}$)', color='#1f77b4', lw=1.5)

# # 3. Add a "Resampling Threshold" line
# # Most filters resample when Neff drops below 50% of the total particles (N)
# # If you used 500 particles, N=500.
# N = 200  # <--- Change this to your actual particle count
# plt.axhline(y=N/2, color='red', linestyle='--', alpha=0.7, label=f'Resampling Trigger (N={N/2})')

# # 4. Formatting for Briefing Slides
# plt.title('Particle Filter Health: Effective Number of Particles ($N_{eff}$)', fontsize=14)
# plt.xlabel('Time (seconds)', fontsize=12)
# plt.ylabel('Count of "Useful" Particles', fontsize=12)
# plt.legend(loc='upper right')
# plt.grid(True, linestyle=':', alpha=0.6)

# plt.show()

# # --- innovation --
# df_inv = extract_innovation(bag_folder, '/pf/particles')

# # Assuming df_inv is your innovation dataframe
# plt.figure(figsize=(12, 5))

# # Zero the time
# t = df_inv['time'] - df_inv['time'].iloc[0]

# plt.plot(t, df_inv['innovation'], color='purple', label='Measurement Innovation')

# # # Mark a specific event (e.g., driving past a glass wall or a person)
# # plt.axvspan(15, 18, color='gray', alpha=0.3, label='Featureless Hallway')

# plt.title('Sensor Model Innovation (Likelihood Mismatch)')
# plt.xlabel('Time (s)')
# plt.ylabel('Innovation (bits/nats)')
# plt.grid(True, alpha=0.3)
# plt.legend()
# plt.show()