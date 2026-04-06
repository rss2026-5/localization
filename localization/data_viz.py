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

            # --- SPECIAL HANDLING FOR TF ---
            if topic_name == '/tf':
                for transform in msg.transforms:
                    # Filter for the specific frame you want (e.g., base_link relative to map)
                    if transform.child_frame_id == 'base_link': 
                        posts.append({
                            'time': timestamp / 1e9,
                            'x': transform.transform.translation.x,
                            'y': transform.transform.translation.y
                        })
            # --- HANDLING FOR ODOMETRY ---
            else:
                posts.append({
                    'time': timestamp / 1e9,
                    'x': msg.pose.pose.position.x,
                    'y': msg.pose.pose.position.y
                })

    return pd.DataFrame(posts)

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
bag_folder = 'bag_files/localization_run_3' 

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

# 1. Load both datasets
df_pf = extract_odom(bag_folder, '/pf/pose/odom')
df_vesc = extract_odom(bag_folder, '/vesc/odom')

# 2. Synchronize VESC data to PF timestamps
# We interpolate VESC (x,y) to match the times when the PF published
vesc_x_sync = np.interp(df_pf['time'], df_vesc['time'], df_vesc['x'])
vesc_y_sync = np.interp(df_pf['time'], df_vesc['time'], df_vesc['y'])

# 3. Calculate Euclidean Distance (Positional Error)
# Note: For a true 'Cross Track' error relative to a path, 
# you'd use a projection, but Euclidean distance is the lab standard for comparing two odoms.
cte = np.sqrt((df_pf['x'] - vesc_x_sync)**2 + (df_pf['y'] - vesc_y_sync)**2)

# 4. Plotting
plt.figure(figsize=(12, 6))
time_axis = df_pf['time'] - df_pf['time'].iloc[0]
plt.plot(time_axis, cte, label='CTE (PF vs VESC)', color='tab:orange', lw=1.5)

plt.axhline(y=np.mean(cte), color='black', linestyle='--', label=f'Mean Error: {np.mean(cte):.3f}m')

plt.title('Cross Track Error: PF Estimate vs. Raw VESC Odometry')
plt.xlabel('Time (seconds)')
plt.ylabel('Distance Error (meters)')
plt.legend()
plt.grid(True, alpha=0.3)

# XY trajectory plot
plt.figure(figsize=(8, 8))
plt.plot(abs(df_vesc['x']), abs(df_vesc['y']), label='Raw VESC (Drifting)', alpha=0.5)
plt.plot(df_pf['x'], df_pf['y'], label='PF Filtered (Corrected)', color='red')
plt.axis('equal')
plt.legend()
plt.title('Spatial Comparison of Trajectories')

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