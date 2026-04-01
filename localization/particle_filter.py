import numpy as np
import threading

from localization.sensor_model import SensorModel
from localization.motion_model import MotionModel

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from sensor_msgs.msg import LaserScan

import tf2_ros
# from tf_transformations import quaternion_from_euler, euler_from_quaternion
import scipy.spatial.transform as sp
from scipy.spatial.transform import Rotation as R

from geometry_msgs.msg import PoseArray, Pose

from rclpy.node import Node
import rclpy

assert rclpy


class ParticleFilter(Node):

    def __init__(self):
        super().__init__("particle_filter")

        self.declare_parameter('particle_filter_frame', "default")
        self.particle_filter_frame = self.get_parameter('particle_filter_frame').get_parameter_value().string_value

        #  *Important Note #1:* It is critical for your particle
        #     filter to obtain the following topic names from the
        #     parameters for the autograder to work correctly. Note
        #     that while the Odometry message contains both a pose and
        #     a twist component, you will only be provided with the
        #     twist component, so you should rely only on that
        #     information, and *not* use the pose component.

        self.declare_parameter('odom_topic', "/odom")
        self.declare_parameter('scan_topic', "/scan")
        self.declare_parameter('num_particles', 200)
        self.declare_parameter('angle_step', 1)

        scan_topic  = self.get_parameter("scan_topic").get_parameter_value().string_value
        odom_topic  = self.get_parameter("odom_topic").get_parameter_value().string_value
        self.num_particles = self.get_parameter("num_particles").get_parameter_value().integer_value
        self.angle_step    = self.get_parameter("angle_step").get_parameter_value().integer_value

        self.laser_sub = self.create_subscription(LaserScan, scan_topic,
                                                  self.laser_callback,
                                                  1)

        self.odom_sub = self.create_subscription(Odometry, odom_topic,
                                                 self.odom_callback,
                                                 1)

        #  *Important Note #2:* You must respond to pose
        #     initialization requests sent to the /initialpose
        #     topic. You can test that this works properly using the
        #     "Pose Estimate" feature in RViz, which publishes to
        #     /initialpose.

        self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, "/initialpose",
                                                 self.pose_callback,
                                                 1)

        #  *Important Note #3:* You must publish your pose estimate to
        #     the following topic. In particular, you must use the
        #     pose field of the Odometry message. You do not need to
        #     provide the twist part of the Odometry message. The
        #     odometry you publish here should be with respect to the
        #     "/map" frame.

        self.odom_pub = self.create_publisher(Odometry, "/pf/pose/odom", 1)
        self.particle_pub = self.create_publisher(PoseArray, "/pf/particles", 1)

        # TF broadcaster for /map → particle_filter_frame
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Initialize the models
        self.motion_model = MotionModel(self)
        self.sensor_model = SensorModel(self)

        # Particle array: Nx3, [x, y, theta]
        self.particles = np.zeros((self.num_particles, 3))
        self.initialized = False

        # Odometry integration state
        self.last_odom_time = None
        self.lock = threading.Lock()

        self.get_logger().info("=============+READY+=============")

    # ---------------------------------------------------------------
    # Initialization
    # ---------------------------------------------------------------

    def pose_callback(self, msg):
        """Initialize particles around the clicked pose from RViz."""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, theta = R.from_quat([q.x, q.y, q.z, q.w]).as_euler("xyz")

        with self.lock:
            self.particles[:, 0] = np.random.normal(x,     0.5,  self.num_particles)
            self.particles[:, 1] = np.random.normal(y,     0.5,  self.num_particles)
            self.particles[:, 2] = np.random.normal(theta, 0.15, self.num_particles)
            self.initialized = True

        self.get_logger().info(f"Particles initialized around ({x:.2f}, {y:.2f}, {theta:.2f})")
        self.publish_pose_estimate()

    # ---------------------------------------------------------------
    # Motion update
    # ---------------------------------------------------------------

    def odom_callback(self, msg):
        """Apply motion model using body-frame twist, integrated over dt."""
        if not self.initialized:
            return

        now = self.get_clock().now().nanoseconds * 1e-9
        if self.last_odom_time is None:
            self.last_odom_time = now
            return

        dt = now - self.last_odom_time
        self.last_odom_time = now

        if dt <= 0 or dt > 1.0:
            return

        # Use ONLY the twist component (as per lab instructions)
        vx    = msg.twist.twist.linear.x
        vy    = msg.twist.twist.linear.y
        omega = msg.twist.twist.angular.z

        odometry = np.array([vx * dt, vy * dt, omega * dt])

        with self.lock:
            self.particles = self.motion_model.evaluate(self.particles, odometry)

        self.publish_pose_estimate()

    # ---------------------------------------------------------------
    # Sensor update + resampling
    # ---------------------------------------------------------------

    def laser_callback(self, msg):
        """Weight and resample particles using the sensor model."""
        if not self.initialized or not self.sensor_model.map_set:
            return

        # Downsample the scan
        ranges = np.array(msg.ranges)
        step = self.angle_step if self.angle_step > 1 else max(1, len(ranges) // self.sensor_model.num_beams_per_particle)
        observation = ranges[::step][:self.sensor_model.num_beams_per_particle]

        # Replace inf/nan with z_max
        z_max_m = (self.sensor_model.table_width - 1) * self.sensor_model.resolution * self.sensor_model.lidar_scale_to_map_scale
        observation = np.where(np.isfinite(observation), observation, z_max_m)

        with self.lock:
            weights = self.sensor_model.evaluate(self.particles, observation, normalize=True)

            if weights is None or np.sum(weights) == 0:
                return

            # Normalize
            weights = weights / np.sum(weights)

            # Resample
            indices = np.random.choice(self.num_particles, self.num_particles, p=weights)
            self.particles = self.particles[indices].copy()

            # Add post-resample noise to maintain diversity
            self.particles[:, 0] += np.random.normal(0, 0.2, self.num_particles)
            self.particles[:, 1] += np.random.normal(0, 0.2, self.num_particles)
            self.particles[:, 2] += np.random.normal(0, 0.05, self.num_particles)

        self.publish_pose_estimate()

    # ---------------------------------------------------------------
    # Pose estimate + TF
    # ---------------------------------------------------------------

    def publish_pose_estimate(self):
        """Compute weighted mean pose from particles and publish."""
        with self.lock:
            mean_x = np.mean(self.particles[:, 0])
            mean_y = np.mean(self.particles[:, 1])
            # Circular mean for theta to handle wrap-around correctly
            mean_theta = np.arctan2(
                np.mean(np.sin(self.particles[:, 2])),
                np.mean(np.cos(self.particles[:, 2]))
            )

        # Build Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "/map"
        odom_msg.child_frame_id = self.particle_filter_frame

        odom_msg.pose.pose.position.x = mean_x
        odom_msg.pose.pose.position.y = mean_y
        odom_msg.pose.pose.position.z = 0.0

        q = R.from_euler("xyz", (0, 0, mean_theta)).as_quat()
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]

        self.odom_pub.publish(odom_msg)

        # Publish particle cloud for RViz visualization
        pa = PoseArray()
        pa.header.stamp = odom_msg.header.stamp
        pa.header.frame_id = "map"
        with self.lock:
            for p in self.particles:
                pose = Pose()
                pose.position.x = float(p[0])
                pose.position.y = float(p[1])
                q = R.from_euler("xyz", (0, 0, float(p[2]))).as_quat()
                pose.orientation.x = q[0]
                pose.orientation.y = q[1]
                pose.orientation.z = q[2]
                pose.orientation.w = q[3]
                pa.poses.append(pose)
        self.particle_pub.publish(pa)

        # Publish TF: /map → particle_filter_frame
        t = TransformStamped()
        t.header.stamp = odom_msg.header.stamp
        t.header.frame_id = "map"
        t.child_frame_id = self.particle_filter_frame.lstrip('/')

        t.transform.translation.x = mean_x
        t.transform.translation.y = mean_y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    pf = ParticleFilter()
    rclpy.spin(pf)
    rclpy.shutdown()
