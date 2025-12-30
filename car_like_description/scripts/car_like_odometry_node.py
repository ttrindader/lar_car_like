#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
car_like_odometry_node.py (ROS1)

Publishes wheel/steering-based odometry for a car-like robot:
- Subscribes:  /joint_states (sensor_msgs/JointState)
- Publishes:   odom (nav_msgs/Odometry)   [default topic name: 'odom']
- Publishes TF: odom -> base_link         [optional]

Model assumptions (typical car-like):
- Rear wheels provide forward motion (angular velocity from joint_states)
- Front steer joints provide steering angle (position from joint_states)
- Kinematic bicycle model:
    v = r * (ω_rl + ω_rr)/2
    yaw_rate = v/L * tan(δ)

This is kinematic wheel odom (no IMU, no slip compensation). Use robot_localization
(EKF) if you want /odometry/filtered and better fusion.

Parameters (private ~):
  ~wheel_radius (m)              [default: 0.0315]
  ~wheelbase (m)                 [default: 0.16]
  ~rear_left_joint               [default: rear_left_wheel_joint]
  ~rear_right_joint              [default: rear_right_wheel_joint]
  ~front_left_steer_joint        [default: front_left_steer_joint]
  ~front_right_steer_joint       [default: front_right_steer_joint]
  ~rear_left_sign                [default: 1.0]  # use -1.0 if your joint sign is flipped
  ~rear_right_sign               [default: 1.0]
  ~use_average_steer             [default: True] # δ = (δ_fl + δ_fr)/2
  ~odom_frame                    [default: odom]
  ~base_frame                    [default: base_link]
  ~publish_tf                    [default: True]
  ~odom_topic                    [default: odom]
  ~rate (Hz)                     [default: 50.0] # publish/integration rate
  ~timeout (s)                   [default: 0.25] # if no joint_states for >timeout, v=0, yaw_rate=0
  ~z (m)                         [default: 0.0]  # odom pose z (usually 0 for 2D)

Tips:
- If your robot moves backwards when commanding forward, set ~rear_left_sign and/or
  ~rear_right_sign to -1.0, OR invert in your controller.
- Frame IDs are typically not namespaced. Use separate TF prefixes if you spawn many robots.

"""

import math
import rospy
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros


def _norm_angle(a: float) -> float:
    """Normalize angle to [-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


class CarLikeOdometry:
    def __init__(self):
        # ----- Params -----
        self.wheel_radius = float(rospy.get_param("~wheel_radius", 0.0315))
        self.wheelbase = float(rospy.get_param("~wheelbase", 0.16))

        self.rear_left_joint = rospy.get_param("~rear_left_joint", "rear_left_wheel_joint")
        self.rear_right_joint = rospy.get_param("~rear_right_joint", "rear_right_wheel_joint")
        self.front_left_steer_joint = rospy.get_param("~front_left_steer_joint", "front_left_steer_joint")
        self.front_right_steer_joint = rospy.get_param("~front_right_steer_joint", "front_right_steer_joint")

        self.rear_left_sign = float(rospy.get_param("~rear_left_sign", 1.0))
        self.rear_right_sign = float(rospy.get_param("~rear_right_sign", 1.0))

        self.use_average_steer = bool(rospy.get_param("~use_average_steer", True))

        self.odom_frame = rospy.get_param("~odom_frame", "odom")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.publish_tf = bool(rospy.get_param("~publish_tf", True))

        self.odom_topic = rospy.get_param("~odom_topic", "odom")
        self.rate_hz = float(rospy.get_param("~rate", 50.0))
        self.timeout = float(rospy.get_param("~timeout", 0.25))
        self.z = float(rospy.get_param("~z", 0.0))

        if self.wheel_radius <= 0.0:
            raise ValueError("~wheel_radius must be > 0")
        if self.wheelbase <= 0.0:
            raise ValueError("~wheelbase must be > 0")

        # ----- State -----
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.last_joint_msg_time = None  # rospy.Time
        self.last_update_time = None     # rospy.Time

        # Last measured values
        self.omega_rl = 0.0
        self.omega_rr = 0.0
        self.delta_fl = 0.0
        self.delta_fr = 0.0
        self.have_rl = False
        self.have_rr = False
        self.have_fl = False
        self.have_fr = False

        # ----- ROS I/O -----
        self.pub_odom = rospy.Publisher(self.odom_topic, Odometry, queue_size=10)
        self.br = tf2_ros.TransformBroadcaster() if self.publish_tf else None

        self.sub_js = rospy.Subscriber("joint_states", JointState, self._on_joint_state, queue_size=50)

        self.timer = rospy.Timer(rospy.Duration(1.0 / max(1e-3, self.rate_hz)), self._on_timer)

        rospy.loginfo("[car_like_odometry] wheel_radius=%.6f wheelbase=%.6f" % (self.wheel_radius, self.wheelbase))
        rospy.loginfo("[car_like_odometry] joints: RL=%s RR=%s FLsteer=%s FRsteer=%s" %
                      (self.rear_left_joint, self.rear_right_joint, self.front_left_steer_joint, self.front_right_steer_joint))
        rospy.loginfo("[car_like_odometry] frames: odom_frame=%s base_frame=%s publish_tf=%s" %
                      (self.odom_frame, self.base_frame, str(self.publish_tf)))
        rospy.loginfo("[car_like_odometry] topic: %s (Odometry), subscribing to joint_states" % self.odom_topic)

    def _on_joint_state(self, msg: JointState):
        idx = {n: i for i, n in enumerate(msg.name)}

        def get_vel(jname: str):
            i = idx.get(jname, None)
            if i is None or i >= len(msg.velocity):
                return None
            return msg.velocity[i]

        def get_pos(jname: str):
            i = idx.get(jname, None)
            if i is None or i >= len(msg.position):
                return None
            return msg.position[i]

        v = get_vel(self.rear_left_joint)
        if v is not None:
            self.omega_rl = float(v) * self.rear_left_sign
            self.have_rl = True

        v = get_vel(self.rear_right_joint)
        if v is not None:
            self.omega_rr = float(v) * self.rear_right_sign
            self.have_rr = True

        p = get_pos(self.front_left_steer_joint)
        if p is not None:
            self.delta_fl = float(p)
            self.have_fl = True

        p = get_pos(self.front_right_steer_joint)
        if p is not None:
            self.delta_fr = float(p)
            self.have_fr = True

        self.last_joint_msg_time = msg.header.stamp if msg.header.stamp != rospy.Time() else rospy.Time.now()

        if (not self.have_rl) or (not self.have_rr) or (not self.have_fl) or (not self.have_fr):
            missing = []
            if not self.have_rl: missing.append(self.rear_left_joint)
            if not self.have_rr: missing.append(self.rear_right_joint)
            if not self.have_fl: missing.append(self.front_left_steer_joint)
            if not self.have_fr: missing.append(self.front_right_steer_joint)
            rospy.logwarn_throttle(2.0, "[car_like_odometry] Missing joints in /joint_states: %s" % ", ".join(missing))

    def _compute_v_delta(self):
        now = rospy.Time.now()
        if self.last_joint_msg_time is None or (now - self.last_joint_msg_time).to_sec() > self.timeout:
            return 0.0, 0.0

        if not (self.have_rl and self.have_rr):
            return 0.0, 0.0

        omega = 0.5 * (self.omega_rl + self.omega_rr)
        v = self.wheel_radius * omega

        if self.use_average_steer and (self.have_fl and self.have_fr):
            delta = 0.5 * (self.delta_fl + self.delta_fr)
        elif self.have_fl:
            delta = self.delta_fl
        elif self.have_fr:
            delta = self.delta_fr
        else:
            delta = 0.0

        return v, delta

    def _on_timer(self, _evt):
        now = rospy.Time.now()
        if self.last_update_time is None:
            self.last_update_time = now
            return

        dt = (now - self.last_update_time).to_sec()
        if dt <= 0.0:
            return
        self.last_update_time = now

        v, delta = self._compute_v_delta()

        yaw_rate = (v / self.wheelbase) * math.tan(delta) if abs(self.wheelbase) > 1e-6 else 0.0

        self.x += v * math.cos(self.yaw) * dt
        self.y += v * math.sin(self.yaw) * dt
        self.yaw = _norm_angle(self.yaw + yaw_rate * dt)

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = self.z

        qz = math.sin(self.yaw * 0.5)
        qw = math.cos(self.yaw * 0.5)
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = yaw_rate

        odom.pose.covariance = [
            0.05, 0,    0,    0,    0,    0,
            0,    0.05, 0,    0,    0,    0,
            0,    0,    9999, 0,    0,    0,
            0,    0,    0,    9999, 0,    0,
            0,    0,    0,    0,    9999, 0,
            0,    0,    0,    0,    0,    0.2
        ]
        odom.twist.covariance = [
            0.05, 0,    0,    0,    0,    0,
            0,    0.05, 0,    0,    0,    0,
            0,    0,    9999, 0,    0,    0,
            0,    0,    0,    9999, 0,    0,
            0,    0,    0,    0,    9999, 0,
            0,    0,    0,    0,    0,    0.2
        ]

        self.pub_odom.publish(odom)

        if self.br is not None:
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = self.z
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.br.sendTransform(t)


def main():
    rospy.init_node("car_like_odometry_node", anonymous=False)
    CarLikeOdometry()
    rospy.spin()


if __name__ == "__main__":
    main()
