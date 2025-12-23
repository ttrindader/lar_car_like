#!/usr/bin/env python3
import math
import rospy
from std_msgs.msg import Float64
from ackermann_msgs.msg import AckermannDriveStamped

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

class VAController:
    def __init__(self):
        ns = rospy.get_param("~ns", "car_like").strip("/")
        self.ns = "/" + ns

        self.wheel_radius = rospy.get_param("~wheel_radius", 0.0315)   # m
        self.wheelbase    = rospy.get_param("~wheelbase", 0.16)        # m (ajuste!)
        self.track        = rospy.get_param("~track", 0.13)            # m (ajuste!)
        self.max_steer    = rospy.get_param("~max_steer", 0.7)         # rad
        self.max_speed    = rospy.get_param("~max_speed", 2.0)         # m/s

        # Publishers
        self.pub_steer_l = rospy.Publisher(self.ns + "/front_left_steer_controller/command",  Float64, queue_size=1)
        self.pub_steer_r = rospy.Publisher(self.ns + "/front_right_steer_controller/command", Float64, queue_size=1)
        self.pub_w_l     = rospy.Publisher(self.ns + "/rear_left_wheel_controller/command",   Float64, queue_size=1)
        self.pub_w_r     = rospy.Publisher(self.ns + "/rear_right_wheel_controller/command",  Float64, queue_size=1)

        rospy.Subscriber(self.ns + "/ackermann_cmd", AckermannDriveStamped, self.cb, queue_size=1)

    def cb(self, msg: AckermannDriveStamped):
        v = clamp(msg.drive.speed, -self.max_speed, self.max_speed)          # m/s
        delta = clamp(msg.drive.steering_angle, -self.max_steer, self.max_steer)  # rad

        # Wheel angular speed (rad/s)
        omega = v / max(1e-6, self.wheel_radius)

        # Ackermann steering (opcional, melhora a curva)
        # Se delta ~ 0, evita divisão por zero e usa igual
        if abs(delta) < 1e-4:
            delta_l = delta_r = 0.0
        else:
            R = self.wheelbase / math.tan(delta)  # raio de curva do centro do eixo traseiro
            # ângulos das rodas dianteiras (aprox. Ackermann)
            delta_l = math.atan(self.wheelbase / (R - self.track/2.0))
            delta_r = math.atan(self.wheelbase / (R + self.track/2.0))

        self.pub_steer_l.publish(Float64(delta_l))
        self.pub_steer_r.publish(Float64(delta_r))
        self.pub_w_l.publish(Float64(omega))
        self.pub_w_r.publish(Float64(omega))

if __name__ == "__main__":
    rospy.init_node("car_like_v_angle_controller")
    VAController()
    rospy.spin()

