#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import tf
import math
import time

class RectangularZigzagVerticalWithPoles:

    def __init__(self):
        rospy.init_node("zigzag_node")

        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("/scan", LaserScan, self.scan_cb)
        rospy.Subscriber("/odom", Odometry, self.odom_cb)

        # ===== PARAMETRELER =====
        self.forward_speed = 0.20
        self.wall_distance = 0.60
        self.shift_distance = 0.75   # Normal kayma mesafesi

        self.Kp_turn = 1.6
        self.max_turn_speed = 0.5
        self.min_turn_speed = 0.12

        self.Kp_align = 1.2
        self.max_align_speed = 0.25
        # ======================

        self.front_dist = float("inf")
        self.front_points = 0
        self.yaw = 0.0

        self.state = "GO_TO_FIRST_WALL"
        self.start_time = 0.0

        self.turn_direction = 1
        self.target_yaw = 0.0

        # Satır yönü (Dikey)
        self.row_heading = math.pi / 2

        # Satır sayacı
        self.row_count = 0

        rospy.loginfo(" ")

    def scan_cb(self, msg):
        vals = []
        close = 0

        for i, r in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment
            if -0.26 < angle < 0.26:
                if not math.isinf(r) and not math.isnan(r):
                    vals.append(r)
                    if r < self.wall_distance:
                        close += 1

        if vals:
            self.front_dist = min(vals)

        self.front_points = close

    def odom_cb(self, msg):
        q = msg.pose.pose.orientation
        (_, _, self.yaw) = tf.transformations.euler_from_quaternion(
            [q.x, q.y, q.z, q.w]
        )

    def angle_error(self, target):
        return math.atan2(
            math.sin(target - self.yaw),
            math.cos(target - self.yaw)
        )

    def run(self):
        rate = rospy.Rate(15)
        cmd = Twist()

        while not rospy.is_shutdown():
            
            # 🛑 8. SATIR KONTROLÜ: Eğer 8 satır tamamlandıysa robotu durdur
            if self.row_count >= 14:
                rospy.loginfo("🏁 8 Satır tamamlandı. Robot durduruluyor...")
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.cmd_pub.publish(cmd)
                rospy.signal_shutdown("Görev başarıyla tamamlandı.")
                break

            # ========= İLK DUVARA GİT =========
            if self.state == "GO_TO_FIRST_WALL":
                cmd.linear.x = self.forward_speed
                cmd.angular.z = 0.0

                if self.front_dist < self.wall_distance:
                    self.state = "TURN_TO_START"
                    self.target_yaw = self.yaw + math.pi / 2

            # ========= BAŞLANGIÇ İÇİN DÖN =========
            elif self.state == "TURN_TO_START":
                error = self.angle_error(self.target_yaw)

                if abs(error) < 0.03:
                    self.row_count = 1
                    rospy.loginfo(" ")
                    self.state = "FORWARD"
                else:
                    w = self.Kp_turn * error
                    w = max(min(w, self.max_turn_speed), -self.max_turn_speed)
                    if abs(w) < self.min_turn_speed:
                        w = self.min_turn_speed * math.copysign(1, w)

                    cmd.linear.x = 0.0
                    cmd.angular.z = w

            # ========= İLERİ GİT (SATIR TARAMA) =========
            elif self.state == "FORWARD":
                err = self.angle_error(self.row_heading)
                w = self.Kp_align * err
                w = max(min(w, self.max_align_speed), -self.max_align_speed)

                cmd.linear.x = self.forward_speed
                cmd.angular.z = w

                # Ön tarafta geniş bir engel (duvar) algılanırsa
                if self.front_dist < self.wall_distance and self.front_points > 8:
                    self.state = "TURN_1"
                    self.target_yaw = self.yaw + self.turn_direction * math.pi / 2

            # ========= DÖNÜŞ 1 (YAN ŞERİDE GEÇİŞ ÖNCESİ) =========
            elif self.state == "TURN_1":
                error = self.angle_error(self.target_yaw)

                if abs(error) < 0.03:
                    self.state = "SHIFT"
                    self.start_time = time.time()
                else:
                    w = self.Kp_turn * error
                    w = max(min(w, self.max_turn_speed), -self.max_turn_speed)
                    if abs(w) < self.min_turn_speed:
                        w = self.min_turn_speed * math.copysign(1, w)

                    cmd.linear.x = 0.0
                    cmd.angular.z = w

            # ========= YANA KAYDIRMA (SHIFT) =========
            elif self.state == "SHIFT":
                # 6. satırdan sonra güvenli geçiş için kayma mesafesini artırır
                if 5 <= self.row_count <= 6:
                    effective_shift = self.shift_distance * 2.0
                else:
                    effective_shift = self.shift_distance

                cmd.linear.x = self.forward_speed
                cmd.angular.z = 0.0

                if time.time() - self.start_time > effective_shift / self.forward_speed:
                    self.state = "TURN_2"
                    self.target_yaw = self.yaw + self.turn_direction * math.pi / 2

            # ========= DÖNÜŞ 2 (YENİ SATIRA BAŞLANGIÇ) =========
            elif self.state == "TURN_2":
                error = self.angle_error(self.target_yaw)

                if abs(error) < 0.03:
                    self.turn_direction *= -1

                    # Yönü 180 derece çevir
                    self.row_heading = math.atan2(
                        math.sin(self.row_heading + math.pi),
                        math.cos(self.row_heading + math.pi)
                    )

                    self.row_count += 1
                    rospy.loginfo(f" satir temizlendi  {self.row_count}")

                    self.state = "FORWARD"
                    self.front_dist = float("inf")
                else:
                    w = self.Kp_turn * error
                    w = max(min(w, self.max_turn_speed), -self.max_turn_speed)
                    if abs(w) < self.min_turn_speed:
                        w = self.min_turn_speed * math.copysign(1, w)

                    cmd.linear.x = 0.0
                    cmd.angular.z = w

            self.cmd_pub.publish(cmd)
            rate.sleep()


if __name__ == "__main__":
    try:
        RectangularZigzagVerticalWithPoles().run()
    except rospy.ROSInterruptException:
        pass
