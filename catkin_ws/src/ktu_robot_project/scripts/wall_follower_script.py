#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math

class RoomOnlyMapper:
    def __init__(self):
        rospy.init_node('room_only_mapper')

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('/scan', LaserScan, self.scan_cb)

        self.cmd = Twist()

        # --- PARAMETRELER ---
        self.WALL_DIST = 0.5
        self.LINEAR = 0.20
        self.TURN = 0.8

    def clean(self, data):
        return min([d for d in data if not math.isinf(d)], default=3.5)

    def scan_cb(self, msg):
        front = self.clean(msg.ranges[350:360] + msg.ranges[0:10])
        right = self.clean(msg.ranges[260:300])

        # --- 1. ÖNDE DUVAR → SOLA DÖN ---
        if front < self.WALL_DIST:
            self.cmd.linear.x = 0.05
            self.cmd.angular.z = self.TURN
            rospy.loginfo("Ön Kapalı → Sola Dön")

        # --- 2. KAPI ALGILANDI (SAĞ BOŞ AMA ÖN AÇIK) ---
        elif right > self.WALL_DIST + 0.6 and front > 1.0:
            # Kapı var ama girmiyoruz
            self.cmd.linear.x = self.LINEAR
            self.cmd.angular.z = 0.0
            rospy.loginfo("Kapı Algılandı → Düz Devam (Girme)")

        # --- 3. DUVAR ÇOK YAKIN ---
        elif right < self.WALL_DIST - 0.1:
            self.cmd.linear.x = self.LINEAR
            self.cmd.angular.z = 0.4
            rospy.loginfo("Duvara Yakın → Sola Açıl")

        # --- 4. DUVAR KAYBOLUYOR (NORMAL KÖŞE) ---
        elif right > self.WALL_DIST + 0.2:
            self.cmd.linear.x = self.LINEAR
            self.cmd.angular.z = -0.4
            rospy.loginfo("Duvar Kayboluyor → Sağa Kır")

        # --- 5. NORMAL İLERLEME ---
        else:
            self.cmd.linear.x = self.LINEAR
            self.cmd.angular.z = 0.0
            rospy.loginfo("Duvar Takibi Normal")

        self.pub.publish(self.cmd)

if __name__ == '__main__':
    try:
        RoomOnlyMapper()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

