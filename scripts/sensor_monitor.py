#!/usr/bin/env python3

import rospy
import socket
import time
from diagnostic_msgs.msg import DiagnosticStatus
from nortek_dvl.msg import DvlStatus
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistWithCovarianceStamped
from diagnostic_updater import DiagnosticTask, Updater

ROS_MESSAGE_LIFETIME = 3

class ExpiringMessage:
    def __init__(self, message_life):
        self._value = None
        self._receive_time = 0
        self._message_life = message_life
    
    def update_value(self, value):
        self._value = value
        self._receive_time = time.time()
    
    def get_value(self):
        if time.time() - self._receive_time < self._message_life:
            return self._value
        else:
            return None

class DVLSensorTask(DiagnosticTask):
    def __init__(self):
        DiagnosticTask.__init__(self, "DVL Sensor")

        self._dvl_status = ExpiringMessage(ROS_MESSAGE_LIFETIME)
        self._dvl_twist = ExpiringMessage(ROS_MESSAGE_LIFETIME)

        rospy.Subscriber('dvl/status', DvlStatus, self.dvl_status_callback, queue_size=1)
        rospy.Subscriber('dvl_twist', TwistWithCovarianceStamped, self.dvl_twist_callback, queue_size=1)

    def dvl_status_callback(self, msg):
        self._dvl_status.update_value(msg)
    
    def dvl_twist_callback(self, msg):
        self._dvl_twist.update_value(True)

    def run(self, stat):
        dvl_status = self._dvl_status.get_value()
        dvl_twist = self._dvl_twist.get_value()

        if dvl_status is None:
            stat.summary(DiagnosticStatus.ERROR, "Not Connected")
        else:
            stat.add("B1 velocity valid", str(dvl_status.b1_vel_valid))
            stat.add("B2 velocity valid", str(dvl_status.b2_vel_valid))
            stat.add("B3 velocity valid", str(dvl_status.b3_vel_valid))
            stat.add("B4 velocity valid", str(dvl_status.b4_vel_valid))
            velocity_ok_count = int(dvl_status.b1_vel_valid) + int(dvl_status.b2_vel_valid) + int(dvl_status.b3_vel_valid) + int(dvl_status.b4_vel_valid)

            stat.add("B1 distance valid", str(dvl_status.b1_dist_valid))
            stat.add("B2 distance valid", str(dvl_status.b2_dist_valid))
            stat.add("B3 distance valid", str(dvl_status.b3_dist_valid))
            stat.add("B4 distance valid", str(dvl_status.b4_dist_valid))
            distance_ok_count = int(dvl_status.b1_dist_valid) + int(dvl_status.b2_dist_valid) + int(dvl_status.b3_dist_valid) + int(dvl_status.b4_dist_valid)

            stat.add("B1 FOM valid", str(dvl_status.b1_fom_valid))
            stat.add("B2 FOM valid", str(dvl_status.b2_fom_valid))
            stat.add("B3 FOM valid", str(dvl_status.b3_fom_valid))
            stat.add("B4 FOM valid", str(dvl_status.b4_fom_valid))
            fom_ok_count = int(dvl_status.b1_fom_valid) + int(dvl_status.b2_fom_valid) + int(dvl_status.b3_fom_valid) + int(dvl_status.b4_fom_valid)

            stat.add("X velocity valid", str(dvl_status.x_vel_valid))
            stat.add("Y velocity valid", str(dvl_status.y_vel_valid))
            stat.add("Z1 velocity valid", str(dvl_status.z1_vel_valid))
            stat.add("Z2 velocity valid", str(dvl_status.z2_vel_valid))
            xy2z_vel_ok_count = int(dvl_status.x_vel_valid) + int(dvl_status.y_vel_valid) + int(dvl_status.z1_vel_valid) + int(dvl_status.z2_vel_valid)

            stat.add("X FOM valid", str(dvl_status.x_fom_valid))
            stat.add("Y FOM valid", str(dvl_status.y_fom_valid))
            stat.add("Z1 FOM valid", str(dvl_status.z1_fom_valid))
            stat.add("Z2 FOM valid", str(dvl_status.z2_fom_valid))
            xy2z_fom_ok_count = int(dvl_status.x_fom_valid) + int(dvl_status.y_fom_valid) + int(dvl_status.z1_fom_valid) + int(dvl_status.z2_fom_valid)

            if dvl_twist is None:
                stat.summary(DiagnosticStatus.WARN, "Unable to get reading")
            else:
                stat.summary(DiagnosticStatus.OK, "OK ({0}/4 vel OK) ({1}/4 dist OK) ({2}/4 fom OK) ({3}/4 xy2z vel OK) ({4}/4 xy2z fom OK)".format(velocity_ok_count, distance_ok_count, fom_ok_count, xy2z_vel_ok_count, xy2z_fom_ok_count))

        return stat

class IMUSensorTask(DiagnosticTask):
    def __init__(self):
        DiagnosticTask.__init__(self, "IMU Sensor")

        self._imu_status = ExpiringMessage(ROS_MESSAGE_LIFETIME)

        rospy.Subscriber('imu/imu', Imu, self.imu_callback, queue_size=1)

    def imu_callback(self, msg):
        self._imu_status.update_value(True)

    def run(self, stat):
        imu_status = self._imu_status.get_value()

        if imu_status is None:
            stat.summary(DiagnosticStatus.ERROR, "Not Connected")
        else:
            stat.summary(DiagnosticStatus.OK, "OK")

        return stat


def main():
    hostname = socket.gethostname()
    rospy.init_node("sensor_monitor")

    updater = Updater()
    updater.setHardwareID(hostname)

    updater.add(DVLSensorTask())
    updater.add(IMUSensorTask())

    rate = rospy.Rate(rospy.get_param("~rate", 1))
    while not rospy.is_shutdown():
        rate.sleep()
        updater.update()


if __name__ == '__main__':
    main()