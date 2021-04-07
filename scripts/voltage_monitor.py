#!/usr/bin/env python

import rospy
import socket
import time
import yaml
from std_msgs.msg import Bool, Float32, Float32MultiArray
from diagnostic_msgs.msg import DiagnosticStatus
from riptide_msgs.msg import SwitchState
from diagnostic_updater import DiagnosticTask, Updater

# Robot Types
PUDDLES_ROBOT = "puddles"
TITAN_ROBOT = "titan"

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

class BatteryVoltageTask(DiagnosticTask):
    def __init__(self, warning_voltage, error_voltage, warning_voltage_diff):
        DiagnosticTask.__init__(self, "Battery Voltage")

        self._warning_voltage = float(warning_voltage)
        self._error_voltage = float(error_voltage)
        self._warning_voltage_diff = float(warning_voltage_diff)

        self._stbd_voltage = ExpiringMessage(ROS_MESSAGE_LIFETIME)
        self._port_voltage = ExpiringMessage(ROS_MESSAGE_LIFETIME)

        rospy.Subscriber('state/battery_voltage', Float32MultiArray, self.battery_voltage_callback, queue_size=1)

    def battery_voltage_callback(self, msg):
        self._port_voltage.update_value(msg.data[0])
        self._stbd_voltage.update_value(msg.data[1])

    def run(self, stat):
        port_voltage = self._port_voltage.get_value()
        stbd_voltage = self._stbd_voltage.get_value()

        if port_voltage is None or stbd_voltage is None:
            stat.summary(DiagnosticStatus.STALE, "No data available from copro publisher")
        else:
            difference_note = ""
            batt_low_note = ""
            if abs(stbd_voltage - port_voltage) >= self._warning_voltage_diff:
                difference_note = " [Voltage difference above nominal value of {}V]".format(self._warning_voltage_diff)
                batt_low_note = "[LARGE VDIF] "
            stat.add("Port Battery", "{:.2f}V".format(port_voltage))
            stat.add("Starboard Battery", "{:.2f}V".format(stbd_voltage))
            stat.add("Thruster Shutoff Voltage", "{}V".format(self._error_voltage))
            stat.add("Starboard-Port Voltage Difference", "{:.2f}V".format(abs(stbd_voltage-port_voltage)) + difference_note)


            if port_voltage <= self._error_voltage or stbd_voltage <= self._error_voltage:
                if port_voltage <= self._error_voltage and stbd_voltage <= self._error_voltage:
                    battery_error = "Both Batteries (P: {:.2f}V - S: {:.2f}V)".format(port_voltage, stbd_voltage)
                elif port_voltage <= self._error_voltage:
                    battery_error = "Port Battery ({:.2f}V)".format(port_voltage)
                elif stbd_voltage <= self._error_voltage:
                    battery_error = "Starboard Battery ({:.2f}V)".format(stbd_voltage)
                stat.summary(DiagnosticStatus.ERROR, batt_low_note + "{} below thruster cutoff voltage ({}V)".format(battery_error, self._error_voltage))

            elif port_voltage <= self._warning_voltage or stbd_voltage <= self._warning_voltage:
                if port_voltage <= self._warning_voltage and stbd_voltage <= self._warning_voltage:
                    battery_error = "Both Batteries (P: {:.2f}V - S: {:.2f}V)".format(port_voltage, stbd_voltage)
                elif port_voltage <= self._warning_voltage:
                    battery_error = "Port Battery ({:.2f}V)".format(port_voltage)
                elif stbd_voltage <= self._warning_voltage:
                    battery_error = "Starboard Battery ({:.2f}V)".format(stbd_voltage)
                stat.summary(DiagnosticStatus.WARN, batt_low_note + "{} below nominal voltage ({}V)".format(battery_error, self._warning_voltage))

            elif abs(stbd_voltage - port_voltage) >= self._warning_voltage_diff:
                stat.summary(DiagnosticStatus.WARN, "Battery voltage difference (P: {:.2f}V - S: {:.2f}V) is outside nominal range ({}V)".format(port_voltage, stbd_voltage, self._warning_voltage_diff))

            else:
                stat.summary(DiagnosticStatus.OK, "Battery Voltages (P: {:.2f}V - S: {:.2f}V) OK".format(port_voltage, stbd_voltage))

        return stat


class BatteryCurrentTask(DiagnosticTask):
    def __init__(self, warning_current, error_current):
        DiagnosticTask.__init__(self, "Battery Current")

        self._warning_current = float(warning_current)
        self._error_current = float(error_current)

        self._stbd_current = ExpiringMessage(ROS_MESSAGE_LIFETIME)
        self._port_current = ExpiringMessage(ROS_MESSAGE_LIFETIME)

        rospy.Subscriber('state/battery_current', Float32MultiArray, self.battery_current_callback, queue_size=1)

    def battery_current_callback(self, msg):
        self._port_current.update_value(msg.data[0])
        self._stbd_current.update_value(msg.data[1])

    def run(self, stat):
        port_current = self._port_current.get_value()
        stbd_current = self._stbd_current.get_value()

        if port_current is None or stbd_current is None:
            stat.summary(DiagnosticStatus.STALE, "No data available from copro publisher")
        else:
            stat.add("Port Battery", "{:.2f}A".format(port_current))
            stat.add("Starboard Battery", "{:.2f}A".format(stbd_current))

            if port_current >= self._error_current or stbd_current >= self._error_current:
                if port_current >= self._error_current and stbd_current >= self._error_current:
                    battery_error = "Both Batteries (P: {:.2f}A - S: {:.2f}A)".format(port_current, stbd_current)
                elif port_current >= self._error_current:
                    battery_error = "Port Battery ({:.2f}A)".format(port_current)
                elif stbd_current >= self._error_current:
                    battery_error = "Starboard Battery ({:.2f}A)".format(stbd_current)
                stat.summary(DiagnosticStatus.ERROR, "{} above battery fuse rating ({}A)".format(battery_error, self._error_current))

            elif port_current >= self._warning_current or stbd_current >= self._warning_current:
                if port_current >= self._warning_current and stbd_current >= self._warning_current:
                    battery_error = "Both Batteries (P: {:.2f}A - S: {:.2f}A)".format(port_current, stbd_current)
                elif port_current >= self._warning_current:
                    battery_error = "Port Battery ({:.2f}A)".format(port_current)
                elif stbd_current >= self._warning_current:
                    battery_error = "Starboard Battery ({:.2f}A)".format(stbd_current)
                stat.summary(DiagnosticStatus.WARN, "{} above nominal current ({}A)".format(battery_error, self._warning_current))
            else:
                stat.summary(DiagnosticStatus.OK, "Battery Current (P: {:.2f}A - S: {:.2f}A) OK".format(port_current, stbd_current))

        return stat


class ThrusterCurrentTask(DiagnosticTask):
    def __init__(self, warning_current, error_current):
        DiagnosticTask.__init__(self, "Thruster Current")

        self._warning_current = float(warning_current)
        self._error_current = float(error_current)

        self._thruster_currents = ExpiringMessage(ROS_MESSAGE_LIFETIME)
        self._kill_switch_engaged = ExpiringMessage(ROS_MESSAGE_LIFETIME)

        rospy.Subscriber('state/thruster_currents', Float32MultiArray, self.thruster_current_callback, queue_size=1)
        rospy.Subscriber('state/switches', SwitchState, self.kill_switch_callback, queue_size=1)

    def kill_switch_callback(self, msg):
        self._kill_switch_engaged.update_value(msg.kill)

    def thruster_current_callback(self, msg):
        self._thruster_currents.update_value(list(map(float, msg.data)))

    def generateThrusterList(self, thruster_list, thruster_currents):
        if len(thruster_list) == 1:
            message = "Thruster "
        else:
            message = "Thrusters "
        for i in range(len(thruster_list)):
            if i != 0:
                message += ", "
            thruster_id = thruster_list[i]
            message += "{} ({:.2f}A)".format(thruster_id+1, thruster_currents[thruster_id])
        return message

    def run(self, stat):
        thruster_currents = self._thruster_currents.get_value()
        kill_switch_engaged = self._kill_switch_engaged.get_value()

        if thruster_currents is None or kill_switch_engaged is None:
            stat.summary(DiagnosticStatus.STALE, "No data available from copro publisher")
        else:
            error_thrusters = []
            warning_thrusters = []
            zero_current_thrusters = []
            current_total = 0
            for i in range(len(thruster_currents)):
                extra_info = ""
                if thruster_currents[i] >= self._error_current:
                    error_thrusters.append(i)
                    extra_info = " [OVER FUSE]"
                elif thruster_currents[i] >= self._warning_current:
                    warning_thrusters.append(i)
                    extra_info = " [WARN]"
                elif thruster_currents[i] == 0:
                    zero_current_thrusters.append(i)
                    extra_info = " [OFF]"
                stat.add("Thruster {} Current".format(i+1), "{:.2f}A".format(thruster_currents[i]) + extra_info)
                current_total += thruster_currents[i]

            stat.add("Total Current", "{:.2f}A".format(current_total))

            if len(error_thrusters) > 0:
                error_message = self.generateThrusterList(error_thrusters, thruster_currents)
                stat.summary(DiagnosticStatus.ERROR, "{} above ESC fuse rating ({}A) (Total Current: {:.2f}A})".format(error_message, self._error_current, current_total))

            elif len(warning_thrusters) > 0:
                warning_message = self.generateThrusterList(warning_thrusters, thruster_currents)
                stat.summary(DiagnosticStatus.WARN, "{} above nominal ESC current ({}A) (Total Current: {:.2f}A)".format(warning_message, self._warning_current, current_total))
            
            #elif len(zero_current_thrusters) > 0 and kill_switch_engaged:
            #    if len(zero_current_thrusters) == len(thruster_currents):
            #        warning_message = "Thrusters"
            #    else:
            #        warning_message = self.generateThrusterList(zero_current_thrusters, thruster_currents)
            #    stat.summary(DiagnosticStatus.WARN, "{} not powered on while kill switch engaged".format(warning_message))
            elif len(zero_current_thrusters) != len(thruster_currents) and not kill_switch_engaged:
                if len(zero_current_thrusters) == 0:
                    warning_message = "Thrusters"
                else:
                    powered_on_thrusters = []
                    for i in range(len(thruster_currents)):
                        if i not in zero_current_thrusters:
                            powered_on_thrusters.append(i)
                    warning_message = self.generateThrusterList(powered_on_thrusters, thruster_currents)
                stat.summary(DiagnosticStatus.WARN, "{} powered on while kill switch is disengaged".format(warning_message))
            else:
                if kill_switch_engaged:
                    esc_state = "ON"
                else:
                    esc_state = "OFF"
                stat.summary(DiagnosticStatus.OK, "Total Thruster Current: {:.2f}A [ESCs {}]".format(current_total, esc_state))

        return stat


class FiveVoltMonitorTask(DiagnosticTask):
    def __init__(self, warning_voltage_min, warning_voltage_max):
        DiagnosticTask.__init__(self, "5V Rail Voltage")

        self._warning_voltage_min = float(warning_voltage_min)
        self._warning_voltage_max = float(warning_voltage_max)

        self._rail_voltage = ExpiringMessage(ROS_MESSAGE_LIFETIME)

        rospy.Subscriber('state/voltage_5', Float32, self.voltage_rail_callback, queue_size=1)

    def voltage_rail_callback(self, msg):
        self._rail_voltage.update_value(float(msg.data))

    def run(self, stat):
        rail_voltage = self._rail_voltage.get_value()

        if rail_voltage is None:
            stat.summary(DiagnosticStatus.STALE, "No data available from copro publisher")
        else:
            stat.add("5V Rail Voltage", "{:.2f}V".format(rail_voltage))

            if rail_voltage <= self._warning_voltage_min:
                stat.summary(DiagnosticStatus.WARN, "Rail ({:.2f}V) under nominal voltage ({}V)".format(rail_voltage, self._warning_voltage_min))
            elif rail_voltage >= self._warning_voltage_max:
                stat.summary(DiagnosticStatus.WARN, "Rail ({:.2f}V) above nominal voltage ({}V)".format(rail_voltage, self._warning_voltage_max))
            else:
                stat.summary(DiagnosticStatus.OK, "Rail at {:.2f}V".format(rail_voltage, rail_voltage))

        return stat


class TwelveVoltMonitorTask(DiagnosticTask):
    def __init__(self, warning_voltage_min, warning_voltage_max):
        DiagnosticTask.__init__(self, "12V Rail Voltage")

        self._warning_voltage_min = float(warning_voltage_min)
        self._warning_voltage_max = float(warning_voltage_max)

        self._rail_voltage = ExpiringMessage(ROS_MESSAGE_LIFETIME)

        rospy.Subscriber('state/voltage_12', Float32, self.voltage_rail_callback, queue_size=1)

    def voltage_rail_callback(self, msg):
        self._rail_voltage.update_value(float(msg.data))

    def run(self, stat):
        rail_voltage = self._rail_voltage.get_value()

        if rail_voltage is None:
            stat.summary(DiagnosticStatus.STALE, "No data available from copro publisher")
        else:
            stat.add("12V Rail Voltage", "{:.2f}V".format(rail_voltage))

            if rail_voltage <= self._warning_voltage_min:
                stat.summary(DiagnosticStatus.WARN, "Rail ({:.2f}V) under nominal voltage ({}V)".format(rail_voltage, self._warning_voltage_min))
            elif rail_voltage >= self._warning_voltage_max:
                stat.summary(DiagnosticStatus.WARN, "Rail ({:.2f}V) above nominal voltage ({}V)".format(rail_voltage, self._warning_voltage_max))
            else:
                stat.summary(DiagnosticStatus.OK, "Rail at {:.2f}V".format(rail_voltage, rail_voltage))

        return stat


class BalancedVoltageMonitorTask(DiagnosticTask):
    def __init__(self, warning_voltage_min, warning_voltage_max):
        DiagnosticTask.__init__(self, "V+ Rail Voltage")

        self._warning_voltage_min = float(warning_voltage_min)
        self._warning_voltage_max = float(warning_voltage_max)

        self._rail_voltage = ExpiringMessage(ROS_MESSAGE_LIFETIME)

        rospy.Subscriber('state/voltage_balanced', Float32, self.voltage_rail_callback, queue_size=1)

    def voltage_rail_callback(self, msg):
        self._rail_voltage.update_value(float(msg.data))

    def run(self, stat):
        rail_voltage = self._rail_voltage.get_value()

        if rail_voltage is None:
            stat.summary(DiagnosticStatus.STALE, "No data available from copro publisher")
        else:
            stat.add("V+ Rail Voltage", "{:.2f}V".format(rail_voltage))

            if rail_voltage <= self._warning_voltage_min:
                stat.summary(DiagnosticStatus.WARN, "Rail ({:.2f}V) under nominal voltage ({}V)".format(rail_voltage, self._warning_voltage_min))
            elif rail_voltage >= self._warning_voltage_max:
                stat.summary(DiagnosticStatus.WARN, "Rail ({:.2f}V) above nominal voltage ({}V)".format(rail_voltage, self._warning_voltage_max))
            else:
                stat.summary(DiagnosticStatus.OK, "Rail at {:.2f}V".format(rail_voltage, rail_voltage))

        return stat


def main():
    hostname = socket.gethostname()
    rospy.init_node("voltage_monitor")

    current_robot = rospy.get_param('~current_robot')

    updater = Updater()
    updater.setHardwareID(hostname)

    with open(rospy.get_param('~diag_thresholds_file'), 'r') as stream:
        thresholds = yaml.safe_load(stream)["volt_cur_thresholds"]
        updater.add(BatteryVoltageTask(thresholds["battery_volt"]["warn"], thresholds["battery_volt"]["thruster_cutoff"], thresholds["battery_volt"]["diff_warn"]))
        updater.add(BatteryCurrentTask(thresholds["battery_current"]["warn"], thresholds["battery_current"]["fuse"]))
        updater.add(ThrusterCurrentTask(thresholds["thruster_current"]["warn"], thresholds["thruster_current"]["fuse"]))
        if current_robot == TITAN_ROBOT:
            updater.add(FiveVoltMonitorTask(thresholds["five_volt"]["warn_min"], thresholds["five_volt"]["warn_max"]))
            updater.add(TwelveVoltMonitorTask(thresholds["twelve_volt"]["warn_min"], thresholds["twelve_volt"]["warn_max"]))
        updater.add(BalancedVoltageMonitorTask(rospy.get_param("~balanced_volt_warn_min", 19.5), rospy.get_param("~twelve_volt_warn_max", 22)))

    rate = rospy.Rate(rospy.get_param("~rate", 1))
    while not rospy.is_shutdown():
        rate.sleep()
        updater.update()


if __name__ == '__main__':
    main()