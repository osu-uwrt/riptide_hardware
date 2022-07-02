#!/usr/bin/env python3

import rclpy
import socket
import yaml
from rclpy.qos import QoSPresetProfiles
import diagnostic_updater
from riptide_msgs2.msg import ElectricalReadings
from diagnostic_msgs.msg import DiagnosticStatus

from .common import ExpiringMessage

class BatteryVoltageTask(diagnostic_updater.DiagnosticTask):
    def __init__(self, electrical_readings_msg: ExpiringMessage, batt_voltage_thresholds):
        diagnostic_updater.DiagnosticTask.__init__(self, "Battery Voltage")

        self._warning_voltage = float(batt_voltage_thresholds["warn"])
        self._error_voltage = float(batt_voltage_thresholds["thruster_cutoff"])
        self._warning_voltage_diff = float(batt_voltage_thresholds["diff_warn"])

        self._electrical_readings_msg = electrical_readings_msg

    def run(self, stat):
        electrical_reading = self._electrical_readings_msg.get_value()
        
        if electrical_reading is None:
            stat.summary(DiagnosticStatus.STALE, "No data available from copro")
            return stat

        port_voltage = electrical_reading.port_voltage
        stbd_voltage = electrical_reading.stbd_voltage
        
        stat.add("Port Battery", "{:.2f}V".format(port_voltage))
        stat.add("Starboard Battery", "{:.2f}V".format(stbd_voltage))
        stat.add("Thruster Shutoff Voltage", "{}V".format(self._error_voltage))

        if port_voltage == ElectricalReadings.NO_READING or stbd_voltage == ElectricalReadings.NO_READING:
            stat.summary(DiagnosticStatus.ERROR, "Unable to read battery voltages")
        else:
            difference_note = ""
            batt_low_note = ""
            if abs(stbd_voltage - port_voltage) >= self._warning_voltage_diff:
                difference_note = " [Voltage difference above nominal value of {}V]".format(self._warning_voltage_diff)
                batt_low_note = "[LARGE VDIF] "
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


class BatteryCurrentTask(diagnostic_updater.DiagnosticTask):
    def __init__(self, electrical_readings_msg: ExpiringMessage, current_thresholds):
        diagnostic_updater.DiagnosticTask.__init__(self, "Battery Current")

        self._warning_current = float(current_thresholds["warn"])
        self._error_current = float(current_thresholds["fuse"])

        self._electrical_readings_msg = electrical_readings_msg

    def run(self, stat):
        electrical_reading = self._electrical_readings_msg.get_value()
        
        if electrical_reading is None:
            stat.summary(DiagnosticStatus.STALE, "No data available from copro")
            return stat
        
        port_current = electrical_reading.port_current
        stbd_current = electrical_reading.stbd_current
        
        stat.add("Port Battery", "{:.2f}A".format(port_current))
        stat.add("Starboard Battery", "{:.2f}A".format(stbd_current))

        if port_current == ElectricalReadings.NO_READING or stbd_current == ElectricalReadings.NO_READING:
            stat.summary(DiagnosticStatus.ERROR, "Unable to read battery currents")
        else:
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


class ThrusterCurrentTask(diagnostic_updater.DiagnosticTask):
    def __init__(self, electrical_readings_msg: ExpiringMessage, current_thresholds):
        diagnostic_updater.DiagnosticTask.__init__(self, "Thruster Current")

        self._warning_current = float(current_thresholds["warn"])
        self._error_current = float(current_thresholds["fuse"])

        self._electrical_readings_msg = electrical_readings_msg

    def kill_switch_callback(self, msg):
        self._kill_switch_engaged.update_value(msg)

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
        electrical_reading = self._electrical_readings_msg.get_value()

        if electrical_reading is None:
            stat.summary(DiagnosticStatus.STALE, "No data available from copro")
            return stat

        thruster_currents = electrical_reading.esc_current

        if ElectricalReadings.NO_READING in thruster_currents:
            stat.summary(DiagnosticStatus.ERROR, "Unable to read thruster currents")
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
            

            # TODO: Fix kill switch detection

            # elif len(zero_current_thrusters) > 0 and kill_switch_engaged:
            #     if len(zero_current_thrusters) == len(thruster_currents):
            #         warning_message = "Thrusters"
            #     else:
            #         warning_message = self.generateThrusterList(zero_current_thrusters, thruster_currents)
            #     stat.summary(DiagnosticStatus.WARN, "{} not powered on while kill switch engaged".format(warning_message))
            # elif len(zero_current_thrusters) != len(thruster_currents) and not kill_switch_engaged:
            #     if len(zero_current_thrusters) == 0:
            #         warning_message = "Thrusters"
            #     else:
            #         powered_on_thrusters = []
            #         for i in range(len(thruster_currents)):
            #             if i not in zero_current_thrusters:
            #                 powered_on_thrusters.append(i)
            #         warning_message = self.generateThrusterList(powered_on_thrusters, thruster_currents)
            #     stat.summary(DiagnosticStatus.WARN, "{} powered on while kill switch is disengaged".format(warning_message))
            else:
                # if kill_switch_engaged:
                #     esc_state = "ON"
                # else:
                #     esc_state = "OFF"
                #stat.summary(DiagnosticStatus.OK, "Total Thruster Current: {:.2f}A [ESCs {}]".format(current_total, esc_state))
                stat.summary(DiagnosticStatus.OK, "Total Thruster Current: {:.2f}A".format(current_total))

        return stat


class FiveVoltMonitorTask(diagnostic_updater.DiagnosticTask):
    def __init__(self, electrical_readings_msg: ExpiringMessage, rail_thresholds):
        diagnostic_updater.DiagnosticTask.__init__(self, "5V Rail Voltage")

        self._warn_voltage_min = float(rail_thresholds["warn_voltage_min"])
        self._warn_voltage_max = float(rail_thresholds["warn_voltage_max"])
        self._warn_current_max = float(rail_thresholds["warn_current_max"])

        self._electrical_readings_msg = electrical_readings_msg

    def run(self, stat):
        electrical_reading = self._electrical_readings_msg.get_value()

        if electrical_reading is None:
            stat.summary(DiagnosticStatus.STALE, "No data available from copro")
            return stat
        
        rail_voltage = electrical_reading.five_volt_voltage
        rail_current = electrical_reading.five_volt_current

        if rail_voltage == ElectricalReadings.NO_READING or rail_current == ElectricalReadings.NO_READING:
            stat.summary(DiagnosticStatus.ERROR, "Unable to read 5V rail")
        else:
            stat.add("5V Rail Voltage", "{:.2f}V".format(rail_voltage))
            stat.add("5V Rail Current", "{:.2f}A".format(rail_current))

            if rail_current >= self._warn_current_max:
                stat.summary(DiagnosticStatus.WARN, "Rail ({:.2f}A) above nominal rail current ({}A)".format(rail_current, self._warn_current_max))
            elif rail_voltage <= self._warn_voltage_min:
                stat.summary(DiagnosticStatus.WARN, "Rail ({:.2f}V) under nominal voltage ({}V)".format(rail_voltage, self._warn_voltage_min))
            elif rail_voltage >= self._warn_voltage_max:
                stat.summary(DiagnosticStatus.WARN, "Rail ({:.2f}V) above nominal voltage ({}V)".format(rail_voltage, self._warn_voltage_max))
            else:
                stat.summary(DiagnosticStatus.OK, "Rail at {:.2f}V {:.2f}A".format(rail_voltage, rail_current))

        return stat


class TwelveVoltMonitorTask(diagnostic_updater.DiagnosticTask):
    def __init__(self, electrical_readings_msg: ExpiringMessage, rail_thresholds):
        diagnostic_updater.DiagnosticTask.__init__(self, "12V Rail Voltage")

        self._warn_voltage_min = float(rail_thresholds["warn_voltage_min"])
        self._warn_voltage_max = float(rail_thresholds["warn_voltage_max"])
        self._warn_current_max = float(rail_thresholds["warn_current_max"])

        self._electrical_readings_msg = electrical_readings_msg

    def run(self, stat):
        electrical_reading = self._electrical_readings_msg.get_value()

        if electrical_reading is None:
            stat.summary(DiagnosticStatus.STALE, "No data available from copro")
            return stat

        rail_voltage = electrical_reading.twelve_volt_voltage
        rail_current = electrical_reading.twelve_volt_current

        if rail_voltage == ElectricalReadings.NO_READING or rail_current == ElectricalReadings.NO_READING:
            stat.summary(DiagnosticStatus.ERROR, "Unable to read 12V rail")
        else:
            stat.add("12V Rail Voltage", "{:.2f}V".format(rail_voltage))
            stat.add("12V Rail Current", "{:.2f}A".format(rail_current))

            if rail_current >= self._warn_current_max:
                stat.summary(DiagnosticStatus.WARN, "Rail ({:.2f}A) above nominal rail current ({}A)".format(rail_current, self._warn_current_max))
            elif rail_voltage <= self._warn_voltage_min:
                stat.summary(DiagnosticStatus.WARN, "Rail ({:.2f}V) under nominal voltage ({}V)".format(rail_voltage, self._warn_voltage_min))
            elif rail_voltage >= self._warn_voltage_max:
                stat.summary(DiagnosticStatus.WARN, "Rail ({:.2f}V) above nominal voltage ({}V)".format(rail_voltage, self._warn_voltage_max))
            else:
                stat.summary(DiagnosticStatus.OK, "Rail at {:.2f}V {:.2f}A".format(rail_voltage, rail_current))

        return stat


class BalancedVoltageMonitorTask(diagnostic_updater.DiagnosticTask):
    def __init__(self, electrical_readings_msg: ExpiringMessage, rail_thresholds):
        diagnostic_updater.DiagnosticTask.__init__(self, "V+ Rail Voltage")

        self._warn_voltage_min = float(rail_thresholds["warn_voltage_min"])
        self._warn_voltage_max = float(rail_thresholds["warn_voltage_max"])

        self._electrical_readings_msg = electrical_readings_msg

    def run(self, stat):
        electrical_reading = self._electrical_readings_msg.get_value()

        if electrical_reading is None:
            stat.summary(DiagnosticStatus.STALE, "No data available from copro")
            return stat
        
        rail_voltage = electrical_reading.balanced_voltage

        if rail_voltage == ElectricalReadings.NO_READING:
            stat.summary(DiagnosticStatus.ERROR, "Unable to read V+ rail voltage")
        else:
            stat.add("V+ Rail Voltage", "{:.2f}V".format(rail_voltage))

            if rail_voltage <= self._warn_voltage_min:
                stat.summary(DiagnosticStatus.WARN, "Rail ({:.2f}V) under nominal voltage ({}V)".format(rail_voltage, self._warn_voltage_min))
            elif rail_voltage >= self._warn_voltage_max:
                stat.summary(DiagnosticStatus.WARN, "Rail ({:.2f}V) above nominal voltage ({}V)".format(rail_voltage, self._warn_voltage_max))
            else:
                stat.summary(DiagnosticStatus.OK, "Rail at {:.2f}V".format(rail_voltage, rail_voltage))

        return stat


class VoltageMonitor:
    def electrical_state_cb(self, msg):
        self.electrical_readings_msg.update_value(msg)

    def run(self):
        hostname = socket.gethostname()
        rclpy.init()
        node = rclpy.create_node('voltage_monitor')
        node.declare_parameter('robot', rclpy.Parameter.Type.STRING)
        node.declare_parameter('diag_thresholds_file', rclpy.Parameter.Type.STRING)
        
        current_robot = node.get_parameter('robot').value

        # Load config file
        with open(node.get_parameter('diag_thresholds_file').value, 'r') as stream:
            thresholds_file = yaml.safe_load(stream)
        message_lifetime = thresholds_file["ros_message_lifetime"]
        thresholds = thresholds_file["volt_cur_thresholds"]

        # Subscribe to messages
        self.electrical_readings_msg = ExpiringMessage(node.get_clock(), message_lifetime)
        node.create_subscription(ElectricalReadings, "state/electrical", self.electrical_state_cb, QoSPresetProfiles.SENSOR_DATA.value)

        # Create diagnostics updater
        updater = diagnostic_updater.Updater(node)
        updater.setHardwareID(hostname)

        #updater.add(BatteryVoltageTask(self.electrical_readings_msg, thresholds["battery_volt"]))
        #updater.add(BatteryCurrentTask(self.electrical_readings_msg, thresholds["battery_current"]))
        #updater.add(ThrusterCurrentTask(self.electrical_readings_msg, thresholds["thruster_current"]))
        #if current_robot == "tempest":
        #    updater.add(FiveVoltMonitorTask(self.electrical_readings_msg, thresholds["five_volt"]))
        #    updater.add(TwelveVoltMonitorTask(self.electrical_readings_msg, thresholds["twelve_volt"]))
        updater.add(BalancedVoltageMonitorTask(self.electrical_readings_msg, thresholds["balanced_volt"]))

        updater.force_update()

        rclpy.spin(node, None)

    @staticmethod
    def main():
        monitor = VoltageMonitor()
        monitor.run()

if __name__ == '__main__':
    VoltageMonitor.main()