#!/usr/bin/env python3

import rclpy
import diagnostic_updater
import socket
import yaml
from rclpy.qos import QoSPresetProfiles
from riptide_msgs2.msg import FirmwareState, KillSwitchReport, RobotState
from diagnostic_msgs.msg import DiagnosticStatus

from .common import ExpiringMessage

ERROR_DESCRIPTIONS = [
    "FAULT_WATCHDOG_RESET",    #  0
    "FAULT_ROS_SOFT_FAIL",     #  1
    "FAULT_ROS_BAD_COMMAND",   #  2
    "FAULT_DSHOT_ERROR",       #  3
    "FAULT_THRUSTER_TIMEOUT",  #  4
    "FAULT_ASYNC_I2C_ERROR",   #  5
    "FAULT_DEPTH_INIT_ERROR",  #  6
    "FAULT_DEPTH_ERROR",       #  7
    "FAULT_ADC_ERROR",         #  8
    "FAULT_BB_ADC_ERROR",      #  9
    "FAULT_ESC_ADC_ERROR",     # 10
    "FAULT_COOLING_STALE",     # 11
    "FAULT_LOWBATT_STALE",     # 12
    "FAULT_LOW_BATTERY",       # 13
    "FAULT_ACTUATOR_FAIL",     # 14
    "FAULT_NO_ACTUATOR",       # 15
]

class CoprocessorStatusTask(diagnostic_updater.DiagnosticTask):
    def __init__(self, firmware_state, warning_percentage):
        diagnostic_updater.DiagnosticTask.__init__(self, "Copro")

        self._warning_percentage = int(warning_percentage)
        self._firmware_state = firmware_state

    def gen_fault_string(self, fault_bits):
        fault_list = []
        fault_num = 0
        while fault_bits != 0:
            if (fault_bits & 1) != 0:
                if fault_num < len(ERROR_DESCRIPTIONS):
                    fault_list.append(ERROR_DESCRIPTIONS[fault_num])
                else:
                    fault_list.append("UNKNOWN_ERR_{0}".format(fault_num))
            fault_num += 1
            fault_bits >>= 1
        return ", ".join(fault_list)

    def run(self, stat):
        firmware_state = self._firmware_state.get_value()

        if firmware_state is None:
            stat.summary(DiagnosticStatus.ERROR, "Not Connected")
            return stat
        
        memory_usage = firmware_state.copro_memory_usage
        copro_faults = firmware_state.copro_faults

        stat.add("Copro Memory Usage", "{:d}%".format(memory_usage))
        stat.add("Copro Faults", self.gen_fault_string(copro_faults))

        if copro_faults != 0:
            stat.summary(DiagnosticStatus.ERROR, "Fault Code " + str(self.gen_fault_string(copro_faults)))
        elif memory_usage > self._warning_percentage:
            stat.summary(DiagnosticStatus.WARN, "Copro Memory {:d}% above {:d}%".format(memory_usage, self._warning_percentage))
        else:
            stat.summary(DiagnosticStatus.OK, "Copro Memory {:d}%".format(memory_usage))

        return stat

class RobotTemperatureTask(diagnostic_updater.DiagnosticTask):
    def __init__(self, robot_state, firmware_state, warning_temp_above):
        diagnostic_updater.DiagnosticTask.__init__(self, "Robot Temperature")

        self._warning_temp_above = int(warning_temp_above)
        self._robot_state = robot_state
        self._firmware_state = firmware_state

    def run(self, stat):
        robot_state = self._robot_state.get_value()
        firmware_state = self._firmware_state.get_value()

        if robot_state is None or firmware_state is None:
            stat.summary(DiagnosticStatus.STALE, "No data available from copro")
            return stat

        temperature = robot_state.robot_temperature
        temp_threshold = firmware_state.peltier_cooling_threshold
        peltier_power = robot_state.peltier_active
        
        stat.add("Cooling Temperature Threshold", str(temp_threshold) + "C")
        stat.add("Peltier Powered", str(peltier_power))
            
        if temperature == RobotState.NO_READING:
            stat.summary(DiagnosticStatus.ERROR, "Unable to read temperature sensor")
        else:
            stat.add("Robot Temperature", "{:.2f}C".format(temperature))

            peltier_status_msg = " [Peltier "
            if peltier_power:
                peltier_status_msg += "ON"
            else:
                peltier_status_msg += "OFF"
            peltier_status_msg += "]"

            if (temperature - temp_threshold) >= self._warning_temp_above:
                stat.summary(DiagnosticStatus.WARN, "Temperature ({:.2f}C) is over {}C above cooling threshold".format(temperature, self._warning_temp_above) + peltier_status_msg)
            else:
                stat.summary(DiagnosticStatus.OK, "Temperature at {:.2f}C".format(temperature) + peltier_status_msg)

        return stat


class WaterTemperatureTask(diagnostic_updater.DiagnosticTask):
    def __init__(self, robot_state, warn_temp_below, warn_temp_above):
        diagnostic_updater.DiagnosticTask.__init__(self, "Water Temperature")

        self._warn_temp_below = float(warn_temp_below)
        self._warn_temp_above = float(warn_temp_above)
        self._robot_state = robot_state

    def run(self, stat):
        robot_state = self._robot_state.get_value()

        if robot_state is None:
            stat.summary(DiagnosticStatus.STALE, "No data available from copro")
            return stat

        water_temp = robot_state.water_temperature

        stat.add("Temperature", "{:.3f}C".format(water_temp))

        if water_temp == RobotState.NO_READING:
            stat.summary(DiagnosticStatus.ERROR, "Unable to read water temp")
        elif water_temp >= self._warn_temp_above:
            stat.summary(DiagnosticStatus.WARN, "Temp {:.2f}C above nominal water temp {:.1f}C".format(water_temp, self._warn_temp_above))
        elif water_temp <= self._warn_temp_below:
            stat.summary(DiagnosticStatus.WARN, "Temp {:.2f}C below nominal water temp {:.1f}C".format(water_temp, self._warn_temp_below))
        else:
            stat.summary(DiagnosticStatus.OK, "{:.2f}C".format(water_temp))

        return stat


class DepthSensorTask(diagnostic_updater.DiagnosticTask):
    def __init__(self, firmware_state):
        diagnostic_updater.DiagnosticTask.__init__(self, "Depth Sensor")

        self._firmware_state = firmware_state

    def run(self, stat):
        firmware_state = self._firmware_state.get_value()

        if firmware_state is None:
            stat.summary(DiagnosticStatus.STALE, "No data available from copro")
            return stat

        if firmware_state.depth_sensor_initialized:
            stat.summary(DiagnosticStatus.OK, "Connected")
        else:
            stat.summary(DiagnosticStatus.ERROR, "Not Connected")

        return stat


class KillSwitchTask(diagnostic_updater.DiagnosticTask):
    def __init__(self, robot_state, firmware_state):
        diagnostic_updater.DiagnosticTask.__init__(self, "Kill Switch")

        self._robot_state = robot_state
        self._firmware_state = firmware_state

    def lookup_kill_switch(self, switch_id):
        if switch_id == KillSwitchReport.KILL_SWITCH_PHYSICAL:
            return "Physical"
        elif switch_id == KillSwitchReport.KILL_SWITCH_RQT_CONTROLLER:
            return "RQT Controller"
        elif switch_id == KillSwitchReport.KILL_SWITCH_TOPSIDE_BUTTON:
            return "Topside Button"
        elif switch_id == KillSwitchReport.KILL_SWITCH_DEBUG:
            return "Debug"
        else:
            return "Unknown ID {0}".format(switch_id)

    def make_bitwise_kill_switch_list(self, switch_bits):
        switch_id = 0
        switch_list = []
        while switch_bits != 0:
            if (switch_bits & 1) != 0:
                switch_list.append(self.lookup_kill_switch(switch_id))
            switch_bits >>= 1
            switch_id += 1
        return switch_list

    def run(self, stat):
        robot_state = self._robot_state.get_value()
        firmware_state = self._firmware_state.get_value()

        if robot_state is None or firmware_state is None:
            stat.summary(DiagnosticStatus.STALE, "No data available from copro")
            return stat
        
        kill_switch_inserted = robot_state.kill_switch_inserted
        kill_switches_enabled = firmware_state.kill_switches_enabled
        kill_switches_asserting_kill = firmware_state.kill_switches_asserting_kill
        kill_switches_needs_update = firmware_state.kill_switches_needs_update
        kill_switches_timed_out = firmware_state.kill_switches_timed_out

        stat.add("Kill Switch State", "Run" if kill_switch_inserted else "Killed")
        stat.add("Active Kill Switches", "\n".join(self.make_bitwise_kill_switch_list(kill_switches_enabled)))
        stat.add("Switches Requiring Update", "\n".join(self.make_bitwise_kill_switch_list(kill_switches_needs_update)))
        stat.add("Switches Asserting Kill", "\n".join(self.make_bitwise_kill_switch_list(kill_switches_asserting_kill)))
        stat.add("Switches Timed Out", "\n".join(self.make_bitwise_kill_switch_list(kill_switches_timed_out)))

        if kill_switch_inserted != (not bool(kill_switches_asserting_kill | kill_switches_timed_out)):
            stat.summary(DiagnosticStatus.ERROR, "Kill state does not match expected state!")
        elif kill_switches_timed_out != 0:
            stat.summary(DiagnosticStatus.WARN, "{} kill switch timed out".format(", ".join(self.make_bitwise_kill_switch_list(kill_switches_timed_out))))
        elif kill_switch_inserted:
            stat.summary(DiagnosticStatus.OK, "Inserted")
        elif (kill_switches_asserting_kill & (1<<KillSwitchReport.KILL_SWITCH_PHYSICAL)) != 0:
            stat.summary(DiagnosticStatus.OK, "Removed")
        elif kill_switches_asserting_kill != 0:
            stat.summary(DiagnosticStatus.OK, "{} switch asserting kill".format(", ".join(self.make_bitwise_kill_switch_list(kill_switches_asserting_kill))))
        else:
            stat.summary(DiagnosticStatus.STALE, "Bad State!")

        return stat


class ActuatorStatusTask(diagnostic_updater.DiagnosticTask):
    def __init__(self, firmware_state):
        diagnostic_updater.DiagnosticTask.__init__(self, "Actuators")

        self._firmware_state = firmware_state

    def run(self, stat):
        firmware_state = self._firmware_state.get_value()

        if firmware_state is None:
            stat.summary(DiagnosticStatus.STALE, "No data available from copro")
            return stat
        
        actuator_connected = firmware_state.actuator_connected
        actuator_fault = firmware_state.actuator_faults

        stat.add("Actuators Connected", str(actuator_connected))
        stat.add("Actuator Faults", str(actuator_fault))
        if not actuator_connected:
            stat.summary(DiagnosticStatus.ERROR, "Not Connected")
        elif actuator_fault > 0:
            stat.summary(DiagnosticStatus.ERROR, "Actuators in fault state")
        else:
            stat.summary(DiagnosticStatus.OK, "OK")

        return stat


class ElectricalMonitor:
    def firmware_state_cb(self, msg):
        self.firmware_state_msg.update_value(msg)
    
    def robot_state_cb(self, msg):
        self.robot_state_msg.update_value(msg)

    def run(self):
        hostname = socket.gethostname()
        rclpy.init()
        node = rclpy.create_node('electrical_monitor')
        node.declare_parameter('robot', rclpy.Parameter.Type.STRING)
        node.declare_parameter('diag_thresholds_file', rclpy.Parameter.Type.STRING)

        current_robot = node.get_parameter('robot').value

        # Load config file
        with open(node.get_parameter('diag_thresholds_file').value, 'r') as stream:
            thresholds_file = yaml.safe_load(stream)
        message_lifetime = float(thresholds_file["ros_message_lifetime"])
        thresholds = thresholds_file["electrical_monitor_thresholds"]

        # Subscribe to messages
        self.firmware_state_msg = ExpiringMessage(node.get_clock(), message_lifetime)
        self.robot_state_msg = ExpiringMessage(node.get_clock(), message_lifetime)
        node.create_subscription(FirmwareState, "state/firmware", self.firmware_state_cb, QoSPresetProfiles.SENSOR_DATA.value)
        node.create_subscription(RobotState, "state/robot", self.robot_state_cb, QoSPresetProfiles.SENSOR_DATA.value)

        # Create diagnostics updater
        updater = diagnostic_updater.Updater(node)
        updater.setHardwareID(hostname)

        updater.add(WaterTemperatureTask(self.robot_state_msg, thresholds["water_low_warn_temp"], thresholds["water_high_warn_temp"]))
        updater.add(RobotTemperatureTask(self.robot_state_msg, self.firmware_state_msg, thresholds["temp_over_target_warn"]))
        updater.add(CoprocessorStatusTask(self.firmware_state_msg, thresholds["copro_mem_warn_percentage"]))
        updater.add(DepthSensorTask(self.firmware_state_msg))
        updater.add(KillSwitchTask(self.robot_state_msg, self.firmware_state_msg))
        updater.add(ActuatorStatusTask(self.firmware_state_msg))

        updater.force_update()

        rclpy.spin(node, None)

    @staticmethod
    def main():
        monitor = ElectricalMonitor()
        monitor.run()


if __name__ == '__main__':
    ElectricalMonitor.main()