#!/usr/bin/env python3

import rospy
import socket
import time
import yaml
from std_msgs.msg import Bool, Float32, UInt8MultiArray, Int8
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_updater import DiagnosticTask, Updater

# Robot Types
PUDDLES_ROBOT = "puddles"
TITAN_ROBOT = "titan"

ROS_MESSAGE_LIFETIME = 3

ERROR_DESCRIPTIONS = [
    "Undefined",
    "PROGRAM_TERMINATED",            # 1
    "MAIN_LOOP_CRASH",               # 2
    "DEPTH_LOOP_CRASH",              # 3
    "BATTERY_CHECKER_CRASH",         # 4
    "AUTO_COOLING_CRASH",            # 5
    "BB_INIT_FAIL",                  # 6
    "ESC_INIT_FAIL",                 # 7
    "DEPTH_INIT_FAIL",               # 8
    "BACKPLANE_INIT_FAIL",           # 9
    "FAULT_STATE_INVALID",           # 10
    "BATT_LOW",                      # 11
    "WATCHDOG_RESET",                # 12
    "CONV_BOARD_INIT_FAIL",          # 13
    "THRUSTER_SAFETY_MONITOR_CRASH", # 14
    "DEVICE_BOOTING",                # 15
]

COMMAND_DESCRIPTIONS = [
    "MOBO_POWER",			#0
    "LIGHTING",         	#1
    "THRUSTER_ENABLE",		#2
    "PELTIER_POWER",	    #3
    "GET_BAT_VOLTS",	    #4
    "GET_BAT_CURRENTS",		#5
    "GET_TEMPERATURE",		#6
    "THRUSTER_FORCE",		#7
    "LOGIC_CURRENTS",		#8
    "LOGIC_VOLTS",			#9
    "SWITCHES",			    #10
    "DEPTH",				#11
    "GET_THRUSTER_CURRENTS",#12
    "TWELVE_POWER",		    #13
    "FIVE_RESET",			#14
    "RESET",				#15
    "ACTUATOR",	     	    #16
    "LATENCY_CHECK",        #17 
    "MEMORY_CHECK",         #18 
    "TEMP_THRESHOLD",       #19 
    "GET_FAULT_STATE",	    #20
    "GET_VERSION_CMD",      #21
    "SAFETY_KEEPALIVE_CMD", #22
]

COMMAND_EXEC_CRASH_FLAG = (1<<7)

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

class CoprocessorStatusTask(DiagnosticTask):
    def __init__(self, warning_percentage):
        DiagnosticTask.__init__(self, "Copro")
        self._warning_percentage = int(warning_percentage)

        self._copro_memory_usage = ExpiringMessage(ROS_MESSAGE_LIFETIME)
        self._copro_connected = ExpiringMessage(ROS_MESSAGE_LIFETIME)
        self._copro_fault_list = ExpiringMessage(ROS_MESSAGE_LIFETIME)

        rospy.Subscriber('state/copro', Bool, self.copro_state_callback, queue_size=1)
        rospy.Subscriber('state/copro_fault', UInt8MultiArray, self.copro_fault_list_callback, queue_size=1)
        rospy.Subscriber('state/copro_memory_usage', Float32, self.copro_memory_callback, queue_size=1)

    def copro_state_callback(self, msg):
        self._copro_connected.update_value(msg.data)
    
    def copro_memory_callback(self, msg):
        self._copro_memory_usage.update_value(msg.data * 100)
    
    def copro_fault_list_callback(self, msg):
        if len(msg.data) > 0 and not isinstance(msg.data[0], int):
            self._copro_fault_list.update_value(list(map(ord, msg.data)))
        else:
            self._copro_fault_list.update_value(msg.data)

    def genFaultList(self):
        fault_string = ""
        fault_list = self._copro_fault_list.get_value()
        for i in range(len(fault_list)):
            if i != 0:
                fault_string += ", "
            if (fault_list[i] & COMMAND_EXEC_CRASH_FLAG) != 0:
                command_id = fault_list[i] - COMMAND_EXEC_CRASH_FLAG
                if len(COMMAND_DESCRIPTIONS) > command_id:
                    fault_string += "CMD_{}_ERROR".format(COMMAND_DESCRIPTIONS[command_id])
                else:
                    fault_string += "UNKNOWN_CMD_{}_ERR".format(command_id)
            elif fault_list[i] < len(ERROR_DESCRIPTIONS):
                fault_string += ERROR_DESCRIPTIONS[fault_list[i]]
            else:
                fault_string += "UNKNOWN_ERR_{0}".format(fault_list[i])
        return fault_string

    def run(self, stat):
        copro_connected = self._copro_connected.get_value()
        memory_usgae = self._copro_memory_usage.get_value()
        fault_list = self._copro_fault_list.get_value()

        if copro_connected is None:
            stat.summary(DiagnosticStatus.STALE, "No connection data from copro publisher")
        elif not copro_connected:
            stat.summary(DiagnosticStatus.ERROR, "Not Connected")
        elif memory_usgae is None or fault_list is None:
            stat.summary(DiagnosticStatus.STALE, "No data available from copro publisher")
        else:
            stat.add("Copro Connected", str(copro_connected))
            stat.add("Copro Memory Usage", "{:.2f}%".format(memory_usgae))
            stat.add("Copro Faults", self.genFaultList())

            if len(fault_list) > 0:
                stat.summary(DiagnosticStatus.ERROR, "Fault Code " + str(self.genFaultList()))
            elif memory_usgae > self._warning_percentage:
                stat.summary(DiagnosticStatus.WARN, "Copro Memory above {:d}%".format(self._warning_percentage))
            else:
                stat.summary(DiagnosticStatus.OK, "Copro Memory {:.2f}%".format(memory_usgae))

        return stat

class TemperatureTask(DiagnosticTask):
    def __init__(self, warning_temp_above):
        DiagnosticTask.__init__(self, "Temperature")

        self._warning_temp_above = int(warning_temp_above)

        self._temperature = ExpiringMessage(ROS_MESSAGE_LIFETIME)
        self._temp_threshold = ExpiringMessage(ROS_MESSAGE_LIFETIME)
        self._peltier_power = ExpiringMessage(ROS_MESSAGE_LIFETIME)

        rospy.Subscriber('state/temperature', Float32, self.temperature_callback, queue_size=1)
        rospy.Subscriber('state/peltier_power', Bool, self.peltier_power_callback, queue_size=1)
        rospy.Subscriber('state/temp_threshold', Int8, self.temp_threshold_callback, queue_size=1)

    def temperature_callback(self, msg):
        self._temperature.update_value(msg.data)
    
    def peltier_power_callback(self, msg):
        self._peltier_power.update_value(msg.data)
    
    def temp_threshold_callback(self, msg):
        self._temp_threshold.update_value(msg.data)

    def run(self, stat):
        temperature = self._temperature.get_value()
        temp_threshold = self._temp_threshold.get_value()
        peltier_power = self._peltier_power.get_value()

        if temp_threshold is None or peltier_power is None:
            stat.summary(DiagnosticStatus.STALE, "No data available from copro publisher")
        elif temperature is None:
            stat.add("Cooling Temperature Threshold", str(temp_threshold) + "C")
            stat.add("Peltier Powered", str(peltier_power))
            stat.summary(DiagnosticStatus.ERROR, "Unable to read temperature sensor")
        else:
            stat.add("Temperature", "{:.2f}C".format(temperature))
            stat.add("Cooling Temperature Threshold", str(temp_threshold) + "C")
            stat.add("Peltier Powered", str(peltier_power))

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

class DepthSensorTask(DiagnosticTask):
    def __init__(self):
        DiagnosticTask.__init__(self, "Depth Sensor")

        self._connectecd = ExpiringMessage(ROS_MESSAGE_LIFETIME)
        rospy.Subscriber('depth/connected', Bool, self.depth_connected_callback, queue_size=1)

    def depth_connected_callback(self, msg):
        self._connectecd.update_value(msg.data)

    def run(self, stat):
        connected = self._connectecd.get_value()

        if connected is None:
            stat.summary(DiagnosticStatus.STALE, "No data available from copro publisher")
        elif connected:
            stat.summary(DiagnosticStatus.OK, "Connected")
        else:
            stat.summary(DiagnosticStatus.ERROR, "Not Connected")

        return stat


class KillSwitchTask(DiagnosticTask):
    def __init__(self):
        DiagnosticTask.__init__(self, "Kill Switch")

        self._switch_engaged = ExpiringMessage(ROS_MESSAGE_LIFETIME)
        rospy.Subscriber('state/kill_switch', Bool, self.kill_switch_callback, queue_size=1)

    def kill_switch_callback(self, msg):
        self._switch_engaged.update_value(msg)

    def run(self, stat):
        switch_engaged = self._switch_engaged.get_value()

        if switch_engaged is None:
            stat.summary(DiagnosticStatus.STALE, "No data available from copro publisher")
        elif switch_engaged is True:
            stat.summary(DiagnosticStatus.OK, "Engaged")
        else:
            stat.summary(DiagnosticStatus.OK, "Disengaged")

        return stat


class ActuatorStatusTask(DiagnosticTask):
    def __init__(self):
        DiagnosticTask.__init__(self, "Actuators")

        self._actuator_fault = ExpiringMessage(ROS_MESSAGE_LIFETIME)
        self._actuator_connected = ExpiringMessage(ROS_MESSAGE_LIFETIME)

        rospy.Subscriber('state/actuator', Bool, self.actuator_state_callback, queue_size=1)
        rospy.Subscriber('state/actuator_fault', Bool, self.actuator_fault_callback, queue_size=1)

    def actuator_state_callback(self, msg):
        self._actuator_connected.update_value(msg.data)
    
    def actuator_fault_callback(self, msg):
        self._actuator_fault.update_value(msg.data)

    def run(self, stat):
        actuator_connected = self._actuator_connected.get_value()
        actuator_fault = self._actuator_fault.get_value()

        if actuator_connected is None or actuator_fault is None:
            stat.summary(DiagnosticStatus.STALE, "No data available from copro publisher")
        else:
            stat.add("Actuators Connected", str(actuator_connected))
            stat.add("Actuators in Fault", str(actuator_fault))
            if not actuator_connected:
                stat.summary(DiagnosticStatus.ERROR, "Not Connected")
            elif actuator_fault:
                stat.summary(DiagnosticStatus.ERROR, "Actuators in fault state")
            else:
                stat.summary(DiagnosticStatus.OK, "OK")

        return stat

def main():
    hostname = socket.gethostname()
    rospy.init_node("electrical_monitor")

    current_robot = rospy.get_param('~current_robot')

    updater = Updater()
    updater.setHardwareID(hostname)

    with open(rospy.get_param('~diag_thresholds_file'), 'r') as stream:
        thresholds = yaml.safe_load(stream)["device_thresholds"]
        updater.add(TemperatureTask(thresholds["temp_over_target_warn"]))
        updater.add(CoprocessorStatusTask(thresholds["copro_mem_warn_percentage"]))
        updater.add(DepthSensorTask())
        updater.add(KillSwitchTask())
        if current_robot == TITAN_ROBOT:
            updater.add(ActuatorStatusTask())

    rate = rospy.get_param("~rate", 1)
    rospy.Timer(rospy.Duration(1 / rate), lambda _: updater.update())

    rospy.spin()


if __name__ == '__main__':
    main()