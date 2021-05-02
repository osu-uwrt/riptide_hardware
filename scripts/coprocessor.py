#!/usr/bin/env python3

"""
Implemented ROS Nodes:

Subscribers:
'control/temp_threshold' (Int8): Sets the temperature threshold (in Deg C) before the peltier turns on
'control/copro_reset' (Bool): Resets the Copro MCU when published with True
'control/actuator_reset' (Bool): Resets the Actuator MCU when published with True
'command/pwm' (Int16MultiArray): 8 PWM Values (in microseconds) for each of the thrusters
'command/light1' (Int8): Sets the light 1 brightness (In Percent)
'command/light2' (Int8): Sets the light 2 brightness (In Percent)
'command/drop' (Int8): Drops the specified marker (0 or 1)
'command/arm' (Bool): Arms/Disarms the Torpedo System
'command/fire' (Int8): Fires the specific torpedo (0 or 1)
'command/grabber' (Int8): Closes (+ Number), Opens (- Number), or Stops Grabber (0)

Publishers:
'depth/raw' (Depth): The raw reading from the depth sensor
'depth/connected' (Bool): If the depth sensor is connected
'state/battery_voltage' (Float32MultiArray): The voltage of port and starboard batteries
'state/voltage_balanced' (Float32): The voltage of the balanced battery (V+) rail
'state/battery_current' (Float32MultiArray): The current of port and starboard batteries
'state/temperature' (Float32): The temperature of the robot electronics
'state/switches' (SwitchState): The readings of the switches (only kill switch and sw1 implemented)
'state/thruster_currents' (Float32MultiArray): The currents of the thrusters
'state/copro_memory_usage' (Float32): The memory usage of the copro (decimal out of 1)
'state/temp_threshold' (Int8): The temperature threshold (in Deg C) before the peltier turns on
'state/peltier_power' (Bool): If the peltier is powered on
'state/copro_fault' (UInt8MultiArray): A list of fault codes from the copro
'state/voltage_12' (Float32): The voltage of the 12V rail
'state/voltage_5' (Float32): The voltage of the 5V rail
'state/actuator' (Bool): If the actuator board is connected
'state/actuator_fault' (Bool): If the actuator is in a fault state
'state/copro' (Bool): If the copro is connected to the robot
"""

import errno
import rospy
import socket
import select
import time
import traceback
from threading import Thread, Lock
from collections import deque
from std_msgs.msg import String, Header, Bool, Float32MultiArray, Int8, UInt16, Float32, Int16MultiArray, UInt8MultiArray
from riptide_msgs.msg import Depth, PwmStamped, StatusLight, SwitchState
from riptide_hardware.cfg import CoprocessorDriverConfig
from dynamic_reconfigure.server import Server
import yaml

# Robot Types
PUDDLES_ROBOT = "puddles"
TITAN_ROBOT = "titan"

# Copro Commands
CONN_HELLO_MESSAGE = "\010UWRT_Hi"
CONN_LATENCY_NEW_WEIGHT = 0.02

MOBO_POWER_CMD = 0
LIGHTING_POWER_CMD = 1      # Implemented - Note: Controls jetson power on Puddles Copro
THRUSTER_ENABLE_CMD = 2
PELTIER_POWER_CMD = 3       # Implemented
BATTERY_VOLTAGE_CMD = 4     # Implemented
BATTERY_CURRENT_CMD = 5     # Implemented
TEMPERATURE_CMD = 6         # Implemented
THRUSTER_FORCE_CMD = 7      # Implemented
LOGIC_CURRENT_CMD = 8           # Note: Not implemented on Titan Copro
LOGIC_VOLTAGE_CMD = 9       # Implemented
GET_SWITCHES_CMD = 10       # Implemented
GET_DEPTH_CMD = 11          # Implemented
THRUSTER_CURRENT_CMD = 12   # Implemented
TWELVE_VOLT_POWER_CMD = 13
FIVE_VOLT_RESET_CMD = 14
COPRO_RESET_CMD = 15        # Implemented
ACTUATOR_CMD = 16           # Implemented
PING_COPRO_CMD = 17         # Implemented
MEMORY_CHECK_CMD = 18       # Implemented
TEMP_THRESHOLD_CMD = 19     # Implemented
GET_FAULT_STATE_CMD = 20    # Implemented

# Actuator Commands
ACTUATOR_SET_TORPEDO_TIMING_CMD = 0     # Implemented
ACTUATOR_RESET_BOARD_CMD = 1            # Implemented
ACTUATOR_GET_FAULT_STATUS_CMD = 2       # Implemented - Note: Only implemented on titan
ACTUATOR_GET_TORPEDO_STATUS_CMD = 3
ACTUATOR_ARM_TORPEDO_CMD = 4            # Implemented
ACUTATOR_FIRE_TORPEDO_CMD = 5           # Implemented
ACTUATOR_RELEASE_MARKER_CMD = 6         # Implemented
ACTUATOR_SET_GRIPPER_PWM_CMD = 7        # Implemented

# Actuator command constant
ACTUATOR_TRY_FAIL = 0xFF

class BaseCoproCommand:
    """List of non critical tasks to monitor"""
    tasks = None

    """List of critical tasks to monitor.
    If any of these tasks stop, the code will restart
    """
    critical_tasks = None

    def __init__(self, driver, command_id):
        """Initializes the Command and registers it with the CoproDriver

        Args:
            driver (CoproDriver): The instance of CoproDriver to register the command with
        """
        self._command_id = command_id
        self.critical_tasks = []
        self.tasks = []

        self.driver = driver
        self.driver.registerCommand(self)

    def getCommandId(self):
        """The command id for the class

        Returns:
            int: The id of the command
        """
        return self._command_id

    def commandCallback(self, response, extra_data):
        """The callback for after a command has been executed

        Args:
            response (list): An integer list of the response from the copro
        """
        raise NotImplementedError()


def toBytes(num):
    """Converts 16-bit integer to 8-bit integer array

    Args:
        num (int): A 16-bit integer

    Returns:
        list: An 8-bit integer list
    """
    return [num // 256, num % 256]


class PwmCommand(BaseCoproCommand):
    def __init__(self, driver):
        BaseCoproCommand.__init__(self, driver, THRUSTER_FORCE_CMD)
        rospy.Subscriber('command/pwm', Int16MultiArray, self.pwm_callback, queue_size=1)

    def pwm_callback(self, pwm_message):
        args = []
        args += toBytes(pwm_message.data[0])
        args += toBytes(pwm_message.data[1])
        args += toBytes(pwm_message.data[2])
        args += toBytes(pwm_message.data[3])
        args += toBytes(pwm_message.data[4])
        args += toBytes(pwm_message.data[5])
        args += toBytes(pwm_message.data[6])
        args += toBytes(pwm_message.data[7])
        self.driver.enqueueCommand(THRUSTER_FORCE_CMD, args)

    def stopThrustersCommand(self):
        # This the command that will put the thrusters into a stopped state
        return bytearray([18, 7, 5, 220, 5, 220, 5, 220, 5, 220, 5, 220, 5, 220, 5, 220, 5, 220])

    def commandCallback(self, response, extra_data):
        if len(response) != 1:
            rospy.logerr("Invalid thruster force response of length %d", len(response))
        elif response[0] == 0:
            rospy.logwarn("Thruster command failed to run!")
        elif response[0] == 1:
            pass  # Thrusters command successfully executed
        else:
            rospy.logerr("Invalid thruster force response of val %d", response[0])


class DepthCommand(BaseCoproCommand):
    def __init__(self, driver):
        BaseCoproCommand.__init__(self, driver, GET_DEPTH_CMD)
        self.depth_pub = rospy.Publisher('depth/raw', Depth, queue_size=1)
        self.depth_connected_pub = rospy.Publisher('depth/connected', Bool, queue_size=1)
        self.depth_connected_pub.publish(False)

        with open(rospy.get_param('vehicle_file'), 'r') as stream:
            self.depthVariance = yaml.safe_load(stream)['depth']['sigma'] ** 2
        
        self.critical_tasks.append(rospy.Timer(rospy.Duration(0.05), self.depth_callback))

    def depth_callback(self, event):
        if self.depth_pub.get_num_connections() > 0 or self.depth_connected_pub.get_num_connections() > 0:
            self.driver.enqueueCommand(GET_DEPTH_CMD)
    
    def commandCallback(self, response, extra_data):
        if len(response) != 4:
            rospy.logerr("Improper depth response: " + str(response))
        else:
            if response[0] == 1:
                depth = (response[1] << 16) + (response[2] << 8) + response[3]
                if response[1] & 0x80 != 0:
                    depth = -1 * ((1<<24) - depth)
                depth = depth / 100000.0
                depth_msg = Depth()
                depth_msg.header.stamp = rospy.Time.now()
                depth_msg.header.frame_id = rospy.get_namespace()[1:]+"pressure_link"
                depth_msg.depth = -depth
                depth_msg.variance = self.depthVariance

                self.depth_connected_pub.publish(True)
                self.depth_pub.publish(depth_msg)
            elif response[0] == 0:
                self.depth_connected_pub.publish(False)
            else:
                rospy.logerr("Improper depth response: " + str(response))


class BatteryVoltageCommand(BaseCoproCommand):
    def __init__(self, driver):
        BaseCoproCommand.__init__(self, driver, BATTERY_VOLTAGE_CMD)
        self.batVoltage_pub = rospy.Publisher('state/battery_voltage', Float32MultiArray, queue_size=1)
        self.balanced_voltage_pub = rospy.Publisher('state/voltage_balanced', Float32, queue_size=1)

        self.tasks.append(rospy.Timer(rospy.Duration(1), self.battery_voltage_callback))

    def battery_voltage_callback(self, event):   
        if self.batVoltage_pub.get_num_connections() > 0 or self.balanced_voltage_pub.get_num_connections() > 0:
            self.driver.enqueueCommand(BATTERY_VOLTAGE_CMD)
    
    def commandCallback(self, response, extra_data):
        if len(response) == 1 and response[0] == 0:
            rospy.logwarn_once("Unable to Read Battery Voltage")
        elif len(response) != 7 or response[0] != 1:
            rospy.logerr("Improper battery voltage response: " + str(response))
        else: 
            port_volt = (response[1] *256.0 + response[2])/100.0
            stbd_volt = (response[3] *256.0 + response[4])/100.0
            balanced_volt = (response[5] *256.0 + response[6])/100.0
            self.batVoltage_pub.publish(Float32MultiArray(data = [port_volt, stbd_volt]))
            self.balanced_voltage_pub.publish(balanced_volt)


class BatteryCurrentCommand(BaseCoproCommand):
    def __init__(self, driver):
        BaseCoproCommand.__init__(self, driver, BATTERY_CURRENT_CMD)
        self.batCurrent_pub = rospy.Publisher('state/battery_current', Float32MultiArray, queue_size=1)
        self.tasks.append(rospy.Timer(rospy.Duration(1), self.battery_current_callback))

    def battery_current_callback(self, event):   
        if self.batCurrent_pub.get_num_connections() > 0:
            self.driver.enqueueCommand(BATTERY_CURRENT_CMD)

    def commandCallback(self, response, extra_data):
        if len(response) == 1 and response[0] == 0:
            rospy.logwarn_once("Unable to Read Battery Currents")
        elif len(response) != 5 or response[0] != 1:
            rospy.logerr("Improper battery current response: " + str(response))
        else:
            port_current = (response[1] *256.0 + response[2])/100.0
            stbd_current = (response[3] *256.0 + response[3])/100.0
            self.batCurrent_pub.publish(Float32MultiArray(data = [port_current, stbd_current]))


class TemperatureCommand(BaseCoproCommand):
    def __init__(self, driver):
        BaseCoproCommand.__init__(self, driver, TEMPERATURE_CMD)
        self.temperature_pub = rospy.Publisher('state/temperature', Float32, queue_size=1)
        self.tasks.append(rospy.Timer(rospy.Duration(0.5), self.temperature_callback))

    def temperature_callback(self, event):   
        if self.temperature_pub.get_num_connections() > 0:
            self.driver.enqueueCommand(TEMPERATURE_CMD)
    
    def commandCallback(self, response, extra_data):
        if len(response) == 1 and response[0] == 0:
            rospy.logwarn_once("Unable to Robot Temperature")
        elif len(response) != 3 or response[0] != 1:
            rospy.logerr("Improper battery current response: " + str(response))
        else:
            temperature = (response[1]*256.0 + response[2])/10.0
            self.temperature_pub.publish(Float32(temperature))


class SwitchCommand(BaseCoproCommand):
    def __init__(self, driver):
        BaseCoproCommand.__init__(self, driver, GET_SWITCHES_CMD)
        self.last_kill_switch_state = False
        self.switch_pub = rospy.Publisher('state/switches', SwitchState, queue_size=1)
        self.critical_tasks.append(rospy.Timer(rospy.Duration(0.2), self.switch_callback))

    def switch_callback(self, event):
        if self.switch_pub.get_num_connections() > 0:
            self.driver.enqueueCommand(GET_SWITCHES_CMD)
    
    def commandCallback(self, response, extra_data):
        if len(response) != 1:
            rospy.logerr("Improper switches response: " + str(response))
        else:
            switch_msg = SwitchState()
            switch_msg.header.stamp = rospy.Time.now()
            switch_msg.kill = bool(response[0] & (1 << 0))
            switch_msg.sw1 = bool(response[0] & (1 << 1))

            if self.last_kill_switch_state is not switch_msg.kill:
                if switch_msg.kill:
                    rospy.loginfo("Kill switch engaged")
                else:
                    rospy.loginfo("Kill switch disengaged")
                self.last_kill_switch_state = switch_msg.kill

            self.switch_pub.publish(switch_msg)


class ThrusterCurrentCommand(BaseCoproCommand):
    def __init__(self, driver):
        BaseCoproCommand.__init__(self, driver, THRUSTER_CURRENT_CMD)
        self.thruster_current_pub = rospy.Publisher('state/thruster_currents', Float32MultiArray, queue_size=1)
        self.tasks.append(rospy.Timer(rospy.Duration(0.2), self.thruster_current_callback))

    def thruster_current_callback(self, event):
        if self.thruster_current_pub.get_num_connections() > 0:
            self.driver.enqueueCommand(THRUSTER_CURRENT_CMD)

    def commandCallback(self, response, extra_data):
        if len(response) == 1 and response[0] == 0:
            rospy.logwarn_once("Unable to Read Thruster Currents")
        elif len(response) != 9 or response[0] != 1:
            rospy.logerr("Improper thruster current response: " + str(response))
        else:
            current_msg = Float32MultiArray(data = [x/25.0 for x in response[1:]])
            self.thruster_current_pub.publish(current_msg)


class CoproMemoryCommand(BaseCoproCommand):
    def __init__(self, driver):
        BaseCoproCommand.__init__(self, driver, MEMORY_CHECK_CMD)
        self.memory_pub = rospy.Publisher('state/copro_memory_usage', Float32, queue_size=1)
        self.tasks.append(rospy.Timer(rospy.Duration(1.0), self.memory_callback))

    def memory_callback(self, event):
        if self.memory_pub.get_num_connections() > 0:
            self.driver.enqueueCommand(MEMORY_CHECK_CMD)
    
    def commandCallback(self, response, extra_data):
        if len(response) != 2:
            rospy.logerr("Improper memory usage response: " + str(response))
        else:
            memory = (response[0]*256.0 + response[1])/(256*256-1)
            self.memory_pub.publish(Float32(memory))


class TempThresholdCommand(BaseCoproCommand):
    def __init__(self, driver):
        BaseCoproCommand.__init__(self, driver, TEMP_THRESHOLD_CMD)
        self.temp_threshold_pub = rospy.Publisher('state/temp_threshold', Int8, queue_size=1)
        self.tasks.append(rospy.Timer(rospy.Duration(1), self.temp_threshold_callback))

        rospy.Subscriber('control/temp_threshold', Int8, self.set_temp_threshold_callback)

    def temp_threshold_callback(self, event):
        if self.temp_threshold_pub.get_num_connections() > 0:
            self.driver.enqueueCommand(TEMP_THRESHOLD_CMD)

    def set_temp_threshold_callback(self, msg):
        self.driver.enqueueCommand(TEMP_THRESHOLD_CMD, [msg.data])

    def commandCallback(self, response, extra_data):
        if len(response) != 1:
            rospy.logerr("Improper temp threshold response: " + str(response))
        else:
            temp_threshold = int(response[0])
            self.temp_threshold_pub.publish(temp_threshold)


class LightingCommand(BaseCoproCommand):
    def __init__(self, driver):
        BaseCoproCommand.__init__(self, driver, LIGHTING_POWER_CMD)

        assert self.driver.current_robot == TITAN_ROBOT

        rospy.Subscriber('command/light1', Int8, self.light1_callback)
        rospy.Subscriber('command/light2', Int8, self.light2_callback)

    def light1_callback(self, msg):
        if msg.data < 0 or msg.data > 100:
            rospy.logerr("Invalid range for lighting request: %d", msg.data)
        else:
            lighting_args = [1, msg.data]
            self.driver.enqueueCommand(LIGHTING_POWER_CMD, args=lighting_args)

    def light2_callback(self, msg):
        if msg.data < 0 or msg.data > 100:
            rospy.logerr("Invalid range for lighting request: %d", msg.data)
        else:
            lighting_args = [2, msg.data]
            self.driver.enqueueCommand(LIGHTING_POWER_CMD, args=lighting_args)
    
    def commandCallback(self, response, extra_data):
        if len(response) != 1:
            rospy.logerr("Improper lighting response: " + str(response))
        elif response[0] == 0:
            rospy.logwarn("Failed to set lighting")
        elif response[0] == 1:
            pass
        else:
            rospy.logerr("Improper lighting response: " + str(response))


class PeltierCommand(BaseCoproCommand):
    def __init__(self, driver):
        BaseCoproCommand.__init__(self, driver, PELTIER_POWER_CMD)

        self.peltier_power_pub = rospy.Publisher('state/peltier_power', Bool, queue_size=1)
        self.tasks.append(rospy.Timer(rospy.Duration(1), self.peltier_callback))

    def peltier_callback(self, event):
        if self.peltier_power_pub.get_num_connections() > 0:
            self.driver.enqueueCommand(PELTIER_POWER_CMD)
    
    def commandCallback(self, response, extra_data):
        if len(response) != 1 or (response[0] != 0 and response[0] != 1):
            rospy.logerr("Improper peltier response: " + str(response))
        else:
            self.peltier_power_pub.publish(bool(response[0]))


class CoproFaultCommand(BaseCoproCommand):
    def __init__(self, driver):
        BaseCoproCommand.__init__(self, driver, GET_FAULT_STATE_CMD)

        self.copro_fault_pub = rospy.Publisher('state/copro_fault', UInt8MultiArray, queue_size=1)
        self.tasks.append(rospy.Timer(rospy.Duration(1), self.copro_fault_callback))
    
    def copro_fault_callback(self, event):
        if self.copro_fault_pub.get_num_connections() > 0:
            self.driver.enqueueCommand(GET_FAULT_STATE_CMD)

    def commandCallback(self, response, extra_data):
        if len(response) == 0:
            rospy.logerr("Empty copro status response")
        elif response[0] != 1 and response[0] != 0:
            rospy.logerr("Invalid copro status %d", response[0])
        else:
            if response[0] == 1:
                rospy.logwarn_once("Copro entered fault state")
            self.copro_fault_pub.publish(UInt8MultiArray(data=response[1:]))


class LogicVoltageCommand(BaseCoproCommand):
    def __init__(self, driver):
        BaseCoproCommand.__init__(self, driver, LOGIC_VOLTAGE_CMD)

        self.logic_12v_pub = rospy.Publisher('state/voltage_12', Float32, queue_size=1)
        self.logic_5v_pub = rospy.Publisher('state/voltage_5', Float32, queue_size=1)
        self.tasks.append(rospy.Timer(rospy.Duration(1), self.logic_voltage_callback))

    def logic_voltage_callback(self, event):
        if self.logic_12v_pub.get_num_connections() > 0 or self.logic_5v_pub.get_num_connections() > 0:
            self.driver.enqueueCommand(LOGIC_VOLTAGE_CMD)
    
    def commandCallback(self, response, extra_data):
        if len(response) == 1 and response[0] == 0:
            rospy.logwarn_once("Unable to Read Logic Voltage")
        elif len(response) != 7 or response[0] != 1:
            rospy.logerr("Invalid Logic Voltage Response: " + str(response))
        else:
            five_volt_value = ((response[3] * 256) + response[4]) / 1000.0
            twelve_volt_value = ((response[5] * 256) + response[6]) / 500.0

            self.logic_5v_pub.publish(five_volt_value)
            self.logic_12v_pub.publish(twelve_volt_value)


class ResetCommand(BaseCoproCommand):
    def __init__(self, driver):
        BaseCoproCommand.__init__(self, driver, COPRO_RESET_CMD)

        rospy.Subscriber('control/copro_reset', Bool, self.reset_callback)

    def reset_callback(self, msg):
        if msg.data:
            self.driver.enqueueCommand(COPRO_RESET_CMD, is_reset=True)

class ActuatorCommand(BaseCoproCommand):
    lastConfig = None

    def __init__(self, driver):
        BaseCoproCommand.__init__(self, driver, ACTUATOR_CMD)

        rospy.Subscriber('command/drop', Int8, self.drop_callback)
        rospy.Subscriber('command/arm', Bool, self.arm_callback)
        rospy.Subscriber('command/fire', Int8, self.fire_callback)
        rospy.Subscriber('command/grabber', Int8, self.grab_callback)
        rospy.Subscriber('control/actuator_reset', Bool, self.reset_callback)

        if self.driver.current_robot == TITAN_ROBOT:
            self.actuator_connection_pub = rospy.Publisher('state/actuator', Bool, queue_size=1)
            self.actuator_connection_pub.publish(False)
            self.actuator_fault_pub = rospy.Publisher('state/actuator_fault', Bool, queue_size=1)

            self.tasks.append(rospy.Timer(rospy.Duration(1), self.status_callback))
        else:
            self.actuator_connection_pub = None
            self.actuator_fault_pub = None

        if self.driver.current_robot == TITAN_ROBOT:
            Server(CoprocessorDriverConfig, self.reconfigure_callback)

    def reconfigure_callback(self, config, level):
        self.lastConfig = config
        for i in range(5):
            start = config["coil_%d_start1" % (i+1)]
            end = config["coil_%d_end1" % (i+1)]
            self.enqueueActuatorCommand(ACTUATOR_SET_TORPEDO_TIMING_CMD, i, [start // 256, start % 256, end // 256, end % 256])

            start = config["coil_%d_start2" % (i+1)]
            end = config["coil_%d_end2" % (i+1)]
            self.enqueueActuatorCommand(ACTUATOR_SET_TORPEDO_TIMING_CMD, i+16, [start // 256, start % 256, end // 256, end % 256])

        self.enqueueActuatorCommand(ACTUATOR_SET_TORPEDO_TIMING_CMD, 5, [0, 0, config["standby"] // 256, config["standby"] % 256])
        self.enqueueActuatorCommand(ACTUATOR_SET_TORPEDO_TIMING_CMD, 21, [0, 0, config["standby"] // 256, config["standby"] % 256])

        return config

    def enqueueActuatorCommand(self, command, lower_bit_args = 0, remaining_args = []):
        assert lower_bit_args < (1<<5)
        assert command < (1<<3)
        
        self.driver.enqueueCommand(ACTUATOR_CMD, [(command << 5) + lower_bit_args] + remaining_args, extra_data=command)

    def reset_callback(self, msg):
        if msg.data:
            self.enqueueActuatorCommand(ACTUATOR_RESET_BOARD_CMD)

    def arm_callback(self, msg):
        if msg.data:
            self.enqueueActuatorCommand(ACTUATOR_ARM_TORPEDO_CMD, 1<<4)
        else:
            self.enqueueActuatorCommand(ACTUATOR_ARM_TORPEDO_CMD, 0)

    def fire_callback(self, msg):
        if msg.data == 0:
            self.enqueueActuatorCommand(ACUTATOR_FIRE_TORPEDO_CMD, 0)
        else:
            self.enqueueActuatorCommand(ACUTATOR_FIRE_TORPEDO_CMD, 1<<4)

    def grab_callback(self, msg):
        if msg.data == 0:
            self.enqueueActuatorCommand(ACTUATOR_SET_GRIPPER_PWM_CMD, 0, toBytes(1500))
        elif msg.data > 0:
            self.enqueueActuatorCommand(ACTUATOR_SET_GRIPPER_PWM_CMD, 0, toBytes(1200))
        else:
            self.enqueueActuatorCommand(ACTUATOR_SET_GRIPPER_PWM_CMD, 0, toBytes(1800))

    def drop_callback(self, msg):
        if msg.data == 0:
            self.enqueueActuatorCommand(ACTUATOR_RELEASE_MARKER_CMD, 0)
        else:
            self.enqueueActuatorCommand(ACTUATOR_RELEASE_MARKER_CMD, 1<<4)

    def status_callback(self, event):
        if self.actuator_connection_pub.get_num_connections() > 0 or self.actuator_fault_pub.get_num_connections() > 0:
            self.enqueueActuatorCommand(ACTUATOR_GET_FAULT_STATUS_CMD)

    def commandCallback(self, response, extra_data):
        command = extra_data

        if self.driver.current_robot == TITAN_ROBOT:
            if len(response) != 2:
                rospy.logerr("Invalid response for actuator command %d: " + str(response), command)
                return
            if response[1] == ACTUATOR_TRY_FAIL:
                if command != ACTUATOR_GET_FAULT_STATUS_CMD and command != ACTUATOR_RESET_BOARD_CMD:
                    # Only warn if code is trying to use the actuators
                    rospy.logwarn_throttle(60, "Unable to contact actuator board")
                self.actuator_connection_pub.publish(False)
            else:
                if response[1] != 1:
                    rospy.logwarn_throttle(60, "Multiple tries required to contact actuator board (check i2c connections)")
                self.actuator_connection_pub.publish(True)
        else:
            if len(response) != 1:
                rospy.logerr("Invalid response for actuator command %d: " + str(response), command)
                return

        if command != ACTUATOR_GET_TORPEDO_STATUS_CMD and command != ACTUATOR_GET_FAULT_STATUS_CMD and command != ACTUATOR_RESET_BOARD_CMD:
            if response[0] == 0:
                rospy.logwarn("Failed to execute actuator command %d", command)
            elif response[0] != 1:
                rospy.logerr("Invalid response for actuator command %d: %d", command, response[0])
        elif command == ACTUATOR_GET_FAULT_STATUS_CMD:
            if response[0] != 0 and response[0] != 1:
                rospy.logerr("Invalid response for actuator command %d: %d", command, response[0])
            else:
                self.actuator_fault_pub.publish(bool(response[0]))


class PingCommand(BaseCoproCommand):
    def __init__(self, driver):
        BaseCoproCommand.__init__(self, driver, PING_COPRO_CMD)

        # Nothing needs to be published or subscribed to. Just to make sure the connection is still alive

        self.last_ping = time.time()
        self.critical_tasks.append(rospy.Timer(rospy.Duration(0.4), self.ping_callback))

    def ping_callback(self, event):
        self.driver.enqueueCommand(PING_COPRO_CMD)
    
    def manual_ping(self):
        self.last_ping = time.time()

    def commandCallback(self, response, extra_data):
        if len(response) != 1 and response[0] != 1:
            rospy.logerr("Invalid ping response: " + str(response))
        else:
            self.last_ping = time.time()


class CoproDriver:
    # The general timeout used for connecting and for connection stalls
    TIMEOUT = 2.0

    # The IP address to use to connect
    IP_ADDR = None

    # The ros publishers for the copro connection state
    connection_pub = None
    connection_latency_pub = None
    connection_tx_queue_len_pub = None
    connection_rx_pending_queue_len_pub = None

    # The timer scheduling the copro communication task
    copro_comm_timer = None

    # The config parameter passed with the current robot
    current_robot = None

    # Commands with specific features used directly in the copro driver
    actuatorCommander = None     # The instance of ActuatorCommand (used for actuator callback configuration)
    pingCommander = None         # The instance of PingCommand (used for timeouts)
    pwmCommander = None          # The instance of PwmCommand (used for getting the stop thruster command during disconnect)

    def __init__(self):
        """Initializes new instance of CoproDriver class
        """

        # The registered commands with the command id as the key and BaseCoproCommand implementation as value
        # Should only be written by registerCommand and deregisterCommand
        self.registered_commands = {}

        ########################################
        ### Note: These variables are safe to be used by all class methods, including enqueueCommand

        # Controls if enqueueCommand can enqueue data to the command_queue
        self.command_queueing_permitted = False

        # The command queue for communicating with copro
        # This contains a full command_data packet:
        # [packet_data (bytearray), command_id (None or int), extra_data (Any), sent_time (float)]
        self.command_queue = deque([], 50)
        ########################################


        ########################################
        ### Note: These variables should only be interacted with by class methods, but not by enqueueCommand

        # The socket connection to copro
        self.copro = None

        # The buffer of data received from copro, keeps track of partially received messages
        self.buffer = []

        # If the copro is currently connected
        self.connected = False

        # Used to print Connecting to Copro only once for each connect attempt
        self.connecting_msg_sent = False        

        # If the copro communication task should shut down
        self.comm_should_shutdown = False

        # Connection latency (ms)
        self.connection_latency = -1

        # The response queue for holding data on packets being processed by the copro
        # This contains a partial command_data packet:
        # [command_id (None or int), extra_data (Any), sent_time (float)]
        self.response_queue = deque([], 50)
        ########################################


    ########################################
    # Connection Management Code           #
    ########################################

    def tryConnect(self):
        try:
            # Try Connect
            self.copro = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.copro.settimeout(self.TIMEOUT)
            self.copro.connect((self.IP_ADDR, 50000))

            # Exchange Hello
            self.copro.settimeout(10)
            self.copro.sendall(CONN_HELLO_MESSAGE)
            data = self.copro.recv(8)
            assert data == CONN_HELLO_MESSAGE

            self.copro.settimeout(self.TIMEOUT)

            self.connected = True
        except:
            # Close down the connection properly for all states of connect
            self.closeConnection()

    def closeConnection(self):
        """Closes the connection to the copro
        Should only be called from copro comm task

        """

        # If the socket is still active, clean it up
        if self.copro is not None:
            try:
                if self.connected:
                    # Stop thrusters
                    rospy.loginfo("Stopping thrusters")
                    self.copro.setblocking(False)
                    try:
                        # Try a clean disconnect
                        self.copro.sendall(self.pwmCommander.stopThrustersCommand())
                        self.copro.sendall(bytearray([0]))
                        self.copro.shutdown(socket.SHUT_RDWR)
                    except:
                        pass
                self.copro.close()
            except Exception as e:
                traceback.print_exc()
                rospy.logerr("Error during connection close: " + str(e))
        
        # Reset connection instance variables
        self.copro = None
        self.connected = False
        self.connection_latency = -1
        self.buffer = []

        # Dump contents of the command queues
        self.command_queueing_permitted = False
        self.command_queue.clear()
        self.response_queue.clear()

    def doCoproCommunication(self):
        """Sends any pending commands to copro and processes any data returned
        """
        # Publish the number of commands in queue before they are handled
        self.connection_tx_queue_len_pub.publish(len(self.command_queue))

        readable, writable, exceptional = select.select([self.copro], [self.copro], [self.copro], 0)

        # Handle outgoing packets if there is room in send buffer
        if len(writable) > 0:
            if len(self.command_queue) > 0:
                command = []
                while len(self.command_queue) != 0:
                    # Make sure there is enough space in the response queue to send another command
                    if len(self.response_queue) >= self.response_queue.maxlen:
                        break
                    
                    # Get the data packet to send from the command data
                    command_data = self.command_queue.popleft()
                    command += command_data.pop(0)

                    # If response is expected, queue the cut command data
                    if command_data[0] is not None:
                        self.response_queue.append(command_data)
                
                self.copro.sendall(bytearray(command))

        # Handle incoming packets if there is data in receive buffer
        if len(readable) > 0:
            # Receive all data until the connection blocks
            try:
                self.copro.setblocking(False)
                while True:
                    recv_data = self.copro.recv(64)

                    # Length of zero from recv means EOF, so socket was shutdown
                    if len(recv_data) == 0:
                        self.closeConnection()
                        break

                    # If this is running in python 2, it will return a string rather than bytes, so convert to int list
                    if not isinstance(recv_data[0], int):
                        recv_data = list(map(ord, recv_data))
                    self.buffer += recv_data
            except socket.error:
                pass
            
            # Continue processing buffer until the buffer doesn't containe the
            # whole the length (first byte) of the packet or the buffer is empty
            while len(self.buffer) > 0 and self.buffer[0] <= len(self.buffer):
                # Decode packet - The first byte is length, remaining is data
                response = self.buffer[1:self.buffer[0]]

                # Retrieve the command id and time of request of this message
                command_data = self.response_queue.popleft()
                command = command_data[0]
                command_latency = int((time.time() - command_data[2]) * 1000)

                # Recalculate connection latency with a weighted average
                if self.connection_latency == -1:
                    self.connection_latency = command_latency
                else:
                    self.connection_latency = (self.connection_latency * (1-CONN_LATENCY_NEW_WEIGHT)) + (command_latency * CONN_LATENCY_NEW_WEIGHT)

                # The copro connection will return an empty packet if it failed to execute the command
                # If this is the case, don't execute the callback, and instead log error
                if len(response) == 0:
                    rospy.logwarn("Command error on command "+str(command))
                
                # Execute command if callback available for it
                elif command in self.registered_commands:
                    self.registered_commands[command].commandCallback(response, command_data[1])
                else:
                    rospy.logerr("Unhandled Command Response from Copro: %d", command)

                # Remove the processed command from the receive buffer
                self.buffer = self.buffer[self.buffer[0]:]
        
        # Publish the connection latency and rx pending queue length
        self.connection_rx_pending_queue_len_pub.publish(len(self.response_queue))
        if self.connection_latency != -1:
            self.connection_latency_pub.publish(self.connection_latency)

        # Check for stalls using ping command data
        if time.time() - self.pingCommander.last_ping > self.TIMEOUT:
            rospy.logwarn("Copro Connection Timed Out... Dropping Connection")
            self.closeConnection()


    ########################################
    # Public Methods                       #
    ########################################

    def enqueueCommand(self, command, args = [], is_reset = False, extra_data = None):
        """Adds command to queue to be sent to copro

        Args:
            command (int): The command ID to be sent
            args (list, optional): Int list of arguments to provide with command. Defaults to [].
            is_reset (bool): True if command is for a reset command so a response won't be expected
            extra_data (Any): Any extra data to be passed to the callback function

        Returns:
            bool: If the command was successfully queued
        """
            
        # Command Pakcet Format
        # Byte 0: Length (Including length byte)
        # Byte 1: Command ID
        # Byte 2-n: Args
        data = [command] + args
        data = [len(data) + 1] + data

        # Make sure that there is enough space in the queue for the command
        if len(self.command_queue) < self.command_queue.maxlen:
            response_command = command
            if is_reset:
                # Resposne isn't expected during reset command
                response_command = None
            
            # Check to make sure command queueing is permitted
            if not self.command_queueing_permitted:
                return False
            
            self.command_queue.append([data, response_command, extra_data, time.time()])
            return True
        else:
            rospy.logwarn_throttle(5, "Copro commands dropped: Command Buffer Full")
            return False

    def registerCommand(self, receiver):
        """Registers a receiver to process a command

        Args:
            receiver (BaseCoproCommand): Instance of BaseCoproCommand that will be called with the specific command id

        Raises:
            RuntimeError: Raised if the command id is already registered
        """
        if receiver.getCommandId() in self.registered_commands:
            raise RuntimeError("Command already registered")
        self.registered_commands[receiver.getCommandId()] = receiver

    def deregisterCommand(self, receiver):
        """Removes the receiver from processing commands

        Args:
            receiver (BaseCoproCommand): Instance of BaseCoproCommand to be deregistered

        Raises:
            KeyError: Raised if the specific command id isn't registered
            ValueError: Raised if the registered receiver is not the receiver passed to the function
        """
        if receiver.getCommandId() not in self.registered_commands:
            raise KeyError("Command not registered")
        if self.registered_commands[receiver.getCommandId()] != receiver:
            raise ValueError("The requested command to deregister is not currently registered")
        del self.registered_commands[receiver.getCommandId()]



    ########################################
    # Copro Communication Task             #
    ########################################

    def coproCommTask(self, event):
        """Manages the connection with the copro
        Should only be called from Timer callback

        Args:
            event (rospy.TimerEvent): The timer event for this call
        """
        if self.comm_should_shutdown:
            self.closeConnection()
            self.copro_comm_timer.shutdown()
            return

        if self.connected:
            self.connection_pub.publish(True)
            try:
                self.doCoproCommunication()
            except Exception as e:
                traceback.print_exc()
                rospy.logerr("Exception Occurred: " + str(e))
                self.closeConnection()
        else:
            self.connection_pub.publish(False)

            if not self.connecting_msg_sent:
                rospy.loginfo("Connecting to copro...")
                self.connecting_msg_sent = True

            self.tryConnect()

            if self.connected:
                self.connecting_msg_sent = False

                rospy.loginfo("Connected to copro!")
                self.buffer = []
                self.pingCommander.manual_ping()

                # Clear any data that entered the queue right after close, then enable queueing
                self.response_queue.clear()
                self.command_queue.clear()
                self.command_queueing_permitted = True
                
                # Send the actuator configuration data
                if self.actuatorCommander.lastConfig is not None:
                    self.actuatorCommander.reconfigure_callback(self.actuatorCommander.lastConfig, 0)


    ########################################
    # Main Methods                         #
    ########################################

    def onShutdown(self):
        """Called when rospy is shutting down
        Should only be called from on_shutdown callback
        """

        # Request comm task shutdown, and wait for the connection to close cleanly
        self.comm_should_shutdown = True
        while self.copro_comm_timer.is_alive():
            rospy.sleep(0.01)


    def main(self):
        """Runs the coprocessor_driver ros node

        Raises:
            RuntimeError: Raised if the specified robot in the config is invalid
        """
        rospy.init_node('coprocessor_driver')

        # Load configuration values depending on current robot
        self.current_robot = rospy.get_param('~current_robot')

        if self.current_robot == PUDDLES_ROBOT:
            self.IP_ADDR = 'localhost'
        elif self.current_robot == TITAN_ROBOT:
            self.IP_ADDR = '192.168.1.43'
        else:
            raise RuntimeError("Invalid Robot Specified")

        self.connection_pub = rospy.Publisher('state/copro', Bool, queue_size=1)
        self.connection_latency_pub = rospy.Publisher('state/copro/latency', UInt16, queue_size=1)
        self.connection_tx_queue_len_pub = rospy.Publisher('state/copro/tx_len', Int8, queue_size=1)
        self.connection_rx_pending_queue_len_pub = rospy.Publisher('state/copro/rx_pending_len', Int8, queue_size=1)
        
        # Implement command loading here
        self.pwmCommander = PwmCommand(self)
        DepthCommand(self)
        BatteryVoltageCommand(self)
        BatteryCurrentCommand(self)
        TemperatureCommand(self)
        SwitchCommand(self)
        ThrusterCurrentCommand(self)
        CoproMemoryCommand(self)
        TempThresholdCommand(self)
        PeltierCommand(self)
        CoproFaultCommand(self)
        LogicVoltageCommand(self)
        ResetCommand(self)
        if self.current_robot == TITAN_ROBOT:
            LightingCommand(self)
        self.actuatorCommander = ActuatorCommand(self)
        self.pingCommander = PingCommand(self)

        # set up clean shutdown
        rospy.on_shutdown(self.onShutdown)

        # Setup monitoring task
        self.copro_comm_timer = rospy.Timer(rospy.Duration(0.01), self.coproCommTask)

        # Task Monitoring
        try:
            while not rospy.is_shutdown():
                rospy.sleep(0.1)

                # Check command tasks
                for command_id in self.registered_commands:
                    
                    # Check critical tasks
                    for command in self.registered_commands[command_id].critical_tasks:
                        if not command.is_alive() and not self.comm_should_shutdown:
                            rospy.logfatal("Killing coprocessor_driver node: {0} critical task thread crashed!".format(self.registered_commands[command_id].__class__.__name__))
                            rospy.signal_shutdown("Shutting Down from thread crash")
                    
                    # Check non critical tasks
                    for command in self.registered_commands[command_id].tasks:
                        if not command.is_alive() and not self.comm_should_shutdown:
                            rospy.logerr_once("{0} task crashed".format(self.registered_commands[command_id].__class__.__name__))
                
                # Check comm task
                if not self.copro_comm_timer.is_alive() and not self.comm_should_shutdown:
                    rospy.logfatal("Killing coprocessor_driver node: Copro Comm thread crashed!")
                    rospy.signal_shutdown("Shutting Down from thread crash")
        except:
            rospy.logfatal("Killing coprocessor_driver node: Task monitor crashed")
            raise

if __name__ == '__main__':
    coproDriver = CoproDriver()
    coproDriver.main()