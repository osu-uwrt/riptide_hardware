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
'state/kill_switch' (Bool): The state of the kill switch
'state/aux_switch' (Bool): The state of the aux switch
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

import random
import rospy
import socket
import select
import threading
import time
import traceback
from collections import deque
from typing import Any, Dict, List, Union
from std_msgs.msg import Bool, Float32MultiArray, Int8, UInt16, Float32, Int16MultiArray, UInt8MultiArray
from riptide_hardware.msg import Depth
from riptide_hardware.cfg import CoprocessorDriverConfig
from dynamic_reconfigure.server import Server
import yaml

# Robot Types
PUDDLES_ROBOT = "puddles"
TEMPEST_ROBOT = "tempest"

# Copro Commands
CONN_EXPECTED_VERSION = b"COPRO-2.0"
CONN_LATENCY_NEW_WEIGHT = 0.02

MOBO_POWER_CMD = 0
LIGHTING_POWER_CMD = 1      # Implemented - Note: Controls jetson power on Puddles Copro
THRUSTER_ENABLE_CMD = 2
PELTIER_POWER_CMD = 3       # Implemented
BATTERY_VOLTAGE_CMD = 4     # Implemented
BATTERY_CURRENT_CMD = 5     # Implemented
TEMPERATURE_CMD = 6         # Implemented
THRUSTER_FORCE_CMD = 7      # Implemented
LOGIC_CURRENT_CMD = 8           # Note: Not implemented on Tempest Copro
LOGIC_VOLTAGE_CMD = 9       # Implemented
GET_SWITCHES_CMD = 10       # Implemented
GET_DEPTH_CMD = 11          # Implemented
THRUSTER_CURRENT_CMD = 12   # Implemented
TWELVE_VOLT_POWER_CMD = 13
FIVE_VOLT_RESET_CMD = 14
COPRO_RESET_CMD = 15        # Implemented
ACTUATOR_CMD = 16           # Implemented
PING_COPRO_CMD = 17
MEMORY_CHECK_CMD = 18       # Implemented
TEMP_THRESHOLD_CMD = 19     # Implemented
GET_FAULT_STATE_CMD = 20    # Implemented
GET_VERSION_CMD = 21        # Implemented
SAFETY_KEEPALIVE_CMD = 22   # Implemented

# Actuator Commands
ACTUATOR_SET_TORPEDO_TIMING_CMD = 0     # Implemented
ACTUATOR_RESET_BOARD_CMD = 1            # Implemented
ACTUATOR_GET_FAULT_STATUS_CMD = 2       # Implemented - Note: Only implemented on tempest
ACTUATOR_GET_TORPEDO_STATUS_CMD = 3
ACTUATOR_ARM_TORPEDO_CMD = 4            # Implemented
ACUTATOR_FIRE_TORPEDO_CMD = 5           # Implemented
ACTUATOR_RELEASE_MARKER_CMD = 6         # Implemented
ACTUATOR_SET_GRIPPER_PWM_CMD = 7        # Implemented

# Actuator command constant
ACTUATOR_TRY_FAIL = 0xFF

########################################
#region Utility/Skeleton Classes
########################################

class QueuedCommand:
    """Holds the data for a command to be sent to the copro
    """

    command_id: int
    """The ID of the command"""

    receives_response: bool
    """True if the command will receive a response from the copro"""

    packet_id: int
    """The packet ID corresponding to this queued command, used to match responses"""

    extra_data: Any
    """Extra data queued with the command"""

    def __init__(self, command_id: int, command_data: List[int], extra_data: Any, receives_response: bool):
        """Initializes a new command instance and sets the create time to the current time

        Args:
            command_id (int): The command id of the command (Must be unsigned 8-bit integer)
            command_data (List[int]): Data to be added to the command
            extra_data (Any): Data to be passed to the command callback
            requires_resopnse (bool): Determines if the command will receive a response from the copro
        """

        # Initialize Public Variables
        self.command_id = command_id
        self.receives_response = receives_response
        self.extra_data = extra_data
        self.packet_id = None

        # Initialize Private Variables
        self._command_data = command_data       # The extra data for the command
        self._create_time = time.time()         # The time the command object was initially created
    
    def generate_packet(self, packet_id: int) -> bytearray:
        """Updates the command to being sent from the copro and returns a bytearray of the command to be sent
        
        Args:
            packet_id: The packet id to give to this packet during transmission (Must be NON-ZERO unsigned 16-bit integer)

        Returns:
            bytes: The encoded command to be sent to the copro
        """
        assert packet_id > 0 and packet_id < 2**16, "Invalid Packet ID provided: {}".format(packet_id)
        assert packet_id is None, "Command {}-{} already has a packet id assigned ({})".format(self.command_id, packet_id, self.packet_id)

        # Store packet id
        self.packet_id = packet_id

        # Command Packet Format
        # Byte 0: Length (Including length byte)
        # Byte 1: Command ID
        # Byte 3-4: Packet ID
        # Byte 2-n: Args
        command_data = self._command_data
        packet_header = [len(command_data) + 4, self.command_id, self.packet_id >> 8, self.packet_id & 0xFF]
        return bytearray(packet_header + command_data)

    def get_elapsed(self) -> float:
        """Calculates the elapsed time of the packet from the given moment from when the packet was created
        
        Returns:
            float: The elapsed time since command creation of this queued command
        """

        return time.time() - self._create_time

class BaseCoproCommand:
    """The base copro command class for which all copro commands must use as a base class
    """

    tasks: List[rospy.Timer]
    """List of non critical tasks to monitor"""

    critical_tasks: List[rospy.Timer]
    """List of critical tasks to monitor.
    If any of these tasks stop, the code will restart
    """

    def __init__(self, driver: 'CoproDriver', command_id: int, receives_response = True):
        """Initializes the Command and registers it with the CoproDriver

        Args:
            driver (CoproDriver): The instance of CoproDriver to register the command with
            command_id (int): The command id to be sent
            receives_response: If the command gets a response from the copro
        """
        # Initialize Private Variables
        self._command_id = command_id
        self._receives_response = receives_response

        # Initialize Task Arrays
        self.critical_tasks = []
        self.tasks = []

        # Register Command
        self.driver: 'CoproDriver' = driver
        self.driver.registerCommand(self)

    def getCommandId(self) -> int:
        """The command id for the class

        Returns:
            int: The id of the command
        """
        return self._command_id

    def enqueueCommand(self, command_data: List[int] = [], extra_data: Any = None) -> None:
        """Enqueues the command to be sent. If replace duplicates is enabled,this will instead
        replace the command currently queued if it has not been sent yet.

        Args:
            command_data (List[int]): Data to be added to the command
            extra_data (Any): Data to be passed to the command callback
        """
        
        command = QueuedCommand(self._command_id, command_data, extra_data, self._receives_response)
        self.driver.enqueueCommand(command)

    def commandCallback(self, response: List[int], extra_data: Any) -> None:
        """The callback for after a command has been executed

        Args:
            response (list): An integer list of the response from the copro
        """
        raise NotImplementedError()


def toBytes(num: int) -> List[int]:
    """Converts 16-bit integer to 8-bit integer array

    Args:
        num (int): A 16-bit integer

    Returns:
        list: An 8-bit integer list
    """
    assert num < 2**16
    return [num // 256, num % 256]

#endregion


########################################
#region Command Classes
########################################

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
        self.enqueueCommand(args)

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
            self.enqueueCommand()
    
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
            self.enqueueCommand()
    
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
            self.enqueueCommand()

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
            self.enqueueCommand()
    
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
        self.kill_switch_pub = rospy.Publisher('state/kill_switch', Bool, queue_size=1)
        self.aux_switch_pub = rospy.Publisher('state/aux_switch', Bool, queue_size=1)
        self.critical_tasks.append(rospy.Timer(rospy.Duration(0.2), self.switch_callback))

    def switch_callback(self, event):
        if self.kill_switch_pub.get_num_connections() > 0 or self.aux_switch_pub.get_num_connections() > 0:
            self.enqueueCommand()
    
    def commandCallback(self, response, extra_data):
        if len(response) != 1:
            rospy.logerr("Improper switches response: " + str(response))
        else:
            kill_state = bool(response[0] & (1 << 0))
            self.kill_switch_pub.publish(kill_state)
            self.aux_switch_pub.publish(bool(response[0] & (1 << 1)))

            if self.last_kill_switch_state is not kill_state:
                if kill_state:
                    rospy.loginfo("Kill switch engaged")
                else:
                    rospy.loginfo("Kill switch disengaged")
                self.last_kill_switch_state = kill_state


class ThrusterCurrentCommand(BaseCoproCommand):
    def __init__(self, driver):
        BaseCoproCommand.__init__(self, driver, THRUSTER_CURRENT_CMD)
        self.thruster_current_pub = rospy.Publisher('state/thruster_currents', Float32MultiArray, queue_size=1)
        self.tasks.append(rospy.Timer(rospy.Duration(0.2), self.thruster_current_callback))

    def thruster_current_callback(self, event):
        if self.thruster_current_pub.get_num_connections() > 0:
            self.enqueueCommand()

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
            self.enqueueCommand()
    
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
            self.enqueueCommand()

    def set_temp_threshold_callback(self, msg):
        self.enqueueCommand([msg.data])

    def commandCallback(self, response, extra_data):
        if len(response) != 1:
            rospy.logerr("Improper temp threshold response: " + str(response))
        else:
            temp_threshold = int(response[0])
            self.temp_threshold_pub.publish(temp_threshold)


class LightingCommand(BaseCoproCommand):
    def __init__(self, driver):
        BaseCoproCommand.__init__(self, driver, LIGHTING_POWER_CMD)

        assert self.driver.current_robot == TEMPEST_ROBOT

        rospy.Subscriber('command/light1', Int8, self.light1_callback)
        rospy.Subscriber('command/light2', Int8, self.light2_callback)

    def light1_callback(self, msg):
        if msg.data < 0 or msg.data > 100:
            rospy.logerr("Invalid range for lighting request: %d", msg.data)
        else:
            lighting_args = [1, msg.data]
            self.enqueueCommand(lighting_args)

    def light2_callback(self, msg):
        if msg.data < 0 or msg.data > 100:
            rospy.logerr("Invalid range for lighting request: %d", msg.data)
        else:
            lighting_args = [2, msg.data]
            self.enqueueCommand(lighting_args)
    
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
            self.enqueueCommand()
    
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
            self.enqueueCommand()

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
            self.enqueueCommand()
    
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
        BaseCoproCommand.__init__(self, driver, COPRO_RESET_CMD, receives_response=False)

        rospy.Subscriber('control/copro_reset', Bool, self.reset_callback)

    def reset_callback(self, msg):
        if msg.data:
            self.enqueueCommand()

class ActuatorCommand(BaseCoproCommand):
    lastConfig = None

    def __init__(self, driver):
        BaseCoproCommand.__init__(self, driver, ACTUATOR_CMD)

        rospy.Subscriber('command/drop', Int8, self.drop_callback)
        rospy.Subscriber('command/arm', Bool, self.arm_callback)
        rospy.Subscriber('command/fire', Int8, self.fire_callback)
        rospy.Subscriber('command/grabber', Int8, self.grab_callback)
        rospy.Subscriber('control/actuator_reset', Bool, self.reset_callback)

        if self.driver.current_robot == TEMPEST_ROBOT:
            self.actuator_connection_pub = rospy.Publisher('state/actuator', Bool, queue_size=1)
            self.actuator_connection_pub.publish(False)
            self.actuator_fault_pub = rospy.Publisher('state/actuator_fault', Bool, queue_size=1)

            self.tasks.append(rospy.Timer(rospy.Duration(1), self.status_callback))
        else:
            self.actuator_connection_pub = None
            self.actuator_fault_pub = None

        if self.driver.current_robot == TEMPEST_ROBOT:
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
        
        self.enqueueCommand([(command << 5) + lower_bit_args] + remaining_args, extra_data=command)

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

        if self.driver.current_robot == TEMPEST_ROBOT:
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


class SafetyKeepaliveCommand(BaseCoproCommand):
    def __init__(self, driver):
        BaseCoproCommand.__init__(self, driver, SAFETY_KEEPALIVE_CMD)

        # Nothing needs to be published or subscribed to. Just to make sure the connection is still alive

        self.last_ping = time.time()
        self.critical_tasks.append(rospy.Timer(rospy.Duration(0.05), self.keepalive_callback))

    def keepalive_callback(self, event):
        self.enqueueCommand([1])
    
    def invalidate_keepalive(self):
        self.enqueueCommand([0])
    
    def manual_ping(self):
        self.last_ping = time.time()
        self.enqueueCommand([1])

    def commandCallback(self, response, extra_data):
        if len(response) != 1 and response[0] != 1:
            rospy.logerr("Invalid keepalive response: " + str(response))
        else:
            self.last_ping = time.time()

#endregion


########################################
#region Copro Driver Class
########################################

class CoproDriver:
    # The timeout to be used to consider when a command is not going to be received, as well as when the connection should be considered dead
    TIMEOUT = 2.0

    # The IP address to use to connect
    IP_ADDR: str

    # The ros publishers for the copro connection state
    connection_pub: rospy.Publisher
    connection_latency_pub: rospy.Publisher
    connection_rx_pending_queue_len_pub: rospy.Publisher

    # The timer scheduling the copro communication task
    copro_comm_timer: rospy.Timer

    # The config parameter passed with the current robot
    current_robot = None

    # Commands with specific features used directly in the copro driver
    actuatorCommander: ActuatorCommand          # The instance of ActuatorCommand (used for actuator callback configuration)
    keepaliveCommander: SafetyKeepaliveCommand  # The instance of SafetyKeepaliveCommand (used for timeouts and controlling safeties)

    def __init__(self):
        """Initializes new instance of CoproDriver class
        """

        # The registered commands with the command id as the key and BaseCoproCommand implementation as value
        # Should only be written by registerCommand and deregisterCommand
        self.registered_commands: Dict[int, BaseCoproCommand] = {}

        ########################################
        ### Note: These variables are safe to be used by all class methods when locked

        # Said lock that protects the following variables when being accessed by ros callbacks
        self.comm_lock = threading.Lock()

        # The socket connection to copro
        # Since it is a UDP socket it can be immediately opened and never needs to be reopened
        self.copro = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.copro.setblocking(False)

        # True when a copro is actively responding, False when a copro hasn't responded within the given timeout
        # Should only written by copro driver thread
        self.connected = False

        ########################################
 

        ########################################
        ### Note: These variables should only be interacted with by class methods running on the copro driver thread

        # Outstanding Commands Dict
        # Has keys with the packet id and the value of the corresponding QueuedCommand
        self.outstanding_commands: 'Dict[int, QueuedCommand]' = {}

        # If the copro communication task should shut down
        self.comm_should_shutdown = False

        # Connection latency (ms)
        self.connection_latency = -1

        # The packet id to be used to identify response packets. Incremented and wraps around at 2**16. Should only be directly accessed by _get_packet_id
        self._packet_id = random.randint(1, 65535)

        ########################################

    def _get_packet_id(self) -> int:
        packet_id = self._packet_id
        self._packet_id += 1
        if self._packet_id > 65535:
            self._packet_id = 1
        return packet_id

    ########################################
    # Connection Management Code           #
    ########################################

    def tryConnect(self) -> None:
        with self.comm_lock:
            if self.connected:
                rospy.logwarn("Trying to connect while already connected")
                return
            packet_id = self._get_packet_id()
            self.copro.sendto(bytearray([5, GET_VERSION_CMD, packet_id >> 8, packet_id & 0xFF, 0]), (self.IP_ADDR, 50000))

            r,_,_ = select.select([self.copro], [], [], self.TIMEOUT)
            if self.copro in r:
                # Crude reading code, doesn't need to be too robust since it's just for the hello packet
                data = self.copro.recv(4096)
                while len(data) > data[0]:
                    packet = data[:data[0]]
                    if (len(packet) > 3) and (packet[1] == self.packet_id >> 8) and (self.packet_id & 0xFF):
                        protocol_version = packet[3:]
                        if protocol_version == CONN_EXPECTED_VERSION:
                            self.connected = True
                            break
                        else:
                            rospy.logwarn_once("Unexpected Copro Protocol Version: "+str(protocol_version))
                    
    def closeConnection(self) -> None:
        """Closes the connection to the copro
        Should only be called from copro comm task

        """

        # Try to send invalidate_keepalive
        self.keepaliveCommander.invalidate_keepalive()
        
        # Reset connection instance variables
        with self.comm_lock:
            self.outstanding_commands = {}
            self.connected = False
            self.connection_latency = -1
            self.buffer = []

    def doCoproCommunication(self) -> None:
        """Processes any data returned from the copro
        """

        with self.comm_lock:
            readable, _, _ = select.select([self.copro], [], [], 0)

            # Handle incoming packets if there is data in receive buffer
            if len(readable) > 0:
                # Receive all data until the connection blocks
                try:
                    while True:
                        recv_data = self.copro.recv(64)
                        self.buffer += recv_data
                except BlockingIOError:
                    pass
            
        # Continue processing buffer until the buffer doesn't containe the
        # whole the length (first byte) of the packet or the buffer is empty
        while len(self.buffer) >= 3 and self.buffer[0] <= len(self.buffer):
            # Decode packet - The first byte is length, remaining is data
            packet_id = (self.buffer[1] << 8) + self.buffer[2]
            response_data = self.buffer[3:self.buffer[0]]

            if packet_id in self.outstanding_commands:
                command = self.outstanding_commands.pop(packet_id)
            
                # Retrieve the command id and time of request of this message
                command_latency = int(command.update_received() * 1000)

                # Recalculate connection latency with a weighted average
                if self.connection_latency == -1:
                    self.connection_latency = command_latency
                else:
                    self.connection_latency = int((self.connection_latency * (1-CONN_LATENCY_NEW_WEIGHT)) + (command_latency * CONN_LATENCY_NEW_WEIGHT))

                # The copro connection will return an empty packet if it failed to execute the command
                # If this is the case, don't execute the callback, and instead log error
                if len(response_data) == 0:
                    rospy.logwarn("Command error on command "+str(command.command_id))
                
                # Execute command if callback available for it
                elif command.command_id in self.registered_commands:
                    self.registered_commands[command.command_id].commandCallback(response_data, command.extra_data)
                else:
                    rospy.logerr("Unhandled Command Response from Copro: %d", command.command_id)
            else:
                rospy.logwarn("Unexpected packet with id %d received", packet_id)

            # Remove the processed command from the receive buffer
            self.buffer = self.buffer[self.buffer[0]:]
        
        for packet_id in self.outstanding_commands:
            if self.outstanding_commands[packet_id].get_elapsed() > self.TIMEOUT:
                cmd = self.outstanding_commands.pop(packet_id)
                rospy.logwarn("Dropping unreceived packet %d (cmd: %d)", cmd.packet_id, cmd.command_id)

        # Publish the connection latency and rx pending queue length
        self.connection_rx_pending_queue_len_pub.publish(len(self.outstanding_commands))
        if self.connection_latency != -1:
            self.connection_latency_pub.publish(self.connection_latency)

        # Check for stalls using ping command data
        if time.time() - self.keepaliveCommander.last_ping > self.TIMEOUT:
            rospy.logwarn("Copro Connection Timed Out... Dropping Connection")
            self.closeConnection()


    ########################################
    # Public Methods                       #
    ########################################

    def enqueueCommand(self, command: QueuedCommand) -> bool:
        """Adds command to queue to be sent to copro

        Args:
            command (QueuedCommand): The command to be sent

        Returns:
            bool: If the command was successfully queued
        """
        with self.comm_lock:
            if self.connected:
                raw_data = command.generate_packet(self._get_packet_id())
                self.copro.sendto(raw_data, (self.IP_ADDR, 50000))

                assert command.packet_id not in self.outstanding_commands
                self.outstanding_commands[command.packet_id] = command

                return True
            else:
                return False

    def registerCommand(self, receiver: BaseCoproCommand) -> None:
        """Registers a receiver to process a command

        Args:
            receiver (BaseCoproCommand): Instance of BaseCoproCommand that will be called with the specific command id

        Raises:
            RuntimeError: Raised if the command id is already registered
        """
        if receiver.getCommandId() in self.registered_commands:
            raise RuntimeError("Command already registered")
        self.registered_commands[receiver.getCommandId()] = receiver

    def deregisterCommand(self, receiver: BaseCoproCommand) -> None:
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

    def coproCommTask(self, event: rospy.timer.TimerEvent) -> None:
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
                self.keepaliveCommander.manual_ping()
                
                # Send the actuator configuration data
                if self.actuatorCommander.lastConfig is not None:
                    self.actuatorCommander.reconfigure_callback(self.actuatorCommander.lastConfig, 0)


    ########################################
    # Main Methods                         #
    ########################################

    def onShutdown(self) -> None:
        """Called when rospy is shutting down
        Should only be called from on_shutdown callback
        """

        # Request comm task shutdown, and wait for the connection to close cleanly
        self.comm_should_shutdown = True
        while self.copro_comm_timer.is_alive():
            rospy.sleep(0.01)


    def main(self) -> None:
        """Runs the coprocessor_driver ros node

        Raises:
            RuntimeError: Raised if the specified robot in the config is invalid
        """
        rospy.init_node('coprocessor_driver')

        # Load configuration values depending on current robot
        self.current_robot = rospy.get_param('~current_robot')

        if self.current_robot == PUDDLES_ROBOT:
            self.IP_ADDR = '192.168.1.42'
        elif self.current_robot == TEMPEST_ROBOT:
            self.IP_ADDR = '192.168.1.43'
        else:
            raise RuntimeError("Invalid Robot Specified")

        self.connection_pub = rospy.Publisher('state/copro', Bool, queue_size=1)
        self.connection_latency_pub = rospy.Publisher('state/copro/latency', UInt16, queue_size=1)
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
        if self.current_robot == TEMPEST_ROBOT:
            LightingCommand(self)
        self.actuatorCommander = ActuatorCommand(self)
        self.keepaliveCommander = SafetyKeepaliveCommand(self)

        # set up clean shutdown
        rospy.on_shutdown(self.onShutdown)

        # Setup monitoring task
        self.copro_comm_timer = rospy.Timer(rospy.Duration(0.01), self.coproCommTask)

        # Task Monitoring
        try:
            while not rospy.is_shutdown():
                rospy.sleep(0.5)

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

#endregion

if __name__ == '__main__':
    coproDriver = CoproDriver()
    coproDriver.main()