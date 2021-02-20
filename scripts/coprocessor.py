#!/usr/bin/env python

import rospy
import socket
import select
import traceback
from threading import Thread, Lock
from collections import deque
from std_msgs.msg import String, Header, Bool, Float32MultiArray, Int8, Float32, Int16MultiArray, UInt8MultiArray
from riptide_msgs.msg import Depth, PwmStamped, StatusLight, SwitchState
from riptide_hardware.cfg import CoprocessorDriverConfig
from dynamic_reconfigure.server import Server
import yaml

# Robot Types
PUDDLES_ROBOT = 1
TITAN_ROBOT = 2

# Copro Commands
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
COPRO_RESET_CMD = 15
ACTUATOR_CMD = 16           # Implemented
PING_COPRO_CMD = 17
MEMORY_CHECK_CMD = 18       # Implemented
TEMP_THRESHOLD_CMD = 19     # Implemented
GET_FAULT_STATE_CMD = 20    # Implemented

# Actuator Commands
ACTUATOR_SET_TORPEDO_TIMING_CMD = 0     # Implemented
ACTUATOR_RESET_BOARD_CMD = 1
ACTUATOR_GET_FAULT_STATUS_CMD = 2       # Implemented - Note: Only implemented on titan
ACTUATOR_GET_TORPEDO_STATUS_CMD = 3
ACTUATOR_ARM_TORPEDO_CMD = 4            # Implemented
ACUTATOR_FIRE_TORPEDO_CMD = 5           # Implemented
ACTUATOR_RELEASE_MARKER_CMD = 6         # Implemented
ACTUATOR_SET_GRIPPER_PWM_CMD = 7        # Implemented

# Actuator command constant
ACTUATOR_TRY_FAIL = 0xFF

class BaseCoproCommand:
    def __init__(self, driver):
        self.driver = driver
        self.driver.registerCommand(self)

    def getCommandId(self):
        raise NotImplementedError()

    def commandCallback(self, response):
        raise NotImplementedError()


def toBytes(num):
    return [num // 256, num % 256]


class PwmCommand(BaseCoproCommand):
    def __init__(self, driver):
        BaseCoproCommand.__init__(self, driver)
        rospy.Subscriber('command/pwm', Int16MultiArray, self.pwm_callback)
    
    def getCommandId(self):
        return THRUSTER_FORCE_CMD
    
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

    def commandCallback(self, response):
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
        BaseCoproCommand.__init__(self, driver)
        self.depth_pub = rospy.Publisher('depth/raw', Depth, queue_size=1)
        self.depth_connected_pub = rospy.Publisher('depth/connected', Bool, queue_size=1)
        self.depth_connected_pub.publish(False)

        with open(rospy.get_param('vehicle_file'), 'r') as stream:
            self.depthVariance = yaml.safe_load(stream)['depth']['sigma'] ** 2

        rospy.Timer(rospy.Duration(0.05), self.depth_callback)
    
    def getCommandId(self):
        return GET_DEPTH_CMD

    def depth_callback(self, event):
        if self.depth_pub.get_num_connections() > 0:
            self.driver.enqueueCommand(GET_DEPTH_CMD)
    
    def commandCallback(self, response):
        if len(response) != 4:
            rospy.logerr("Improper depth response: " + str(response))
        else:
            if response[0] == 1:
                depth = (response[1] << 16) + (response[2] << 8) + response[3]
                if response[0] & 0x80 != 0:
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
        BaseCoproCommand.__init__(self, driver)
        self.batVoltage_pub = rospy.Publisher('state/battery_voltage', Float32MultiArray, queue_size=1)
        self.balanced_voltage_pub = rospy.Publisher('state/voltage_balanced', Float32, queue_size=1)
        rospy.Timer(rospy.Duration(1), self.battery_voltage_callback)

    def getCommandId(self):
        return BATTERY_VOLTAGE_CMD

    def battery_voltage_callback(self, event):   
        if self.batVoltage_pub.get_num_connections() > 0 or self.balanced_voltage_pub.get_num_connections() > 0:
            self.driver.enqueueCommand(BATTERY_VOLTAGE_CMD)
    
    def commandCallback(self, response):
        if len(response) != 6:
            rospy.logerr("Improper battery voltage response: " + str(response))
        else: 
            port_volt = (response[0] *256.0 + response[1])/100.0
            stbd_volt = (response[2] *256.0 + response[3])/100.0
            balanced_volt = (response[4] *256.0 + response[5])/100.0
            self.batVoltage_pub.publish(Float32MultiArray(data = [port_volt, stbd_volt]))
            self.balanced_voltage_pub.publish(balanced_volt)


class BatteryCurrentCommand(BaseCoproCommand):
    def __init__(self, driver):
        BaseCoproCommand.__init__(self, driver)
        self.batCurrent_pub = rospy.Publisher('state/battery_current', Float32MultiArray, queue_size=1)
        rospy.Timer(rospy.Duration(1), self.battery_current_callback)
    
    def getCommandId(self):
        return BATTERY_CURRENT_CMD

    def battery_current_callback(self, event):   
        if self.batCurrent_pub.get_num_connections() > 0:
            self.driver.enqueueCommand(BATTERY_CURRENT_CMD)

    def commandCallback(self, response):
        if len(response) != 4:
            rospy.logerr("Improper battery current response: " + str(response))
        else:
            port_current = (response[0] *256.0 + response[1])/100.0
            stbd_current = (response[2] *256.0 + response[3])/100.0
            self.batCurrent_pub.publish(Float32MultiArray(data = [port_current, stbd_current]))


class TemperatureCommand(BaseCoproCommand):
    def __init__(self, driver):
        BaseCoproCommand.__init__(self, driver)
        self.temperature_pub = rospy.Publisher('state/temperature', Float32, queue_size=1)
        rospy.Timer(rospy.Duration(0.5), self.temperature_callback)

    def getCommandId(self):
        return TEMPERATURE_CMD

    def temperature_callback(self, event):   
        if self.temperature_pub.get_num_connections() > 0:
            self.driver.enqueueCommand(TEMPERATURE_CMD)
    
    def commandCallback(self, response):
        if len(response) != 2:
            rospy.logerr("Improper battery current response: " + str(response))
        else:
            temperature = (response[0]*256.0 + response[1])/10.0
            self.temperature_pub.publish(Float32(temperature))


class SwitchCommand(BaseCoproCommand):
    def __init__(self, driver):
        BaseCoproCommand.__init__(self, driver)
        self.last_kill_switch_state = False
        self.switch_pub = rospy.Publisher('state/switches', SwitchState, queue_size=1)
        rospy.Timer(rospy.Duration(0.2), self.switch_callback)

    def getCommandId(self):
        return GET_SWITCHES_CMD
    
    def switch_callback(self, event):
        if self.switch_pub.get_num_connections() > 0:
            self.driver.enqueueCommand(GET_SWITCHES_CMD)
    
    def commandCallback(self, response):
        if len(response) != 1:
            rospy.logerr("Improper switches response: " + str(response))
        else:
            switch_msg = SwitchState()
            switch_msg.header.stamp = rospy.Time.now()
            switch_msg.kill = True if response[0] & 2 else False
            switch_msg.sw1 = True if response[0] & 1 else False

            if self.last_kill_switch_state is not switch_msg.kill:
                if switch_msg.kill:
                    rospy.loginfo("Kill switch engaged")
                else:
                    rospy.loginfo("Kill switch disengaged")
                self.last_kill_switch_state = switch_msg.kill

            self.switch_pub.publish(switch_msg)


class ThrusterCurrentCommand(BaseCoproCommand):
    def __init__(self, driver):
        BaseCoproCommand.__init__(self, driver)
        self.thruster_current_pub = rospy.Publisher('state/thruster_currents', Float32MultiArray, queue_size=1)
        rospy.Timer(rospy.Duration(0.2), self.thruster_current_callback)

    def getCommandId(self):
        return THRUSTER_CURRENT_CMD

    def thruster_current_callback(self, event):
        if self.thruster_current_pub.get_num_connections() > 0:
            self.driver.enqueueCommand(THRUSTER_CURRENT_CMD)

    def commandCallback(self, response):
        if len(response) != 8:
            rospy.logerr("Improper thruster current response: " + str(response))
        else:
            current_msg = Float32MultiArray(data = [x/25.0 for x in response])
            self.thruster_current_pub.publish(current_msg)


class CoproMemoryCommand(BaseCoproCommand):
    def __init__(self, driver):
        BaseCoproCommand.__init__(self, driver)
        self.memory_pub = rospy.Publisher('state/memory_usage', Float32, queue_size=1)
        rospy.Timer(rospy.Duration(1.0), self.memory_callback)

    def getCommandId(self):
        return MEMORY_CHECK_CMD

    def memory_callback(self, event):
        if self.memory_pub.get_num_connections() > 0:
            self.driver.enqueueCommand(MEMORY_CHECK_CMD)
    
    def commandCallback(self, response):
        if len(response) != 2:
            rospy.logerr("Improper memory usage response: " + str(response))
        else:
            memory = (response[0]*256.0 + response[1]*256.0)/(256*256-1)
            self.memory_pub.publish(Float32(memory))


class TempThresholdCommand(BaseCoproCommand):
    def __init__(self, driver):
        BaseCoproCommand.__init__(self, driver)
        self.temp_threshold_pub = rospy.Publisher('state/temp_threshold', Int8, queue_size=1)
        rospy.Timer(rospy.Duration(1), self.temp_threshold_callback)

        self.set_temp_threshold_pub = rospy.Subscriber('control/temp_threshold', Int8, self.set_temp_threshold_callback)
    
    def getCommandId(self):
        return TEMP_THRESHOLD_CMD

    def temp_threshold_callback(self, event):
        if self.temp_threshold_pub.get_num_connections() > 0:
            self.driver.enqueueCommand(TEMP_THRESHOLD_CMD)

    def set_temp_threshold_callback(self, msg):
        self.driver.enqueueCommand(TEMP_THRESHOLD_CMD, [msg.data])

    def commandCallback(self, response):
        if len(response) != 1:
            rospy.logerr("Improper temp threshold response: " + str(response))
        else:
            temp_threshold = int(response[0])
            self.temp_threshold_pub.publish(temp_threshold)


class LightingCommand(BaseCoproCommand):
    def __init__(self, driver):
        BaseCoproCommand.__init__(self, driver)

        assert self.driver.current_robot == TITAN_ROBOT

        rospy.Subscriber('command/light1', Int8, self.light1_callback)
        rospy.Subscriber('command/light2', Int8, self.light2_callback)
    
    def getCommandId(self):
        return LIGHTING_POWER_CMD

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
    
    def commandCallback(self, response):
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
        BaseCoproCommand.__init__(self, driver)

        self.peltier_power_pub = rospy.Publisher('state/peltier_power', Bool, queue_size=1)
        rospy.Timer(rospy.Duration(1), self.peltier_callback)
    
    def getCommandId(self):
        return PELTIER_POWER_CMD

    def peltier_callback(self, event):
        if self.peltier_power_pub.get_num_connections() > 0:
            self.driver.enqueueCommand(PELTIER_POWER_CMD)
    
    def commandCallback(self, response):
        if len(response) != 1 or (response[0] != 0 and response[0] != 1):
            rospy.logerr("Improper peltier response: " + str(response))
        else:
            self.peltier_power_pub.publish(bool(response[0]))


class CoproFaultCommand(BaseCoproCommand):
    def __init__(self, driver):
        BaseCoproCommand.__init__(self, driver)

        self.copro_fault_pub = rospy.Publisher('state/copro_fault', UInt8MultiArray, queue_size=1)
        rospy.Timer(rospy.Duration(1), self.copro_fault_callback)
    
    def getCommandId(self):
        return GET_FAULT_STATE_CMD
    
    def copro_fault_callback(self, event):
        if self.copro_fault_pub.get_num_connections() > 0:
            self.driver.enqueueCommand(GET_FAULT_STATE_CMD)

    def commandCallback(self, response):
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
        BaseCoproCommand.__init__(self, driver)

        self.logic_12v_pub = rospy.Publisher('state/voltage_12', Float32, queue_size=1)
        self.logic_5v_pub = rospy.Publisher('state/voltage_5', Float32, queue_size=1)
        rospy.Timer(rospy.Duration(1), self.logic_voltage_callback)

    def getCommandId(self):
        return LOGIC_VOLTAGE_CMD

    def logic_voltage_callback(self, event):
        if self.logic_12v_pub.get_num_connections() > 0 or self.logic_5v_pub.get_num_connections() > 0:
            self.driver.enqueueCommand(LOGIC_VOLTAGE_CMD)
    
    def commandCallback(self, response):
        if len(response) != 6:
            rospy.logerr("Invalid Logic Voltage Response: " + str(response))
        else:
            five_volt_value = ((response[2] * 256) + response[3]) / 1000.0
            twelve_volt_value = ((response[4] * 256) + response[5]) / 500.0

            self.logic_5v_pub.publish(five_volt_value)
            self.logic_12v_pub.publish(twelve_volt_value)
    

class ActuatorCommand(BaseCoproCommand):
    lastConfig = None

    def __init__(self, driver):
        BaseCoproCommand.__init__(self, driver)

        self.response_queue = deque([], 50)

        rospy.Subscriber('command/drop', Int8, self.drop_callback)
        rospy.Subscriber('command/arm', Bool, self.arm_callback)
        rospy.Subscriber('command/fire', Int8, self.fire_callback)
        rospy.Subscriber('command/grabber', Int8, self.grab_callback)

        if self.driver.current_robot == TITAN_ROBOT:
            self.actuator_connection_pub = rospy.Publisher('state/actuator', Bool, queue_size=1)
            self.actuator_connection_pub.publish(False)
            self.actuator_fault_pub = rospy.Publisher('state/actuator_fault', Bool, queue_size=1)

            rospy.Timer(rospy.Duration(1), self.status_callback)
        else:
            self.actuator_connection_pub = None
            self.actuator_fault_pub = None

        Server(CoprocessorDriverConfig, self.reconfigure_callback)

    def getCommandId(self):
        return ACTUATOR_CMD

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

        if self.driver.enqueueCommand(ACTUATOR_CMD, [(command << 5) + lower_bit_args] + remaining_args):
            self.response_queue.append(command)

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

    def commandCallback(self, response):
        command = self.response_queue.popleft()

        if self.driver.current_robot == TITAN_ROBOT:
            if len(response) != 2:
                rospy.logerr("Invalid response for actuator command %d: " + str(response), command)
                return
            if response[1] == ACTUATOR_TRY_FAIL:
                if command != ACTUATOR_GET_FAULT_STATUS_CMD:
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

        if command != ACTUATOR_GET_TORPEDO_STATUS_CMD and command != ACTUATOR_GET_FAULT_STATUS_CMD:
            if response[0] == 0:
                rospy.logwarn("Failed to execute actuator command %d", command)
            elif response[0] != 1:
                rospy.logerr("Invalid response for actuator command %d: %d", command, response[0])
        elif command == ACTUATOR_GET_FAULT_STATUS_CMD:
            if response[0] != 0 and response[0] != 1:
                rospy.logerr("Invalid response for actuator command %d: %d", command, response[0])
            else:
                self.actuator_fault_pub.publish(bool(response[0]))


class CoproDriver:
    IP_ADDR = None
    connection_pub = None
    current_robot = None
    actuatorCommander = None

    def __init__(self):
        self.copro = None
        self.connected = False
        # only add byte arrays to this queue
        self.command_queue = deque([], 50)
        # after appending command to command_queue,
        # append the command id to the response queue
        self.response_queue = deque([], 50)
        self.buffer = []
        self.registered_commands = {}

        self.command_lock = Lock()
        

    def connect(self, timeout) :
        try:
            self.copro = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.copro.settimeout(timeout)
            self.copro.connect((self.IP_ADDR, 50000))
            self.connected = True
            return True
        except:
            self.connected = False
            self.copro = None
            return False

    def enqueueCommand(self, command, args = []):
        if not self.connected:
            return False

        self.command_lock.acquire()
        data = [command] + args
        data = [len(data) + 1] + data
        if (len(self.command_queue) + len(data)) <= self.command_queue.maxlen and (len(self.response_queue)+1) <= self.response_queue.maxlen:
            self.command_queue.append(data)
            self.response_queue.append(command)
            self.command_lock.release()
            return True
        else:
            rospy.logwarn_throttle(5, "Copro commands dropped: Command Buffer Full")
            self.command_lock.release()
            return False

    def registerCommand(self, receiver):
        if receiver.getCommandId() in self.registered_commands:
            raise RuntimeError("Command already registered")
        self.registered_commands[receiver.getCommandId()] = receiver

    def deregisterCommand(self, receiver):
        if receiver.getCommandId() not in self.registered_commands:
            raise KeyError("Command not registered")
        if self.registered_commands[receiver.getCommandId()] != receiver:
            raise ValueError("The requested command to deregister is not currently registered")
        del self.registered_commands[receiver.getCommandId()]

    def shutdown_copro(self):
        if self.connected:
            self.connected = False
            # Stop thrusters
            rospy.loginfo("Stopping thrusters")
            self.copro.sendall(bytearray([18, 7, 5, 220, 5, 220, 5, 220, 5, 220, 5, 220, 5, 220, 5, 220, 5, 220]))
            self.copro.sendall(bytearray([0]))
            rospy.sleep(.1)
            self.copro.close()

    def doCoproCommunication(self):
        readable, writable, exceptional = select.select([self.copro], [self.copro], [self.copro], 0)

        if len(writable) > 0 and len(self.command_queue) > 0:
            command = []
            while len(self.command_queue) != 0:
                command += self.command_queue.popleft()
            self.copro.sendall(bytearray(command))
        if len(readable) > 0:
            recv_data = self.copro.recv(64)
            if len(recv_data) == 0:
                self.connected = False
                return
            if not isinstance(recv_data[0], int):
                recv_data = list(map(ord, recv_data))
            self.buffer += recv_data
            while len(self.buffer) > 0 and self.buffer[0] <= len(self.buffer):
                response = self.buffer[1:self.buffer[0]]
                command = self.response_queue.popleft()

                if len(response) == 0:
                    rospy.logwarn("Protocol error on command "+str(command))

                    # If the command is for the actuator, remove the subcommand from queue to keep it lined up
                    if command == self.actuatorCommander.getCommandId():
                        self.actuatorCommander.response_queue.popleft()
                elif command in self.registered_commands:
                    self.registered_commands[command].commandCallback(response)
                else:
                    rospy.logerr("Unhandled Command Response from Copro: %d", command)

                self.buffer = self.buffer[self.buffer[0]:]

    def main(self):
        rospy.init_node('coprocessor_driver')

        self.current_robot = rospy.get_param('current_robot')

        if self.current_robot == PUDDLES_ROBOT:
            self.IP_ADDR = '192.168.1.42'
        elif self.current_robot == TITAN_ROBOT:
            self.IP_ADDR = '192.168.1.43'
        else:
            raise RuntimeError("Invalid Robot Specified")

        self.connection_pub = rospy.Publisher('state/copro', Bool, queue_size=1)
        
        # Implement command loading here
        PwmCommand(self)
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
        if self.current_robot == TITAN_ROBOT:
            LightingCommand(self)
        self.actuatorCommander = ActuatorCommand(self)
        
        # 200 Hz rate for checking for messages to send
        rate = rospy.Rate(100)
        
        # set up clean shutdown
        rospy.on_shutdown(self.shutdown_copro)

        while not rospy.is_shutdown():
            if self.connected:
                try:
                    self.doCoproCommunication()
                except Exception as e:
                    traceback.print_exc()
                    rospy.logerr("Exception Occurred: " + str(e))
                    self.command_queue.clear()
                    self.response_queue.clear()
                    self.actuatorCommander.response_queue.clear()
                    self.buffer = []
                    try:
                        self.shutdown_copro()
                    except:
                        pass
                    self.copro = None
                    self.connected = False
            else:
                rospy.loginfo("Connecting to copro...")
                while not self.connect(1.0):
                    self.connection_pub.publish(False)
                    if rospy.is_shutdown():
                        return
                rospy.loginfo("Connected to copro!")
                
                self.buffer = []
                self.connection_pub.publish(True)
                self.response_queue.clear()
                self.command_queue.clear()
                self.actuatorCommander.response_queue.clear()
                if self.actuatorCommander.lastConfig is not None:
                    self.actuatorCommander.reconfigure_callback(self.actuatorCommander.lastConfig, 0)

            rate.sleep()

if __name__ == '__main__':
    coproDriver = CoproDriver()
    coproDriver.main()