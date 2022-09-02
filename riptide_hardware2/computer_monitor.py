#!/usr/bin/env python3

import rclpy
import socket
import psutil
from diagnostic_msgs.msg import DiagnosticStatus
import diagnostic_updater
import subprocess
import yaml


class CpuTask(diagnostic_updater.DiagnosticTask):
    def __init__(self, warning_percentage):
        diagnostic_updater.DiagnosticTask.__init__(self, "CPU Information")
        self._warning_percentage = int(warning_percentage)


    def run(self, stat):
        cpu_percentages = (psutil.cpu_percent(percpu=True))
        cpu_average = sum(cpu_percentages) / len(cpu_percentages)

        stat.add("CPU Load Average", "{:.2f}".format(cpu_average))

        for idx, val in enumerate(cpu_percentages):
            stat.add("CPU {} Load".format(idx), "{:.2f}".format(val))

        if cpu_average > self._warning_percentage:
            stat.summary(DiagnosticStatus.WARN,
                         "Average CPU usage exceeds {:d}%".format(self._warning_percentage))
        else:
            stat.summary(DiagnosticStatus.OK, "Average CPU utilization {:.2f}%".format(cpu_average))

        return stat

class CoreTempTask(diagnostic_updater.DiagnosticTask):
    class CoreTempSysfs:
        critical = 80.0
        high = 70.0
        
        def __init__(self, id):
            with open("/sys/class/thermal/thermal_zone%d/temp" % id) as f:
                self.current = int(f.readline().strip()) / 1000.0

            with open("/sys/class/thermal/thermal_zone%d/type" % id) as f:
                self.label = f.readline().strip()
            

    def __init__(self, warning_percentage):
        diagnostic_updater.DiagnosticTask.__init__(self, "Core Temperature")
        self._warning_percentage = int(warning_percentage)

    def run(self, stat):
        #core_temps = psutil.sensors_temperatures()["coretemp"]
        core_temps = [self.CoreTempSysfs(0), self.CoreTempSysfs(1), self.CoreTempSysfs(5),
                      self.CoreTempSysfs(6), self.CoreTempSysfs(7), self.CoreTempSysfs(9)]

        warn = False
        error = False
        max_temp = 0
        for temp in core_temps:
            stat.add(temp.label, "{:.2f} C".format(temp.current))
            
            if temp.current >= temp.critical * self._warning_percentage / 100.0:
                max_temp = temp.critical * self._warning_percentage / 100.0
                error = True
                break

            if temp.current >= temp.high * self._warning_percentage / 100.0:
                max_temp = temp.high * self._warning_percentage / 100.0
                warn = True

        if error:
            stat.summary(DiagnosticStatus.ERROR,
                         "Core temp exceeds {:.2f} C".format(max_temp))
        elif warn:
            stat.summary(DiagnosticStatus.WARN,
                         "Core temp exceeds {:.2f} C".format(max_temp))
        else:
            temps = [x.current for x in core_temps]
            average_temp = sum(temps) / len(temps)
            stat.summary(DiagnosticStatus.OK, "Average core temp {:.2f} C".format(average_temp))

        return stat

    @classmethod
    def has_hardware(cls):
        #return "coretemp" in psutil.sensors_temperatures()
        try:
            core_temps = [cls.CoreTempSysfs(0), cls.CoreTempSysfs(1), cls.CoreTempSysfs(5),
                      cls.CoreTempSysfs(6), cls.CoreTempSysfs(7), cls.CoreTempSysfs(9)]
            return True
        except:
            return False

class ComputerTempTask(diagnostic_updater.DiagnosticTask):
    def __init__(self, warning_percentage, error_temp):
        diagnostic_updater.DiagnosticTask.__init__(self, "Computer Temperature")
        self._warning_percentage = int(warning_percentage)
        self._error_temp = error_temp

    def run(self, stat):
        computer_temp = psutil.sensors_temperatures()["thermal-fan-est"][0].current

        error = computer_temp > self._error_temp
        warn = computer_temp > (self._error_temp * self._warning_percentage / 100.0)

        if error:
            stat.summary(DiagnosticStatus.ERROR,
                         "Core temp exceeds {:.2f} C".format(computer_temp))
        elif warn:
            stat.summary(DiagnosticStatus.WARN,
                         "Core temp exceeds {:.2f} C".format(computer_temp))
        else:
            stat.summary(DiagnosticStatus.OK, "Computer temp {:.2f} C".format(computer_temp))

        return stat

    @staticmethod
    def has_hardware():
        return "thermal-fan-est" in psutil.sensors_temperatures()

class MemoryTask(diagnostic_updater.DiagnosticTask):
    def __init__(self, warning_percentage):
        diagnostic_updater.DiagnosticTask.__init__(self, "Memory Information")
        self._warning_percentage = int(warning_percentage)

    def run(self, stat):
        memory_info = psutil.virtual_memory()

        stat.add("Total Memory", "{:.2f} GB".format(memory_info.total / (1024.0 ** 3)))
        stat.add("Used Memory", "{:.2f} GB".format(memory_info.used / (1024.0 ** 3)))
        stat.add("Percent Used", "{:.2f}%".format(memory_info.percent))

        if memory_info.percent > self._warning_percentage:
            stat.summary(DiagnosticStatus.WARN,
                         "Memory usage exceeds {:d} percent".format(self._warning_percentage))
        else:
            stat.summary(DiagnosticStatus.OK, "Memory usage {:.2f}%".format(memory_info.percent))

        return stat

class DiskTask(diagnostic_updater.DiagnosticTask):
    def __init__(self, warning_percentage):
        diagnostic_updater.DiagnosticTask.__init__(self, "Disk Information")
        self._warning_percentage = int(warning_percentage)

    def run(self, stat):
        all_partitions = psutil.disk_partitions()

        warn = False
        for partition in all_partitions:
            if "snap" in partition.mountpoint:
                continue
            if "loop" in partition.device:
                continue

            disk_usage = psutil.disk_usage(partition.mountpoint)

            used = disk_usage.used / (1024.0 ** 3)
            total = disk_usage.total / (1024.0 ** 3)
            stat.add("\"{:s}\" Usage".format(partition.mountpoint), "{:.2f} GB / {:.2f} GB ({:.2f}%)".format(used, total, disk_usage.percent))

            if disk_usage.percent > self._warning_percentage:
                warn = True


        if warn:
            stat.summary(DiagnosticStatus.WARN,
                         "Disk usage exceeds {:d} percent".format(self._warning_percentage))
        else:
            stat.summary(DiagnosticStatus.OK, "Disk usage is OK")

        return stat

class GPUTask(diagnostic_updater.DiagnosticTask):
    def __init__(self, warning_percentage):
        diagnostic_updater.DiagnosticTask.__init__(self, "GPU Information")
        self._warning_percentage = int(warning_percentage)

    def run(self, stat):
        p = subprocess.Popen('nvidia-smi --query-gpu=index,gpu_name,memory.used,memory.total,utilization.gpu --format=csv', stdout = subprocess.PIPE,
                         stderr = subprocess.PIPE, shell = True)
        (o,e) = p.communicate()

        warn = False
        gpu_utils = []
        gpu_lines = o.strip().split("\n")[1:]
        for gpu_line in gpu_lines:
            values = gpu_line.split(",")
            id = int(values[0])
            name = values[1]
            used = int(values[2].replace(" MiB", ""))
            total = int(values[3].replace(" MiB", ""))
            util = int(values[4].replace(" %", ""))

            stat.add("GPU {:d} ({:s}) Memory".format(id, name), "{:d} MB / {:d} MB ({:.2f}%)".format(used, total, 100.0 * used / total))
            stat.add("GPU {:d} ({:s}) Utilization".format(id, name), "{:d}%".format(util))
            gpu_utils.append(util)

            if util > self._warning_percentage:
                warn = True

        if warn:
            stat.summary(DiagnosticStatus.WARN,
                         "GPU utilization exceeds {:d}%".format(self._warning_percentage))
        else:
            stat.summary(DiagnosticStatus.OK, "Average GPU utilization {:.2f}%".format(sum(gpu_utils) / len(gpu_utils)))

        return stat

    @staticmethod
    def has_hardware():
        try:
            p = subprocess.Popen('nvidia-smi --query-gpu=index --format=csv', stdout = subprocess.PIPE,
                         stderr = subprocess.PIPE, shell = True)
            (o,e) = p.communicate()
            return len(o.strip().split("\n")) > 1
        except Exception:
            return False



def main():
    hostname = socket.gethostname()
    rclpy.init()
    node = rclpy.create_node('computer_monitor')
    node.declare_parameter('diag_thresholds_file', rclpy.Parameter.Type.STRING)

    # Load config file
    with open(node.get_parameter('diag_thresholds_file').value, 'r') as stream:
        thresholds_file = yaml.safe_load(stream)
    thresholds = thresholds_file["computer_monitor_thresholds"]

    # Create diagnostics updater
    updater = diagnostic_updater.Updater(node)
    updater.setHardwareID(hostname)

    updater.add(CpuTask(thresholds["cpu_warning_percentage"]))
    updater.add(MemoryTask(thresholds["memory_warning_percentage"]))
    updater.add(DiskTask(thresholds["disk_warning_percentage"]))
    if ComputerTempTask.has_hardware():
        updater.add(ComputerTempTask(thresholds["temp_warning_percentage"], thresholds["computer_error_temp_c"]))
    if CoreTempTask.has_hardware():
        updater.add(CoreTempTask(thresholds["temp_warning_percentage"]))
    if GPUTask.has_hardware():
        updater.add(GPUTask(thresholds["gpu_warning_percentage"]))

    updater.force_update()

    rclpy.spin(node, None)


if __name__ == '__main__':
    main()