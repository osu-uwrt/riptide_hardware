#!/usr/bin/env python

import socket
import psutil
import rospy
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_updater import DiagnosticTask, Updater
import shutil
import subprocess


class CpuTask(DiagnosticTask):
    def __init__(self, warning_percentage):
        DiagnosticTask.__init__(self, "CPU Information")
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

class CoreTempTask(DiagnosticTask):
    def __init__(self, warning_percentage):
        DiagnosticTask.__init__(self, "Core Temperature")
        self._warning_percentage = int(warning_percentage)

    def run(self, stat):
        core_temps = psutil.sensors_temperatures()["coretemp"]

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

    @staticmethod
    def has_hardware():
        return "coretemp" in psutil.sensors_temperatures()


class MemoryTask(DiagnosticTask):
    def __init__(self, warning_percentage):
        DiagnosticTask.__init__(self, "Memory Information")
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

class DiskTask(DiagnosticTask):
    def __init__(self, warning_percentage):
        DiagnosticTask.__init__(self, "Disk Information")
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

class GPUTask(DiagnosticTask):
    def __init__(self, warning_percentage):
        DiagnosticTask.__init__(self, "GPU Information")
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
    rospy.init_node("computer_monitor")

    updater = Updater()
    updater.setHardwareID(hostname)

    updater.add(CpuTask(rospy.get_param("~cpu_warning_percentage", 90)))
    updater.add(MemoryTask(rospy.get_param("~memory_warning_percentage", 90)))
    updater.add(DiskTask(rospy.get_param("~disk_warning_percentage", 90)))
    if CoreTempTask.has_hardware():
        updater.add(CoreTempTask(rospy.get_param("~temp_warning_percentage", 90)))
    if GPUTask.has_hardware():
        updater.add(GPUTask(rospy.get_param("~gpu_warning_percentage", 90)))

    rate = rospy.Rate(rospy.get_param("~rate", 1))
    while not rospy.is_shutdown():
        rate.sleep()
        updater.update()


if __name__ == '__main__':
    main()