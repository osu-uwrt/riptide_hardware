import rclpy.clock

def get_time_latency(clock, start_time):
    now = clock.now().to_msg()
    return (now.sec - start_time.sec) + ((now.nanosec - start_time.nanosec) / 1000000000.0)

class ExpiringMessage:
    def __init__(self, clock: rclpy.clock.Clock, message_life: float, has_header=False):
        self._clock = clock
        self._has_header = has_header

        self._value = None
        self._receive_time = None
        self._message_life = message_life
    
    def update_value(self, value):
        self._value = value
        if self._has_header:
            self._receive_time = value.header.stamp
        else:
            self._receive_time = self._clock.now().to_msg()
    
    def get_value(self):
        if self._receive_time is None:
            return None

        latency = get_time_latency(self._clock, self._receive_time)
        if latency < self._message_life:
            return self._value
        else:
            return None