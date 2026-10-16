"""Small blocking client used by the CLI tools (goto, characterize)."""
import time

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32MultiArray, Int32, String

from scara_control import protocol as P


class ScaraClient(Node):

    def __init__(self, name='scara_client', print_events=True):
        super().__init__(name)
        self.state = None
        self.events = []
        self.print_events = print_events
        self.pub_pose = self.create_publisher(Point, P.TOPIC_CMD_POSE, 10)
        self.pub_cmd = self.create_publisher(Int32, P.TOPIC_CMD, 10)
        self.create_subscription(Float32MultiArray, P.TOPIC_STATE, self._on_state, qos_profile_sensor_data)
        self.create_subscription(String, P.TOPIC_EVENT, self._on_event, 50)

    def _on_state(self, msg):
        self.state = list(msg.data)

    def _on_event(self, msg):
        self.events.append(msg.data)
        if self.print_events:
            print(f'  [esp32] {msg.data}', flush=True)

    # ------------------------------------------------------------------ utils
    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait_until(self, pred, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)
            if self.state is not None and pred(self.state):
                return True
        return False

    def wait_for_robot(self, timeout=10.0):
        """Wait until /scara/state arrives (agent + ESP32 connected)."""
        return self.wait_until(lambda s: True, timeout)

    def flags(self):
        return P.flags_of(self.state)

    def homed(self):
        return bool(self.flags() & P.FLAG_HOMED)

    # --------------------------------------------------------------- commands
    def send_cmd(self, cmd):
        self.pub_cmd.publish(Int32(data=int(cmd)))

    def home(self, timeout=60.0):
        self.send_cmd(P.CMD_HOME)
        self.wait_until(lambda s: int(s[P.ST_MODE]) == P.MODE_HOMING, 2.0)
        ok = self.wait_until(
            lambda s: int(s[P.ST_MODE]) != P.MODE_HOMING, timeout)
        return ok and self.homed()

    def goto(self, x, y, z, timeout=30.0):
        """Send a Cartesian target [m] and block until it is reached.

        Returns (ok: bool, info: str).
        """
        if self.state is None and not self.wait_for_robot():
            return False, 'no /scara/state (is the agent running and the ESP32 connected?)'
        prev_id = int(self.state[P.ST_MOVE_ID])
        self.pub_pose.publish(Point(x=float(x), y=float(y), z=float(z)))

        if not self.wait_until(lambda s: int(s[P.ST_MOVE_ID]) > prev_id, 3.0):
            return False, 'firmware did not acknowledge the target'

        def done(s):
            err = int(s[P.ST_ERROR])
            return (P.flags_of(s) & P.FLAG_REACHED) or err in P.REJECT_ERRORS \
                or int(s[P.ST_MODE]) == P.MODE_FAULT

        if not self.wait_until(done, timeout):
            return False, 'timeout'
        s = self.state
        err = int(s[P.ST_ERROR])
        if int(s[P.ST_MODE]) == P.MODE_FAULT:
            return False, f'FAULT: {P.ERR_NAMES[err]}'
        if err in P.REJECT_ERRORS:
            return False, f'rejected: {P.ERR_NAMES[err]}'
        return True, 'reached' + (' (settle timeout)' if err == P.ERR_SETTLE else '')
