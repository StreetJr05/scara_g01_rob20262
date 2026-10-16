"""scara_bridge: glue between the ESP32 (micro-ROS) and the rest of ROS 2.

* Holds every runtime-tunable controller value as a ROS 2 parameter
  (loaded from config/scara_params.yaml) and pushes them to the firmware on
  /scara/params: at start-up, whenever the ESP32 (re)connects or reboots, and
  whenever a parameter changes:
      ros2 param set /scara_bridge j1_kp 5.0
* /scara/state  -> /joint_states (RViz + robot_state_publisher)
                -> /scara/tool_pose (PointStamped, from the firmware FK)
* /scara/cmd_pose -> /scara/target_marker (RViz sphere at the requested point)
* /scara/event  -> console log
"""
import rclpy
from geometry_msgs.msg import Point, PointStamped
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray, String
from visualization_msgs.msg import Marker

from scara_control import protocol as P


class ScaraBridge(Node):

    def __init__(self):
        super().__init__('scara_bridge')
        self.declare_parameter('p_stroke', 0.04)        # must match the xacro
        self.declare_parameter('frame_id', 'world')
        any_number = ParameterDescriptor(dynamic_typing=True)
        for name in P.PARAM_IDS:
            self.declare_parameter(name, 0.0, any_number)

        self.pub_params = self.create_publisher(Float32MultiArray, P.TOPIC_PARAMS, 10)
        self.pub_js = self.create_publisher(JointState, '/joint_states', 10)
        self.pub_tool = self.create_publisher(PointStamped, '/scara/tool_pose', 10)
        self.pub_marker = self.create_publisher(Marker, '/scara/target_marker', 10)

        self.create_subscription(Float32MultiArray, P.TOPIC_STATE, self.on_state, qos_profile_sensor_data)
        self.create_subscription(String, P.TOPIC_EVENT, self.on_event, 50)
        self.create_subscription(Point, P.TOPIC_CMD_POSE, self.on_target, 10)

        self.add_on_set_parameters_callback(self.on_param_change)
        self.last_uptime = None
        self.last_state_time = None
        self.need_push = True
        self.create_timer(0.5, self.watchdog)
        self.get_logger().info('scara_bridge ready, waiting for /scara/state ...')

    # ----------------------------------------------------------- parameters
    def push_params(self):
        data = []
        for name, pid in P.PARAM_IDS.items():
            data += [float(pid), float(self.get_parameter(name).value)]
        if data:
            self.pub_params.publish(Float32MultiArray(data=data))
            self.get_logger().info(f'sent {len(data) // 2} controller parameters to the ESP32')

    def on_param_change(self, params):
        # The new values are not committed yet, so send them from `params`.
        data = []
        for p in params:
            if p.name in P.PARAM_IDS:
                data += [float(P.PARAM_IDS[p.name]), float(p.value)]
        if data:
            self.pub_params.publish(Float32MultiArray(data=data))
            self.get_logger().info(f'updated {len(data) // 2} parameter(s) on the ESP32')
        return SetParametersResult(successful=True)

    def watchdog(self):
        now = self.get_clock().now()
        if self.last_state_time is not None and (now - self.last_state_time).nanoseconds > 1.5e9:
            if not self.need_push:
                self.get_logger().warn('lost /scara/state (ESP32 disconnected?)')
            self.need_push = True

    # ---------------------------------------------------------------- state
    def on_state(self, msg):
        s = msg.data
        if len(s) < P.ST_LEN:
            return
        self.last_state_time = self.get_clock().now()
        uptime = s[P.ST_UPTIME]
        if self.last_uptime is not None and uptime + 0.5 < self.last_uptime:
            self.get_logger().warn('ESP32 rebooted')
            self.need_push = True
        self.last_uptime = uptime
        if self.need_push:
            self.need_push = False
            self.push_params()

        stamp = self.get_clock().now().to_msg()
        flags = P.flags_of(s)
        js = JointState()
        js.header.stamp = stamp
        js.name = ['joint1', 'joint2', 'joint3']
        js.position = [float(s[P.ST_Q1]), float(s[P.ST_Q2]),
                       float(self.get_parameter('p_stroke').value) if flags & P.FLAG_P_DOWN else 0.0]
        js.velocity = [float(s[P.ST_DQ1]), float(s[P.ST_DQ2]), 0.0]
        self.pub_js.publish(js)

        tool = PointStamped()
        tool.header.stamp = stamp
        tool.header.frame_id = self.get_parameter('frame_id').value
        tool.point = Point(x=float(s[P.ST_X]), y=float(s[P.ST_Y]), z=float(s[P.ST_Z]))
        self.pub_tool.publish(tool)

    def on_event(self, msg):
        text = msg.data
        if text.startswith(('FAULT', 'REJECT', 'WARN')):
            self.get_logger().warn(text)
        else:
            self.get_logger().info(text)

    def on_target(self, p):
        m = Marker()
        m.header.frame_id = self.get_parameter('frame_id').value
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns, m.id, m.type, m.action = 'target', 0, Marker.SPHERE, Marker.ADD
        m.pose.position = p
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.012
        m.color.r, m.color.g, m.color.b, m.color.a = 0.1, 0.9, 0.2, 0.9
        self.pub_marker.publish(m)


def main():
    rclpy.init()
    node = ScaraBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
