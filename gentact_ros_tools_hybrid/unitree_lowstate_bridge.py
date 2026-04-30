#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from unitree_hg.msg import LowState


class UnitreeLowStateBridge(Node):
    def __init__(self):
        super().__init__("unitree_lowstate_bridge")

        self.declare_parameter("lowstate_topic", "/lowstate")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("num_joints", 27)
        self.declare_parameter("joint_names", [""])

        ls_topic = self.get_parameter("lowstate_topic").value
        js_topic = self.get_parameter("joint_states_topic").value
        self.n = int(self.get_parameter("num_joints").value)
        self.names = list(self.get_parameter("joint_names").value)

        if len(self.names) != self.n:
            raise RuntimeError(
                f"joint_names ({len(self.names)}) != num_joints ({self.n})"
            )

        # H1-2 /lowstate publisher is RELIABLE per ros2 topic info,
        # but BEST_EFFORT also matches and is safer for high-rate streams.
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.pub = self.create_publisher(JointState, js_topic, 10)
        self.sub = self.create_subscription(LowState, ls_topic, self._cb, sensor_qos)

        self._msg_count = 0
        self.create_timer(2.0, self._heartbeat)

        self.get_logger().info(
            f"Bridge running: {ls_topic} -> {js_topic} ({self.n} joints)"
        )

    def _heartbeat(self):
        if self._msg_count == 0:
            self.get_logger().warn(
                "No LowState messages received in last 2s — check topic name and source."
            )
        self._msg_count = 0

    def _cb(self, msg):
        try:
            if len(msg.motor_state) < self.n:
                self.get_logger().warn(
                    f"motor_state has {len(msg.motor_state)} entries, need {self.n}",
                    throttle_duration_sec=2.0,
                )
                return
            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()
            js.name = self.names
            js.position = [float(msg.motor_state[i].q) for i in range(self.n)]
            js.velocity = [float(msg.motor_state[i].dq) for i in range(self.n)]
            js.effort = [float(msg.motor_state[i].tau_est) for i in range(self.n)]
            self.pub.publish(js)
            self._msg_count += 1
        except Exception as e:
            self.get_logger().error(f"callback failed: {e}", throttle_duration_sec=1.0)


def main():
    rclpy.init()
    node = UnitreeLowStateBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
